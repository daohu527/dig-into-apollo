<a name="radar_module" />

## radar

毫米波雷达的处理流程相对比较简单，毫米波雷达的入口在"RadarDetectionComponent"中，在"RadarDetectionComponent"中首先对毫米波雷达数据做预处理(ContiArsPreprocessor)，实际上就是对毫米波雷达每一帧的时间做校正，之后在对毫米波雷达障碍物做识别"RadarObstaclePerception"，由于毫米波雷达可以直接输出障碍物的信息，识别的工作实际上已经简化了，识别出障碍物之后再根据ROI区域对检测结果做过滤，最后再采用卡尔曼滤波和匈牙利匹配对障碍物做追踪和匹配，上述这种追踪算法实际上是SORT算法，论文是2016年发表的"Simple Online and Realtime Tracking"。  

下图是毫米波雷达模块的整个调用流程，可以看到在"RadarDetectionComponent"模块中实现了预处理(ContiArsPreprocessor)和障碍物检测(RadarObstaclePerception)。而障碍物检测"RadarObstaclePerception"在radar目录中实现，主要分为检测、感兴趣区域过滤、追踪3个步骤。  
![radar_process](../img/radar_process.jpg)  

**疑问**  
不知道毫米波雷达对静态障碍物的检测和识别效果怎么样？  

下面我们主要介绍radar目录的具体实现，实际上radar,camera,lidar的目录结构都大体类似，在app中申明功能，在lib中实现。  
```
.
├── app           // 功能定义
├── common
└── lib
    ├── detector  // 物体检测
    ├── dummy
    ├── interface
    ├── preprocessor  // 预处理
    ├── roi_filter    // 感兴趣区域过滤
    └── tracker       // 目标追踪
        ├── common      
        ├── conti_ars_tracker
        ├── filter          // 卡尔曼滤波
        └── matcher         // 匈牙利算法
```

## app
毫米波雷达的实现在"radar_obstacle_perception.cc"中实现，实现的类为"RadarObstaclePerception"。主要调用了"lib"目录中的方法，来实现雷达检测目标的输出。  

#### Init初始化
Init初始化中会指定预处理器，识别器，追踪器等几个组件，接下来毫米波雷达会采用上述组件进行障碍物的识别和追踪。  
```c++
bool RadarObstaclePerception::Init(const std::string& pipeline_name) {
  std::string model_name = pipeline_name;
  // 1. 读取配置
  const ModelConfig* model_config = nullptr;
  ACHECK(ConfigManager::Instance()->GetModelConfig(model_name, &model_config))
      << "not found model: " << model_name;

  std::string detector_name;
  ACHECK(model_config->get_value("Detector", &detector_name))
      << "Detector not found";

  std::string roi_filter_name;
  ACHECK(model_config->get_value("RoiFilter", &roi_filter_name))
      << "RoiFilter not found";

  std::string tracker_name;
  ACHECK(model_config->get_value("Tracker", &tracker_name))
      << "Tracker not found";
  
  // 2. 给指定的识别器，过滤器，追踪器赋值
  BaseDetector* detector =
      BaseDetectorRegisterer::GetInstanceByName(detector_name);
  CHECK_NOTNULL(detector);
  detector_.reset(detector);

  BaseRoiFilter* roi_filter =
      BaseRoiFilterRegisterer::GetInstanceByName(roi_filter_name);
  CHECK_NOTNULL(roi_filter);
  roi_filter_.reset(roi_filter);

  BaseTracker* tracker = BaseTrackerRegisterer::GetInstanceByName(tracker_name);
  CHECK_NOTNULL(tracker);
  tracker_.reset(tracker);
  
  // 3. 识别器，过滤器，追踪器初始化
  ACHECK(detector_->Init()) << "radar detector init error";
  ACHECK(roi_filter_->Init()) << "radar roi filter init error";
  ACHECK(tracker_->Init()) << "radar tracker init error";

  return true;
}
```

#### Perceive识别
接下来是整个识别流程，输入为障碍物，配置选项，输出为障碍物对象。其中处理器，识别器，追踪器具体的实现在"lib"目录中。  
```c++
bool RadarObstaclePerception::Perceive(
    const drivers::ContiRadar& corrected_obstacles,
    const RadarPerceptionOptions& options,
    std::vector<base::ObjectPtr>* objects) {

  const std::string& sensor_name = options.sensor_name;
  base::FramePtr detect_frame_ptr(new base::Frame());
  // 1. 识别障碍物
  if (!detector_->Detect(corrected_obstacles, options.detector_options,
                         detect_frame_ptr)) {
    AERROR << "radar detect error";
    return false;
  }

  // 2. 感兴趣区域过滤
  if (!roi_filter_->RoiFilter(options.roi_filter_options, detect_frame_ptr)) {
    ADEBUG << "All radar objects were filtered out";
  }

  // 3. 追踪
  base::FramePtr tracker_frame_ptr = std::make_shared<base::Frame>();
  if (!tracker_->Track(*detect_frame_ptr, options.track_options,
                       tracker_frame_ptr)) {
    AERROR << "radar track error";
    return false;
  }
  
  // 4. 输出结果
  *objects = tracker_frame_ptr->objects;

  return true;
}
```


## lib目录

#### preprocessor 预处理
预处理主要是对时间戳做预处理。  


#### detector 识别
这里不是获取的雷达的raw_data而是直接获取的radar输出的感知结果。实现的类为"ContiArsDetector"主要实现了"Detect"方法。  
```c++
bool ContiArsDetector::Detect(const drivers::ContiRadar& corrected_obstacles,
                              const DetectorOptions& options,
                              base::FramePtr radar_frame) {
  // 1. 通过校准好的数据填充雷达帧
  RawObs2Frame(corrected_obstacles, options, radar_frame);
  radar_frame->timestamp = corrected_obstacles.header().timestamp_sec();
  radar_frame->sensor2world_pose = *(options.radar2world_pose);
  return true;
}
```

具体在雷达帧中填充了什么数据呢？下面我们来看下具体的实现。  
```c++
void ContiArsDetector::RawObs2Frame(
    const drivers::ContiRadar& corrected_obstacles,
    const DetectorOptions& options, base::FramePtr radar_frame) {
  // 1. 雷达到世界，雷达到IMU的位置关系，以及车的角速度
  const Eigen::Matrix4d& radar2world = *(options.radar2world_pose);
  const Eigen::Matrix4d& radar2novatel = *(options.radar2novatel_trans);
  const Eigen::Vector3f& angular_speed = options.car_angular_speed;
  Eigen::Matrix3d rotation_novatel;

  rotation_novatel << 0, -angular_speed(2), angular_speed(1), angular_speed(2),
      0, -angular_speed(0), -angular_speed(1), angular_speed(0), 0;
  Eigen::Matrix3d rotation_radar = radar2novatel.topLeftCorner(3, 3).inverse() *
                                   rotation_novatel *
                                   radar2novatel.topLeftCorner(3, 3);
  Eigen::Matrix3d radar2world_rotate = radar2world.block<3, 3>(0, 0);
  Eigen::Matrix3d radar2world_rotate_t = radar2world_rotate.transpose();
  // Eigen::Vector3d radar2world_translation = radar2world.block<3, 1>(0, 3);
  ADEBUG << "radar2novatel: " << radar2novatel;
  ADEBUG << "angular_speed: " << angular_speed;
  ADEBUG << "rotation_radar: " << rotation_radar;
  for (const auto radar_obs : corrected_obstacles.contiobs()) {
    base::ObjectPtr radar_object = std::make_shared<base::Object>();
    radar_object->id = radar_obs.obstacle_id();
    radar_object->track_id = radar_obs.obstacle_id();
    Eigen::Vector4d local_loc(radar_obs.longitude_dist(),
                              radar_obs.lateral_dist(), 0, 1);
    Eigen::Vector4d world_loc =
        static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(radar2world *
                                                          local_loc);
    radar_object->center = world_loc.block<3, 1>(0, 0);
    radar_object->anchor_point = radar_object->center;

    Eigen::Vector3d local_vel(radar_obs.longitude_vel(),
                              radar_obs.lateral_vel(), 0);

    Eigen::Vector3d angular_trans_speed =
        rotation_radar * local_loc.topLeftCorner(3, 1);
    Eigen::Vector3d world_vel =
        static_cast<Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
            radar2world_rotate * (local_vel + angular_trans_speed));
    Eigen::Vector3d vel_temp =
        world_vel + options.car_linear_speed.cast<double>();
    radar_object->velocity = vel_temp.cast<float>();

    Eigen::Matrix3d dist_rms;
    dist_rms.setZero();
    Eigen::Matrix3d vel_rms;
    vel_rms.setZero();
    dist_rms(0, 0) = radar_obs.longitude_dist_rms();
    dist_rms(1, 1) = radar_obs.lateral_dist_rms();
    vel_rms(0, 0) = radar_obs.longitude_vel_rms();
    vel_rms(1, 1) = radar_obs.lateral_vel_rms();
    radar_object->center_uncertainty =
        (radar2world_rotate * dist_rms * dist_rms.transpose() *
         radar2world_rotate_t)
            .cast<float>();

    radar_object->velocity_uncertainty =
        (radar2world_rotate * vel_rms * vel_rms.transpose() *
         radar2world_rotate_t)
            .cast<float>();
    double local_obj_theta = radar_obs.oritation_angle() / 180.0 * PI;
    Eigen::Vector3f direction(static_cast<float>(cos(local_obj_theta)),
                              static_cast<float>(sin(local_obj_theta)), 0.0f);
    direction = radar2world_rotate.cast<float>() * direction;
    radar_object->direction = direction;
    radar_object->theta = std::atan2(direction(1), direction(0));
    radar_object->theta_variance =
        static_cast<float>(radar_obs.oritation_angle_rms() / 180.0 * PI);
    radar_object->confidence = static_cast<float>(radar_obs.probexist());

    int motion_state = radar_obs.dynprop();
    double prob_target = radar_obs.probexist();
    if ((prob_target > MIN_PROBEXIST) &&
        (motion_state == CONTI_MOVING || motion_state == CONTI_ONCOMING ||
         motion_state == CONTI_CROSSING_MOVING)) {
      radar_object->motion_state = base::MotionState::MOVING;
    } else if (motion_state == CONTI_DYNAMIC_UNKNOWN) {
      radar_object->motion_state = base::MotionState::UNKNOWN;
    } else {
      radar_object->motion_state = base::MotionState::STATIONARY;
      radar_object->velocity.setZero();
    }

    int cls = radar_obs.obstacle_class();
    if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
      radar_object->type = base::ObjectType::VEHICLE;
    } else if (cls == CONTI_PEDESTRIAN) {
      radar_object->type = base::ObjectType::PEDESTRIAN;
    } else if (cls == CONTI_MOTOCYCLE || cls == CONTI_BICYCLE) {
      radar_object->type = base::ObjectType::BICYCLE;
    } else {
      radar_object->type = base::ObjectType::UNKNOWN;
    }

    radar_object->size(0) = static_cast<float>(radar_obs.length());
    radar_object->size(1) = static_cast<float>(radar_obs.width());
    radar_object->size(2) = 2.0f;  // vehicle template (pnc required)
    if (cls == CONTI_POINT) {
      radar_object->size(0) = 1.0f;
      radar_object->size(1) = 1.0f;
    }
    // extreme case protection
    if (radar_object->size(0) * radar_object->size(1) < 1.0e-4) {
      if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
        radar_object->size(0) = 4.0f;
        radar_object->size(1) = 1.6f;  // vehicle template
      } else {
        radar_object->size(0) = 1.0f;
        radar_object->size(1) = 1.0f;
      }
    }
    MockRadarPolygon(radar_object);

    float local_range = static_cast<float>(local_loc.head(2).norm());
    float local_angle =
        static_cast<float>(std::atan2(local_loc(1), local_loc(0)));
    radar_object->radar_supplement.range = local_range;
    radar_object->radar_supplement.angle = local_angle;

    radar_frame->objects.push_back(radar_object);

    ADEBUG << "obs_id: " << radar_obs.obstacle_id() << ", "
           << "long_dist: " << radar_obs.longitude_dist() << ", "
           << "lateral_dist: " << radar_obs.lateral_dist() << ", "
           << "long_vel: " << radar_obs.longitude_vel() << ", "
           << "latera_vel: " << radar_obs.lateral_vel() << ", "
           << "rcs: " << radar_obs.rcs() << ", "
           << "meas_state: " << radar_obs.meas_state();
  }
}
```


#### roi_filter  过滤
根据地图过滤感兴趣的区域。主要的实现在"HdmapRadarRoiFilter"中，主要是判断障碍物是否在感兴趣区域之内。  
```c++
bool HdmapRadarRoiFilter::RoiFilter(const RoiFilterOptions& options,
                                    base::FramePtr radar_frame) {
  std::vector<base::ObjectPtr> origin_objects = radar_frame->objects;
  // 1. 判断障碍物是否在感兴趣区域之内
  return common::ObjectInRoiCheck(options.roi, origin_objects,
                                  &radar_frame->objects);
}
```

**疑问**  
1. 感兴趣区域是如何组成的，为何定义的是点云的格式？？？  


## tracker 追踪
追踪的主要实现在"ContiArsTracker"中，用的算法是SORT算法，先对目标做检测，然后用卡尔曼滤波对物体的运动做估计，然后用匈牙利算法对多个目标做匹配，得到多个目标的跟踪结果。  

#### ContiArsTracker
初始化
```c++
bool ContiArsTracker::Init() {
  std::string model_name = name_;
  const lib::ModelConfig *model_config = nullptr;
  bool state = true;
  if (!lib::ConfigManager::Instance()->GetModelConfig(model_name,
                                                      &model_config)) {
    AERROR << "not found model: " << model_name;
    state = false;
  }
  if (!model_config->get_value("tracking_time_window", &s_tracking_time_win_)) {
    AERROR << "track_time_window is not found.";
    state = false;
  }
  if (!model_config->get_value("macher_name", &matcher_name_)) {
    AERROR << "macher_name is not found.";
    state = false;
  }
  std::string chosen_filter;
  if (!model_config->get_value("chosen_filter", &chosen_filter)) {
    AERROR << "chosen_filter is not found.";
    state = false;
  }
  RadarTrack::SetChosenFilter(chosen_filter);
  int tracked_times_threshold;
  if (!model_config->get_value("tracked_times_threshold",
                               &tracked_times_threshold)) {
    AERROR << "tracked_times_threshold is not found.";
    state = false;
  }
  RadarTrack::SetTrackedTimesThreshold(tracked_times_threshold);
  bool use_filter;
  if (!model_config->get_value("use_filter", &use_filter)) {
    AERROR << "use_filter is not found.";
    state = false;
  }
  RadarTrack::SetUseFilter(use_filter);
  // Or use register class instead.
  if (matcher_name_ == "HMMatcher") {
    matcher_ = new HMMatcher();
    matcher_->Init();  //  use proto later
  } else {
    AERROR << "Not supported matcher : " << matcher_name_;
    state = false;
  }

  track_manager_ = new RadarTrackManager();
  ACHECK(track_manager_ != nullptr)
      << "Failed to get RadarTrackManager instance.";
  return state;
}
```

追踪
```c++
bool ContiArsTracker::Track(const base::Frame &detected_frame,
                            const TrackerOptions &options,
                            base::FramePtr tracked_frame) {
  TrackObjects(detected_frame);
  CollectTrackedFrame(tracked_frame);
  return true;
}
```


#### AdaptiveKalmanFilter
自适应卡尔曼滤波器


#### HMMatcher
匈牙利匹配  
1. 配置文件目录  
配置文件在"modules\perception\production\data\perception\radar\models\tracker\hm_matcher.conf"中。  
```
max_match_distance : 2.5
bound_match_distance : 10.0
```

#### Match
首先是找到没有追踪到的目标，和丢失追踪的目标？这里的"unassigned_tracks"和"unassigned_objects"如何定义呢？  
```c++
bool HMMatcher::Match(const std::vector<RadarTrackPtr> &radar_tracks,
                      const base::Frame &radar_frame,
                      const TrackObjectMatcherOptions &options,
                      std::vector<TrackObjectPair> *assignments,
                      std::vector<size_t> *unassigned_tracks,
                      std::vector<size_t> *unassigned_objects) {
  // 1. traceid相等，并且距离小于max_match_distance
  IDMatch(radar_tracks, radar_frame, assignments, unassigned_tracks,
          unassigned_objects);
  TrackObjectPropertyMatch(radar_tracks, radar_frame, assignments,
                           unassigned_tracks, unassigned_objects);
  return true;
}
```


#### TrackObjectPropertyMatch

匹配目标检测和追踪  
```c++
void HMMatcher::TrackObjectPropertyMatch(
    const std::vector<RadarTrackPtr> &radar_tracks,
    const base::Frame &radar_frame, std::vector<TrackObjectPair> *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
  // 1. 如果任意一个为空，则结束，那么物体和追踪是如何对应的？？？  
  if (unassigned_tracks->empty() || unassigned_objects->empty()) {
    return;
  }
  std::vector<std::vector<double> > association_mat(unassigned_tracks->size());
  for (size_t i = 0; i < association_mat.size(); ++i) {
    association_mat[i].resize(unassigned_objects->size(), 0);
  }
  // 计算追踪物体和当前帧物体的距离，并且保存在association_mat中。
  ComputeAssociationMat(radar_tracks, radar_frame, *unassigned_tracks,
                        *unassigned_objects, &association_mat);

  // 把association_mat赋值给global_costs
  common::SecureMat<double> *global_costs =
      hungarian_matcher_.mutable_global_costs();
  global_costs->Resize(unassigned_tracks->size(), unassigned_objects->size());
  for (size_t i = 0; i < unassigned_tracks->size(); ++i) {
    for (size_t j = 0; j < unassigned_objects->size(); ++j) {
      (*global_costs)(i, j) = association_mat[i][j];
    }
  }
  // 通过直方图来计算匹配？？？  
  std::vector<TrackObjectPair> property_assignments;
  std::vector<size_t> property_unassigned_tracks;
  std::vector<size_t> property_unassigned_objects;
  hungarian_matcher_.Match(
      BaseMatcher::GetMaxMatchDistance(), BaseMatcher::GetBoundMatchDistance(),
      common::GatedHungarianMatcher<double>::OptimizeFlag::OPTMIN,
      &property_assignments, &property_unassigned_tracks,
      &property_unassigned_objects);

  // 是否目标和追踪到的目标数量一定相等？？？  
  // assignments 存放匹配好的对象，unassigned_tracks没有匹配的追踪，unassigned_objects没有匹配的目标
  for (size_t i = 0; i < property_assignments.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_assignments[i].first);
    size_t go_idx = unassigned_objects->at(property_assignments[i].second);
    assignments->push_back(std::pair<size_t, size_t>(gt_idx, go_idx));
  }
  std::vector<size_t> temp_unassigned_tracks;
  std::vector<size_t> temp_unassigned_objects;
  for (size_t i = 0; i < property_unassigned_tracks.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_unassigned_tracks[i]);
    temp_unassigned_tracks.push_back(gt_idx);
  }
  for (size_t i = 0; i < property_unassigned_objects.size(); ++i) {
    size_t go_idx = unassigned_objects->at(property_unassigned_objects[i]);
    temp_unassigned_objects.push_back(go_idx);
  }
  *unassigned_tracks = temp_unassigned_tracks;
  *unassigned_objects = temp_unassigned_objects;
}
```

匈牙利匹配算法？？？  
采用的是"Kuhn-Munkres Algorithm"来进行多目标追踪的。  
```c++
template <typename T>
void GatedHungarianMatcher<T>::Match(
    T cost_thresh, T bound_value, OptimizeFlag opt_flag,
    std::vector<std::pair<size_t, size_t>>* assignments,
    std::vector<size_t>* unassigned_rows,
    std::vector<size_t>* unassigned_cols) {

  /* initialize matcher */
  cost_thresh_ = cost_thresh;
  opt_flag_ = opt_flag;
  bound_value_ = bound_value;
  assignments_ptr_ = assignments;
  // 这里为何要检查cost_thresh < bound_value
  MatchInit();

  /* compute components */
  std::vector<std::vector<size_t>> row_components;
  std::vector<std::vector<size_t>> col_components;
  this->ComputeConnectedComponents(&row_components, &col_components);
  CHECK_EQ(row_components.size(), col_components.size());

  /* compute assignments */
  assignments_ptr_->clear();
  assignments_ptr_->reserve(std::max(rows_num_, cols_num_));
  for (size_t i = 0; i < row_components.size(); ++i) {
    this->OptimizeConnectedComponent(row_components[i], col_components[i]);
  }

  this->GenerateUnassignedData(unassigned_rows, unassigned_cols);
}
```
