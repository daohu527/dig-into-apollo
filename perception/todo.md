# URL
https://blog.csdn.net/jinzhuojun/article/details/80875264  
https://blog.csdn.net/jinzhuojun/article/details/83038279  
https://zhuanlan.zhihu.com/p/33416142  
https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md  


# 感知综述
https://zhuanlan.zhihu.com/p/33416142  



## ContiArsDetector::RawObs2Frame
功能: 雷达原始数据转换为一帧  
输入: 其中ContiRadar的消息格式在"modules/drivers/proto/conti_radar.proto"中，这部分是Radar输出的数据。  
输出: 一帧数据

```
void ContiArsDetector::RawObs2Frame(
    const drivers::ContiRadar& corrected_obstacles,
    const DetectorOptions& options, base::FramePtr radar_frame) {
  // 雷达到世界姿势？
  const Eigen::Matrix4d& radar2world = *(options.radar2world_pose);
  // ?
  const Eigen::Matrix4d& radar2novatel = *(options.radar2novatel_trans);
  // 角速度
  const Eigen::Vector3f& angular_speed = options.car_angular_speed;
  Eigen::Matrix3d rotation_novatel;
  rotation_novatel << 0, -angular_speed(2), angular_speed(1), angular_speed(2),
      0, -angular_speed(0), -angular_speed(1), angular_speed(0), 0;
  Eigen::Matrix3d rotation_radar = radar2novatel.topLeftCorner(3, 3).inverse() *
                                   rotation_novatel *
                                   radar2novatel.topLeftCorner(3, 3);
  // 旋转
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
    if (motion_state == CONTI_MOVING || motion_state == CONTI_ONCOMING ||
        motion_state == CONTI_CROSSING_MOVING) {
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
    // 为什么mock的都是一个点
    MockRadarPolygon(radar_object);

    float local_range = static_cast<float>(local_loc.head(2).norm());
    float local_angle =
        static_cast<float>(std::atan2(local_loc(1), local_loc(0)));
    // 范围
    radar_object->radar_supplement.range = local_range;
    // 角度
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

## HdmapRadarRoiFilter::RoiFilter
功能: 过滤感兴趣区域
```
bool HdmapRadarRoiFilter::RoiFilter(const RoiFilterOptions& options,
                                    base::FramePtr radar_frame) {
  std::vector<base::ObjectPtr> origin_objects = radar_frame->objects;
  return common::ObjectInRoiCheck(options.roi, origin_objects,
                                  &radar_frame->objects);
}
```

#### ObjectInRoiCheck
perception/common/geometry/roi_filter.cc
确认目标是否在感兴趣区域
```
bool ObjectInRoiCheck(const HdmapStructConstPtr roi,
                      const std::vector<ObjectPtr>& objects,
                      std::vector<ObjectPtr>* valid_objects) {
  if (roi == nullptr ||
      (roi->road_polygons.empty() && roi->junction_polygons.empty())) {
    valid_objects->assign(objects.begin(), objects.end());
    return true;
  }

  valid_objects->clear();
  valid_objects->reserve(objects.size());
  for (std::size_t i = 0; i < objects.size(); i++) {
    if (IsObjectInRoi(roi, objects[i])) {
      valid_objects->push_back(objects[i]);
    }
  }

  return valid_objects->size() > 0;
}
```

## ContiArsTracker::Track
跟踪
```
bool ContiArsTracker::Track(const base::Frame &detected_frame,
                            const TrackerOptions &options,
                            base::FramePtr tracked_frame) {
  // 跟踪物体
  TrackObjects(detected_frame);
  
  // 收集跟踪Frame
  CollectTrackedFrame(tracked_frame);
  return true;
}
```

## ContiArsTracker::UpdateAssignedTracks
更新分配了的track
```
void ContiArsTracker::UpdateAssignedTracks(
    const base::Frame &radar_frame, std::vector<TrackObjectPair> assignments) {
  auto &radar_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < assignments.size(); ++i) {
    radar_tracks[assignments[i].first]->UpdataObsRadar(
        radar_frame.objects[assignments[i].second], radar_frame.timestamp);
  }
}
```

## ContiArsTracker::UpdateUnassignedTracks
更新未分配的track，主要是把没有用的设置成dead状态
```
void ContiArsTracker::UpdateUnassignedTracks(
    const base::Frame &radar_frame,
    const std::vector<size_t> &unassigned_tracks) {
  double timestamp = radar_frame.timestamp;
  auto &radar_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    if (radar_tracks[unassigned_tracks[i]]->GetObs() != nullptr) {
      double radar_time = radar_tracks[unassigned_tracks[i]]->GetTimestamp();
      double time_diff = fabs(timestamp - radar_time);
      if (time_diff > s_tracking_time_win_) {
        radar_tracks[unassigned_tracks[i]]->SetDead();
      }
    } else {
      radar_tracks[unassigned_tracks[i]]->SetDead();
    }
  }
}
```

## DeleteLostTracks
删除丢失的track，承接上一步

```
void ContiArsTracker::DeleteLostTracks() { track_manager_->RemoveLostTracks(); }
```
#### RadarTrackManager::RemoveLostTracks
把dead状态的track删除
```
int RadarTrackManager::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (!tracks_[i]->IsDead()) {
      if (i != track_count) {
        tracks_[track_count] = tracks_[i];
      }
      ++track_count;
    }
  }
  int removed_count = static_cast<int>(tracks_.size() - track_count);
  ADEBUG << "Remove " << removed_count << " tracks";
  tracks_.resize(track_count);
  return static_cast<int>(track_count);
}
```


## 

