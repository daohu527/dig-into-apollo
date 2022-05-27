## DetectionComponent
DetectionComponent主要的目的是用来做物体识别。接收点云信息，最后输出感知到的结果。
输入TOPIC： drivers::PointCloud
输出TOPIC： LidarFrameMessage


实际上在BUILD文件中，DetectionComponent等几个模块编译为一个模块，最后的可执行文件为"libperception_component_lidar"。也就是说DetectionComponent的配置文件在DAG中查找libperception_component_lidar中对应的"DetectionComponent"的配置。
疑问：  
目前并没有在dag中找到对应的配置文件，看起来这个模块是给第三方雷达感知算法提供的接口？？？  

#### 初始化(Init)
```c++
bool DetectionComponent::Init() {
  LidarDetectionComponentConfig comp_config;
  // 1. 读取配置文件
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  ADEBUG << "Lidar Component Configs: " << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  lidar2novatel_tf2_child_frame_id_ =
      comp_config.lidar2novatel_tf2_child_frame_id();
  lidar_query_tf_offset_ =
      static_cast<float>(comp_config.lidar_query_tf_offset());
  enable_hdmap_ = comp_config.enable_hdmap();
  // 2. 注册发布消息
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);
  
  // 3. 初始化算法插件
  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection component algorithm plugin.";
    return false;
  }
  return true;
}
```

#### InitAlgorithmPlugin

```c++
bool DetectionComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));
  // 1. 设置雷达障碍物识别器
  detector_.reset(new lidar::LidarObstacleDetection);

  lidar::LidarObstacleDetectionInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  init_options.enable_hdmap_input =
      FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  // 2. 初始化识别器，调用LidarObstacleDetection的Init方法
  if (!detector_->Init(init_options)) {
    AINFO << "sensor_name_ "
          << "Failed to init detection.";
    return false;
  }
  // 3. 初始化坐标转换关系
  lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
  return true;
}
```


#### 执行(Proc)
执行主要在Proc，而Proc主要是调用内部的实现"InternalProc"。  
```c++
bool DetectionComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  // 1. 为什么要加锁，难道有多个雷达的情况？？？
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(sensor_name_);
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }

  // 2. 初始化信息帧
  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = s_seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;

  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;

  PERCEPTION_PERF_BLOCK_START();
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      timestamp - lidar_query_tf_offset_ * 0.001;
  // 3. 获取车的姿态
  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp,
                                               &pose)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: " << lidar_query_tf_timestamp;
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      sensor_name_, "detection_1::get_lidar_to_world_pose");

  frame->lidar2world_pose = pose;

  lidar::LidarObstacleDetectionOptions detect_opts;
  detect_opts.sensor_name = sensor_name_;
  lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2novatel_extrinsics);

  // 4. 开始物体识别，调用LidarObstacleDetection的Process方法，下文会分析具体的实现
  lidar::LidarProcessResult ret =
      detector_->Process(detect_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name_,
                                           "detection_2::detect_obstacle");

  return true;
}
```

## LidarObstacleDetection
在DetectionComponent中会初始化LidarObstacleDetection，并且调用类的"Process()"方法，那么LidarObstacleDetection中究竟实现了哪些功能？  

下面先分析Init方法
#### 初始化Init
```c++
bool LidarObstacleDetection::Init(
    const LidarObstacleDetectionInitOptions& options) {
  // 1. 根据传感器名称获取模型配置
  auto& sensor_name = options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  // 2. 获取配置文件
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);
  config_file = cyber::common::GetAbsolutePath(
      config_file, "lidar_obstacle_detection.conf");

  // 3. 从配置文件中获取，探测器的名称，是否采用地图管理，过滤目录参数
  LidarObstacleDetectionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  detector_name_ = config.detector();
  use_map_manager_ = config.use_map_manager();
  use_object_filter_bank_ = config.use_object_filter_bank();

  use_map_manager_ = use_map_manager_ && options.enable_hdmap_input;

  // 4. 初始化场景管理
  SceneManagerInitOptions scene_manager_init_options;
  ACHECK(SceneManager::Instance().Init(scene_manager_init_options));

  // 5. 点云预处理
  PointCloudPreprocessorInitOptions preprocessor_init_options;
  preprocessor_init_options.sensor_name = sensor_name;
  ACHECK(cloud_preprocessor_.Init(preprocessor_init_options));

  // 6. 是否采用地图管理
  if (use_map_manager_) {
    MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }

  // 7. 初始化探测器为PointPillarsDetection
  detector_.reset(new PointPillarsDetection);
  // detector_.reset(
  //    BaseSegmentationRegisterer::GetInstanceByName(segmentor_name_));
  CHECK_NOTNULL(detector_.get());
  DetectionInitOptions detection_init_options;
  // segmentation_init_options.sensor_name = sensor_name;
  ACHECK(detector_->Init(detection_init_options));

  return true;
}
```


接着看LidarObstacleDetection如何进行物体识别
#### 执行Process
Process有多态实现，主要的区别为是否传入点云信息message。  
```c++
LidarProcessResult LidarObstacleDetection::Process(
    const LidarObstacleDetectionOptions& options,
    const std::shared_ptr<apollo::drivers::PointCloud const>& message,
    LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(options.sensor_name);

  PERCEPTION_PERF_BLOCK_START();
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "preprocess");

  // 1. 点云预处理
  if (cloud_preprocessor_.Preprocess(preprocessor_options, message, frame)) {
    // 2. 点云计算
    return ProcessCommon(options, frame);
  }
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}
```

#### 点云计算
```c++
LidarProcessResult LidarObstacleDetection::ProcessCommon(
    const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  if (use_map_manager_) {
    MapManagerOptions map_manager_options;
    // 1. 更新地图选项
    if (!map_manager_.Update(map_manager_options, frame)) {
      return LidarProcessResult(LidarErrorCode::MapManagerError,
                                "Failed to update map structure.");
    }
  }

  // 2. 进行物体识别
  DetectionOptions detection_options;
  if (!detector_->Detect(detection_options, frame)) {
    return LidarProcessResult(LidarErrorCode::DetectionError,
                              "Failed to detect.");
  }

  return LidarProcessResult(LidarErrorCode::Succeed);
}
```

上述的识别过程实际上在Init中设置为PointPillarsDetection，也就是说点云识别在PointPillarsDetection中实现。  


## PointPillarsDetection
PointPillarsDetection主要的实现在Init和Proc中。  

#### 初始化
在PointPillarsDetection中初始化"PointPillars"类
```c++
bool PointPillarsDetection::Init(const DetectionInitOptions& options) {
  point_pillars_ptr_.reset(
      new PointPillars(reproduce_result_mode_, score_threshold_,
                       nms_overlap_threshold_, FLAGS_pfe_onnx_file,
                       FLAGS_rpn_onnx_file));
  return true;
}
```

#### 实现(detect)
```c++
bool PointPillarsDetection::Detect(const DetectionOptions& options,
                                   LidarFrame* frame) {

  // record input cloud and lidar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();

  Timer timer;
  
  // 1. 设置GPU id
  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  // 2. 转化点云为数组
  float* points_array = new float[original_cloud_->size() * 4];
  PclToArray(original_cloud_, points_array, kNormalizingFactor);

  // 3. 进行推理
  std::vector<float> out_detections;
  point_pillars_ptr_->doInference(points_array, original_cloud_->size(),
                                  &out_detections);
  inference_time_ = timer.toc(true);

  // 4. 输出障碍物的boundbox
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
             &out_detections);

  AINFO << "PointPillars: inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;
  return true;
}
```

#### 输出障碍物
获取目标做分类，点云模型目前只输出了车的分类，因此无法做其他分类
```c++
void PointPillarsDetection::GetObjects(
    std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
    std::vector<float>* detections) {
  Timer timer;
  int num_objects = detections->size() / kOutputNumBoxFeature;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    float x = detections->at(i * kOutputNumBoxFeature + 0);
    float y = detections->at(i * kOutputNumBoxFeature + 1);
    float z = detections->at(i * kOutputNumBoxFeature + 2);
    float dx = detections->at(i * kOutputNumBoxFeature + 4);
    float dy = detections->at(i * kOutputNumBoxFeature + 3);
    float dz = detections->at(i * kOutputNumBoxFeature + 5);
    float yaw = detections->at(i * kOutputNumBoxFeature + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    float dx2cos = dx * cosf(yaw) / 2;
    float dy2sin = dy * sinf(yaw) / 2;
    float dx2sin = dx * sinf(yaw) / 2;
    float dy2cos = dy * cosf(yaw) / 2;
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    for (int j = 0; j < 2; ++j) {
      PointF point0, point1, point2, point3;
      float vz = z + (j == 0 ? 0 : dz);
      point0.x = x + dx2cos + dy2sin;
      point0.y = y + dx2sin - dy2cos;
      point0.z = vz;
      point1.x = x + dx2cos - dy2sin;
      point1.y = y + dx2sin + dy2cos;
      point1.z = vz;
      point2.x = x - dx2cos - dy2sin;
      point2.y = y - dx2sin + dy2cos;
      point2.z = vz;
      point3.x = x - dx2cos + dy2sin;
      point3.y = y - dx2sin - dy2cos;
      point3.z = vz;
      object->lidar_supplement.cloud.push_back(point0);
      object->lidar_supplement.cloud.push_back(point1);
      object->lidar_supplement.cloud.push_back(point2);
      object->lidar_supplement.cloud.push_back(point3);
    }
    for (auto& pt : object->lidar_supplement.cloud) {
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      object->lidar_supplement.cloud_world.push_back(world_point);
    }

    // classification (only detect vehicles so far)
    // TODO(chenjiahao): Fill object types completely
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->lidar_supplement.raw_probs
        .back()[static_cast<int>(base::ObjectType::VEHICLE)] = 1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
    object->type = static_cast<base::ObjectType>(
        std::distance(object->type_probs.begin(),
                      std::max_element(object->type_probs.begin(),
                                       object->type_probs.end())));
  }

  collect_time_ = timer.toc(true);
}
```

## PointPillars
PointPillars，一种新颖的编码器，它利用PointNets来学习在垂直列柱体组织中的点云的特征。 Apollo中的PointPillars是基于autoware。  
疑问：  
1. 具体的实现是如何实现的，大量用到了cuda
```c++
PointPillars::PointPillars(const bool reproduce_result_mode,
                           const float score_threshold,
                           const float nms_overlap_threshold,
                           const std::string pfe_onnx_file,
                           const std::string rpn_onnx_file)
```
其中网络在"pfe_onnx_file"和"rpn_onnx_file"中，那么大概判断PointPillars是否是自己实现了cuda对神经网络的加速计算？？？  

综上所述DetectionComponent主要实现了雷达的识别，识别的具体功能实现在"perception/lidar"中，同时"perception/lidar"还实现了雷达的分割和追踪，后面我们会接着分析这个模块。  





## FusionCameraDetectionComponent


#### 初始化Init

```c++
bool FusionCameraDetectionComponent::Init() {
  // 1. 初始化配置
  if (InitConfig() != cyber::SUCC) {
    AERROR << "InitConfig() failed.";
    return false;
  }
  // 2. 感知结果
  writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_obstacles_channel_name_);
  // 3. 提前融合传感器消息帧
  sensorframe_writer_ =
      node_->CreateWriter<SensorFrameMessage>(prefused_channel_name_);
  // 4. 相机可视化消息
  camera_viz_writer_ = node_->CreateWriter<CameraPerceptionVizMessage>(
      camera_perception_viz_message_channel_name_);
  // 5. 相机调试消息
  camera_debug_writer_ =
      node_->CreateWriter<apollo::perception::camera::CameraDebug>(
          camera_debug_channel_name_);
  // 6. 初始化场景
  if (InitSensorInfo() != cyber::SUCC) {
    AERROR << "InitSensorInfo() failed.";
    return false;
  }
  // 7. 初始化算法
  if (InitAlgorithmPlugin() != cyber::SUCC) {
    AERROR << "InitAlgorithmPlugin() failed.";
    return false;
  }
  // 8. 初始化相机帧
  if (InitCameraFrames() != cyber::SUCC) {
    AERROR << "InitCameraFrames() failed.";
    return false;
  }
  // 9. 初始化矩阵
  if (InitProjectMatrix() != cyber::SUCC) {
    AERROR << "InitProjectMatrix() failed.";
    return false;
  }
  // 10. 初始化相机监听
  if (InitCameraListeners() != cyber::SUCC) {
    AERROR << "InitCameraListeners() failed.";
    return false;
  }
  // 11. 初始化移动服务？？？
  if (InitMotionService() != cyber::SUCC) {
    AERROR << "InitMotionService() failed.";
    return false;
  }

  // 12. 设置相机高度和角度
  SetCameraHeightAndPitch();

  // Init visualizer
  // TODO(techoe, yg13): homography from image to ground should be
  // computed from camera height and pitch.
  // Apply online calibration to adjust pitch/height automatically
  // Temporary code is used here for test

  double pitch_adj_degree = 0.0;
  double yaw_adj_degree = 0.0;
  double roll_adj_degree = 0.0;
  // load in lidar to imu extrinsic
  Eigen::Matrix4d ex_lidar2imu;
  LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" +
                     "velodyne128_novatel_extrinsics.yaml",
                 &ex_lidar2imu);
  AINFO << "velodyne128_novatel_extrinsics: " << ex_lidar2imu;

  // 13. 可视化
  ACHECK(visualize_.Init_all_info_single_camera(
      camera_names_, visual_camera_, intrinsic_map_, extrinsic_map_,
      ex_lidar2imu, pitch_adj_degree, yaw_adj_degree, roll_adj_degree,
      image_height_, image_width_));

  homography_im2car_ = visualize_.homography_im2car(visual_camera_);
  camera_obstacle_pipeline_->SetIm2CarHomography(homography_im2car_);

  // 14. 车道前方最危险目标检测
  if (enable_cipv_) {
    cipv_.Init(homography_im2car_, min_laneline_length_for_cipv_,
               average_lane_width_in_meter_, max_vehicle_width_in_meter_,
               average_frame_rate_, image_based_cipv_, debug_level_);
  }
  
  // 15. 使能可视化
  if (enable_visualization_) {
    if (write_visual_img_) {
      visualize_.write_out_img_ = true;
      visualize_.SetDirectory(visual_debug_folder_);
    }
  }

  return true;
}
```

算法部分的实现在"InitAlgorithmPlugin()"中初始化，具体的实现在"ObstacleCameraPerception"中
## ObstacleCameraPerception

#### 初始化Init
相机检测的初始化比较复杂，分为了几个方面。  
其中读取的配置在文件"perception\production\conf\perception\camera\obstacle.pt"中，这里的".pt"文件是否是pytorch传统概念上的pt文件？？？如果打开实际上是一个配置文件，并且指定了具体的pt文件在哪个目录。
```c++
bool ObstacleCameraPerception::Init(
    const CameraPerceptionInitOptions &options) {

  // 1. 初始化探测器
  base::BaseCameraModelPtr model;
  for (int i = 0; i < perception_param_.detector_param_size(); ++i) {
    ObstacleDetectorInitOptions detector_init_options;
    app::DetectorParam detector_param = perception_param_.detector_param(i);
    auto plugin_param = detector_param.plugin_param();
    detector_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    detector_init_options.conf_file = plugin_param.config_file();
    detector_init_options.gpu_id = perception_param_.gpu_id();
    
    // 1.1 获取模型
    model = common::SensorManager::Instance()->GetUndistortCameraModel(
        detector_param.camera_name());
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    name_intrinsic_map_.insert(std::pair<std::string, Eigen::Matrix3f>(
        detector_param.camera_name(), pinhole->get_intrinsic_params()));
    detector_init_options.base_camera_model = model;
    std::shared_ptr<BaseObstacleDetector> detector_ptr(
        BaseObstacleDetectorRegisterer::GetInstanceByName(plugin_param.name()));
    // 1.2 探测器
    name_detector_map_.insert(
        std::pair<std::string, std::shared_ptr<BaseObstacleDetector>>(
            detector_param.camera_name(), detector_ptr));
    // 1.3 初始化探测器
    ACHECK(name_detector_map_.at(detector_param.camera_name())
               ->Init(detector_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // 2. 初始化追踪器
  ACHECK(perception_param_.has_tracker_param()) << "Failed to init tracker.";
  {
    ObstacleTrackerInitOptions tracker_init_options;
    tracker_init_options.image_width = static_cast<float>(model->get_width());
    tracker_init_options.image_height = static_cast<float>(model->get_height());
    tracker_init_options.gpu_id = perception_param_.gpu_id();
    auto plugin_param = perception_param_.tracker_param().plugin_param();
    tracker_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    tracker_init_options.conf_file = plugin_param.config_file();
    tracker_.reset(
        BaseObstacleTrackerRegisterer::GetInstanceByName(plugin_param.name()));
    ACHECK(tracker_ != nullptr);
    ACHECK(tracker_->Init(tracker_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // 3. 初始化转换器
  ACHECK(perception_param_.has_transformer_param())
      << "Failed to init transformer.";
  {
    ObstacleTransformerInitOptions transformer_init_options;
    auto plugin_param = perception_param_.transformer_param().plugin_param();
    transformer_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    transformer_init_options.conf_file = plugin_param.config_file();
    transformer_.reset(BaseObstacleTransformerRegisterer::GetInstanceByName(
        plugin_param.name()));
    ACHECK(transformer_ != nullptr);
    ACHECK(transformer_->Init(transformer_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // 4. 初始化障碍物后处理
  ACHECK(perception_param_.has_postprocessor_param())
      << "Failed to init obstacle postprocessor.";
  {
    ObstaclePostprocessorInitOptions obstacle_postprocessor_init_options;
    auto plugin_param = perception_param_.postprocessor_param().plugin_param();
    obstacle_postprocessor_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    obstacle_postprocessor_init_options.conf_file = plugin_param.config_file();
    obstacle_postprocessor_.reset(
        BaseObstaclePostprocessorRegisterer::GetInstanceByName(
            plugin_param.name()));
    ACHECK(obstacle_postprocessor_ != nullptr);
    ACHECK(obstacle_postprocessor_->Init(obstacle_postprocessor_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // 5. 初始化特征展开
  if (!perception_param_.has_feature_param()) {
    AINFO << "No feature config found.";
    extractor_ = nullptr;
  } else {
    FeatureExtractorInitOptions init_options;
    auto plugin_param = perception_param_.feature_param().plugin_param();
    init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
    init_options.conf_file = plugin_param.config_file();
    extractor_.reset(
        BaseFeatureExtractorRegisterer::GetInstanceByName(plugin_param.name()));
    ACHECK(extractor_ != nullptr);
    ACHECK(extractor_->Init(init_options))
        << "Failed to init: " << plugin_param.name();
  }

  lane_calibration_working_sensor_name_ =
      options.lane_calibration_working_sensor_name;

  // 6. 初始化车道
  InitLane(work_root, perception_param_);

  // 7. 初始化校准服务
  InitCalibrationService(work_root, model, perception_param_);

  // 8. 初始化调试信息
  if (perception_param_.has_debug_param()) {
    // Init debug info
    if (perception_param_.debug_param().has_track_out_file()) {
      out_track_.open(perception_param_.debug_param().track_out_file(),
                      std::ofstream::out);
    }
    if (perception_param_.debug_param().has_camera2world_out_file()) {
      out_pose_.open(perception_param_.debug_param().camera2world_out_file(),
                     std::ofstream::out);
    }
  }

  // 9. 初始化对象模板
  if (perception_param_.has_object_template_param()) {
    ObjectTemplateManagerInitOptions init_options;
    auto plugin_param =
        perception_param_.object_template_param().plugin_param();
    init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
    init_options.conf_file = plugin_param.config_file();
    ACHECK(ObjectTemplateManager::Instance()->Init(init_options));
  }
  return true;
}
```


#### 相机感知(Perception)
相机的感知包括车道线和障碍物的感知，障碍物追踪几个功能。  
```c++
bool ObstacleCameraPerception::Perception(
    const CameraPerceptionOptions &options, CameraFrame *frame) {

  inference::CudaUtil::set_device_id(perception_param_.gpu_id());
  ObstacleDetectorOptions detector_options;
  ObstacleTransformerOptions transformer_options;
  ObstaclePostprocessorOptions obstacle_postprocessor_options;
  ObstacleTrackerOptions tracker_options;
  FeatureExtractorOptions extractor_options;

  frame->camera_k_matrix =
      name_intrinsic_map_.at(frame->data_provider->sensor_name());
  if (frame->calibration_service == nullptr) {
    AERROR << "Calibraion service is not available";
    return false;
  }

  // 1. 车道线识别
  LaneDetectorOptions lane_detetor_options;
  LanePostprocessorOptions lane_postprocessor_options;
  if (!lane_detector_->Detect(lane_detetor_options, frame)) {
    AERROR << "Failed to detect lane.";
    return false;
  }

  // 2. 车道线后处理
  if (!lane_postprocessor_->Process2D(lane_postprocessor_options, frame)) {
    AERROR << "Failed to postprocess lane 2D.";
    return false;
  }

  // 3. 校准服务
  frame->calibration_service->Update(frame);
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "CalibrationService");

  if (!lane_postprocessor_->Process3D(lane_postprocessor_options, frame)) {
    AERROR << "Failed to postprocess lane 3D.";
    return false;
  }

  // 4. 输出车道信息到文件
  if (write_out_lane_file_) {
    std::string lane_file_path =
        absl::StrCat(out_lane_dir_, "/", frame->frame_id, ".txt");
    WriteLanelines(write_out_lane_file_, lane_file_path, frame->lane_objects);
  }

  // 5. 输出校准信息到文件
  if (write_out_calib_file_) {
    std::string calib_file_path =
        absl::StrCat(out_calib_dir_, "/", frame->frame_id, ".txt");
    WriteCalibrationOutput(write_out_calib_file_, calib_file_path, frame);
  }

  // 6. 障碍物追踪
  if (!tracker_->Predict(tracker_options, frame)) {
    AERROR << "Failed to predict.";
    return false;
  }

  std::shared_ptr<BaseObstacleDetector> detector =
      name_detector_map_.at(frame->data_provider->sensor_name());
  
  // 7. 障碍物识别
  if (!detector->Detect(detector_options, frame)) {
    AERROR << "Failed to detect.";
    return false;
  }

  // 8. 保存障碍物信息为KITTI格式
  WriteDetections(
      perception_param_.debug_param().has_detection_out_dir(),
      absl::StrCat(perception_param_.debug_param().detection_out_dir(), "/",
                   frame->frame_id, ".txt"),
      frame->detected_objects);
  if (extractor_ && !extractor_->Extract(extractor_options, frame)) {
    AERROR << "Failed to extractor";
    return false;
  }

  // Save detection results with bbox, detection_feature
  WriteDetections(
      perception_param_.debug_param().has_detect_feature_dir(),
      absl::StrCat(perception_param_.debug_param().detect_feature_dir(), "/",
                   frame->frame_id, ".txt"),
      frame);
  // Set the sensor name of each object
  for (size_t i = 0; i < frame->detected_objects.size(); ++i) {
    frame->detected_objects[i]->camera_supplement.sensor_name =
        frame->data_provider->sensor_name();
  }
  if (!tracker_->Associate2D(tracker_options, frame)) {
    AERROR << "Failed to associate2d.";
    return false;
  }

  // 9. 进行坐标转换
  if (!transformer_->Transform(transformer_options, frame)) {
    AERROR << "Failed to transform.";
    return false;
  }

  // 10. 障碍物后处理
  obstacle_postprocessor_options.do_refinement_with_calibration_service =
      frame->calibration_service != nullptr;
  if (!obstacle_postprocessor_->Process(obstacle_postprocessor_options,
                                        frame)) {
    AERROR << "Failed to post process obstacles.";
    return false;
  }

  if (!tracker_->Associate3D(tracker_options, frame)) {
    AERROR << "Failed to Associate3D.";
    return false;
  }

  // 11. 追踪障碍物信息
  if (!tracker_->Track(tracker_options, frame)) {
    AERROR << "Failed to track.";
    return false;
  }

  if (perception_param_.has_debug_param()) {
    if (perception_param_.debug_param().has_camera2world_out_file()) {
      WriteCamera2World(out_pose_, frame->frame_id, frame->camera2world_pose);
    }
    if (perception_param_.debug_param().has_track_out_file()) {
      WriteTracking(out_track_, frame->frame_id, frame->tracked_objects);
    }
  }

  // 12. 保存障碍物追踪信息为KITTI格式
  WriteDetections(
      perception_param_.debug_param().has_tracked_detection_out_dir(),
      absl::StrCat(perception_param_.debug_param().tracked_detection_out_dir(),
                   "/", frame->frame_id, ".txt"),
      frame->tracked_objects);

  // 13. 保存结果？？？
  for (auto &obj : frame->tracked_objects) {
    FillObjectPolygonFromBBox3D(obj.get());
    obj->anchor_point = obj->center;
  }

  return true;
}
```

  
#### 前方最危险目标检测(Cipv)
Cipv在"perception/camera"中，


## FusionComponent
融合感知模块采用的是"ObstacleMultiSensorFusion"中的实现。当消息到来的时候会执行Proc，而Proc是调用内部实现"InternalProc"。  
```c++
bool FusionComponent::InternalProc(
    const std::shared_ptr<SensorFrameMessage const>& in_message,
    std::shared_ptr<PerceptionObstacles> out_message,
    std::shared_ptr<SensorFrameMessage> viz_message) {
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }

  PERCEPTION_PERF_BLOCK_START();
  const double timestamp = in_message->timestamp_;
  const uint64_t lidar_timestamp = in_message->lidar_timestamp_;
  std::vector<base::ObjectPtr> valid_objects;
  if (in_message->error_code_ != apollo::common::ErrorCode::OK) {
    if (!MsgSerializer::SerializeMsg(
            timestamp, lidar_timestamp, in_message->seq_num_, valid_objects,
            in_message->error_code_, out_message.get())) {
      AERROR << "Failed to gen PerceptionObstacles object.";
      return false;
    }
    if (FLAGS_obs_enable_visualization) {
      viz_message->process_stage_ = ProcessStage::SENSOR_FUSION;
      viz_message->error_code_ = in_message->error_code_;
    }
    AERROR << "Fusion receive message with error code, skip it.";
    return true;
  }
  base::FramePtr frame = in_message->frame_;
  frame->timestamp = in_message->timestamp_;

  std::vector<base::ObjectPtr> fused_objects;
  if (!fusion_->Process(frame, &fused_objects)) {
    AERROR << "Failed to call fusion plugin.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(std::string("fusion_process"),
                                           in_message->sensor_id_);

  if (in_message->sensor_id_ != fusion_main_sensor_) {
    return true;
  }

  Eigen::Matrix4d sensor2world_pose =
      in_message->frame_->sensor2world_pose.matrix();
  if (object_in_roi_check_ && FLAGS_obs_enable_hdmap_input) {
    // get hdmap
    base::HdmapStructPtr hdmap(new base::HdmapStruct());
    if (hdmap_input_) {
      base::PointD position;
      position.x = sensor2world_pose(0, 3);
      position.y = sensor2world_pose(1, 3);
      position.z = sensor2world_pose(2, 3);
      hdmap_input_->GetRoiHDMapStruct(position, radius_for_roi_object_check_,
                                      hdmap);
      // TODO(use check)
      // ObjectInRoiSlackCheck(hdmap, fused_objects, &valid_objects);
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    } else {
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    }
  } else {
    valid_objects.assign(fused_objects.begin(), fused_objects.end());
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(std::string("fusion_roi_check"),
                                           in_message->sensor_id_);

  // produce visualization msg
  if (FLAGS_obs_enable_visualization) {
    viz_message->timestamp_ = in_message->timestamp_;
    viz_message->seq_num_ = in_message->seq_num_;
    viz_message->frame_ = base::FramePool::Instance().Get();
    viz_message->frame_->sensor2world_pose =
        in_message->frame_->sensor2world_pose;
    viz_message->sensor_id_ = in_message->sensor_id_;
    viz_message->hdmap_ = in_message->hdmap_;
    viz_message->process_stage_ = ProcessStage::SENSOR_FUSION;
    viz_message->error_code_ = in_message->error_code_;
    viz_message->frame_->objects = fused_objects;
  }
  // produce pb output msg
  apollo::common::ErrorCode error_code = apollo::common::ErrorCode::OK;
  if (!MsgSerializer::SerializeMsg(timestamp, lidar_timestamp,
                                   in_message->seq_num_, valid_objects,
                                   error_code, out_message.get())) {
    AERROR << "Failed to gen PerceptionObstacles object.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      std::string("fusion_serialize_message"), in_message->sensor_id_);

  const double cur_time = apollo::common::time::Clock::NowInSeconds();
  const double latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Obstacle:End:msg_time[" << timestamp
        << "]:cur_time[" << cur_time << "]:cur_latency[" << latency
        << "]:obj_cnt[" << valid_objects.size() << "]";
  AINFO << "publish_number: " << valid_objects.size() << " obj";
  return true;
}
```


#### ObstacleMultiSensorFusion
ObstacleMultiSensorFusion实现了障碍物的融合，在目录"modules\perception\fusion"中。  



## LaneDetectionComponent

## LidarOutputComponent

## RecognitionComponent

## RadarDetectionComponent

## SegmentationComponent

## TrafficLightsPerceptionComponent  

至此，整个感知模块的分析就完成了，可以看到感知模块主要是负责获取障碍物的类型、以及车道等信息反馈给车。  