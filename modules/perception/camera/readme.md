<a name="camera_module" />

## camera
camera模块的结构和radar类似，目录如下：  
```
.
├── app        \\ 主程序
├── common     \\ 公共程序
├── lib         \\ 库，用来做红绿灯、障碍物检测等功能
├── test        \\ 测试用例
└── tools       \\ 工具，用来做车道线和红绿灯识别结果展示
```

## lib目录

## traffic_light
红绿灯的识别分为3个阶段，先预处理，然后识别，然后跟踪？
preprocessor // 预处理
detector - detection     // 检测
         - recognition   // 识别，通过上面的detection检测？？？
tracker // 追踪红绿灯

#### preprocessor预处理
首先来看红绿灯预处理，主要有个文件"pose.cc","multi_camera_projection.cc"和"tl_preprocessor.cc"，先看"pose.cc"的实现。  

在"pose.cc"中实现了"CarPose"类，主要作用是获取车的位置和姿态，从而获取到相机的位置和姿态。在"CarPose"中定义了如下数据结构。  
```
  Eigen::Matrix4d pose_;  // car(novatel) to world pose
  std::map<std::string, Eigen::Matrix4d> c2w_poses_;  // camera to world poses
  double timestamp_;
```
其中"c2w_poses_"存放了不同焦距的相机到世界坐标的转换矩阵。  

获取车的位置和姿态的实现如下，这里是默认车的坐标为0？  
```c++
// 初始化车到世界坐标的转换矩阵
bool CarPose::Init(double ts, const Eigen::Matrix4d &pose) {
  timestamp_ = ts;
  pose_ = pose;
  return true;
}

const Eigen::Matrix4d CarPose::getCarPose() const { return pose_; }

const Eigen::Vector3d CarPose::getCarPosition() const {
  Eigen::Vector3d p;
  p[0] = pose_(0, 3);
  p[1] = pose_(1, 3);
  p[2] = pose_(2, 3);

  return p;
}
```

获取相机到世界坐标的转换矩阵。  
```c++
void CarPose::SetCameraPose(const std::string &camera_name,
                            const Eigen::Matrix4d &c2w_pose) {
  c2w_poses_[camera_name] = c2w_pose;
}

bool CarPose::GetCameraPose(const std::string &camera_name,
                            Eigen::Matrix4d *c2w_pose) const {
  if (c2w_pose == nullptr) {
    AERROR << "c2w_pose is not available";
    return false;
  }
  if (c2w_poses_.find(camera_name) == c2w_poses_.end()) {
    return false;
  }
  *c2w_pose = c2w_poses_.at(camera_name);

  return true;
}
```

删除某个相机到世界坐标的转换矩阵。  
```c++
void CarPose::ClearCameraPose(const std::string &camera_name) {
  auto it = c2w_poses_.find(camera_name);
  if (it != c2w_poses_.end()) {
    c2w_poses_.erase(it);
  }
}
```

这里还提供了重载的输入输出流"<<"，有个疑问这里的"pose_"类型为"Eigen::Matrix4d"打印的时候会直接展开吗？  
```c++
std::ostream &operator<<(std::ostream &os, const CarPose &pose) {
  os << pose.pose_;
  return os;
}
```

"multi_camera_projection.cc"中实现了"MultiCamerasProjection"类，先看数据结构。  
```c++
  // sorted by focal length in descending order
  // 1. 相机名称，按照焦距升序排列
  std::vector<std::string> camera_names_;
  // camera_name -> camera_model
  // 2. 根据相机名称，获取对应的相机模型
  std::map<std::string, base::BrownCameraDistortionModelPtr> camera_models_;
```

首先看"MultiCamerasProjection"类的初始化流程。  
```c++
bool MultiCamerasProjection::Init(const MultiCamerasInitOption& options) {
  // 1. 初始化sensor_manager
  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  if (!sensor_manager->Init()) {
    AERROR << "sensor_manager init failed";
  }

  // 2. 遍历相机列表
  for (size_t i = 0; i < options.camera_names.size(); ++i) {
    const std::string& cur_camera_name = options.camera_names.at(i);

    // 3. 查看相机是否存在
    if (!sensor_manager->IsSensorExist(cur_camera_name)) {
      AERROR << "sensor " << cur_camera_name << " do not exist";
      return false;
    }

    // 4. 从sensor_manager中获取相机模型
    camera_models_[cur_camera_name] =
        std::dynamic_pointer_cast<base::BrownCameraDistortionModel>(
            sensor_manager->GetDistortCameraModel(cur_camera_name));

    camera_names_.push_back(cur_camera_name);
  }
  // sort camera_names_ by focal lengths (descending order)
  std::sort(camera_names_.begin(), camera_names_.end(),
            [&](const std::string& lhs, const std::string& rhs) {
              const auto lhs_cam_intrinsics =
                  camera_models_[lhs]->get_intrinsic_params();
              const auto rhs_cam_intrinsics =
                  camera_models_[rhs]->get_intrinsic_params();
              // 5. 计算焦点长度，并且比较大小
              auto lhs_focal_length =
                  0.5 * (lhs_cam_intrinsics(0, 0) + lhs_cam_intrinsics(1, 1));
              auto rhs_focal_length =
                  0.5 * (rhs_cam_intrinsics(0, 0) + rhs_cam_intrinsics(1, 1));
              return lhs_focal_length > rhs_focal_length;
            });
  // 6. 打印相机名称数组，用空格隔开
  AINFO << "camera_names sorted by descending focal lengths: "
        << std::accumulate(camera_names_.begin(), camera_names_.end(),
                           std::string(""),
                           [](std::string& sum, const std::string& s) {
                             return sum + s + " ";
                           });

  return true;
}
```

投影  
```c++
bool MultiCamerasProjection::Project(const CarPose& pose,
                                     const ProjectOption& option,
                                     base::TrafficLight* light) const {
  Eigen::Matrix4d c2w_pose;
  // 1. 通过相机名称获取相机到世界坐标的转换矩阵
  c2w_pose = pose.c2w_poses_.at(option.camera_name);

  bool ret = false;
  
  // 2. 根据 红绿灯区域坐标点 获取 相机上红绿灯投影点
  ret = BoundaryBasedProject(camera_models_.at(option.camera_name), c2w_pose,
                             light->region.points, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_name: " << option.camera_name;
    return false;
  }
  return true;
}
```

再接着看"BoundaryBasedProject"的实现。  
```c++
bool MultiCamerasProjection::BoundaryBasedProject(
    const base::BrownCameraDistortionModelPtr camera_model,
    const Eigen::Matrix4d& c2w_pose,
    const std::vector<base::PointXYZID>& points,
    base::TrafficLight* light) const {
  // 1. 获取照片的宽度和高度
  int width = static_cast<int>(camera_model->get_width());
  int height = static_cast<int>(camera_model->get_height());
  // 2. 获取红绿灯框，必须大于等于4个点
  int bound_size = static_cast<int>(points.size());

  std::vector<Eigen::Vector2i> pts2d(bound_size);
  // 3. 相机到世界坐标的逆矩阵
  auto c2w_pose_inverse = c2w_pose.inverse();

  for (int i = 0; i < bound_size; ++i) {
    const auto& pt3d_world = points.at(i);
    // 4. 红绿灯世界坐标到相机坐标
    Eigen::Vector3d pt3d_cam =
        (c2w_pose_inverse *
         Eigen::Vector4d(pt3d_world.x, pt3d_world.y, pt3d_world.z, 1.0))
            .head(3);
    // 5. 如果距离小于0，则表示已经经过了红绿灯（红绿灯在车后面）
    if (std::islessequal(pt3d_cam[2], 0.0)) {
      AWARN << "light bound point behind the car: " << pt3d_cam;
      return false;
    }
    // 6. 从3D投影到2D
    pts2d[i] = camera_model->Project(pt3d_cam.cast<float>()).cast<int>();
  }

  // 7. 在一系列点中找到左下角和右上角的点，刚好能够包围投影出来的点
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();
  for (const auto& pt : pts2d) {
    min_x = std::min(pt[0], min_x);
    max_x = std::max(pt[0], max_x);
    min_y = std::min(pt[1], min_y);
    max_y = std::max(pt[1], max_y);
  }
  // 8. 构造图片中感兴趣的区域位置，也就是红绿灯的位置
  base::BBox2DI roi(min_x, min_y, max_x, max_y);
  // 9. 感兴趣区域为0，或者超过图片的范围，图片的坐标起点为（0，0）
  if (OutOfValidRegion(roi, width, height) || roi.Area() == 0) {
    AWARN << "Projection get ROI outside the image. ";
    return false;
  }
  light->region.projection_roi = base::RectI(roi);
  return true;
}
```
**疑问**：  
1. 这里的图片的坐标系是如何的？坐标起点为图片左上角，向下为y轴，向右为x轴？？？  


"tl_preprocessor.cc"中实现了"TLPreprocessor"类。  
```c++
bool TLPreprocessor::Init(const TrafficLightPreprocessorInitOptions &options) {
  camera::MultiCamerasInitOption projection_init_option;
  projection_init_option.camera_names = options.camera_names;
  // 1. 初始化 MultiCamerasProjection projection_
  if (!projection_.Init(projection_init_option)) {
    AERROR << "init multi_camera_projection failed.";
    return false;
  }

  num_cameras_ = projection_.getCameraNamesByDescendingFocalLen().size();
  // 2. 这里申明2个数组，分别代表红绿灯在图片内，和红绿灯不在图片内
  lights_on_image_array_.resize(num_cameras_);
  lights_outside_image_array_.resize(num_cameras_);
  // 3. 同步间隔
  sync_interval_seconds_ = options.sync_interval_seconds;

  return true;
}
```


更新选择的相机
```c++
bool TLPreprocessor::UpdateCameraSelection(
    const CarPose &pose, const TLPreprocessorOption &option,
    std::vector<base::TrafficLightPtr> *lights) {
  const double &timestamp = pose.getTimestamp();
  // 1. 选择当前时间，最大焦距
  selected_camera_name_.first = timestamp;
  selected_camera_name_.second = GetMaxFocalLenWorkingCameraName();

  // 2. 
  if (!ProjectLightsAndSelectCamera(pose, option,
                                    &(selected_camera_name_.second), lights)) {
    AERROR << "project_lights_and_select_camera failed, ts: " << timestamp;
  }

  return true;
}
```
接下来我们接着看"ProjectLightsAndSelectCamera"。
```c++
bool TLPreprocessor::ProjectLightsAndSelectCamera(
    const CarPose &pose, const TLPreprocessorOption &option,
    std::string *selected_camera_name,
    std::vector<base::TrafficLightPtr> *lights) {
  // 1. 清除数组
  for (auto &light_ptrs : lights_on_image_array_) {
    light_ptrs.clear();
  }
  for (auto &light_ptrs : lights_outside_image_array_) {
    light_ptrs.clear();
  }

  // 2. 根据焦距获取相机名称
  const auto &camera_names = projection_.getCameraNamesByDescendingFocalLen();
  for (size_t cam_id = 0; cam_id < num_cameras_; ++cam_id) {
    const std::string &camera_name = camera_names[cam_id];
    // 3. 获取投影
    if (!ProjectLights(pose, camera_name, lights,
                       &(lights_on_image_array_[cam_id]),
                       &(lights_outside_image_array_[cam_id]))) {
      AERROR << "select_camera_by_lights_projection project lights on "
             << camera_name << " image failed";
      return false;
    }
  }

  projections_outside_all_images_ = !lights->empty();
  for (size_t cam_id = 0; cam_id < num_cameras_; ++cam_id) {
    projections_outside_all_images_ =
        projections_outside_all_images_ &&
        (lights_on_image_array_[cam_id].size() < lights->size());
  }
  if (projections_outside_all_images_) {
    AWARN << "lights projections outside all images";
  }

  // 4. 选择相机
  SelectCamera(&lights_on_image_array_, &lights_outside_image_array_, option,
               selected_camera_name);

  return true;
}
```

投影红绿灯  
```c++
bool TLPreprocessor::ProjectLights(
    const CarPose &pose, const std::string &camera_name,
    std::vector<base::TrafficLightPtr> *lights,
    base::TrafficLightPtrs *lights_on_image,
    base::TrafficLightPtrs *lights_outside_image) {

  // 1. 查看相机是否工作，通过查找"camera_is_working_flags_"
  bool is_working = false;
  if (!GetCameraWorkingFlag(camera_name, &is_working) || !is_working) {
    AWARN << "TLPreprocessor::project_lights not project lights, "
          << "camera is not working, camera_name: " << camera_name;
    return true;
  }

  // 2. 遍历红绿灯，并且找到红绿灯在相机上的投影
  for (size_t i = 0; i < lights->size(); ++i) {
    base::TrafficLightPtr light_proj(new base::TrafficLight);
    auto light = lights->at(i);
    if (!projection_.Project(pose, ProjectOption(camera_name), light.get())) {
      // 3. 没有红绿灯投影在相机上，放入 lights_outside_image
      light->region.outside_image = true;
      *light_proj = *light;
      lights_outside_image->push_back(light_proj);
    } else {
      // 4. 有红绿灯投影在相机上，放入 lights_on_image
      light->region.outside_image = false;
      *light_proj = *light;
      lights_on_image->push_back(light_proj);
    }
  }
  return true;
}
```
**疑问**：  
1. 智能指针先指向一个对象，后面重新指向另外一个对象，前面这个对象会自动释放内存？？？  


选择相机，这里如何选择的？？？  
```c++
void TLPreprocessor::SelectCamera(
    std::vector<base::TrafficLightPtrs> *lights_on_image_array,
    std::vector<base::TrafficLightPtrs> *lights_outside_image_array,
    const TLPreprocessorOption &option, std::string *selected_camera_name) {
  // 1. 获取焦距最小的相机名称
  auto min_focal_len_working_camera = GetMinFocalLenWorkingCameraName();

  const auto &camera_names = projection_.getCameraNamesByDescendingFocalLen();
  for (size_t cam_id = 0; cam_id < lights_on_image_array->size(); ++cam_id) {
    const auto &camera_name = camera_names[cam_id];
    bool is_working = false;
    // 2. 如果相机没有工作，则跳过
    if (!GetCameraWorkingFlag(camera_name, &is_working) || !is_working) {
      AINFO << "camera " << camera_name << "is not working";
      continue;
    }

    bool ok = true;
    if (camera_name != min_focal_len_working_camera) {
      // 如果投影在外的相机大于0
      if (lights_outside_image_array->at(cam_id).size() > 0) {
        continue;
      }
      auto lights = lights_on_image_array->at(cam_id);
      for (const auto light : lights) {
        // 如果红绿灯超出范围
        if (OutOfValidRegion(light->region.projection_roi,
                             projection_.getImageWidth(camera_name),
                             projection_.getImageHeight(camera_name),
                             option.image_borders_size->at(camera_name))) {
          ok = false;
          break;
        }
      }
    } else {
      // 如果是最小的焦距，红绿灯的个数是否大于0
      ok = (lights_on_image_array->at(cam_id).size() > 0);
    }

    if (ok) {
      *selected_camera_name = camera_name;
      break;
    }
  }
  AINFO << "select_camera selection: " << *selected_camera_name;
}
```

更新红绿灯投影  
```c++
bool TLPreprocessor::UpdateLightsProjection(
    const CarPose &pose, const TLPreprocessorOption &option,
    const std::string &camera_name,
    std::vector<base::TrafficLightPtr> *lights) {
  lights_on_image_.clear();
  lights_outside_image_.clear();

  AINFO << "clear lights_outside_image_ " << lights_outside_image_.size();

  if (lights->empty()) {
    AINFO << "No lights to be projected";
    return true;
  }

  if (!ProjectLights(pose, camera_name, lights, &lights_on_image_,
                     &lights_outside_image_)) {
    AERROR << "update_lights_projection project lights on " << camera_name
           << " image failed";
    return false;
  }

  if (lights_outside_image_.size() > 0) {
    AERROR << "update_lights_projection failed,"
           << "lights_outside_image->size() " << lights_outside_image_.size()
           << " ts: " << pose.getTimestamp();
    return false;
  }

  auto min_focal_len_working_camera = GetMinFocalLenWorkingCameraName();
  if (camera_name == min_focal_len_working_camera) {
    return lights_on_image_.size() > 0;
  }
  for (const base::TrafficLightPtr &light : lights_on_image_) {
    if (OutOfValidRegion(light->region.projection_roi,
                         projection_.getImageWidth(camera_name),
                         projection_.getImageHeight(camera_name),
                         option.image_borders_size->at(camera_name))) {
      AINFO << "update_lights_projection light project out of image region. "
            << "camera_name: " << camera_name;
      return false;
    }
  }

  AINFO << "UpdateLightsProjection success";
  return true;
}
```

更新信息
```c++
bool TLPreprocessor::SyncInformation(const double image_timestamp,
                                     const std::string &cam_name) {
  const double &proj_ts = selected_camera_name_.first;
  const std::string &proj_camera_name = selected_camera_name_.second;

  if (!projection_.HasCamera(cam_name)) {
    AERROR << "sync_image failed, "
           << "get invalid camera_name: " << cam_name;
    return false;
  }

  if (image_timestamp < last_pub_img_ts_) {
    AWARN << "TLPreprocessor reject the image pub ts:" << image_timestamp
          << " which is earlier than last output ts:" << last_pub_img_ts_
          << ", image_camera_name: " << cam_name;
    return false;
  }

  if (proj_camera_name != cam_name) {
    AWARN << "sync_image failed - find close enough projection,"
          << "but camera_name not match.";
    return false;
  }

  last_pub_img_ts_ = image_timestamp;
  return true;
}
```

#### detector 检测
红绿灯的检测分为2部分，一部分为检测，一部分为识别，分别在"detection"和"recognition"2个目录中。我们先看"detection"的实现。  

在"cropbox.h"和"cropbox.cc"中获取裁剪框。首先是初始化裁切尺度"crop_scale"和最小裁切大小"min_crop_size"
```c++
void CropBox::Init(float crop_scale, int min_crop_size) {
  crop_scale_ = crop_scale;
  min_crop_size_ = min_crop_size;
}
CropBox::CropBox(float crop_scale, int min_crop_size) {
  Init(crop_scale, min_crop_size);
}
```

CropBoxWholeImage根据指定的长、宽裁切图片，如果红绿灯在指定的长、宽之内，则返回裁切框。    
```c++
void CropBoxWholeImage::getCropBox(const int width, const int height,
                                   const base::TrafficLightPtr &light,
                                   base::RectI *crop_box) {
  // 1. 红绿灯在裁剪范围（给定长、宽）内
  if (!OutOfValidRegion(light->region.projection_roi, width, height) &&
      light->region.projection_roi.Area() > 0) {
    crop_box->x = crop_box->y = 0;
    crop_box->width = width;
    crop_box->height = height;
    return;
  }
  // 2. 否则返回全0
  crop_box->x = 0;
  crop_box->y = 0;
  crop_box->width = 0;
  crop_box->height = 0;
}
```

裁切图片。  
```c++
void CropBox::getCropBox(const int width, const int height,
                         const base::TrafficLightPtr &light,
                         base::RectI *crop_box) {
  int rows = height;
  int cols = width;
  // 1. 如果超出范围，则返回0
  if (OutOfValidRegion(light->region.projection_roi, width, height) ||
      light->region.projection_roi.Area() <= 0) {
    crop_box->x = 0;
    crop_box->y = 0;
    crop_box->width = 0;
    crop_box->height = 0;
    return;
  }
  // 2. 获取灯的感兴趣区域
  int xl = light->region.projection_roi.x;
  int yt = light->region.projection_roi.y;
  int xr = xl + light->region.projection_roi.width - 1;
  int yb = yt + light->region.projection_roi.height - 1;

  // scale
  int center_x = (xr + xl) / 2;
  int center_y = (yb + yt) / 2;
  // 3. 感兴趣区域的2.5倍，crop_scale_为裁切比例
  int resize =
      static_cast<int>(crop_scale_ * static_cast<float>(std::max(
                                         light->region.projection_roi.width,
                                         light->region.projection_roi.height)));

  // 4. 根据裁切比例和长、宽，取最小值
  resize = std::max(resize, min_crop_size_);
  resize = std::min(resize, width);
  resize = std::min(resize, height);

  // 5. 计算并且赋值
  xl = center_x - resize / 2 + 1;
  xl = (xl < 0) ? 0 : xl;
  yt = center_y - resize / 2 + 1;
  yt = (yt < 0) ? 0 : yt;
  xr = xl + resize - 1;
  yb = yt + resize - 1;
  if (xr >= cols - 1) {
    xl -= xr - cols + 1;
    xr = cols - 1;
  }

  if (yb >= rows - 1) {
    yt -= yb - rows + 1;
    yb = rows - 1;
  }
  
  crop_box->x = xl;
  crop_box->y = yt;
  crop_box->width = xr - xl + 1;
  crop_box->height = yb - yt + 1;
}
```
**疑问**  
1. 第5步中的实现逻辑是什么样？？？  


"select.h"和"select.cc"选择红绿灯。  
首先看Select类的数据结构，其中用到了匈牙利优化器？  
```c++
common::HungarianOptimizer<float> munkres_;
```

初始化  
```c++
bool Select::Init(int rows, int cols) {
  if (rows < 0 || cols < 0) {
    return false;
  }

  munkres_.costs()->Reserve(rows, cols);

  return true;
}
```
**疑问**  
1. 优化器的cost为什么需要行和列数据？  


计算高斯分数？？  
```c++
double Select::Calc2dGaussianScore(base::Point2DI p1, base::Point2DI p2,
                                   float sigma1, float sigma2) {
  return std::exp(-0.5 * (static_cast<float>((p1.x - p2.x) * (p1.x - p2.x)) /
                              (sigma1 * sigma1) +
                          (static_cast<float>((p1.y - p2.y) * (p1.y - p2.y)) /
                           (sigma2 * sigma2))));
}
```

选择红绿灯。  
```c++
void Select::SelectTrafficLights(
    const std::vector<base::TrafficLightPtr> &refined_bboxes,
    std::vector<base::TrafficLightPtr> *hdmap_bboxes) {
  std::vector<std::pair<size_t, size_t> > assignments;
  munkres_.costs()->Resize(hdmap_bboxes->size(), refined_bboxes.size());

  for (size_t row = 0; row < hdmap_bboxes->size(); ++row) {
    auto center_hd = (*hdmap_bboxes)[row]->region.detection_roi.Center();
    if ((*hdmap_bboxes)[row]->region.outside_image) {
      AINFO << "projection_roi outside image, set score to 0.";
      for (size_t col = 0; col < refined_bboxes.size(); ++col) {
        (*munkres_.costs())(row, col) = 0.0;
      }
      continue;
    }
    for (size_t col = 0; col < refined_bboxes.size(); ++col) {
      float gaussian_score = 100.0f;
      auto center_refine = refined_bboxes[col]->region.detection_roi.Center();
      // use gaussian score as metrics of distance and width
      double distance_score = Calc2dGaussianScore(
          center_hd, center_refine, gaussian_score, gaussian_score);

      double max_score = 0.9;
      auto detect_score = refined_bboxes[col]->region.detect_score;
      double detection_score =
          detect_score > max_score ? max_score : detect_score;

      double distance_weight = 0.7;
      double detection_weight = 1 - distance_weight;
      (*munkres_.costs())(row, col) =
          static_cast<float>(detection_weight * detection_score +
                             distance_weight * distance_score);
      const auto &crop_roi = (*hdmap_bboxes)[row]->region.crop_roi;
      const auto &detection_roi = refined_bboxes[col]->region.detection_roi;
      // 1. crop roi在detection roi之外
      if ((detection_roi & crop_roi) != detection_roi) {
        (*munkres_.costs())(row, col) = 0.0;
      }
      AINFO << "score " << (*munkres_.costs())(row, col);
    }
  }
  
  munkres_.Maximize(&assignments);

  for (size_t i = 0; i < hdmap_bboxes->size(); ++i) {
    (*hdmap_bboxes)[i]->region.is_selected = false;
    (*hdmap_bboxes)[i]->region.is_detected = false;
  }

  // 2. 把结果放入hdmap_bbox_region
  for (size_t i = 0; i < assignments.size(); ++i) {
    if (static_cast<size_t>(assignments[i].first) >= hdmap_bboxes->size() ||
        static_cast<size_t>(
            assignments[i].second >= refined_bboxes.size() ||
            (*hdmap_bboxes)[assignments[i].first]->region.is_selected ||
            refined_bboxes[assignments[i].second]->region.is_selected)) {
    } else {
      auto &refined_bbox_region = refined_bboxes[assignments[i].second]->region;
      auto &hdmap_bbox_region = (*hdmap_bboxes)[assignments[i].first]->region;
      refined_bbox_region.is_selected = true;
      hdmap_bbox_region.is_selected = true;

      const auto &crop_roi = hdmap_bbox_region.crop_roi;
      const auto &detection_roi = refined_bbox_region.detection_roi;
      bool outside_crop_roi = ((crop_roi & detection_roi) != detection_roi);
      if (hdmap_bbox_region.outside_image || outside_crop_roi) {
        hdmap_bbox_region.is_detected = false;
      } else {
        hdmap_bbox_region.detection_roi = refined_bbox_region.detection_roi;
        hdmap_bbox_region.detect_class_id = refined_bbox_region.detect_class_id;
        hdmap_bbox_region.detect_score = refined_bbox_region.detect_score;
        hdmap_bbox_region.is_detected = refined_bbox_region.is_detected;
        hdmap_bbox_region.is_selected = refined_bbox_region.is_selected;
      }
    }
  }
}
```
**疑问**  
功能如何实现的？有什么作用？  

接下来我们看"detection.h"和"detection.cc"。首先看一下"TrafficLightDetection"类的参数。  
```c++
  traffic_light::detection::DetectionParam detection_param_;
  DataProvider::ImageOptions data_provider_image_option_;
  std::shared_ptr<inference::Inference> rt_net_ = nullptr;
  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<base::Blob<float>> param_blob_;
  std::shared_ptr<base::Blob<float>> mean_buffer_;
  std::shared_ptr<IGetBox> crop_;
  std::vector<base::TrafficLightPtr> detected_bboxes_;
  std::vector<base::TrafficLightPtr> selected_bboxes_;
  std::vector<std::string> net_inputs_;
  std::vector<std::string> net_outputs_;
  Select select_;
  int max_batch_size_ = 4;
  int param_blob_length_ = 6;
  float mean_[3];
  std::vector<base::RectI> crop_box_list_;
  std::vector<float> resize_scale_list_;
  int gpu_id_ = 0;
```

Init初始化。  
```c++
bool TrafficLightDetection::Init(
    const camera::TrafficLightDetectorInitOptions &options) {
  std::string proto_path = GetAbsolutePath(options.root_dir, options.conf_file);
  // 1. 获取检测参数
  if (!cyber::common::GetProtoFromFile(proto_path, &detection_param_)) {
    AINFO << "load proto param failed, root dir: " << options.root_dir;
    return false;
  }

  std::string param_str;
  google::protobuf::TextFormat::PrintToString(detection_param_, &param_str);
  AINFO << "TL detection param: " << param_str;

  std::string model_root =
      GetAbsolutePath(options.root_dir, detection_param_.model_name());
  AINFO << "model_root " << model_root;

  std::string proto_file =
      GetAbsolutePath(model_root, detection_param_.proto_file());
  AINFO << "proto_file " << proto_file;

  std::string weight_file =
      GetAbsolutePath(model_root, detection_param_.weight_file());
  AINFO << "weight_file " << weight_file;

  if (detection_param_.is_bgr()) {
    data_provider_image_option_.target_color = base::Color::BGR;
    mean_[0] = detection_param_.mean_b();
    mean_[1] = detection_param_.mean_g();
    mean_[2] = detection_param_.mean_r();
  } else {
    data_provider_image_option_.target_color = base::Color::RGB;
    mean_[0] = detection_param_.mean_r();
    mean_[1] = detection_param_.mean_g();
    mean_[2] = detection_param_.mean_b();
  }

  net_inputs_.push_back(detection_param_.input_blob_name());
  net_inputs_.push_back(detection_param_.im_param_blob_name());
  net_outputs_.push_back(detection_param_.output_blob_name());

  AINFO << "net input blobs: "
        << std::accumulate(net_inputs_.begin(), net_inputs_.end(),
                           std::string(""),
                           [](std::string &sum, const std::string &s) {
                             return sum + "\n" + s;
                           });
  AINFO << "net output blobs: "
        << std::accumulate(net_outputs_.begin(), net_outputs_.end(),
                           std::string(""),
                           [](std::string &sum, const std::string &s) {
                             return sum + "\n" + s;
                           });

  const auto &model_type = detection_param_.model_type();
  AINFO << "model_type: " << model_type;

  rt_net_.reset(inference::CreateInferenceByName(model_type, proto_file,
                                                 weight_file, net_outputs_,
                                                 net_inputs_, model_root));

  AINFO << "rt_net_ create succeed";
  rt_net_->set_gpu_id(options.gpu_id);
  AINFO << "set gpu id " << options.gpu_id;
  gpu_id_ = options.gpu_id;

  int resize_height = detection_param_.min_crop_size();
  int resize_width = detection_param_.min_crop_size();
  max_batch_size_ = detection_param_.max_batch_size();
  param_blob_length_ = 6;

  CHECK_GT(resize_height, 0);
  CHECK_GT(resize_width, 0);
  CHECK_GT(max_batch_size_, 0);

  std::vector<int> shape_input = {max_batch_size_, resize_height, resize_width,
                                  3};
  std::vector<int> shape_param = {max_batch_size_, 1, param_blob_length_, 1};

  std::map<std::string, std::vector<int>> input_reshape;
  input_reshape.insert(
      (std::pair<std::string, std::vector<int>>(net_inputs_[0], shape_input)));
  input_reshape.insert(
      (std::pair<std::string, std::vector<int>>(net_inputs_[1], shape_param)));

  if (!rt_net_->Init(input_reshape)) {
    AINFO << "net init fail.";
    return false;
  }
  AINFO << "net init success.";

  mean_buffer_.reset(new base::Blob<float>(1, resize_height, resize_height, 3));

  param_blob_ = rt_net_->get_blob(net_inputs_[1]);
  float *param_data = param_blob_->mutable_cpu_data();
  for (int i = 0; i < max_batch_size_; ++i) {
    auto offset = i * param_blob_length_;
    param_data[offset + 0] = static_cast<float>(resize_width);
    param_data[offset + 1] = static_cast<float>(resize_height);
    param_data[offset + 2] = 1;
    param_data[offset + 3] = 1;
    param_data[offset + 4] = 0;
    param_data[offset + 5] = 0;
  }

  switch (detection_param_.crop_method()) {
    default:
    case 0:
      crop_.reset(new CropBox(detection_param_.crop_scale(),
                              detection_param_.min_crop_size()));
      break;
    case 1:
      crop_.reset(new CropBoxWholeImage());
      break;
  }

  select_.Init(resize_width, resize_height);
  image_.reset(
      new base::Image8U(resize_height, resize_width, base::Color::BGR));
  return true;
}
``` 



#### tracker 追踪
追踪功能在"SemanticReviser"类中实现的。先看下"SemanticReviser"类中的参数。  
```c++
  traffic_light::tracker::SemanticReviseParam semantic_param_;
  // 1. 修订时间
  float revise_time_s_ = 1.5f;
  // 2. 闪烁阈值
  float blink_threshold_s_ = 0.4f;
  // 3. 不闪烁阈值
  float non_blink_threshold_s_ = 0.8f;
  // 4. 滞后阈值
  int hysteretic_threshold_ = 1;
  // 5. 历史语义
  std::vector<SemanticTable> history_semantic_;
```
接着看下"SemanticTable"结构，后面会用到。  
```c++
struct SemanticTable {
  double time_stamp = 0.0;
  double last_bright_time_stamp = 0.0;
  double last_dark_time_stamp = 0.0;
  bool blink = false;
  std::string semantic;
  std::vector<int> light_ids;
  base::TLColor color;
  HystereticWindow hystertic_window;
};
```

#### Init初始化
从配置文件中读取参数，其中"non_blink_threshold_s_"是"blink_threshold_s_"参数的2倍。  
```c++
bool SemanticReviser::Init(const TrafficLightTrackerInitOptions &options) {
  std::string proto_path =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(proto_path, &semantic_param_)) {
    AERROR << "load proto param failed, root dir: " << options.root_dir;
    return false;
  }

  int non_blink_coef = 2;
  revise_time_s_ = semantic_param_.revise_time_second();
  blink_threshold_s_ = semantic_param_.blink_threshold_second();
  hysteretic_threshold_ = semantic_param_.hysteretic_threshold_count();
  non_blink_threshold_s_ =
      blink_threshold_s_ * static_cast<float>(non_blink_coef);

  return true;
}
```

#### Track 追踪
```c++
bool SemanticReviser::Track(const TrafficLightTrackerOptions &options,
                            CameraFrame *frame) {
  double time_stamp = frame->timestamp;
  std::vector<base::TrafficLightPtr> &lights_ref = frame->traffic_lights;
  std::vector<SemanticTable> semantic_table;

  // 1. 如果没有红绿灯，则退出修订
  if (lights_ref.empty()) {
    history_semantic_.clear();
    ADEBUG << "no lights to revise, return";
    return true;
  }

  // 2. 遍历红绿灯，找到发生变化的表
  for (size_t i = 0; i < lights_ref.size(); i++) {
    base::TrafficLightPtr light = lights_ref.at(i);
    int cur_semantic = light->semantic;

    SemanticTable tmp;
    std::stringstream ss;

    if (cur_semantic > 0) {
      ss << "Semantic_" << cur_semantic;
    } else {
      ss << "No_semantic_light_" << light->id;
    }

    tmp.semantic = ss.str();
    tmp.light_ids.push_back(static_cast<int>(i));
    tmp.color = light->status.color;
    tmp.time_stamp = time_stamp;
    tmp.blink = false;
    auto iter =
        std::find_if(std::begin(semantic_table), std::end(semantic_table),
                     boost::bind(compare, _1, tmp));

    if (iter != semantic_table.end()) {
      iter->light_ids.push_back(static_cast<int>(i));
    } else {
      semantic_table.push_back(tmp);
    }
  }

  // 3. 更新红绿灯序列
  for (size_t i = 0; i < semantic_table.size(); ++i) {
    SemanticTable cur_semantic_table = semantic_table.at(i);
    ReviseByTimeSeries(time_stamp, cur_semantic_table, &lights_ref);
  }

  return true;
}
```

根据时间序列更新红绿灯的状态。    
```c++
void SemanticReviser::ReviseByTimeSeries(
    double time_stamp, SemanticTable semantic_table,
    std::vector<base::TrafficLightPtr> *lights) {

  std::vector<base::TrafficLightPtr> &lights_ref = *lights;
  base::TLColor cur_color = ReviseBySemantic(semantic_table, lights);
  base::TLColor pre_color = base::TLColor::TL_UNKNOWN_COLOR;
  semantic_table.color = cur_color;
  semantic_table.time_stamp = time_stamp;
  ADEBUG << "revise same semantic lights";
  ReviseLights(lights, semantic_table.light_ids, cur_color);

  std::vector<SemanticTable>::iterator iter =
      std::find_if(std::begin(history_semantic_), std::end(history_semantic_),
                   boost::bind(compare, _1, semantic_table));

  if (iter != history_semantic_.end()) {
    pre_color = iter->color;
    if (time_stamp - iter->time_stamp < revise_time_s_) {
      ADEBUG << "revise by time series";
      switch (cur_color) {
        case base::TLColor::TL_YELLOW:
          if (iter->color == base::TLColor::TL_RED) {
            ReviseLights(lights, semantic_table.light_ids, iter->color);
            iter->time_stamp = time_stamp;
            iter->hystertic_window.hysteretic_count = 0;
          } else {
            UpdateHistoryAndLights(semantic_table, lights, &iter);
            ADEBUG << "High confidence color " << s_color_strs[cur_color];
          }
          break;
        case base::TLColor::TL_RED:
        case base::TLColor::TL_GREEN:
          UpdateHistoryAndLights(semantic_table, lights, &iter);
          if (time_stamp - iter->last_bright_time_stamp > blink_threshold_s_ &&
              iter->last_dark_time_stamp > iter->last_bright_time_stamp) {
            iter->blink = true;
          }
          iter->last_bright_time_stamp = time_stamp;
          ADEBUG << "High confidence color " << s_color_strs[cur_color];
          break;
        case base::TLColor::TL_BLACK:
          iter->last_dark_time_stamp = time_stamp;
          iter->hystertic_window.hysteretic_count = 0;
          if (iter->color == base::TLColor::TL_UNKNOWN_COLOR ||
              iter->color == base::TLColor::TL_BLACK) {
            iter->time_stamp = time_stamp;
            UpdateHistoryAndLights(semantic_table, lights, &iter);
          } else {
            ReviseLights(lights, semantic_table.light_ids, iter->color);
          }
          break;
        case base::TLColor::TL_UNKNOWN_COLOR:
        default:
          ReviseLights(lights, semantic_table.light_ids, iter->color);
          break;
      }
    } else {
      iter->time_stamp = time_stamp;
      iter->color = cur_color;
    }

    // set blink status
    if (pre_color != iter->color ||
        fabs(iter->last_dark_time_stamp - iter->last_bright_time_stamp) >
            non_blink_threshold_s_) {
      iter->blink = false;
    }

    for (auto index : semantic_table.light_ids) {
      lights_ref[index]->status.blink =
          (iter->blink && iter->color == base::TLColor::TL_GREEN);
    }

  } else {
    semantic_table.last_dark_time_stamp = semantic_table.time_stamp;
    semantic_table.last_bright_time_stamp = semantic_table.time_stamp;
    history_semantic_.push_back(semantic_table);
  }
}
```


修改红绿灯为目标颜色。  
```c++
void SemanticReviser::ReviseLights(std::vector<base::TrafficLightPtr> *lights,
                                   const std::vector<int> &light_ids,
                                   base::TLColor dst_color) {
  // 1. 遍历红绿灯数组，并且修改颜色为dst_color
  for (auto index : light_ids) {
    lights->at(index)->status.color = dst_color;
  }
}
```

通过语义判别红绿灯颜色。  
```c++
base::TLColor SemanticReviser::ReviseBySemantic(
    SemanticTable semantic_table, std::vector<base::TrafficLightPtr> *lights) {
  std::vector<int> vote(static_cast<int>(base::TLColor::TL_TOTAL_COLOR_NUM), 0);
  std::vector<base::TrafficLightPtr> &lights_ref = *lights;
  base::TLColor max_color = base::TLColor::TL_UNKNOWN_COLOR;

  // 1. 遍历红绿灯，把出现的颜色放入投票桶
  for (size_t i = 0; i < semantic_table.light_ids.size(); ++i) {
    int index = semantic_table.light_ids.at(i);
    base::TrafficLightPtr light = lights_ref[index];
    auto color = light->status.color;
    vote.at(static_cast<int>(color))++;
  }
  
  // 2.如果红绿灯红、黄、绿的颜色为0
  if ((vote.at(static_cast<size_t>(base::TLColor::TL_RED)) == 0) &&
      (vote.at(static_cast<size_t>(base::TLColor::TL_GREEN)) == 0) &&
      (vote.at(static_cast<size_t>(base::TLColor::TL_YELLOW)) == 0)) {
    // 3. 红绿灯黑色的次数大于0
    if (vote.at(static_cast<size_t>(base::TLColor::TL_BLACK)) > 0) {
      return base::TLColor::TL_BLACK;
    } else {
      return base::TLColor::TL_UNKNOWN_COLOR;
    }
  }

  vote.at(static_cast<size_t>(base::TLColor::TL_BLACK)) = 0;
  vote.at(static_cast<size_t>(base::TLColor::TL_UNKNOWN_COLOR)) = 0;
  // 4. 找到最多出现的次数
  auto biggest = std::max_element(std::begin(vote), std::end(vote));

  int max_color_num = *biggest;
  max_color = base::TLColor(std::distance(std::begin(vote), biggest));

  vote.erase(biggest);
  // 5. 找到出现第二多的次数
  auto second_biggest = std::max_element(std::begin(vote), std::end(vote));

  // 6. 如果第一多和第二多的相等，则返回UNKNOWN
  if (max_color_num == *second_biggest) {
    return base::TLColor::TL_UNKNOWN_COLOR;
  } else {
    // 7. 返回出现最多的颜色
    return max_color;
  }
}
```

更新历史记录。  
```c++
void SemanticReviser::UpdateHistoryAndLights(
    const SemanticTable &cur, std::vector<base::TrafficLightPtr> *lights,
    std::vector<SemanticTable>::iterator *history) {
  (*history)->time_stamp = cur.time_stamp;
  if ((*history)->color == base::TLColor::TL_BLACK) {
    if ((*history)->hystertic_window.hysteretic_color == cur.color) {
      (*history)->hystertic_window.hysteretic_count++;
    } else {
      (*history)->hystertic_window.hysteretic_color = cur.color;
      (*history)->hystertic_window.hysteretic_count = 1;
    }

    if ((*history)->hystertic_window.hysteretic_count > hysteretic_threshold_) {
      (*history)->color = cur.color;
      (*history)->hystertic_window.hysteretic_count = 0;
      ADEBUG << "Black lights hysteretic change to " << s_color_strs[cur.color];
    } else {
      ReviseLights(lights, cur.light_ids, (*history)->color);
    }
  } else {
    (*history)->color = cur.color;
  }
}
```





## obstacle障碍物
首先obstacle的目录结构如下
```
.
├── detector    // 检测
├── postprocessor  // 后处理？做了什么处理???
├── tracker  // 追踪
└── transformer  // 坐标转换
```
下面我们分别分析下每个模块具体的实现。  

## detector检测
障碍物检测用到的是YOLO深度学习模型，下面我们看下YoloObstacleDetector类的实现。  
初始化Yolo网络  
```c++
bool YoloObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  gpu_id_ = options.gpu_id;
  BASE_CUDA_CHECK(cudaSetDevice(gpu_id_));
  BASE_CUDA_CHECK(cudaStreamCreate(&stream_));

  base_camera_model_ = options.base_camera_model;
  ACHECK(base_camera_model_ != nullptr) << "base_camera_model is nullptr!";
  std::string config_path =
      GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(config_path, &yolo_param_)) {
    AERROR << "read proto_config fail";
    return false;
  }
  const auto &model_param = yolo_param_.model_param();
  std::string model_root =
      GetAbsolutePath(options.root_dir, model_param.model_name());
  std::string anchors_file =
      GetAbsolutePath(model_root, model_param.anchors_file());
  std::string types_file =
      GetAbsolutePath(model_root, model_param.types_file());
  std::string expand_file =
      GetAbsolutePath(model_root, model_param.expand_file());
  LoadInputShape(model_param);
  LoadParam(yolo_param_);
  min_dims_.min_2d_height /= static_cast<float>(height_);

  if (!LoadAnchors(anchors_file, &anchors_)) {
    return false;
  }
  if (!LoadTypes(types_file, &types_)) {
    return false;
  }
  if (!LoadExpand(expand_file, &expands_)) {
    return false;
  }
  ACHECK(expands_.size() == types_.size());
  if (!InitNet(yolo_param_, model_root)) {
    return false;
  }
  InitYoloBlob(yolo_param_.net_param());
  if (!InitFeatureExtractor(model_root)) {
    return false;
  }
  return true;
}
```
特征提取采用的是"TrackingFeatureExtractor"
```c++
bool YoloObstacleDetector::InitFeatureExtractor(const std::string &root_dir) {
  FeatureExtractorInitOptions feat_options;
  feat_options.conf_file = yolo_param_.model_param().feature_file();
  feat_options.root_dir = root_dir;
  feat_options.gpu_id = gpu_id_;
  auto feat_blob_name = yolo_param_.net_param().feat_blob();
  feat_options.feat_blob = inference_->get_blob(feat_blob_name);
  feat_options.input_height = height_;
  feat_options.input_width = width_;
  feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
      "TrackingFeatureExtractor"));
  if (!feature_extractor_->Init(feat_options)) {
    return false;
  }
  return true;
}
```

下面我们再看下如何用YOLO做目标检测。  
```c++
bool YoloObstacleDetector::Detect(const ObstacleDetectorOptions &options,
                                  CameraFrame *frame) {

  Timer timer;
  // 1. 设置GPU设备
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    return false;
  }

  auto input_blob = inference_->get_blob(yolo_param_.net_param().input_blob());

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  image_options.crop_roi = base::RectI(
      0, offset_y_, static_cast<int>(base_camera_model_->get_width()),
      static_cast<int>(base_camera_model_->get_height()) - offset_y_);
  image_options.do_crop = true;
  frame->data_provider->GetImage(image_options, image_.get());

  inference::ResizeGPU(*image_, input_blob, frame->data_provider->src_width(),
                       0);


  // 2. 检测
  inference_->Infer();
  get_objects_gpu(yolo_blobs_, stream_, types_, nms_, yolo_param_.model_param(),
                  light_vis_conf_threshold_, light_swt_conf_threshold_,
                  overlapped_.get(), idx_sm_.get(), &(frame->detected_objects));


  filter_bbox(min_dims_, &(frame->detected_objects));
  FeatureExtractorOptions feat_options;
  feat_options.normalized = true;

  feature_extractor_->Extract(feat_options, frame);

  recover_bbox(frame->data_provider->src_width(),
               frame->data_provider->src_height() - offset_y_, offset_y_,
               &frame->detected_objects);

  // 3. 后处理
  int left_boundary =
      static_cast<int>(border_ratio_ * static_cast<float>(image_->cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio_) *
                                        static_cast<float>(image_->cols()));
  for (auto &obj : frame->detected_objects) {
    // recover alpha
    obj->camera_supplement.alpha /= ori_cycle_;
    // get area_id from visible_ratios
    if (yolo_param_.model_param().num_areas() == 0) {
      obj->camera_supplement.area_id =
          get_area_id(obj->camera_supplement.visible_ratios);
    }
    // clear cut off ratios
    auto &box = obj->camera_supplement.box;
    if (box.xmin >= left_boundary) {
      obj->camera_supplement.cut_off_ratios[2] = 0;
    }
    if (box.xmax <= right_boundary) {
      obj->camera_supplement.cut_off_ratios[3] = 0;
    }
  }

  return true;
}
```

其中推理过程有GPU实现也有CPU实现。 通过"get_objects_gpu"获取物体的信息。  


## tracker追踪
tracker追踪在"OMTObstacleTracker"类中实现。  
初始化Init
```c++
bool OMTObstacleTracker::Init(const ObstacleTrackerInitOptions &options) {
  std::string omt_config = GetAbsolutePath(options.root_dir, options.conf_file);
  // 1. 加载omt配置
  if (!cyber::common::GetProtoFromFile(omt_config, &omt_param_)) {
    AERROR << "Read config failed: " << omt_config;
    return false;
  }
  
  AINFO << "load omt parameters from " << omt_config
        << " \nimg_capability: " << omt_param_.img_capability()
        << " \nlost_age: " << omt_param_.lost_age()
        << " \nreserve_age: " << omt_param_.reserve_age()
        << " \nborder: " << omt_param_.border()
        << " \ntarget_thresh: " << omt_param_.target_thresh()
        << " \ncorrect_type: " << omt_param_.correct_type();

  // 2. 初始化参数
  track_id_ = 0;
  frame_num_ = 0;
  frame_list_.Init(omt_param_.img_capability());
  gpu_id_ = options.gpu_id;
  similar_map_.Init(omt_param_.img_capability(), gpu_id_);
  similar_.reset(new GPUSimilar);
  width_ = options.image_width;
  height_ = options.image_height;
  reference_.Init(omt_param_.reference(), width_, height_);
  std::string type_change_cost =
      GetAbsolutePath(options.root_dir, omt_param_.type_change_cost());
  std::ifstream fin(type_change_cost);
  ACHECK(fin.is_open());
  kTypeAssociatedCost_.clear();
  int n_type = static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE);
  for (int i = 0; i < n_type; ++i) {
    kTypeAssociatedCost_.emplace_back(std::vector<float>(n_type, 0));
    for (int j = 0; j < n_type; ++j) {
      fin >> kTypeAssociatedCost_[i][j];
    }
  }
  targets_.clear();
  used_.clear();

  // Init object template
  object_template_manager_ = ObjectTemplateManager::Instance();
  return true;
}
```

计算运动相似度  
```c++
float OMTObstacleTracker::ScoreMotion(const Target &target,
                                      TrackObjectPtr track_obj) {
  Eigen::Vector4d x = target.image_center.get_state();
  float target_centerx = static_cast<float>(x[0]);
  float target_centery = static_cast<float>(x[1]);
  base::Point2DF center = track_obj->projected_box.Center();
  base::RectF rect(track_obj->projected_box);
  float s = gaussian(center.x, target_centerx, rect.width) *
            gaussian(center.y, target_centery, rect.height);
  return s;
}
```

计算形状相似度  
```c++
float OMTObstacleTracker::ScoreShape(const Target &target,
                                     TrackObjectPtr track_obj) {
  Eigen::Vector2d shape = target.image_wh.get_state();
  base::RectF rect(track_obj->projected_box);
  float s = static_cast<float>((shape[1] - rect.height) *
                               (shape[0] - rect.width) / (shape[1] * shape[0]));
  return -std::abs(s);
}
```

计算显示相似度  
```c++
float OMTObstacleTracker::ScoreAppearance(const Target &target,
                                          TrackObjectPtr track_obj) {
  float energy = 0.0f;
  int count = 0;
  auto sensor_name = track_obj->indicator.sensor_name;
  for (int i = target.Size() - 1; i >= 0; --i) {
    if (target[i]->indicator.sensor_name != sensor_name) {
      continue;
    }
    PatchIndicator p1 = target[i]->indicator;
    PatchIndicator p2 = track_obj->indicator;

    energy += similar_map_.sim(p1, p2);
    count += 1;
  }

  return energy / (0.1f + static_cast<float>(count) * 0.9f);
}
```

计算重叠相似度  
```c++
float OMTObstacleTracker::ScoreOverlap(const Target &target,
                                       TrackObjectPtr track_obj) {
  Eigen::Vector4d center = target.image_center.get_state();
  Eigen::VectorXd wh = target.image_wh.get_state();
  base::BBox2DF box_target;
  box_target.xmin = static_cast<float>(center[0] - wh[0] * 0.5);
  box_target.xmax = static_cast<float>(center[0] + wh[0] * 0.5);
  box_target.ymin = static_cast<float>(center[1] - wh[1] * 0.5);
  box_target.ymax = static_cast<float>(center[1] + wh[1] * 0.5);

  auto box_obj = track_obj->projected_box;

  float iou = common::CalculateIOUBBox(box_target, box_obj);
  return iou;
}
```

找到最匹配的障碍物。  
```c++
void OMTObstacleTracker::GenerateHypothesis(const TrackObjectPtrs &objects) {
  std::vector<Hypothesis> score_list;
  Hypothesis hypo;
  for (size_t i = 0; i < targets_.size(); ++i) {
    ADEBUG << "Target " << targets_[i].id;
    for (size_t j = 0; j < objects.size(); ++j) {
      hypo.target = static_cast<int>(i);
      hypo.object = static_cast<int>(j);
      float sa = ScoreAppearance(targets_[i], objects[j]);
      float sm = ScoreMotion(targets_[i], objects[j]);
      float ss = ScoreShape(targets_[i], objects[j]);
      float so = ScoreOverlap(targets_[i], objects[j]);
      if (sa == 0) {
        hypo.score = omt_param_.weight_diff_camera().motion() * sm +
                     omt_param_.weight_diff_camera().shape() * ss +
                     omt_param_.weight_diff_camera().overlap() * so;
      } else {
        hypo.score = (omt_param_.weight_same_camera().appearance() * sa +
                      omt_param_.weight_same_camera().motion() * sm +
                      omt_param_.weight_same_camera().shape() * ss +
                      omt_param_.weight_same_camera().overlap() * so);
      }
      int change_from_type = static_cast<int>(targets_[i].type);
      int change_to_type = static_cast<int>(objects[j]->object->sub_type);
      hypo.score += -kTypeAssociatedCost_[change_from_type][change_to_type];
      ADEBUG << "Detection " << objects[j]->indicator.frame_id << "(" << j
             << ") sa:" << sa << " sm: " << sm << " ss: " << ss << " so: " << so
             << " score: " << hypo.score;

      // 95.44% area is range [mu - sigma*2, mu + sigma*2]
      // don't match if motion is beyond the range
      if (sm < 0.045 || hypo.score < omt_param_.target_thresh()) {
        continue;
      }
      score_list.push_back(hypo);
    }
  }

  sort(score_list.begin(), score_list.end(), std::greater<Hypothesis>());
  std::vector<bool> used_target(targets_.size(), false);
  for (auto &pair : score_list) {
    if (used_target[pair.target] || used_[pair.object]) {
      continue;
    }
    Target &target = targets_[pair.target];
    auto det_obj = objects[pair.object];
    target.Add(det_obj);
    used_[pair.object] = true;
    used_target[pair.target] = true;
    AINFO << "Target " << target.id << " match " << det_obj->indicator.frame_id
          << " (" << pair.object << ")"
          << "at " << pair.score << " size: " << target.Size();
  }
}
```

合并重复的障碍物  
```c++
bool OMTObstacleTracker::CombineDuplicateTargets() {
  std::vector<Hypothesis> score_list;
  Hypothesis hypo;
  // 1. targets_为数组，数组元素为Target类
  for (size_t i = 0; i < targets_.size(); ++i) {
    if (targets_[i].Size() == 0) {
      continue;
    }
    for (size_t j = i + 1; j < targets_.size(); ++j) {
      if (targets_[j].Size() == 0) {
        continue;
      }
      int count = 0;
      float score = 0.0f;
      int index1 = 0;
      int index2 = 0;
      while (index1 < targets_[i].Size() && index2 < targets_[j].Size()) {
        auto p1 = targets_[i][index1];
        auto p2 = targets_[j][index2];
        if (std::abs(p1->timestamp - p2->timestamp) <
            omt_param_.same_ts_eps()) {
          if (p1->indicator.sensor_name != p2->indicator.sensor_name) {
            auto box1 = p1->projected_box;
            auto box2 = p2->projected_box;
            score += common::CalculateIOUBBox(box1, box2);
            base::RectF rect1(box1);
            base::RectF rect2(box2);
            score -= std::abs((rect1.width - rect2.width) *
                              (rect1.height - rect2.height) /
                              (rect1.width * rect1.height));
            count += 1;
          }
          ++index1;
          ++index2;
        } else {
          if (p1->timestamp > p2->timestamp) {
            ++index2;
          } else {
            ++index1;
          }
        }
      }
      ADEBUG << "Overlap: (" << targets_[i].id << "," << targets_[j].id
             << ") score " << score << " count " << count;
      hypo.target = static_cast<int>(i);
      hypo.object = static_cast<int>(j);
      hypo.score = (count > 0) ? score / static_cast<float>(count) : 0;
      if (hypo.score < omt_param_.target_combine_iou_threshold()) {
        continue;
      }
      score_list.push_back(hypo);
    }
  }
  sort(score_list.begin(), score_list.end(), std::greater<Hypothesis>());
  std::vector<bool> used_target(targets_.size(), false);
  for (auto &pair : score_list) {
    if (used_target[pair.target] || used_target[pair.object]) {
      continue;
    }
    int index1 = pair.target;
    int index2 = pair.object;
    if (targets_[pair.target].id > targets_[pair.object].id) {
      index1 = pair.object;
      index2 = pair.target;
    }
    Target &target_save = targets_[index1];
    Target &target_del = targets_[index2];
    for (int i = 0; i < target_del.Size(); i++) {
      // no need to change track_id of all objects in target_del
      target_save.Add(target_del[i]);
    }
    std::sort(
        target_save.tracked_objects.begin(), target_save.tracked_objects.end(),
        [](const TrackObjectPtr object1, const TrackObjectPtr object2) -> bool {
          return object1->indicator.frame_id < object2->indicator.frame_id;
        });
    target_save.latest_object = target_save.get_object(-1);
    base::ObjectPtr object = target_del.latest_object->object;
    target_del.Clear();
    AINFO << "Target " << target_del.id << " is merged into Target "
          << target_save.id << " with iou " << pair.score;
    used_target[pair.object] = true;
    used_target[pair.target] = true;
  }
  return true;
}
```



预测
```c++
bool OMTObstacleTracker::Predict(const ObstacleTrackerOptions &options,
                                 CameraFrame *frame) {
  for (auto &target : targets_) {
    target.Predict(frame);
    auto obj = target.latest_object;
    frame->proposed_objects.push_back(obj->object);
  }
  return true;
}
```

创建新目标  
```c++
int OMTObstacleTracker::CreateNewTarget(const TrackObjectPtrs &objects) {
  const TemplateMap &kMinTemplateHWL =
      object_template_manager_->MinTemplateHWL();
  std::vector<base::RectF> target_rects;
  for (auto &&target : targets_) {
    if (!target.isTracked() || target.isLost()) {
      continue;
    }
    base::RectF target_rect(target[-1]->object->camera_supplement.box);
    target_rects.push_back(target_rect);
  }
  int created_count = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (!used_[i]) {
      bool is_covered = false;
      const auto &sub_type = objects[i]->object->sub_type;
      base::RectF rect(objects[i]->object->camera_supplement.box);
      auto &min_tmplt = kMinTemplateHWL.at(sub_type);
      if (OutOfValidRegion(rect, width_, height_, omt_param_.border())) {
        continue;
      }
      for (auto &&target_rect : target_rects) {
        if (IsCovered(rect, target_rect, 0.4f) ||
            IsCoveredHorizon(rect, target_rect, 0.5f)) {
          is_covered = true;
          break;
        }
      }
      if (is_covered) {
        continue;
      }
      if (min_tmplt.empty()  // unknown type
          || rect.height > min_tmplt[0] * omt_param_.min_init_height_ratio()) {
        Target target(omt_param_.target_param());
        target.Add(objects[i]);
        targets_.push_back(target);
        AINFO << "Target " << target.id << " is created by "
              << objects[i]->indicator.frame_id << " ("
              << objects[i]->indicator.patch_id << ")";
        created_count += 1;
      }
    }
  }
  return created_count;
}
```




## postprocessor后处理
后处理的入口在"LocationRefinerObstaclePostprocessor"中，下面我们分析下具体实现。  
是否在ROI区域  
```c++
  bool is_in_roi(const float pt[2], float img_w, float img_h, float v,
                 float h_down) const {
    float x = pt[0];
    float y = pt[1];
    if (y < v) {
      return false;
    } else if (y > (img_h - h_down)) {
      return true;
    }
    float img_w_half = img_w / 2.0f;
    float slope = img_w_half * common::IRec(img_h - h_down - v);
    float left = img_w_half - slope * (y - v);
    float right = img_w_half + slope * (y - h_down);
    return x > left && x < right;
  }
```

LocationRefinerObstaclePostprocessor类，初始化Init
```c++
bool LocationRefinerObstaclePostprocessor::Init(
    const ObstaclePostprocessorInitOptions &options) {
  std::string postprocessor_config =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);
  // 1. 读取配置
  if (!cyber::common::GetProtoFromFile(postprocessor_config,
                                       &location_refiner_param_)) {
    AERROR << "Read config failed: " << postprocessor_config;
    return false;
  }
  
  // 2. 打印配置
  AINFO << "Load postprocessor parameters from " << postprocessor_config
        << " \nmin_dist_to_camera: "
        << location_refiner_param_.min_dist_to_camera()
        << " \nroi_h2bottom_scale: "
        << location_refiner_param_.roi_h2bottom_scale();
  return true;
}
```

处理过程  
```c++
bool LocationRefinerObstaclePostprocessor::Process(
    const ObstaclePostprocessorOptions &options, CameraFrame *frame) {

  Eigen::Vector4d plane;
  // 1. 校准服务是否有地平面
  if (options.do_refinement_with_calibration_service &&
      !frame->calibration_service->QueryGroundPlaneInCameraFrame(&plane)) {
    AINFO << "No valid ground plane in the service.";
  }

  // 2. 
  float query_plane[4] = {
      static_cast<float>(plane(0)), static_cast<float>(plane(1)),
      static_cast<float>(plane(2)), static_cast<float>(plane(3))};
  const auto &camera_k_matrix = frame->camera_k_matrix;
  float k_mat[9] = {0};
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      k_mat[i3 + j] = camera_k_matrix(i, j);
    }
  }

  // 3. 相机K矩阵
  AINFO << "Camera k matrix input to obstacle postprocessor: \n"
        << k_mat[0] << ", " << k_mat[1] << ", " << k_mat[2] << "\n"
        << k_mat[3] << ", " << k_mat[4] << ", " << k_mat[5] << "\n"
        << k_mat[6] << ", " << k_mat[7] << ", " << k_mat[8] << "\n";

  const int width_image = frame->data_provider->src_width();
  const int height_image = frame->data_provider->src_height();
  // 4. 初始化后处理过程
  postprocessor_->Init(k_mat, width_image, height_image);
  ObjPostProcessorOptions obj_postprocessor_options;

  int nr_valid_obj = 0;
  // 5. 遍历检测到的障碍物
  for (auto &obj : frame->detected_objects) {
    ++nr_valid_obj;
    // 6. 获取物体的中心
    float object_center[3] = {obj->camera_supplement.local_center(0),
                              obj->camera_supplement.local_center(1),
                              obj->camera_supplement.local_center(2)};
    float bbox2d[4] = {
        obj->camera_supplement.box.xmin, obj->camera_supplement.box.ymin,
        obj->camera_supplement.box.xmax, obj->camera_supplement.box.ymax};

    float bottom_center[2] = {(bbox2d[0] + bbox2d[2]) / 2, bbox2d[3]};
    float h_down = (static_cast<float>(height_image) - k_mat[5]) *
                   location_refiner_param_.roi_h2bottom_scale();
    // 7. 是否在ROI中
    bool is_in_rule_roi =
        is_in_roi(bottom_center, static_cast<float>(width_image),
                  static_cast<float>(height_image), k_mat[5], h_down);
    float dist2camera = common::ISqrt(common::ISqr(object_center[0]) +
                                      common::ISqr(object_center[2]));

    if (dist2camera > location_refiner_param_.min_dist_to_camera() ||
        !is_in_rule_roi) {
      ADEBUG << "Pass for obstacle postprocessor.";
      continue;
    }

    float dimension_hwl[3] = {obj->size(2), obj->size(1), obj->size(0)};
    float box_cent_x = (bbox2d[0] + bbox2d[2]) / 2;
    Eigen::Vector3f image_point_low_center(box_cent_x, bbox2d[3], 1);
    Eigen::Vector3f point_in_camera =
        static_cast<Eigen::Matrix<float, 3, 1, 0, 3, 1>>(
            camera_k_matrix.inverse() * image_point_low_center);
    float theta_ray =
        static_cast<float>(atan2(point_in_camera.x(), point_in_camera.z()));
    float rotation_y =
        theta_ray + static_cast<float>(obj->camera_supplement.alpha);

    // enforce the ry to be in the range [-pi, pi)
    // 8. 射线的属性？？？
    const float PI = common::Constant<float>::PI();
    if (rotation_y < -PI) {
      rotation_y += 2 * PI;
    } else if (rotation_y >= PI) {
      rotation_y -= 2 * PI;
    }

    // process
    memcpy(obj_postprocessor_options.bbox, bbox2d, sizeof(float) * 4);
    obj_postprocessor_options.check_lowerbound = true;
    camera::LineSegment2D<float> line_seg(bbox2d[0], bbox2d[3], bbox2d[2],
                                          bbox2d[3]);
    obj_postprocessor_options.line_segs.push_back(line_seg);
    memcpy(obj_postprocessor_options.hwl, dimension_hwl, sizeof(float) * 3);
    obj_postprocessor_options.ry = rotation_y;
    // refine with calibration service, support ground plane model currently
    // {0.0f, cos(tilt), -sin(tilt), -camera_ground_height}
    memcpy(obj_postprocessor_options.plane, query_plane, sizeof(float) * 4);

    // changed to touching-ground center
    object_center[1] += dimension_hwl[0] / 2;
    // 9. 后处理包括地面
    postprocessor_->PostProcessObjWithGround(
        obj_postprocessor_options, object_center, dimension_hwl, &rotation_y);
    object_center[1] -= dimension_hwl[0] / 2;

    float z_diff_camera =
        object_center[2] - obj->camera_supplement.local_center(2);

    // fill back results
    obj->camera_supplement.local_center(0) = object_center[0];
    obj->camera_supplement.local_center(1) = object_center[1];
    obj->camera_supplement.local_center(2) = object_center[2];

    obj->center(0) = static_cast<double>(object_center[0]);
    obj->center(1) = static_cast<double>(object_center[1]);
    obj->center(2) = static_cast<double>(object_center[2]);
    obj->center = frame->camera2world_pose * obj->center;

    AINFO << "diff on camera z: " << z_diff_camera;
    AINFO << "Obj center from postprocessor: " << obj->center.transpose();
  }
  return true;
}
```





对象后处理过程在"ObjPostProcessor"类中，下面我们看下它的具体实现。  
设置默认参数。  
```c++
void ObjPostProcessorParams::set_default() {
  max_nr_iter = 5;
  sampling_ratio_low = 0.1f;
  weight_iou = 3.0f;
  learning_r = 0.2f;
  learning_r_decay = 0.9f;
  dist_far = 15.0f;
  shrink_ratio_iou = 0.9f;
  iou_good = 0.5f;
}
```

后处理包括地面  
```c++
bool ObjPostProcessor::PostProcessObjWithGround(
    const ObjPostProcessorOptions &options, float center[3], float hwl[3],
    float *ry) {
  memcpy(hwl, options.hwl, sizeof(float) * 3);
  float bbox[4] = {0};
  memcpy(bbox, options.bbox, sizeof(float) * 4);
  *ry = options.ry;

  // 软约束
  bool adjust_soft =
      AdjustCenterWithGround(bbox, hwl, *ry, options.plane, center);
  if (center[2] > params_.dist_far) {
    return adjust_soft;
  }

  // 硬约束
  bool adjust_hard = PostRefineCenterWithGroundBoundary(
      bbox, hwl, *ry, options.plane, options.line_segs, center,
      options.check_lowerbound);

  return adjust_soft || adjust_hard;
}
```

通过地面调整中心？？？
```c++
bool ObjPostProcessor::AdjustCenterWithGround(const float *bbox,
                                              const float *hwl, float ry,
                                              const float *plane,
                                              float *center) const {
  float iou_ini = GetProjectionScore(ry, bbox, hwl, center);
  if (iou_ini < params_.iou_good) {  // ini pos is not good enough
    return false;
  }
  const float MIN_COST = hwl[2] * params_.sampling_ratio_low;
  const float EPS_COST_DELTA = 1e-1f;
  const float WEIGHT_IOU = params_.weight_iou;
  const int MAX_ITERATION = params_.max_nr_iter;

  float lr = params_.learning_r;
  float cost_pre = FLT_MAX;
  float cost_delta = 0.0f;
  float center_input[3] = {center[0], center[1], center[2]};
  float center_test[3] = {0};
  float x[3] = {0};
  int iter = 1;
  bool stop = false;

  // std::cout << "start to update the center..." << std::endl;
  while (!stop) {
    common::IProjectThroughIntrinsic(k_mat_, center, x);
    x[0] *= common::IRec(x[2]);
    x[1] *= common::IRec(x[2]);
    bool in_front = common::IBackprojectPlaneIntersectionCanonical(
        x, k_mat_, plane, center_test);
    if (!in_front) {
      memcpy(center, center_input, sizeof(float) * 3);
      return false;
    }
    float iou_cur = GetProjectionScore(ry, bbox, hwl, center);
    float iou_test = GetProjectionScore(ry, bbox, hwl, center_test);
    float dist = common::ISqrt(common::ISqr(center[0] - center_test[0]) +
                               common::ISqr(center[2] - center_test[2]));
    float cost_cur = dist + WEIGHT_IOU * (1.0f - (iou_cur + iou_test) / 2);
    // std::cout << "cost___ " << cost_cur << "@" << iter << std::endl;
    if (cost_cur >= cost_pre) {
      stop = true;
    } else {
      cost_delta = (cost_pre - cost_cur) / cost_pre;
      cost_pre = cost_cur;
      center[0] += (center_test[0] - center[0]) * lr;
      center[2] += (center_test[2] - center[2]) * lr;
      ++iter;
      stop = iter >= MAX_ITERATION || cost_delta < EPS_COST_DELTA ||
             cost_pre < MIN_COST;
    }
    lr *= params_.learning_r_decay;
  }
  float iou_res = GetProjectionScore(ry, bbox, hwl, center);
  if (iou_res < iou_ini * params_.shrink_ratio_iou) {
    memcpy(center, center_input, sizeof(float) * 3);
    return false;
  }
  return true;
}
```

根据地面边界获取细化的中心？？？  
```c++
bool ObjPostProcessor::PostRefineCenterWithGroundBoundary(
    const float *bbox, const float *hwl, float ry, const float *plane,
    const std::vector<LineSegment2D<float>> &line_seg_limits, float *center,
    bool check_lowerbound) const {
  bool truncated_on_bottom =
      bbox[3] >= static_cast<float>(height_) -
                     (bbox[3] - bbox[1]) * params_.sampling_ratio_low;
  if (truncated_on_bottom) {
    return false;
  }

  float iou_before = GetProjectionScore(ry, bbox, hwl, center);
  int nr_line_segs = static_cast<int>(line_seg_limits.size());
  float depth_pts[4] = {0};
  float pts_c[12] = {0};
  int x_pts[4] = {0};

  GetDepthXPair(bbox, hwl, center, ry, depth_pts, x_pts, &pts_c[0]);

  float dxdz_acc[2] = {0};
  float ratio_x_over_z = center[0] * common::IRec(center[2]);
  for (int i = 0; i < nr_line_segs; ++i) {
    float dxdz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(line_seg_limits[i], plane, pts_c, k_mat_,
                                      width_, height_, ratio_x_over_z, dxdz,
                                      check_lowerbound);
    dxdz_acc[0] += dxdz[0];
    dxdz_acc[1] += dxdz[1];
  }
  center[0] += dxdz_acc[0];
  center[2] += dxdz_acc[1];

  float iou_after = GetProjectionScore(ry, bbox, hwl, center);
  if (iou_after < iou_before * params_.shrink_ratio_iou) {
    center[0] -= dxdz_acc[0];
    center[2] -= dxdz_acc[1];
    return false;
  }
  return true;
}
```



获取深度信息？？？
```c++
int ObjPostProcessor::GetDepthXPair(const float *bbox, const float *hwl,
                                    const float *center, float ry,
                                    float *depth_pts, int *x_pts,
                                    float *pts_c) const {
  int y_min = height_;
  float w_half = hwl[1] / 2;
  float l_half = hwl[2] / 2;
  float x_cor[4] = {l_half, l_half, -l_half, -l_half};
  float z_cor[4] = {w_half, -w_half, -w_half, w_half};
  float pts[12] = {x_cor[0], 0.0f, z_cor[0], x_cor[1], 0.0f, z_cor[1],
                   x_cor[2], 0.0f, z_cor[2], x_cor[3], 0.0f, z_cor[3]};
  float rot[9] = {0};
  GenRotMatrix(ry, rot);
  float pt_proj[3] = {0};
  float pt_c[3] = {0};
  float *pt = pts;
  bool save_pts_c = pts_c != nullptr;
  for (int i = 0; i < 4; ++i) {
    common::IProjectThroughExtrinsic(rot, center, pt, pt_c);
    common::IProjectThroughIntrinsic(k_mat_, pt_c, pt_proj);
    depth_pts[i] = pt_c[2];
    x_pts[i] = common::IRound(pt_proj[0] * common::IRec(pt_proj[2]));
    int y_proj = common::IRound(pt_proj[1] * common::IRec(pt_proj[2]));
    if (y_proj < y_min) {
      y_min = y_proj;
    }
    if (save_pts_c) {
      int i3 = i * 3;
      pts_c[i3] = pt_c[0];
      pts_c[i3 + 1] = pt_c[1];
      pts_c[i3 + 2] = pt_c[2];
    }
    pt += 3;
  }
  return y_min;
}
```

## transformer转换
MultiCueObstacleTransformer类，实现对障碍物做转换。  
初始化Init  
```c++
bool MultiCueObstacleTransformer::Init(
    const ObstacleTransformerInitOptions &options) {
  std::string transformer_config =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);
  // 1. 获取参数
  if (!cyber::common::GetProtoFromFile(transformer_config, &multicue_param_)) {
    AERROR << "Read config failed: " << transformer_config;
    return false;
  }
  AINFO << "Load transformer parameters from " << transformer_config
        << " \nmin dimension: " << multicue_param_.min_dimension_val()
        << " \ndo template search: " << multicue_param_.check_dimension();

  // 2. 初始化对象模板
  object_template_manager_ = ObjectTemplateManager::Instance();

  return true;
}
```

SetObjMapperOptions设置对象地图属性
```c++
void MultiCueObstacleTransformer::SetObjMapperOptions(
    base::ObjectPtr obj, Eigen::Matrix3f camera_k_matrix, int width_image,
    int height_image, ObjMapperOptions *obj_mapper_options, float *theta_ray) {
  // prepare bbox2d
  float bbox2d[4] = {
      obj->camera_supplement.box.xmin, obj->camera_supplement.box.ymin,
      obj->camera_supplement.box.xmax, obj->camera_supplement.box.ymax};
  // input insanity check
  bbox2d[0] = std::max(0.0f, bbox2d[0]);
  bbox2d[1] = std::max(0.0f, bbox2d[1]);
  bbox2d[2] = std::min(static_cast<float>(width_image) - 1.0f, bbox2d[2]);
  bbox2d[3] = std::min(static_cast<float>(height_image) - 1.0f, bbox2d[3]);
  bbox2d[2] = std::max(bbox2d[0] + 1.0f, bbox2d[2]);
  bbox2d[3] = std::max(bbox2d[1] + 1.0f, bbox2d[3]);

  // prepare dimension_hwl
  float dimension_hwl[3] = {obj->size(2), obj->size(1), obj->size(0)};

  // prepare rotation_y
  float box_cent_x = (bbox2d[0] + bbox2d[2]) / 2;
  Eigen::Vector3f image_point_low_center(box_cent_x, bbox2d[3], 1);
  Eigen::Vector3f point_in_camera =
      static_cast<Eigen::Matrix<float, 3, 1, 0, 3, 1>>(
          camera_k_matrix.inverse() * image_point_low_center);
  *theta_ray =
      static_cast<float>(atan2(point_in_camera.x(), point_in_camera.z()));
  float rotation_y =
      *theta_ray + static_cast<float>(obj->camera_supplement.alpha);
  base::ObjectSubType sub_type = obj->sub_type;

  // enforce rotation_y to be in the range [-pi, pi)
  const float PI = common::Constant<float>::PI();
  if (rotation_y < -PI) {
    rotation_y += 2 * PI;
  } else if (rotation_y >= PI) {
    rotation_y -= 2 * PI;
  }

  memcpy(obj_mapper_options->bbox, bbox2d, sizeof(float) * 4);
  memcpy(obj_mapper_options->hwl, dimension_hwl, sizeof(float) * 3);
  obj_mapper_options->ry = rotation_y;
  obj_mapper_options->is_veh = (obj->type == base::ObjectType::VEHICLE);
  obj_mapper_options->check_dimension = multicue_param_.check_dimension();
  obj_mapper_options->type_min_vol_index =
      MatchTemplates(sub_type, dimension_hwl);
  
  // 2D到3D转换
  ADEBUG << "#2D-to-3D for one obj:";
  ADEBUG << "Obj pred ry:" << rotation_y;
  ADEBUG << "Obj pred type: " << static_cast<int>(sub_type);
  ADEBUG << "Bbox: " << bbox2d[0] << ", " << bbox2d[1] << ", " << bbox2d[2]
         << ", " << bbox2d[3];
}
```

匹配模板，返回值type_min_vol_index的作用是什么？？？  
```c++
int MultiCueObstacleTransformer::MatchTemplates(base::ObjectSubType sub_type,
                                                float *dimension_hwl) {
  const TemplateMap &kMinTemplateHWL =
      object_template_manager_->MinTemplateHWL();
  const TemplateMap &kMidTemplateHWL =
      object_template_manager_->MidTemplateHWL();
  const TemplateMap &kMaxTemplateHWL =
      object_template_manager_->MaxTemplateHWL();

  int type_min_vol_index = 0;
  float min_dimension_val =
      std::min(std::min(dimension_hwl[0], dimension_hwl[1]), dimension_hwl[2]);

  const std::map<TemplateIndex, int> &kLookUpTableMinVolumeIndex =
      object_template_manager_->LookUpTableMinVolumeIndex();

  switch (sub_type) {
    // 1. 交通锥
    case base::ObjectSubType::TRAFFICCONE: {
      const float *min_tmplt_cur_type = kMinTemplateHWL.at(sub_type).data();
      const float *max_tmplt_cur_type = kMaxTemplateHWL.at(sub_type).data();
      dimension_hwl[0] = std::min(dimension_hwl[0], max_tmplt_cur_type[0]);
      dimension_hwl[0] = std::max(dimension_hwl[0], min_tmplt_cur_type[0]);
      break;
    }
    // 2. 行人、自行车、摩托车
    case base::ObjectSubType::PEDESTRIAN:
    case base::ObjectSubType::CYCLIST:
    case base::ObjectSubType::MOTORCYCLIST: {
      const float *min_tmplt_cur_type = kMinTemplateHWL.at(sub_type).data();
      const float *mid_tmplt_cur_type = kMidTemplateHWL.at(sub_type).data();
      const float *max_tmplt_cur_type = kMaxTemplateHWL.at(sub_type).data();
      float dh_min = fabsf(dimension_hwl[0] - min_tmplt_cur_type[0]);
      float dh_mid = fabsf(dimension_hwl[0] - mid_tmplt_cur_type[0]);
      float dh_max = fabsf(dimension_hwl[0] - max_tmplt_cur_type[0]);
      std::vector<std::pair<float, float>> diff_hs;
      diff_hs.push_back(std::make_pair(dh_min, min_tmplt_cur_type[0]));
      diff_hs.push_back(std::make_pair(dh_mid, mid_tmplt_cur_type[0]));
      diff_hs.push_back(std::make_pair(dh_max, max_tmplt_cur_type[0]));
      sort(diff_hs.begin(), diff_hs.end(),
           [](const std::pair<float, float> &a,
              const std::pair<float, float> &b) -> bool {
             return a.first < b.first;
           });
      dimension_hwl[0] = diff_hs[0].second;
      break;
    }
    // 3. 汽车
    case base::ObjectSubType::CAR:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::CAR_MIN_VOLUME_INDEX);
      break;
    case base::ObjectSubType::VAN:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::VAN_MIN_VOLUME_INDEX);
      break;
    // 5. 卡车
    case base::ObjectSubType::TRUCK:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::TRUCK_MIN_VOLUME_INDEX);
      break;
    case base::ObjectSubType::BUS:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::BUS_MIN_VOLUME_INDEX);
      break;
    default:
      if (min_dimension_val < multicue_param_.min_dimension_val()) {
        common::IScale3(dimension_hwl, multicue_param_.min_dimension_val() *
                                           common::IRec(min_dimension_val));
      }
      break;
  }
  return type_min_vol_index;
}
```

填充结果，设置物体的大小，朝向等信息。  
```c++
void MultiCueObstacleTransformer::FillResults(
    float object_center[3], float dimension_hwl[3], float rotation_y,
    Eigen::Affine3d camera2world_pose, float theta_ray, base::ObjectPtr obj) {
  if (obj == nullptr) {
    return;
  }
  object_center[1] -= dimension_hwl[0] / 2;
  obj->camera_supplement.local_center(0) = object_center[0];
  obj->camera_supplement.local_center(1) = object_center[1];
  obj->camera_supplement.local_center(2) = object_center[2];
  ADEBUG << "Obj id: " << obj->track_id;
  ADEBUG << "Obj type: " << static_cast<int>(obj->sub_type);
  ADEBUG << "Obj ori dimension: " << obj->size[2] << ", " << obj->size[1]
         << ", " << obj->size[0];
  obj->center(0) = static_cast<double>(object_center[0]);
  obj->center(1) = static_cast<double>(object_center[1]);
  obj->center(2) = static_cast<double>(object_center[2]);
  obj->center = camera2world_pose * obj->center;

  obj->size(2) = dimension_hwl[0];
  obj->size(1) = dimension_hwl[1];
  obj->size(0) = dimension_hwl[2];

  Eigen::Matrix3d pos_var = mapper_->get_position_uncertainty();
  obj->center_uncertainty(0) = static_cast<float>(pos_var(0));
  obj->center_uncertainty(1) = static_cast<float>(pos_var(1));
  obj->center_uncertainty(2) = static_cast<float>(pos_var(2));

  float theta = rotation_y;
  Eigen::Vector3d dir = (camera2world_pose.matrix().block(0, 0, 3, 3) *
                         Eigen::Vector3d(cos(theta), 0, -sin(theta)));
  obj->direction[0] = static_cast<float>(dir[0]);
  obj->direction[1] = static_cast<float>(dir[1]);
  obj->direction[2] = static_cast<float>(dir[2]);
  obj->theta = static_cast<float>(atan2(dir[1], dir[0]));
  obj->theta_variance = static_cast<float>((mapper_->get_orientation_var())(0));

  obj->camera_supplement.alpha = rotation_y - theta_ray;

  ADEBUG << "Dimension hwl: " << dimension_hwl[0] << ", " << dimension_hwl[1]
         << ", " << dimension_hwl[2];
  ADEBUG << "Obj ry:" << rotation_y;
  ADEBUG << "Obj theta: " << obj->theta;
  ADEBUG << "Obj center from transformer: " << obj->center.transpose();
}
```

转换  
```c++
bool MultiCueObstacleTransformer::Transform(
    const ObstacleTransformerOptions &options, CameraFrame *frame) {
  // 1. 相机k矩阵内参矩阵
  const auto &camera_k_matrix = frame->camera_k_matrix;
  float k_mat[9] = {0};
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      k_mat[i3 + j] = camera_k_matrix(i, j);
    }
  }

  const int width_image = frame->data_provider->src_width();
  const int height_image = frame->data_provider->src_height();
  const auto &camera2world_pose = frame->camera2world_pose;
  mapper_->Init(k_mat, width_image, height_image);

  ObjMapperOptions obj_mapper_options;
  float object_center[3] = {0};
  float dimension_hwl[3] = {0};
  float rotation_y = 0.0f;

  int nr_transformed_obj = 0;
  for (auto &obj : frame->detected_objects) {

    // 1. 设置对象字典属性
    float theta_ray = 0.0f;
    SetObjMapperOptions(obj, camera_k_matrix, width_image, height_image,
                        &obj_mapper_options, &theta_ray);

    // 2. 执行
    mapper_->Solve3dBbox(obj_mapper_options, object_center, dimension_hwl,
                         &rotation_y);

    // 3. 填充结果
    FillResults(object_center, dimension_hwl, rotation_y, camera2world_pose,
                theta_ray, obj);

    ++nr_transformed_obj;
  }
  return nr_transformed_obj > 0;
}
```

#### ObjMapper
ObjMapper找到3D框
```c++
bool ObjMapper::Solve3dBbox(const ObjMapperOptions &options, float center[3],
                            float hwl[3], float *ry) {
  // set default value for variance
  set_default_variance();
  float var_yaw = 0.0f;
  float var_z = 0.0f;

  // get input from options
  memcpy(hwl, options.hwl, sizeof(float) * 3);
  float bbox[4] = {0};
  memcpy(bbox, options.bbox, sizeof(float) * 4);
  *ry = options.ry;
  bool check_dimension = options.check_dimension;
  int type_min_vol_index = options.type_min_vol_index;

  // check input hwl insanity
  if (options.is_veh && check_dimension) {
    assert(type_min_vol_index >= 0);
    const std::vector<float> &kVehHwl = object_template_manager_->VehHwl();
    const float *tmplt_with_min_vol = &kVehHwl[type_min_vol_index];
    float min_tmplt_vol =
        tmplt_with_min_vol[0] * tmplt_with_min_vol[1] * tmplt_with_min_vol[2];
    float shrink_ratio_vol = common::ISqr(sqrtf(params_.iou_high));
    shrink_ratio_vol *= shrink_ratio_vol;
    // float shrink_ratio_vol = sqrt(params_.iou_high);
    if (hwl[0] < params_.abnormal_h_veh ||
        hwl[0] * hwl[1] * hwl[2] < min_tmplt_vol * shrink_ratio_vol) {
      memcpy(hwl, tmplt_with_min_vol, sizeof(float) * 3);
    } else {
      float hwl_tmplt[3] = {hwl[0], hwl[1], hwl[2]};
      int tmplt_index = -1;
      float score = object_template_manager_->VehObjHwlBySearchTemplates(
          hwl_tmplt, &tmplt_index);
      float thres_min_score = shrink_ratio_vol;

      const int kNrDimPerTmplt = object_template_manager_->NrDimPerTmplt();
      bool search_success = score > thres_min_score;
      bool is_same_type = (type_min_vol_index / kNrDimPerTmplt) == tmplt_index;
      const std::map<TemplateIndex, int> &kLookUpTableMinVolumeIndex =
          object_template_manager_->LookUpTableMinVolumeIndex();
      bool is_car_pred =
          type_min_vol_index ==
          kLookUpTableMinVolumeIndex.at(TemplateIndex::CAR_MIN_VOLUME_INDEX);

      bool hwl_is_reliable = search_success && is_same_type;
      if (hwl_is_reliable) {
        memcpy(hwl, hwl_tmplt, sizeof(float) * 3);
      } else if (is_car_pred) {
        const float *tmplt_with_median_vol =
            tmplt_with_min_vol + kNrDimPerTmplt;
        memcpy(hwl, tmplt_with_median_vol, sizeof(float) * 3);
      }
    }
  }

  // call 3d solver
  bool success =
      Solve3dBboxGivenOneFullBboxDimensionOrientation(bbox, hwl, ry, center);

  // calculate variance for yaw & z
  float yaw_score_mean =
      common::IMean(ry_score_.data(), static_cast<int>(ry_score_.size()));
  float yaw_score_sdv = common::ISdv(ry_score_.data(), yaw_score_mean,
                                     static_cast<int>(ry_score_.size()));
  var_yaw = common::ISqrt(common::IRec(yaw_score_sdv + params_.eps_mapper));

  float z = center[2];
  float rz = z * params_.rz_ratio;
  float nr_bins_z = static_cast<float>(params_.nr_bins_z);
  std::vector<float> buffer(static_cast<size_t>(2 * nr_bins_z), 0);
  float *score_z = buffer.data();
  float dz = 2 * rz / nr_bins_z;
  float z_start = std::max(z - rz, params_.depth_min);
  float z_end = z + rz;
  int count_z_test = 0;
  for (float z_test = z_start; z_test <= z_end; z_test += dz) {
    float center_test[3] = {center[0], center[1], center[2]};
    float sf = z_test * common::IRec(center_test[2]);
    common::IScale3(center_test, sf);
    float score_test = GetProjectionScore(*ry, bbox, hwl, center_test);
    score_z[count_z_test++] = score_test;
  }
  float z_score_mean = common::IMean(score_z, count_z_test);
  float z_score_sdv = common::ISdv(score_z, z_score_mean, count_z_test);
  var_z = common::ISqr(common::IRec(z_score_sdv + params_.eps_mapper));

  // fill the position_uncertainty_ and orientation_variance_
  orientation_variance_(0) = var_yaw;
  float bbox_cx = (bbox[0] + bbox[2]) / 2;
  float focal = (k_mat_[0] + k_mat_[4]) / 2;
  float sf_z_to_x = fabsf(bbox_cx - k_mat_[2]) * common::IRec(focal);
  float var_x = var_z * common::ISqr(sf_z_to_x);
  float var_xz = sf_z_to_x * var_z;
  position_uncertainty_(0, 0) = var_x;
  position_uncertainty_(2, 2) = var_z;
  position_uncertainty_(0, 2) = position_uncertainty_(2, 0) = var_xz;
  return success;
}
```


## lane车道线

## postprocessor 后处理

#### Process2D
```c++
bool DarkSCNNLanePostprocessor::Process2D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {

  frame->lane_objects.clear();
  auto start = std::chrono::high_resolution_clock::now();

  // 1. 拷贝检测到车道线数据到 lane_map
  cv::Mat lane_map(lane_map_height_, lane_map_width_, CV_32FC1);
  memcpy(lane_map.data, frame->lane_detected_blob->cpu_data(),
         lane_map_width_ * lane_map_height_ * sizeof(float));

  // 2. 从lane_map中采样点，并且投影他们到世界坐标
  // TODO(techoe): Should be fixed
  int y = static_cast<int>(lane_map.rows * 0.9 - 1);
  // TODO(techoe): Should be fixed
  int step_y = (y - 40) * (y - 40) / 6400 + 1;

  xy_points.clear();
  xy_points.resize(lane_type_num_);
  uv_points.clear();
  uv_points.resize(lane_type_num_);

  // 3.1 如果y大于0，每次移动step
  while (y > 0) {
    // 3.2 每次移动一列
    for (int x = 1; x < lane_map.cols - 1; ++x) {
      // 3.3 获取最大行，最小列的点的值
      int value = static_cast<int>(round(lane_map.at<float>(y, x)));
      
      // 3.4 如果lane在车道左边，value的值在spatialLUTind中      
      if ((value > 0 && value < 5) || value == 11) {
        // right edge (inner) of the lane
        if (value != static_cast<int>(round(lane_map.at<float>(y, x + 1)))) {
          Eigen::Matrix<float, 3, 1> img_point(
              static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_),
              1.0);
          Eigen::Matrix<float, 3, 1> xy_p;
          xy_p = trans_mat_ * img_point;
          Eigen::Matrix<float, 2, 1> xy_point;
          Eigen::Matrix<float, 2, 1> uv_point;
          if (std::fabs(xy_p(2)) < 1e-6) continue;
          xy_point << xy_p(0) / xy_p(2), xy_p(1) / xy_p(2);

          // Filter out lane line points
          if (xy_point(0) < 0.0 ||  // This condition is only for front camera
              xy_point(0) > max_longitudinal_distance_ ||
              std::abs(xy_point(1)) > 30.0) {
            continue;
          }
          uv_point << static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_);
          // 3.4 将xy_point放入xy_points，将uv_point放入uv_points
          // xy_points存放的是Lane的世界坐标，uv_points存放的是图像坐标
          if (xy_points[value].size() < minNumPoints_ || xy_point(0) < 50.0f ||
              std::fabs(xy_point(1) - xy_points[value].back()(1)) < 1.0f) {
            xy_points[value].push_back(xy_point);
            uv_points[value].push_back(uv_point);
          }
        }
      } else if (value >= 5 && value < lane_type_num_) {
        // Left edge (inner) of the lane
        if (value != static_cast<int>(round(lane_map.at<float>(y, x - 1)))) {
          Eigen::Matrix<float, 3, 1> img_point(
              static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_),
              1.0);
          Eigen::Matrix<float, 3, 1> xy_p;
          xy_p = trans_mat_ * img_point;
          Eigen::Matrix<float, 2, 1> xy_point;
          Eigen::Matrix<float, 2, 1> uv_point;
          if (std::fabs(xy_p(2)) < 1e-6) continue;
          xy_point << xy_p(0) / xy_p(2), xy_p(1) / xy_p(2);
          // Filter out lane line points
          if (xy_point(0) < 0.0 ||  // This condition is only for front camera
              xy_point(0) > max_longitudinal_distance_ ||
              std::abs(xy_point(1)) > 30.0) {
            continue;
          }
          uv_point << static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_);
          if (xy_points[value].size() < minNumPoints_ || xy_point(0) < 50.0f ||
              std::fabs(xy_point(1) - xy_points[value].back()(1)) < 1.0f) {
            xy_points[value].push_back(xy_point);
            uv_points[value].push_back(uv_point);
          }
        } else if (value >= lane_type_num_) {
          AWARN << "Lane line value shouldn't be equal or more than: "
                << lane_type_num_;
        }
      }
    }
    step_y = (y - 45) * (y - 45) / 6400 + 1;
    y -= step_y;
  }

  auto elapsed_1 = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds_1 =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
  time_1 += microseconds_1;

  // 4.移除异常值并进行ransac拟合
  std::vector<Eigen::Matrix<float, 4, 1>> coeffs;
  std::vector<Eigen::Matrix<float, 4, 1>> img_coeffs;
  std::vector<Eigen::Matrix<float, 2, 1>> selected_xy_points;
  coeffs.resize(lane_type_num_);
  img_coeffs.resize(lane_type_num_);
  for (int i = 1; i < lane_type_num_; ++i) {
    coeffs[i] << 0, 0, 0, 0;
    // 4.1 xy_points[i]即是每种类型lane中抽样的点的数组
    if (xy_points[i].size() < minNumPoints_) continue;
    Eigen::Matrix<float, 4, 1> coeff;
    // Solve linear system to estimate polynomial coefficients
    if (RansacFitting<float>(xy_points[i], &selected_xy_points, &coeff, 200,
                             static_cast<int>(minNumPoints_), 0.1f)) {
      coeffs[i] = coeff;

      xy_points[i].clear();
      xy_points[i] = selected_xy_points;
    } else {
      xy_points[i].clear();
    }
  }

  auto elapsed_2 = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds_2 =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_2).count();
  time_2 += microseconds_2 - microseconds_1;

  // 3. Write values into lane_objects
  std::vector<float> c0s(lane_type_num_, 0);
  for (int i = 1; i < lane_type_num_; ++i) {
    if (xy_points[i].size() < minNumPoints_) continue;
    c0s[i] = GetPolyValue(
        static_cast<float>(coeffs[i](3)), static_cast<float>(coeffs[i](2)),
        static_cast<float>(coeffs[i](1)), static_cast<float>(coeffs[i](0)),
        static_cast<float>(3.0));
  }
  // 在特殊情况下确定车道空间标签
  // 1.检查左边车道线是否小于最小采样点 2.检查右边车道线是否小于最小采样点
  if (xy_points[4].size() < minNumPoints_ &&
      xy_points[5].size() >= minNumPoints_) {
    std::swap(xy_points[4], xy_points[5]);
    std::swap(uv_points[4], uv_points[5]);
    std::swap(coeffs[4], coeffs[5]);
    std::swap(c0s[4], c0s[5]);
  } else if (xy_points[6].size() < minNumPoints_ &&
             xy_points[5].size() >= minNumPoints_) {
    std::swap(xy_points[6], xy_points[5]);
    std::swap(uv_points[6], uv_points[5]);
    std::swap(coeffs[6], coeffs[5]);
    std::swap(c0s[6], c0s[5]);
  }

  if (xy_points[4].size() < minNumPoints_ &&
      xy_points[11].size() >= minNumPoints_) {
    // Use left lane boundary as the right most missing left lane,
    bool use_boundary = true;
    for (int k = 3; k >= 1; --k) {
      if (xy_points[k].size() >= minNumPoints_) {
        use_boundary = false;
        break;
      }
    }
    if (use_boundary) {
      std::swap(xy_points[4], xy_points[11]);
      std::swap(uv_points[4], uv_points[11]);
      std::swap(coeffs[4], coeffs[11]);
      std::swap(c0s[4], c0s[11]);
    }
  }

  if (xy_points[6].size() < minNumPoints_ &&
      xy_points[12].size() >= minNumPoints_) {
    // Use right lane boundary as the left most missing right lane,
    bool use_boundary = true;
    for (int k = 7; k <= 9; ++k) {
      if (xy_points[k].size() >= minNumPoints_) {
        use_boundary = false;
        break;
      }
    }
    if (use_boundary) {
      std::swap(xy_points[6], xy_points[12]);
      std::swap(uv_points[6], uv_points[12]);
      std::swap(coeffs[6], coeffs[12]);
      std::swap(c0s[6], c0s[12]);
    }
  }

  for (int i = 1; i < lane_type_num_; ++i) {
    base::LaneLine cur_object;
    if (xy_points[i].size() < minNumPoints_) {
      continue;
    }

    // [2] Set spatial label
    cur_object.pos_type = spatialLUT[i];

    // [3] Determine which lines are valid according to the y value at x = 3
    if ((i < 5 && c0s[i] < c0s[i + 1]) ||
        (i > 5 && i < 10 && c0s[i] > c0s[i - 1])) {
      continue;
    }
    if (i == 11 || i == 12) {
      std::sort(c0s.begin(), c0s.begin() + 10);
      if ((c0s[i] > c0s[0] && i == 12) || (c0s[i] < c0s[9] && i == 11)) {
        continue;
      }
    }
    // [4] 结果写入cur_object
    cur_object.curve_car_coord.x_start =
        static_cast<float>(xy_points[i].front()(0));
    cur_object.curve_car_coord.x_end =
        static_cast<float>(xy_points[i].back()(0));
    cur_object.curve_car_coord.a = static_cast<float>(coeffs[i](3));
    cur_object.curve_car_coord.b = static_cast<float>(coeffs[i](2));
    cur_object.curve_car_coord.c = static_cast<float>(coeffs[i](1));
    cur_object.curve_car_coord.d = static_cast<float>(coeffs[i](0));
    // if (cur_object.curve_car_coord.x_end -
    //     cur_object.curve_car_coord.x_start < 5) continue;
    // cur_object.order = 2;
    cur_object.curve_car_coord_point_set.clear();
    for (size_t j = 0; j < xy_points[i].size(); ++j) {
      base::Point2DF p_j;
      p_j.x = static_cast<float>(xy_points[i][j](0));
      p_j.y = static_cast<float>(xy_points[i][j](1));
      cur_object.curve_car_coord_point_set.push_back(p_j);
    }

    cur_object.curve_image_point_set.clear();
    for (size_t j = 0; j < uv_points[i].size(); ++j) {
      base::Point2DF p_j;
      p_j.x = static_cast<float>(uv_points[i][j](0));
      p_j.y = static_cast<float>(uv_points[i][j](1));
      cur_object.curve_image_point_set.push_back(p_j);
    }

    // cur_object.confidence.push_back(1);
    cur_object.confidence = 1.0f;
    frame->lane_objects.push_back(cur_object);
  }

  // Special case riding on a lane:
  // 0: no center lane, 1: center lane as left, 2: center lane as right
  int has_center_ = 0;
  for (auto lane_ : frame->lane_objects) {
    if (lane_.pos_type == base::LaneLinePositionType::EGO_CENTER) {
      if (lane_.curve_car_coord.d >= 0) {
        has_center_ = 1;
      } else if (lane_.curve_car_coord.d < 0) {
        has_center_ = 2;
      }
      break;
    }
  }
  // Change labels for all lanes from one side
  if (has_center_ == 1) {
    for (auto& lane_ : frame->lane_objects) {
      int spatial_id = spatialLUTind[lane_.pos_type];
      if (spatial_id >= 1 && spatial_id <= 5) {
        lane_.pos_type = spatialLUT[spatial_id - 1];
      }
    }
  } else if (has_center_ == 2) {
    for (auto& lane_ : frame->lane_objects) {
      int spatial_id = spatialLUTind[lane_.pos_type];
      if (spatial_id >= 5 && spatial_id <= 9) {
        lane_.pos_type = spatialLUT[spatial_id + 1];
      }
    }
  }

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  // AINFO << "Time for writing: " << microseconds - microseconds_2 << " us";
  time_3 += microseconds - microseconds_2;
  ++time_num;

  ADEBUG << "frame->lane_objects.size(): " << frame->lane_objects.size();

  ADEBUG << "Avg sampling time: " << time_1 / time_num
         << " Avg fitting time: " << time_2 / time_num
         << " Avg writing time: " << time_3 / time_num;
  ADEBUG << "darkSCNN lane_postprocess done!";
  return true;
}
```


#### DarkSCNN
```c++
// Produce laneline output in camera coordinates (optional)
bool DarkSCNNLanePostprocessor::Process3D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  ConvertImagePoint2Camera(frame);
  PolyFitCameraLaneline(frame);
  return true;
}
```

图片坐标转换到相机
```c++
void DarkSCNNLanePostprocessor::ConvertImagePoint2Camera(CameraFrame* frame) {
  float pitch_angle = frame->calibration_service->QueryPitchAngle();
  float camera_ground_height =
      frame->calibration_service->QueryCameraToGroundHeight();
  const Eigen::Matrix3f& intrinsic_params = frame->camera_k_matrix;
  const Eigen::Matrix3f& intrinsic_params_inverse = intrinsic_params.inverse();
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  for (int line_index = 0; line_index < laneline_num; ++line_index) {
    std::vector<base::Point2DF>& image_point_set =
        lane_objects[line_index].curve_image_point_set;
    std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    for (int i = 0; i < static_cast<int>(image_point_set.size()); i++) {
      base::Point3DF camera_point;
      Eigen::Vector3d camera_point3d;
      const base::Point2DF& image_point = image_point_set[i];
      ImagePoint2Camera(image_point, pitch_angle, camera_ground_height,
                        intrinsic_params_inverse, &camera_point3d);
      camera_point.x = static_cast<float>(camera_point3d(0));
      camera_point.y = static_cast<float>(camera_point3d(1));
      camera_point.z = static_cast<float>(camera_point3d(2));
      camera_point_set.push_back(camera_point);
    }
  }
}
```

获取车道线点
```c++
void DarkSCNNLanePostprocessor::PolyFitCameraLaneline(CameraFrame* frame) {
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  for (int line_index = 0; line_index < laneline_num; ++line_index) {
    const std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    // z: longitudinal direction
    // x: latitudinal direction
    float x_start = camera_point_set[0].z;
    float x_end = 0.0f;
    Eigen::Matrix<float, max_poly_order + 1, 1> camera_coeff;
    std::vector<Eigen::Matrix<float, 2, 1>> camera_pos_vec;
    for (int i = 0; i < static_cast<int>(camera_point_set.size()); ++i) {
      x_end = std::max(camera_point_set[i].z, x_end);
      x_start = std::min(camera_point_set[i].z, x_start);
      Eigen::Matrix<float, 2, 1> camera_pos;
      camera_pos << camera_point_set[i].z, camera_point_set[i].x;
      camera_pos_vec.push_back(camera_pos);
    }

    bool is_x_axis = true;
    bool fit_flag =
        PolyFit(camera_pos_vec, max_poly_order, &camera_coeff, is_x_axis);
    if (!fit_flag) {
      continue;
    }
    lane_objects[line_index].curve_camera_coord.a = camera_coeff(3, 0);
    lane_objects[line_index].curve_camera_coord.b = camera_coeff(2, 0);
    lane_objects[line_index].curve_camera_coord.c = camera_coeff(1, 0);
    lane_objects[line_index].curve_camera_coord.d = camera_coeff(0, 0);
    lane_objects[line_index].curve_camera_coord.x_start = x_start;
    lane_objects[line_index].curve_camera_coord.x_end = x_end;
    lane_objects[line_index].use_type = base::LaneLineUseType::REAL;
  }
}
```





#### 数据集
[CULane Dataset](https://xingangpan.github.io/projects/CULane.html)  
[bdd](https://bdd-data.berkeley.edu/)  
[tusimple](https://github.com/TuSimple/tusimple-benchmark)  


#### 参考
[awesome-lane-detection](https://github.com/amusi/awesome-lane-detection)  



## calibration_service & calibrator
首先看OnlineCalibrationService类的实现。

#### OnlineCalibrationService
初始化Init  
```c++
bool OnlineCalibrationService::Init(
    const CalibrationServiceInitOptions &options) {
  master_sensor_name_ = options.calibrator_working_sensor_name;
  sensor_name_ = options.calibrator_working_sensor_name;
  // 1. 初始化K矩阵
  auto &name_intrinsic_map = options.name_intrinsic_map;
  ACHECK(name_intrinsic_map.find(master_sensor_name_) !=
         name_intrinsic_map.end());
  CameraStatus camera_status;
  name_camera_status_map_.clear();
  for (auto iter = name_intrinsic_map.begin(); iter != name_intrinsic_map.end();
       ++iter) {
    camera_status.k_matrix[0] = static_cast<double>(iter->second(0, 0));
    camera_status.k_matrix[4] = static_cast<double>(iter->second(1, 1));
    camera_status.k_matrix[2] = static_cast<double>(iter->second(0, 2));
    camera_status.k_matrix[5] = static_cast<double>(iter->second(1, 2));
    camera_status.k_matrix[8] = 1.0;
    name_camera_status_map_.insert(
        std::pair<std::string, CameraStatus>(iter->first, camera_status));
  }
  // 初始化校准参数
  CalibratorInitOptions calibrator_init_options;
  calibrator_init_options.image_width = options.image_width;
  calibrator_init_options.image_height = options.image_height;
  calibrator_init_options.focal_x = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[0]);
  calibrator_init_options.focal_y = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[4]);
  calibrator_init_options.cx = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[2]);
  calibrator_init_options.cy = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[5]);
  calibrator_.reset(
      BaseCalibratorRegisterer::GetInstanceByName(options.calibrator_method));
  ACHECK(calibrator_ != nullptr);
  ACHECK(calibrator_->Init(calibrator_init_options))
      << "Failed to init " << options.calibrator_method;
  return true;
}
```

更新帧信息  
```c++
void OnlineCalibrationService::Update(CameraFrame *frame) {

  sensor_name_ = frame->data_provider->sensor_name();
  if (sensor_name_ == master_sensor_name_) {
    CalibratorOptions calibrator_options;
    calibrator_options.lane_objects =
        std::make_shared<std::vector<base::LaneLine>>(frame->lane_objects);
    calibrator_options.camera2world_pose =
        std::make_shared<Eigen::Affine3d>(frame->camera2world_pose);
    calibrator_options.timestamp = &(frame->timestamp);
    float pitch_angle = 0.f;
    // 1. 校准传感器参数
    bool updated = calibrator_->Calibrate(calibrator_options, &pitch_angle);
    // rebuild the service when updated
    if (updated) {
      name_camera_status_map_[master_sensor_name_].pitch_angle = pitch_angle;
      for (auto iter = name_camera_status_map_.begin();
           iter != name_camera_status_map_.end(); iter++) {
        // 2. 更新俯仰角
        iter->second.pitch_angle =
            iter->second.pitch_angle_diff +
            name_camera_status_map_[master_sensor_name_].pitch_angle;
        // 3. 更新地平面
        iter->second.ground_plane[1] = cos(iter->second.pitch_angle);
        iter->second.ground_plane[2] = -sin(iter->second.pitch_angle);
      }
    }
  }
  auto iter = name_camera_status_map_.find(sensor_name_);
  AINFO << "camera_ground_height: " << iter->second.camera_ground_height
        << " meter.";
  AINFO << "pitch_angle: " << iter->second.pitch_angle * 180.0 / M_PI
        << " degree.";
  is_service_ready_ = true;
}
```

#### calibrator
LaneLineCalibrator依赖LaneBasedCalibrator。  
```c++
bool LaneLineCalibrator::Calibrate(const CalibratorOptions &options,
                                   float *pitch_angle) {
  // 1. 加载当前车道线
  EgoLane ego_lane;
  if (!LoadEgoLaneline(*options.lane_objects, &ego_lane)) {
    AINFO << "Failed to get the ego lane.";
    return false;
  }

  double cam_ori[4] = {0};
  cam_ori[3] = 1.0;

  // 2. 相机坐标到世界坐标
  Eigen::Affine3d c2w = *options.camera2world_pose;

  double p2w[12] = {
      c2w(0, 0), c2w(0, 1), c2w(0, 2), c2w(0, 3), c2w(1, 0), c2w(1, 1),
      c2w(1, 2), c2w(1, 3), c2w(2, 0), c2w(2, 1), c2w(2, 2), c2w(2, 3),
  };

  ADEBUG << "c2w transform this frame:\n"
         << p2w[0] << ", " << p2w[1] << ", " << p2w[2] << ", " << p2w[3] << "\n"
         << p2w[4] << ", " << p2w[5] << ", " << p2w[6] << ", " << p2w[7] << "\n"
         << p2w[8] << ", " << p2w[9] << ", " << p2w[10] << ", " << p2w[11];

  common::IMultAx3x4(p2w, cam_ori, cam_coord_cur_);
  time_diff_ = kTimeDiffDefault;
  yaw_rate_ = kYawRateDefault;
  velocity_ = kVelocityDefault;

  timestamp_cur_ = *options.timestamp;
  if (!is_first_frame_) {
    time_diff_ = fabsf(static_cast<float>(timestamp_cur_ - timestamp_pre_));
    ADEBUG << timestamp_cur_ << " " << timestamp_pre_ << std::endl;
    camera::GetYawVelocityInfo(time_diff_, cam_coord_cur_, cam_coord_pre_,
                               cam_coord_pre_pre_, &yaw_rate_, &velocity_);
    std::string timediff_yawrate_velocity_text =
        absl::StrCat("time_diff_: ", std::to_string(time_diff_).substr(0, 4),
                     " | yaw_rate_: ", std::to_string(yaw_rate_).substr(0, 4),
                     " | velocity_: ", std::to_string(velocity_).substr(0, 4));
    ADEBUG << timediff_yawrate_velocity_text << std::endl;
  }

  bool updated =
      calibrator_.Process(ego_lane, velocity_, yaw_rate_, time_diff_);
  if (updated) {
    *pitch_angle = calibrator_.get_pitch_estimation();
    float vanishing_row = calibrator_.get_vanishing_row();
    AINFO << "#updated pitch angle: " << *pitch_angle;
    AINFO << "#vanishing row: " << vanishing_row;
  }

  if (!is_first_frame_) {
    memcpy(cam_coord_pre_pre_, cam_coord_pre_, sizeof(double) * 3);
  }
  is_first_frame_ = false;
  memcpy(cam_coord_pre_, cam_coord_cur_, sizeof(double) * 3);
  timestamp_pre_ = timestamp_cur_;
  return updated;
}
```

LaneBasedCalibrator的校准过程。  
```c++
bool LaneBasedCalibrator::Process(const EgoLane &lane, const float &velocity,
                                  const float &yaw_rate,
                                  const float &time_diff) {
  float distance_traveled_in_meter = velocity * time_diff;
  float vehicle_yaw_changed = yaw_rate * time_diff;

  // 1. 检查是否直行
  if (!IsTravelingStraight(vehicle_yaw_changed)) {
    AINFO << "Do not calibate if not moving straight: "
          << "yaw angle changed " << vehicle_yaw_changed;
    vp_buffer_.clear();
    return false;
  }

  VanishingPoint vp_cur;
  VanishingPoint vp_work;

  // 2. 从车道获取当前消失点的估计值
  if (!GetVanishingPoint(lane, &vp_cur)) {
    AINFO << "Lane is not valid for calibration.";
    return false;
  }
  vp_cur.distance_traveled = distance_traveled_in_meter;

  // Push vanishing point into buffer
  PushVanishingPoint(vp_cur);
  if (!PopVanishingPoint(&vp_work)) {
    AINFO << "Driving distance is not long enough";
    return false;
  }

  // 获取当前的音高估计
  pitch_cur_ = 0.0f;
  if (!GetPitchFromVanishingPoint(vp_work, &pitch_cur_)) {
    AINFO << "Failed to estimate pitch from vanishing point.";
    return false;
  }
  vanishing_row_ = vp_work.pixel_pos[1];

  // Get the filtered output using histogram
  if (!AddPitchToHistogram(pitch_cur_)) {
    AINFO << "Calculated pitch is out-of-range.";
    return false;
  }

  accumulated_straight_driving_in_meter_ += distance_traveled_in_meter;

  if (accumulated_straight_driving_in_meter_ >
          params_.min_distance_to_update_calibration_in_meter &&
      pitch_histogram_.Process()) {
    pitch_estimation_ = pitch_histogram_.get_val_estimation();
    const float cy = k_mat_[5];
    const float fy = k_mat_[4];
    vanishing_row_ = tanf(pitch_estimation_) * fy + cy;
    accumulated_straight_driving_in_meter_ = 0.0f;
    return true;
  }
  return false;
}
```


## feature_extractor
TrackingFeatureExtractor追踪特征提取器。  
```c++
bool TrackingFeatureExtractor::Init(
    const FeatureExtractorInitOptions &init_options) {
  //  setup bottom and top
  int feat_height = init_options.feat_blob->shape(2);
  int feat_width = init_options.feat_blob->shape(3);
  input_height_ =
      init_options.input_height == 0 ? feat_height : init_options.input_height;
  input_width_ =
      init_options.input_width == 0 ? feat_width : init_options.input_width;
  tracking_feature::FeatureParam feat_param;
  std::string config_path = cyber::common::GetAbsolutePath(
      init_options.root_dir, init_options.conf_file);
  // 1. 获取配置  
  if (!cyber::common::GetProtoFromFile(config_path, &feat_param)) {
    return false;
  }
  if (feat_param.extractor_size() != 1) {
    return false;
  }
  CHECK_EQ(input_height_ / feat_height, input_width_ / feat_width)
      << "Invalid aspect ratio: " << feat_height << "x" << feat_width
      << " from " << input_height_ << "x" << input_width_;

  // 2. 获取ROI区域pooling
  feat_blob_ = init_options.feat_blob;
  for (int i = 0; i < feat_param.extractor_size(); i++) {
    switch (feat_param.extractor(i).feat_type()) {
      case tracking_feature::ExtractorParam_FeatureType_ROIPooling:
        init_roipooling(init_options,
                        feat_param.extractor(i).roi_pooling_param());
        break;
    }
  }
  if (roi_poolings_.empty()) {
    AERROR << "no proper extractor";
    return false;
  }

  return true;
}
```

初始化感兴趣区域？
```c++
void TrackingFeatureExtractor::init_roipooling(
    const FeatureExtractorInitOptions &options,
    const tracking_feature::ROIPoolingParam &param) {
  int feat_channel = options.feat_blob->shape(1);
  feat_height_ = options.feat_blob->shape(2);
  feat_width_ = options.feat_blob->shape(3);

  std::shared_ptr<FeatureExtractorLayer> feature_extractor_layer_ptr;
  feature_extractor_layer_ptr.reset(new FeatureExtractorLayer());
  std::vector<int> shape{1, 5};
  feature_extractor_layer_ptr->rois_blob.reset(new base::Blob<float>(shape));
  int pooled_w = param.pooled_w();
  int pooled_h = param.pooled_h();
  bool use_floor = param.use_floor();
  feature_extractor_layer_ptr->pooling_layer.reset(
      new inference::ROIPoolingLayer<float>(pooled_h, pooled_w, use_floor, 1,
                                            feat_channel));
  feature_extractor_layer_ptr->top_blob.reset(
      new base::Blob<float>(1, feat_blob_->channels(), pooled_h, pooled_w));
  roi_poolings_.push_back(feature_extractor_layer_ptr);
}
```

解压tracking特征  
```c++
bool TrackingFeatureExtractor::Extract(const FeatureExtractorOptions &options,
                                       CameraFrame *frame) {

  if (frame->detected_objects.empty()) {
    return true;
  }
  if (!options.normalized) {
    encode_bbox(&(frame->detected_objects));
  }
  for (auto feature_extractor_layer_ptr : roi_poolings_) {
    feature_extractor_layer_ptr->rois_blob->Reshape(
        {static_cast<int>(frame->detected_objects.size()), 5});
    float *rois_data =
        feature_extractor_layer_ptr->rois_blob->mutable_cpu_data();
    for (const auto &obj : frame->detected_objects) {
      rois_data[0] = 0;
      rois_data[1] =
          obj->camera_supplement.box.xmin * static_cast<float>(feat_width_);
      rois_data[2] =
          obj->camera_supplement.box.ymin * static_cast<float>(feat_height_);
      rois_data[3] =
          obj->camera_supplement.box.xmax * static_cast<float>(feat_width_);
      rois_data[4] =
          obj->camera_supplement.box.ymax * static_cast<float>(feat_height_);
      ADEBUG << rois_data[0] << " " << rois_data[1] << " " << rois_data[2]
             << " " << rois_data[3] << " " << rois_data[4];
      rois_data += feature_extractor_layer_ptr->rois_blob->offset(1);
    }
    feature_extractor_layer_ptr->pooling_layer->ForwardGPU(
        {feat_blob_, feature_extractor_layer_ptr->rois_blob},
        {frame->track_feature_blob});

    if (!options.normalized) {
      decode_bbox(&(frame->detected_objects));
    }
  }
  norm_.L2Norm(frame->track_feature_blob.get());
  return true;
}
```


ProjectFeature投影特征  
```c++
bool ProjectFeature::Init(const FeatureExtractorInitOptions &options) {
  std::string efx_config = GetAbsolutePath(options.root_dir, options.conf_file);
  ACHECK(cyber::common::GetProtoFromFile(efx_config, &param_))
      << "Read config failed: " << efx_config;
  AINFO << "Load config Success: " << param_.ShortDebugString();
  std::string proto_file =
      GetAbsolutePath(options.root_dir, param_.proto_file());
  std::string weight_file =
      GetAbsolutePath(options.root_dir, param_.weight_file());
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  input_names.push_back(param_.input_blob());
  output_names.push_back(param_.feat_blob());
  const auto &model_type = param_.model_type();
  AINFO << "model_type=" << model_type;
  inference_.reset(inference::CreateInferenceByName(
      model_type, proto_file, weight_file, output_names, input_names,
      options.root_dir));
  ACHECK(nullptr != inference_) << "Failed to init CNNAdapter";
  gpu_id_ = GlobalConfig::Instance()->track_feature_gpu_id;
  inference_->set_gpu_id(gpu_id_);
  inference_->set_max_batch_size(100);
  std::vector<int> shape = {5, 64, 3, 3};
  std::map<std::string, std::vector<int>> shape_map{
      {param_.input_blob(), shape}};

  ACHECK(inference_->Init(shape_map));
  inference_->Infer();
  return true;
}
```

解压特征  
```c++
bool ProjectFeature::Extract(const FeatureExtractorOptions &options,
                             CameraFrame *frame) {
  auto input_blob = inference_->get_blob(param_.input_blob());
  auto output_blob = inference_->get_blob(param_.feat_blob());
  if (frame->detected_objects.empty()) {
    return true;
  }
  input_blob->Reshape(frame->track_feature_blob->shape());
  cudaMemcpy(
      input_blob->mutable_gpu_data(), frame->track_feature_blob->gpu_data(),
      frame->track_feature_blob->count() * sizeof(float), cudaMemcpyDefault);

  cudaDeviceSynchronize();
  inference_->Infer();
  cudaDeviceSynchronize();
  frame->track_feature_blob->Reshape(
      {static_cast<int>(frame->detected_objects.size()), output_blob->shape(1),
       output_blob->shape(2), output_blob->shape(3)});

  cudaMemcpy(
      frame->track_feature_blob->mutable_gpu_data(), output_blob->gpu_data(),
      frame->track_feature_blob->count() * sizeof(float), cudaMemcpyDefault);

  norm_.L2Norm(frame->track_feature_blob.get());
  return true;
}
```

ExternalFeatureExtractor扩展特征初始化
```c++
bool ExternalFeatureExtractor::Init(
    const FeatureExtractorInitOptions &options) {
  std::string efx_config = GetAbsolutePath(options.root_dir, options.conf_file);
  ACHECK(cyber::common::GetProtoFromFile(efx_config, &param_))
      << "Read config failed: " << efx_config;
  AINFO << "Load config Success: " << param_.ShortDebugString();
  std::string proto_file =
      GetAbsolutePath(options.root_dir, param_.proto_file());
  std::string weight_file =
      GetAbsolutePath(options.root_dir, param_.weight_file());
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  input_names.push_back(param_.input_blob());
  output_names.push_back(param_.feat_blob());
  height_ = param_.resize_height();
  width_ = param_.resize_width();
  const auto &model_type = param_.model_type();
  AINFO << "model_type=" << model_type;
  inference_.reset(inference::CreateInferenceByName(
      model_type, proto_file, weight_file, output_names, input_names,
      options.root_dir));
  ACHECK(nullptr != inference_) << "Failed to init CNNAdapter";
  gpu_id_ = GlobalConfig::Instance()->track_feature_gpu_id;
  inference_->set_gpu_id(gpu_id_);
  std::vector<int> shape = {1, height_, width_, 3};
  std::map<std::string, std::vector<int>> shape_map{
      {param_.input_blob(), shape}};

  ACHECK(inference_->Init(shape_map));
  inference_->Infer();
  InitFeatureExtractor(options.root_dir);
  image_.reset(new base::Image8U(height_, width_, base::Color::BGR));
  return true;
}
```
扩展特征解压  
```c++
bool ExternalFeatureExtractor::Extract(const FeatureExtractorOptions &options,
                                       CameraFrame *frame) {
  int raw_height = frame->data_provider->src_height();
  int raw_width = frame->data_provider->src_width();
  auto input_blob = inference_->get_blob(param_.input_blob());
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  auto offset_y_ = static_cast<int>(
      param_.offset_ratio() * static_cast<float>(raw_height) + 0.5f);
  image_options.crop_roi =
      base::RectI(0, offset_y_, raw_width, raw_height - offset_y_);
  image_options.do_crop = true;
  // Timer timer;
  frame->data_provider->GetImage(image_options, image_.get());
  inference::ResizeGPU(*image_, input_blob, raw_width, 0);
  inference_->Infer();
  FeatureExtractorOptions feat_options;
  feat_options.normalized = false;
  feature_extractor_->set_roi(
      image_options.crop_roi.x, image_options.crop_roi.y,
      image_options.crop_roi.width, image_options.crop_roi.height);
  feature_extractor_->Extract(feat_options, frame);
  AINFO << "Extract Done";
  return true;
}
```
