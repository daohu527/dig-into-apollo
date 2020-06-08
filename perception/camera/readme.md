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





## obstacle
detector // 检测
postprocessor // 后处理？做了什么处理???
tracker // 追踪
transformer // 坐标转换

## lane
detector // 检测
postprocessor // 后处理


**数据集**
[CULane Dataset](https://xingangpan.github.io/projects/CULane.html)  
[bdd](https://bdd-data.berkeley.edu/)  
[tusimple](https://github.com/TuSimple/tusimple-benchmark)  



**参考**
[awesome-lane-detection](https://github.com/amusi/awesome-lane-detection)  



#### calibration_service & calibrator



#### feature_extractor ???


