# Dig into Apollo - Localization ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout) 

## 目录
```
.
├── BUILD
├── localization_pose_buffer.cc
├── localization_pose_buffer.h
├── localization_pose_buffer_test.cc
├── map_creation
├── ndt_localization.cc
├── ndt_localization_component.cc
├── ndt_localization_component.h
├── ndt_localization.h
├── ndt_localization_test.cc
├── ndt_locator
├── README.md
└── test_data
```

## map_creation
加载pcd pose文件
```c++
    apollo::localization::msf::velodyne::LoadPcdPoses(
        pose_files[i], &pcd_poses[i], &time_stamps[i], &pcd_indices[i]);
```


## 创建NDT地图
创建ndt地图
```c++
apollo::localization::msf::pyramid_map::NdtMap
apollo::localization::msf::pyramid_map::NdtMapNodePool
```

## 平面提取

```c++
apollo::localization::msf::FeatureXYPlane
```

下面主要介绍一下local_map中的base_map
## base_map

#### base_map
BaseMap中包括map_node_pool_，也就是说一些map_node_pool_缓存了一些map_node。因为加载map_node需要一些时间，因此采用了提前加载并且缓存的方式。map_node_cache_lvl1_和map_node_cache_lvl2_都是一个LRU的cache。
```c++
BaseMap::BaseMap(BaseMapConfig* map_config)
    : map_config_(map_config),
      map_node_cache_lvl1_(nullptr),
      map_node_cache_lvl2_(nullptr),
      map_node_pool_(nullptr) {}
```

先从map_node_cache_lvl1_中获取map node，如果没有然后从map_node_cache_lvl2_中获取，并且存放到map_node_cache_lvl1_，如果都没有，则从磁盘中读取，磁盘中读取直接放入map_node_cache_lvl2_，然后放入map_node_cache_lvl1_并且返回。
```
BaseMapNode* BaseMap::GetMapNodeSafe(const MapNodeIndex& index)
```


提前加载地图节点
```c++
void BaseMap::PreloadMapNodes(std::set<MapNodeIndex>* map_ids) {
  DCHECK_LE(static_cast<int>(map_ids->size()),
            map_node_cache_lvl2_->Capacity());
  // check in cacheL2
  typename std::set<MapNodeIndex>::iterator itr = map_ids->begin();
  while (itr != map_ids->end()) {
    boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
    bool is_exist = map_node_cache_lvl2_->IsExist(*itr);
    lock.unlock();
    if (is_exist) {
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }

  // check whether in already preloading index set
  itr = map_ids->begin();
  auto preloading_itr = map_preloading_task_index_.end();
  while (itr != map_ids->end()) {
    boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
    preloading_itr = map_preloading_task_index_.find(*itr);
    lock.unlock();
    if (preloading_itr !=
        map_preloading_task_index_.end()) {  // already preloading
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }

  // load form disk sync
  std::vector<std::future<void>> preload_futures;
  itr = map_ids->begin();
  AINFO << "Preload map node size: " << map_ids->size();
  while (itr != map_ids->end()) {
    AINFO << "Preload map node: " << *itr << std::endl;
    boost::unique_lock<boost::recursive_mutex> lock3(map_load_mutex_);
    map_preloading_task_index_.insert(*itr);
    lock3.unlock();
    preload_futures.emplace_back(
        cyber::Async(&BaseMap::LoadMapNodeThreadSafety, this, *itr, false));
    ++itr;
  }
  return;
}
```

加载地图节点
```c++
void BaseMap::LoadMapNodes(std::set<MapNodeIndex>* map_ids) {

  // 先在cacheL1中查找，如果找到则从map_ids删除找到的节点，最终剩下没有找到的节点
  typename std::set<MapNodeIndex>::iterator itr = map_ids->begin();
  while (itr != map_ids->end()) {
    if (map_node_cache_lvl1_->IsExist(*itr)) {
      // std::cout << "LoadMapNodes find in L1 cache" << std::endl;
      boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
      map_node_cache_lvl2_->IsExist(*itr);  // fresh lru list
      lock.unlock();
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }

  // 接着在cacheL2中查找，如果找到则从map_ids删除找到的节点，最终剩下没有找到的节点
  itr = map_ids->begin();
  BaseMapNode* node = nullptr;
  boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
  while (itr != map_ids->end()) {
    if (map_node_cache_lvl2_->Get(*itr, &node)) {
      // std::cout << "LoadMapNodes find in L2 cache" << std::endl;
      node->SetIsReserved(true);
      map_node_cache_lvl1_->Put(*itr, node);
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }
  lock.unlock();

  // 并发从硬盘中加载map_id的节点
  std::vector<std::future<void>> load_futures;
  itr = map_ids->begin();
  while (itr != map_ids->end()) {
    AERROR << "Preload map node failed!";
    load_futures.emplace_back(
        cyber::Async(&BaseMap::LoadMapNodeThreadSafety, this, *itr, true));
    ++itr;
  }

  for (auto& future : load_futures) {
    if (future.valid()) {
      future.get();
    }
  }
  // 然后查看cacheL2是否有对应的map_id的节点，上一步从硬盘中读取的map_node会先放入le_cache。
  itr = map_ids->begin();
  node = nullptr;
  boost::unique_lock<boost::recursive_mutex> lock2(map_load_mutex_);
  while (itr != map_ids->end()) {
    if (map_node_cache_lvl2_->Get(*itr, &node)) {
      AINFO << "LoadMapNodes: preload missed, load this node in main thread.\n"
            << *itr;
      node->SetIsReserved(true);
      map_node_cache_lvl1_->Put(*itr, node);
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }
  lock2.unlock();

  // 检查是否所有节点都已经加载，如果不是则报错
  CHECK(map_ids->empty());
  return;
}
```

根据index从硬盘中加载地图，map_node_pool_和map_node_cache_lvl2_的关系是什么？
```c++
void BaseMap::LoadMapNodeThreadSafety(MapNodeIndex index, bool is_reserved) {
  BaseMapNode* map_node = nullptr;
  while (map_node == nullptr) {
    map_node = map_node_pool_->AllocMapNode();
    if (map_node == nullptr) {
      boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
      BaseMapNode* node_remove = map_node_cache_lvl2_->ClearOne();
      if (node_remove) {
        map_node_pool_->FreeMapNode(node_remove);
      }
    }
  }
  // 初始化map_node，并且加载
  map_node->Init(map_config_, index, false);
  if (!map_node->Load()) {
    AERROR << "Created map node: " << index;
  } else {
    AERROR << " Loaded map node: " << index;
  }
  map_node->SetIsReserved(is_reserved);

  // 添加到map_node_cache_lvl2_
  boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
  BaseMapNode* node_remove = map_node_cache_lvl2_->Put(index, map_node);
  // if the node already added into cacheL2, erase it from preloading set
  auto itr = map_preloading_task_index_.find(index);
  if (itr != map_preloading_task_index_.end()) {
    map_preloading_task_index_.erase(itr);
  }
  // 在map_node_pool_中释放node_remove
  if (node_remove) {
    map_node_pool_->FreeMapNode(node_remove);
  }
  return;
}
```


提前加载地图区域
```c++
void BaseMap::PreloadMapArea(const Eigen::Vector3d& location,
                             const Eigen::Vector3d& trans_diff,
                             unsigned int resolution_id, unsigned int zone_id) {
  // 四象限中的位置
  int x_direction = trans_diff[0] > 0 ? 1 : -1;
  int y_direction = trans_diff[1] > 0 ? 1 : -1;

  // 获取地图的精度
  std::set<MapNodeIndex> map_ids;
  float map_pixel_resolution =
      this->map_config_->map_resolutions_[resolution_id];

  /// 车的位置平移一个map_node的距离
  // 根据top_left, 查找map_id，其中resolution_id代表精度数组的编号
  Eigen::Vector3d pt_top_left;
  pt_top_left[0] =
      location[0] - (static_cast<float>(this->map_config_->map_node_size_x_) *
                     map_pixel_resolution / 2.0f);
  pt_top_left[1] =
      location[1] - (static_cast<float>(this->map_config_->map_node_size_y_) *
                     map_pixel_resolution / 2.0f);
  pt_top_left[2] = 0;
  
  MapNodeIndex map_id = MapNodeIndex::GetMapNodeIndex(
      *(this->map_config_), pt_top_left, resolution_id, zone_id);
  map_ids.insert(map_id);

  // 同时根据以下几个坐标，查找map_id
  /// top center
  /// top right
  /// middle left
  /// middle center
  /// middle right
  /// bottom left
  /// bottom center
  /// bottom right

  // 根据x_direction * 1.5，y_direction * 1.5和(x_direction * 1.5, y_direction * 1.5)
  // 查找map_id
  for (int i = -1; i < 2; ++i) {
    Eigen::Vector3d pt;
    pt[0] = location[0] + x_direction * 1.5 *
                              this->map_config_->map_node_size_x_ *
                              map_pixel_resolution;
    pt[1] = location[1] + static_cast<double>(i) *
                              this->map_config_->map_node_size_y_ *
                              map_pixel_resolution;
    pt[2] = 0;
    map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt,
                                           resolution_id, zone_id);
    map_ids.insert(map_id);
  }
  // y_direction * 1.5
  // (x_direction * 1.5, y_direction * 1.5)

  this->PreloadMapNodes(&map_ids);
  return;
}
```

根据seed_pt3d的位置，加载地图区域
```c++
bool BaseMap::LoadMapArea(const Eigen::Vector3d& seed_pt3d,
                          unsigned int resolution_id, unsigned int zone_id,
                          int filter_size_x, int filter_size_y) {
  CHECK_NOTNULL(map_node_pool_);
  std::set<MapNodeIndex> map_ids;
  float map_pixel_resolution =
      this->map_config_->map_resolutions_[resolution_id];
  /// top left
  Eigen::Vector3d pt_top_left;
  pt_top_left[0] = seed_pt3d[0] -
                   (static_cast<float>(this->map_config_->map_node_size_x_) *
                    map_pixel_resolution / 2.0f) -
                   static_cast<float>(filter_size_x / 2) * map_pixel_resolution;
  pt_top_left[1] = seed_pt3d[1] -
                   (static_cast<float>(this->map_config_->map_node_size_y_) *
                    map_pixel_resolution / 2.0f) -
                   static_cast<float>(filter_size_y / 2) * map_pixel_resolution;
  pt_top_left[2] = 0;
  MapNodeIndex map_id = MapNodeIndex::GetMapNodeIndex(
      *(this->map_config_), pt_top_left, resolution_id, zone_id);
  map_ids.insert(map_id);

  /// top center
  /// top right
  /// middle left
  /// middle center
  /// middle right
  /// bottom left
  /// bottom center
  /// bottom right

  this->LoadMapNodes(&map_ids);
  return true;
}
```


## NdtMapNode
NdtMapNode是地图的单元格，主要是用来确定无人车的位置，这里的x,y是相对node的位置，然后根据(x,y)的坐标转换为UTM的绝对位置。
```c++
Eigen::Vector3d NdtMapNode::GetCoordinate3D(unsigned int x, unsigned int y,
                                            int altitude_index) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  Eigen::Vector2d coord_2d;
  coord_2d[0] =
      left_top_corner[0] + (static_cast<double>(x)) * GetMapResolution();
  coord_2d[1] =
      left_top_corner[1] + (static_cast<double>(y)) * GetMapResolution();

  double altitude =
      NdtMapCells::CalAltitude(GetMapResolutionZ(), altitude_index);
  Eigen::Vector3d coord_3d;
  coord_3d[0] = coord_2d[0];
  coord_3d[1] = coord_2d[1];
  coord_3d[2] = altitude;

  return coord_3d;
}
```



同时还涉及2个Node节点如何做合并，这里主要是调用了NdtMapMatrix的合并方法。
```c++
void NdtMapNode::Reduce(NdtMapNode* map_node, const NdtMapNode& map_node_new) {
  assert(map_node->index_.m_ == map_node_new.index_.m_);
  assert(map_node->index_.n_ == map_node_new.index_.n_);
  assert(map_node->index_.resolution_id_ == map_node_new.index_.resolution_id_);
  assert(map_node->index_.zone_id_ == map_node_new.index_.zone_id_);
  NdtMapMatrix::Reduce(
      static_cast<NdtMapMatrix*>(map_node->map_matrix_),
      static_cast<const NdtMapMatrix&>(*map_node_new.map_matrix_));
}
```

NdtMapMatrix包含(m,n)的NdtMapCells矩阵，这里遍历合并2个矩阵。我们主要看下如何合并的逻辑
```c++
void NdtMapMatrix::Reduce(NdtMapMatrix* cells, const NdtMapMatrix& cells_new) {
  for (unsigned int y = 0; y < cells->GetRows(); ++y) {
    for (unsigned int x = 0; x < cells->GetCols(); ++x) {
      NdtMapCells& cell = cells->GetMapCell(y, x);
      const NdtMapCells& cell_new = cells_new.GetMapCell(y, x);
      NdtMapCells::Reduce(&cell, cell_new);
    }
  }
}
```

最后合并2个NdtMapCells。
```c++
void NdtMapCells::Reduce(NdtMapCells* cell, const NdtMapCells& cell_new) {
  // Reduce cells
  for (auto it = cell_new.cells_.begin(); it != cell_new.cells_.end(); ++it) {
    int altitude_index = it->first;
    auto got = cell->cells_.find(altitude_index);
    if (got != cell->cells_.end()) {
      cell->cells_[altitude_index].MergeCell(it->second);
    } else {
      cell->cells_[altitude_index] = NdtMapSingleCell(it->second);
    }
  }

  if (cell_new.max_altitude_index_ > cell->max_altitude_index_) {
    cell->max_altitude_index_ = cell_new.max_altitude_index_;
  }

  if (cell_new.min_altitude_index_ < cell->min_altitude_index_) {
    cell->min_altitude_index_ = cell_new.min_altitude_index_;
  }

  for (auto it_new = cell_new.road_cell_indices_.begin();
       it_new != cell_new.road_cell_indices_.end(); ++it_new) {
    auto got_it = std::find(cell->road_cell_indices_.begin(),
                            cell->road_cell_indices_.end(), *it_new);
    if (got_it != cell->road_cell_indices_.end()) {
      *got_it += *it_new;
    } else {
      cell->road_cell_indices_.push_back(*it_new);
    }
  }
}
```


## maptool

异步保存地图
```c++
map_node_pool_->FreeMapNode(node_remove)
```


介绍完了制作地图的过程，下面开始介绍NDT定位的过程。

## NDT定位模块
NDTLocalizationComponent进行初始化，并且根据输入的gps消息结合点云信息输出定位和坐标translation关系。  

#### NDTLocalization
定位的具体功能是在NDTLocalization中实现的。NDT的匹配在`lidar_locator_`中，而融合的部分在
```c++
  if (!lidar_locator_.IsInitialized()) {
    lidar_locator_.Init(odometry_pose, resolution_id_, zone_id_);
    return;
  }
  lidar_locator_.Update(frame_idx++, odometry_pose, lidar_frame);
  lidar_pose_ = lidar_locator_.GetPose();
  pose_buffer_.UpdateLidarPose(time_stamp, lidar_pose_, odometry_pose);
  ComposeLidarResult(time_stamp, lidar_pose_, &lidar_localization_result_);
  ndt_score_ = lidar_locator_.GetFitnessScore();
```



#### LocalizationPoseBuffer
LocalizationPoseBuffer是一个buffer，大小默认为20，会存储历史的lidar姿态和IMU姿态，然后根据当前的IMU姿态推断最新的lidar姿态。  


## LidarLocatorNdt
NDT定位的主要实现在LidarLocatorNdt模块中。 根据输入的pose然后再进行ndt匹配，这样在IMU有偏差的情况下，可以有一定程度的校准，如果长时间GPS和IMU的定位不准确，那么当前的方案可能就不太可行。另外这里没有尝试纯NDT定位的方案，如果需要测试纯NDT定位的效果，也需要修改部分代码才能实现。  

主要的处理函数在`LidarLocatorNdt::Update`中实现。

```c++
int LidarLocatorNdt::Update(unsigned int frame_idx, const Eigen::Affine3d& pose,
                            const LidarFrame& lidar_frame) {
  // Increasement from INSPVA
  Eigen::Vector3d trans_diff =
      pose.translation() - pre_input_location_.translation();
  Eigen::Vector3d trans_pre_local =
      pre_estimate_location_.translation() + trans_diff;
  Eigen::Quaterniond quatd(pose.linear());
  Eigen::Translation3d transd(trans_pre_local);
  Eigen::Affine3d center_pose = transd * quatd;

  Eigen::Quaterniond pose_qbn(pose.linear());
  AINFO << "original pose: " << std::setprecision(15) << pose.translation()[0]
        << ", " << pose.translation()[1] << ", " << pose.translation()[2]
        << ", " << pose_qbn.x() << ", " << pose_qbn.y() << ", " << pose_qbn.z()
        << ", " << pose_qbn.w();

  // Get lidar pose Twv = Twb * Tbv
  Eigen::Affine3d transform = center_pose * velodyne_extrinsic_;
  predict_location_ = center_pose;

// Pre-load the map nodes
#ifdef USE_PRELOAD_MAP_NODE
  bool map_is_ready =
      map_.LoadMapArea(center_pose.translation(), resolution_id_, zone_id_,
                       filter_x_, filter_y_);
  map_.PreloadMapArea(center_pose.translation(), trans_diff, resolution_id_,
                      zone_id_);
#endif

  // Online pointcloud are projected into a ndt map node. (filtered)
  double lt_x = pose.translation()[0];
  double lt_y = pose.translation()[1];
  double map_resolution = map_.GetConfig().map_resolutions_[resolution_id_];
  lt_x -= (map_.GetConfig().map_node_size_x_ * map_resolution / 2.0);
  lt_y -= (map_.GetConfig().map_node_size_y_ * map_resolution / 2.0);

  // Start Ndt method
  // Convert online points to pcl pointcloud
  apollo::common::time::Timer online_filtered_timer;
  online_filtered_timer.Start();
  pcl::PointCloud<pcl::PointXYZ>::Ptr online_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < lidar_frame.pt_xs.size(); ++i) {
    pcl::PointXYZ p(lidar_frame.pt_xs[i], lidar_frame.pt_ys[i],
                    lidar_frame.pt_zs[i]);
    online_points->push_back(p);
  }

  // Filter online points
  AINFO << "Online point cloud leaf size: " << proj_reslution_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr online_points_filtered(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(online_points);
  sor.setLeafSize(proj_reslution_, proj_reslution_, proj_reslution_);
  sor.filter(*online_points_filtered);
  AINFO << "Online Pointcloud size: " << online_points->size() << "/"
        << online_points_filtered->size();
  online_filtered_timer.End("online point calc end.");

  //  Obtain map pointcloud
  apollo::common::time::Timer map_timer;
  map_timer.Start();
  Eigen::Vector2d left_top_coord2d(lt_x, lt_y);
  ComposeMapCells(left_top_coord2d, zone_id_, resolution_id_,
                  map_.GetConfig().map_resolutions_[resolution_id_],
                  transform.inverse());

  // Convert map pointcloud to local corrdinate
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < cell_map_.size(); ++i) {
    Leaf& le = cell_map_[i];
    float mean_0 = static_cast<float>(le.mean_(0));
    float mean_1 = static_cast<float>(le.mean_(1));
    float mean_2 = static_cast<float>(le.mean_(2));
    pcl_map_point_cloud->push_back(pcl::PointXYZ(mean_0, mean_1, mean_2));
  }
  map_timer.End("Map create end.");
  // Set left top corner for reg
  reg_.SetLeftTopCorner(map_left_top_corner_);
  // Ndt calculation
  reg_.SetInputTarget(cell_map_, pcl_map_point_cloud);
  reg_.SetInputSource(online_points_filtered);

  apollo::common::time::Timer ndt_timer;
  ndt_timer.Start();
  Eigen::Matrix3d inv_R = transform.inverse().linear();
  Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity();
  init_matrix.block<3, 3>(0, 0) = inv_R.inverse();

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  reg_.Align(output_cloud, init_matrix.cast<float>());
  ndt_timer.End("Ndt Align End.");

  fitness_score_ = reg_.GetFitnessScore();
  bool has_converged = reg_.HasConverged();
  int iteration = reg_.GetFinalNumIteration();
  Eigen::Matrix4d ndt_pose = reg_.GetFinalTransformation().cast<double>();
  AINFO << "Ndt summary:";
  AINFO << "Fitness Score: " << fitness_score_;
  AINFO << "Has_converged: " << has_converged;
  AINFO << "Iteration: %d: " << iteration;
  AINFO << "Relative Ndt pose: " << ndt_pose(0, 3) << ", " << ndt_pose(1, 3)
        << ", " << ndt_pose(2, 3);

  // Twv
  Eigen::Affine3d lidar_location = Eigen::Affine3d::Identity();
  lidar_location = transform.matrix() * init_matrix.inverse() * ndt_pose;

  // Save results
  location_ = lidar_location * velodyne_extrinsic_.inverse();
  pre_input_location_ = pose;
  pre_estimate_location_ = location_;
  pre_imu_height_ = location_.translation()(2);

  if (map_is_ready) {
    return 0;
  } else {
    return -1;
  }
  return 0;
}
```



#### ndt_voxel_grid_covariance
叶子节点中点的个数，以及平局值和协方差。
```c++
struct Leaf {
  Leaf()
      : nr_points_(0),
        mean_(Eigen::Vector3d::Zero()),
        icov_(Eigen::Matrix3d::Zero()) {}

  /**@brief Get the number of points contained by this voxel. */
  int GetPointCount() const { return nr_points_; }

  /**@brief Get the voxel centroid. */
  Eigen::Vector3d GetMean() const { return mean_; }

  /**@brief Get the inverse of the voxel covariance. */
  Eigen::Matrix3d GetInverseCov() const { return icov_; }

  /**@brief Number of points contained by voxel. */
  int nr_points_;
  /**@brief 3D voxel centroid. */
  Eigen::Vector3d mean_;
  /**@brief Inverse of voxel covariance matrix. */
  Eigen::Matrix3d icov_;
};
```

#### VoxelGridCovariance
Voxel的协方差计算，VoxelGridCovariance包含了一系列的叶子节点(Leaf)。  

初始化voxel结构体，一是SetMap，二是构建kdtree_
```c++
  /**@brief Initializes voxel structure. */
  inline void filter(const std::vector<Leaf> &cell_leaf,
                     bool searchable = true) {
    voxel_centroids_ = PointCloudPtr(new PointCloud);
    SetMap(cell_leaf, voxel_centroids_);
    if (voxel_centroids_->size() > 0) {
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }
```

把map_leaves中的值赋值给leaves_，并且把index保存到voxel_centroids_leaf_indices_中
```c++
template <typename PointT>
void VoxelGridCovariance<PointT>::SetMap(const std::vector<Leaf>& map_leaves,
                                         PointCloudPtr output) {
  voxel_centroids_leaf_indices_.clear();

  // input_输入点云不为空

  // Copy the header + allocate enough space for points
  output->height = 1;
  output->is_dense = true;
  output->points.clear();

  // 获取最大和最小的点云坐标
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D<PointT>(*input_, min_p, max_p);

  Eigen::Vector4f left_top = Eigen::Vector4f::Zero();
  left_top.block<3, 1>(0, 0) = map_left_top_corner_.cast<float>();
  min_p -= left_top;
  max_p -= left_top;

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int>(min_p[0] * inverse_leaf_size_[0]);
  max_b_[0] = static_cast<int>(max_p[0] * inverse_leaf_size_[0]);
  min_b_[1] = static_cast<int>(min_p[1] * inverse_leaf_size_[1]);
  max_b_[1] = static_cast<int>(max_p[1] * inverse_leaf_size_[1]);
  min_b_[2] = static_cast<int>(min_p[2] * inverse_leaf_size_[2]);
  max_b_[2] = static_cast<int>(max_p[2] * inverse_leaf_size_[2]);

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
  div_b_[3] = 0;

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

  // Clear the leaves
  leaves_.clear();

  output->points.reserve(map_leaves.size());
  voxel_centroids_leaf_indices_.reserve(leaves_.size());

  for (unsigned int i = 0; i < map_leaves.size(); ++i) {
    const Leaf& cell_leaf = map_leaves[i];
    Eigen::Vector3d local_mean = cell_leaf.mean_ - map_left_top_corner_;
    int ijk0 =
        static_cast<int>(local_mean(0) * inverse_leaf_size_[0]) - min_b_[0];
    int ijk1 =
        static_cast<int>(local_mean(1) * inverse_leaf_size_[1]) - min_b_[1];
    int ijk2 =
        static_cast<int>(local_mean(2) * inverse_leaf_size_[2]) - min_b_[2];

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    Leaf& leaf = leaves_[idx];
    leaf = cell_leaf;

    if (cell_leaf.nr_points_ >= min_points_per_voxel_) {
      output->push_back(PointT());
      output->points.back().x = static_cast<float>(leaf.mean_[0]);
      output->points.back().y = static_cast<float>(leaf.mean_[1]);
      output->points.back().z = static_cast<float>(leaf.mean_[2]);
      voxel_centroids_leaf_indices_.push_back(idx);
    }
  }
  output->width = static_cast<uint32_t>(output->points.size());
}
```

这里为什么可以根据index找到k_leaves？
```c++
template <typename PointT>
int VoxelGridCovariance<PointT>::RadiusSearch(
    const PointT& point, double radius, std::vector<LeafConstPtr>* k_leaves,
    std::vector<float>* k_sqr_distances, unsigned int max_nn) {
  k_leaves->clear();

  // Find neighbors within radius in the occupied voxel centroid cloud
  std::vector<int> k_indices;
  int k =
      kdtree_.radiusSearch(point, radius, k_indices, *k_sqr_distances, max_nn);

  // Find leaves corresponding to neighbors
  k_leaves->reserve(k);
  for (std::vector<int>::iterator iter = k_indices.begin();
       iter != k_indices.end(); iter++) {
    k_leaves->push_back(&leaves_[voxel_centroids_leaf_indices_[*iter]]);
  }
  return k;
}
```

通过Leaf的质心随机生成100个点进行点云的显示。
```
template <typename PointT>
void VoxelGridCovariance<PointT>::GetDisplayCloud(
    pcl::PointCloud<pcl::PointXYZ>* cell_cloud)
```


#### NormalDistributionsTransform
设置目标点云
```c++
  inline void SetInputTarget(const std::vector<Leaf> &cell_leaf,
                             const PointCloudTargetConstPtr &cloud) {
    if (cell_leaf.empty()) {
      AWARN << "Input leaf is empty.";
      return;
    }
    if (cloud->points.empty()) {
      AWARN << "Input target is empty.";
      return;
    }

    // 设置目标点云，其中target_cells_为VoxelGridCovariance
    target_ = cloud;
    target_cells_.SetVoxelGridResolution(resolution_, resolution_, resolution_);
    target_cells_.SetInputCloud(cloud);
    target_cells_.filter(cell_leaf, true);
  }
```

resolution_代表体素网格分辨率

step_size_牛顿线性查找最大步长

trans_probability_ 注册概率

nr_iterations_ 迭代次数

final_transformation_ 最后的转移矩阵


```c++
template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<PointSource, PointTarget>::
    ComputeTransformation(PointCloudSourcePtr output,
                          const Eigen::Matrix4f &guess) {
  apollo::common::time::Timer timer;
  timer.Start();

  nr_iterations_ = 0;
  converged_ = false;
  double gauss_c1, gauss_c2, gauss_d3;

  // Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  gauss_c1 = 10 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  gauss_d3 = -log(gauss_c2);
  gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3;
  gauss_d2_ =
      -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity()) {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud(*output, *output, guess);
  }

  // Initialize Point Gradient and Hessian
  point_gradient_.setZero();
  point_gradient_.block<3, 3>(0, 0).setIdentity();
  point_hessian_.setZero();

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix() = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation();
  Eigen::Vector3f init_rotation =
      eig_transformation.rotation().eulerAngles(0, 1, 2);
  p << init_translation(0), init_translation(1), init_translation(2),
      init_rotation(0), init_rotation(1), init_rotation(2);

  Eigen::Matrix<double, 6, 6> hessian;
  double score = 0;
  double delta_p_norm;

  // Calculate derivates of initial transform vector, subsequent derivative
  // calculations are done in the step length determination.
  score = ComputeDerivatives(&score_gradient, &hessian, output, &p);
  timer.End("Ndt ComputeDerivatives");

  apollo::common::time::Timer loop_timer;
  loop_timer.Start();
  while (!converged_) {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for decent direction using newton method, line 23 in Algorithm
    // 2 [Magnusson 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(
        hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    delta_p = sv.solve(-score_gradient);

    // Calculate step length with guarnteed sufficient decrease [More,
    // Thuente 1994]
    delta_p_norm = delta_p.norm();

    if (delta_p_norm == 0 || delta_p_norm != delta_p_norm) {
      trans_probability_ = score / static_cast<double>(input_->points.size());
      converged_ = delta_p_norm == delta_p_norm;
      return;
    }

    delta_p.normalize();
    delta_p_norm = ComputeStepLengthMt(p, &delta_p, delta_p_norm, step_size_,
                                       transformation_epsilon_ / 2, &score,
                                       &score_gradient, &hessian, output);
    delta_p *= delta_p_norm;

    transformation_ =
        (Eigen::Translation<float, 3>(static_cast<float>(delta_p(0)),
                                      static_cast<float>(delta_p(1)),
                                      static_cast<float>(delta_p(2))) *
         Eigen::AngleAxis<float>(static_cast<float>(delta_p(3)),
                                 Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxis<float>(static_cast<float>(delta_p(4)),
                                 Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxis<float>(static_cast<float>(delta_p(5)),
                                 Eigen::Vector3f::UnitZ()))
            .matrix();

    p = p + delta_p;

    if (nr_iterations_ > max_iterations_ ||
        (nr_iterations_ &&
         (std::fabs(delta_p_norm) < transformation_epsilon_))) {
      converged_ = true;
    }

    nr_iterations_++;
  }
  loop_timer.End("Ndt loop.");

  // Store transformation probability.  The relative differences within each
  // scan registration are accurate but the normalization constants need to be
  // modified for it to be globally accurate
  trans_probability_ = score / static_cast<double>(input_->points.size());
}
```

其中的公式推导需要接着分析`ndt_solver.hpp`。  


## ndt_map
ndt_map为ndt地图的主要说明目录，


#### BaseMapNodePool
BaseMapNodePool是一个map node对象池，其中is_fixed_size_表示大小固定，不会新增加大小。free_list_中存放还没有使用的节点，busy_nodes_中存放正在使用的节点，当需要释放的时候可以用ResetMapNode来进行资源的释放。

其中比较有意思的点在于，BaseMapNodePool在析构函数中进行资源释放，同时结合`ndt_map_node->SetIsChanged(true)`来保存node节点到硬盘，从而实现地图的制作过程。  



## poses_interpolation.cc
对pose进行插值，主要的处理在函数中
```c++
void PosesInterpolation::DoInterpolation() {
  // 读取pose信息
  std::vector<Eigen::Vector3d> input_stds;
  velodyne::LoadPosesAndStds(input_poses_path_, &input_poses_, &input_stds,
                             &input_poses_timestamps_);

  // 读取pcd的index和时间戳
  LoadPCDTimestamp();

  // Interpolation
  PoseInterpolationByTime(input_poses_, input_poses_timestamps_,
                          ref_timestamps_, ref_ids_, &out_indexes_,
                          &out_timestamps_, &out_poses_);

  // 保存pcd文件
  WritePCDPoses();
}
```

插值的主要逻辑在如下函数中，主要是根据lidar的时间戳，查找位姿，并且做插值，最后保存到文件。  
```c++
in_poses  // 定位的pose列表
in_timestamps // 定位的时间戳
ref_timestamps  // 点云的时间戳
ref_indexes   // 点云的index
out_indexes  // 输出的index
out_timestamps  // 输出的时间戳
out_poses   // 输出的姿态

void PosesInterpolation::PoseInterpolationByTime(
    const std::vector<Eigen::Affine3d> &in_poses,
    const std::vector<double> &in_timestamps,
    const std::vector<double> &ref_timestamps,
    const std::vector<unsigned int> &ref_indexes,
    std::vector<unsigned int> *out_indexes, std::vector<double> *out_timestamps,
    std::vector<Eigen::Affine3d> *out_poses) {
  out_indexes->clear();
  out_timestamps->clear();
  out_poses->clear();

  unsigned int index = 0;
  for (size_t i = 0; i < ref_timestamps.size(); i++) {
    double ref_timestamp = ref_timestamps[i];
    unsigned int ref_index = ref_indexes[i];

    while (index < in_timestamps.size() &&
           in_timestamps.at(index) < ref_timestamp) {
      ++index;
    }

    if (index < in_timestamps.size()) {
      if (index >= 1) {
        double cur_timestamp = in_timestamps[index];
        double pre_timestamp = in_timestamps[index - 1];
        assert(cur_timestamp != pre_timestamp);

        double t =
            (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
        assert(t >= 0.0);
        assert(t <= 1.0);

        Eigen::Affine3d pre_pose = in_poses[index - 1];
        Eigen::Affine3d cur_pose = in_poses[index];
        Eigen::Quaterniond pre_quatd(pre_pose.linear());
        Eigen::Translation3d pre_transd(pre_pose.translation());
        Eigen::Quaterniond cur_quatd(cur_pose.linear());
        Eigen::Translation3d cur_transd(cur_pose.translation());

        Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

        Eigen::Translation3d re_transd;
        re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
        re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
        re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);

        out_poses->push_back(re_transd * res_quatd);
        out_indexes->push_back(ref_index);
        out_timestamps->push_back(ref_timestamp);
      }
    } else {
      AWARN << "[WARN] No more poses. Exit now.";
      break;
    }
    ADEBUG << "Frame_id: " << i;
  }
}
```


