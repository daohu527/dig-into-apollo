## radar

## lib目录
先预处理，再识别，然后过滤，最后追踪？？？

#### preprocessor 预处理
预处理主要是对时间戳做预处理。  


#### detector 识别
这里不是获取的raw_data而是直接获取的radar输出的感知结果。  


#### roi_filter  过滤
根据地图过滤感兴趣的区域。
**疑问**  
1. 感兴趣区域是如何组成的，为何定义的是点云的格式？？？  


## tracker 追踪

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



