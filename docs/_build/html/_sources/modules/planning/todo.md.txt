## Frame
Frame代表了规划模块中的一帧，包括了多种信息。

#### FindDriveReferenceLineInfo
找到代价最小并且可以行驶的参考线，然后返回结果。
```c++
const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  // 遍历找到最小代价，并且可以行驶的reference line
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}
```

#### FindTargetReferenceLineInfo
返回第一个变道类型的参考线，如果没有找到，则返回最后一个参考线。
```c++
const ReferenceLineInfo *Frame::FindTargetReferenceLineInfo() {
  const ReferenceLineInfo *target_reference_line_info = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
    target_reference_line_info = &reference_line_info;
  }
  return target_reference_line_info;
}
```

#### FindFailedReferenceLineInfo
找到是变道类型并且不能行驶的参考线，如果没有找到则返回空。
```c++
const ReferenceLineInfo *Frame::FindFailedReferenceLineInfo() {
  for (const auto &reference_line_info : reference_line_info_) {
    // Find the unsuccessful lane-change path
    if (!reference_line_info.IsDrivable() &&
        reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
  }
  return nullptr;
}
```

#### DriveReferenceLineInfo
返回FindDriveReferenceLineInfo中找到的参考线。即代价最小并且可以行驶的参考线。
```c++
const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}
```

#### UpdateReferenceLinePriority
更新参考线的优先级，其中key为lane的id，value为优先级。(todo)这里的lanes是`hdmap::RouteSegments`类型，为什么id只有一个？
```c++
void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {
  for (const auto &pair : id_to_priority) {
    const auto id = pair.first;
    const auto priority = pair.second;
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority);
    }
  }
}
```

#### CreateReferenceLineInfo
根据reference_lines创建reference_line_info_（也是一个数组），并且添加障碍物到数组。
```c++
bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  // 1. reference_line_info_添加成员，(todo)这里ref_line_iter和segments_iter是一一对应的吗？
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }

  // 2. 如果reference_line_info_大小为2
  if (reference_line_info_.size() == 2) {
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
    common::SLPoint first_sl;
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    common::SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    // 2.1 根据车当前的位置求出到起点的距离
    const double offset = first_sl.l() - second_sl.l();
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  // 3. 初始化障碍物，如果有一个成功则表示有合理的参考线
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
      AERROR << "Failed to init reference line";
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}
```

