# Dig into Apollo - Reference line ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 尽吾志也而不能至者，可以无悔矣


## Table of Contents
- [介绍](#introduction)
- [参考线提供者](#rf_provider)
- [参考线](#rf_line)
- [平滑器](#rf_smoother)  


<a name="introduction" />

## 参考线介绍(Reference line)  

参考线是根据routing规划的路线，生成一系列参考轨迹，提供给规划算法做为参考，从而生成最终的规划轨迹。
为什么要提供参考呢？因为道路是结构化道路，在没有参考的情况下，需要通过搜索算法来查找路线，这种场景在机器人路径规划中比较普遍，机器人在一个开放空间只要没有障碍物它就可以行走，而车不一样，车是在道路上行驶的，在提供参考的情况下，节省了查找的时间和复杂度，降低了算法的难度，这也就是参考线的意义。  

1. ReferenceLine和ReferenceLineInfo的关系
ReferenceLine提供的是轨迹信息，而ReferenceLineInfo在ReferenceLine的基础上新添加了决策信息。  


<a name="rf_provider" />

## ReferenceLineProvider  

单独的线程并发执行ReferenceLineProvider。  
[reference_line](../img/reference_line.jpg)

1. 整个流程的过程是怎样的？
2. 如何生成的参考线，输入是什么？输出是什么？
3. 参考线用图来形象的表示？


我们先从ReferenceLine本身开始分析，先搞清楚参考线的**数据结构**。  

## ReferenceLine
参考线的数据结构非常简单，由速度限制数组、参考点数组、路径和优先级4部分组成。`map_path_`中的点和`reference_points_`中的点个数相同。
```c++
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  hdmap::Path map_path_;
  uint32_t priority_ = 0;
```

接下来我们分析下参考点的结构
#### ReferencePoint
参考点的数据结构是路径和路径的曲率（kappa_）以及dkappa_

#### 
根据s，返回初始化的`PathPoint`。
```c++
common::PathPoint ReferencePoint::ToPathPoint(double s) const {
  return common::util::PointFactory::ToPathPoint(x(), y(), 0.0, s, heading(),
                                                 kappa_, dkappa_);
}
```

#### RemoveDuplicates
移除重复的点，如果重复则把点添加到`lane_waypoints_`，(todo)这里lane_waypoints_有什么作用，为什么不采用单独的点？  
```c++
void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}
```


#### 
知道了数据结构接下来我们分析下其中的方法。



#### GetReferenceLines
获取参考线
```c++
bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
	...
  // 1. 如果有单独的线程，则直接赋值 
  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    if (!reference_lines_.empty()) {
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
      segments->assign(route_segments_.begin(), route_segments_.end());
      return true;
    }
  } else {
    double start_time = Clock::NowInSeconds();
    // 2. 否则，创建并且更新参考线
    if (CreateReferenceLine(reference_lines, segments)) {
      UpdateReferenceLine(*reference_lines, *segments);
      double end_time = Clock::NowInSeconds();
      last_calculation_time_ = end_time - start_time;
      return true;
    }
  }

  AWARN << "Reference line is NOT ready.";
  if (reference_line_history_.empty()) {
    AERROR << "Failed to use reference line latest history";
    return false;
  }
  // 3. 如果失败，则采用历史规划轨迹
  reference_lines->assign(reference_line_history_.back().begin(),
                          reference_line_history_.back().end());
  segments->assign(route_segments_history_.back().begin(),
                   route_segments_history_.back().end());
  AWARN << "Use reference line from history!";
  return true;
}
```

#### CreateReferenceLine
创建参考线
```c++
bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {

  // 1. 获取车辆状态
  common::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }
  // 2. 获取routing
  routing::RoutingResponse routing;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing = routing_;
  }
  bool is_new_routing = false;
  {
    // 2.1 如果是新routing，那么更新routing
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (pnc_map_->IsNewRouting(routing)) {
      is_new_routing = true;
      if (!pnc_map_->UpdateRoutingResponse(routing)) {
        AERROR << "Failed to update routing in pnc map";
        return false;
      }
    }
  }

  // 3. 创建routing segment
  if (!CreateRouteSegments(vehicle_state, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }
  if (is_new_routing || !FLAGS_enable_reference_line_stitching) {
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      // 4.1.1 平滑routing segment
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        common::SLPoint sl;
        if (!reference_lines->back().XYToSL(vehicle_state, &sl)) {
          AWARN << "Failed to project point: {" << vehicle_state.x() << ","
                << vehicle_state.y() << "} to stitched reference line";
        }
        // 4.1.2 收缩参考线
        Shrink(sl, &reference_lines->back(), &(*iter));
        ++iter;
      }
    }
    return true;
  } else {  // stitching reference line
  	// 4.2 根据routing segment缝合参考线
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
  }
  return true;
}
```

#### UpdatedReferenceLine
只是更新标签
```c++
  bool UpdatedReferenceLine() { return is_reference_line_updated_.load(); }
```



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

<a name="rf_line" />

## ReferenceLineInfo
参考线信息，在参考线的基础添加了决策信息，ST图等。


<a name="rf_smoother" />

## 平滑器
