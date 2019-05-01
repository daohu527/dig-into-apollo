## sim_control

初始化，是否设置开始节点？
```
void SimControl::Init(bool set_start_point, double start_velocity,
                      double start_acceleration) {
  // 1. 设置开始点
  // 2. 没有使用导航模式
  if (set_start_point && !FLAGS_use_navigation_mode) {
    InitStartPoint(start_velocity, start_acceleration);
  }
}
```

下面在进一步看如何初始化起点：  
```
void SimControl::InitStartPoint(double start_velocity,
                                double start_acceleration) {
  TrajectoryPoint point;
  // Use the latest localization position as start point,
  // fall back to a dummy point from map
  
  // 什么作用？
  localization_reader_->Observe();
  // 如果读取不到Localization TOPIC
  if (localization_reader_->Empty()) {
    start_point_from_localization_ = false;
    apollo::common::PointENU start_point;
    // 从地图读取起点
    if (!map_service_->GetStartPoint(&start_point)) {
      AWARN << "Failed to get a dummy start point from map!";
      return;
    }
    point.mutable_path_point()->set_x(start_point.x());
    point.mutable_path_point()->set_y(start_point.y());
    point.mutable_path_point()->set_z(start_point.z());
    double theta = 0.0;
    double s = 0.0;
    // 从地图获取
    map_service_->GetPoseWithRegardToLane(start_point.x(), start_point.y(),
                                          &theta, &s);
    point.mutable_path_point()->set_theta(theta);
    point.set_v(start_velocity);
    point.set_a(start_acceleration);
  } else {
    start_point_from_localization_ = true;
    // 从Localization读取新的位置
    const auto& localization = localization_reader_->GetLatestObserved();
    const auto& pose = localization->pose();
    point.mutable_path_point()->set_x(pose.position().x());
    point.mutable_path_point()->set_y(pose.position().y());
    point.mutable_path_point()->set_z(pose.position().z());
    point.mutable_path_point()->set_theta(pose.heading());
    point.set_v(
        std::hypot(pose.linear_velocity().x(), pose.linear_velocity().y()));
    // Calculates the dot product of acceleration and velocity. The sign
    // of this projection indicates whether this is acceleration or
    // deceleration.
    double projection =
        pose.linear_acceleration().x() * pose.linear_velocity().x() +
        pose.linear_acceleration().y() * pose.linear_velocity().y();

    // Calculates the magnitude of the acceleration. Negate the value if
    // it is indeed a deceleration.
    double magnitude = std::hypot(pose.linear_acceleration().x(),
                                  pose.linear_acceleration().y());
    point.set_a(std::signbit(projection) ? -magnitude : magnitude);
  }
  SetStartPoint(point);
}
```
