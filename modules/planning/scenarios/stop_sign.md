# stop_sign
停车汇入场景

# 流程


# StopSignUnprotectedScenario

```
stage_type: STOP_SIGN_UNPROTECTED_PRE_STOP
stage_type: STOP_SIGN_UNPROTECTED_STOP
stage_type: STOP_SIGN_UNPROTECTED_CREEP
stage_type: STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE

stage_config: {
  stage_type: STOP_SIGN_UNPROTECTED_PRE_STOP
  enabled: true
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
}

stage_config: {
  stage_type: STOP_SIGN_UNPROTECTED_STOP
  enabled: true
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
}

stage_config: {
  stage_type: STOP_SIGN_UNPROTECTED_CREEP
  enabled: true
  task_type: CREEP_DECIDER
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
}

stage_config: {
  stage_type: STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE
  enabled: true
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
}
```

# StopSignUnprotectedStagePreStop

## Process
```c++
Stage::StageStatus StopSignUnprotectedStagePreStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - current_stop_sign_overlap->start_s;
  if (distance_adc_pass_stop_sign <= kPassStopLineBuffer) {
    // not passed stop line, check valid stop
    if (CheckADCStop(adc_front_edge_s, current_stop_sign_overlap->start_s)) {
      return FinishStage();
    }
  } else {
    // passed stop line
    return FinishStage();
  }

  // PRE-STOP
  const PathDecision& path_decision = reference_line_info.path_decision();
  auto& watch_vehicles = GetContext()->watch_vehicles;

  std::vector<std::string> watch_vehicle_ids;
  for (const auto& vehicle : watch_vehicles) {
    std::copy(vehicle.second.begin(), vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));

    // for debug string
    std::string associated_lane_id = vehicle.first;
    std::string s;
    for (const std::string& vehicle_id : vehicle.second) {
      s = s.empty() ? vehicle_id : s + "," + vehicle_id;
    }
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  for (const auto* obstacle : path_decision.obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*obstacle, &watch_vehicles);
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus StopSignUnprotectedStagePreStop::FinishStage() {
  GetContext()->stop_start_time = Clock::NowInSeconds();
  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP;

  return Stage::FINISHED;
}
```

# StopSignUnprotectedStageStop

## Process
```c++
Stage::StageStatus StopSignUnprotectedStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // refresh overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  static constexpr double kPassStopLineBuffer = 1.0;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - stop_sign_start_s;
  // passed stop line too far
  if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
    return FinishStage();
  }

  // check on wait-time
  auto start_time = GetContext()->stop_start_time;
  const double wait_time = Clock::NowInSeconds() - start_time;
  ADEBUG << "stop_start_time[" << start_time << "] wait_time[" << wait_time
         << "]";
  if (wait_time < scenario_config_.stop_duration_sec()) {
    return Stage::RUNNING;
  }

  // check on watch_vehicles
  auto& watch_vehicles = GetContext()->watch_vehicles;
  if (watch_vehicles.empty()) {
    return FinishStage();
  }

  // get all vehicles currently watched
  std::vector<std::string> watch_vehicle_ids;
  for (const auto& watch_vehicle : watch_vehicles) {
    std::copy(watch_vehicle.second.begin(), watch_vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));
    // for debug
    std::string s;
    for (const std::string& vehicle : watch_vehicle.second) {
      s = s.empty() ? vehicle : s + "," + vehicle;
    }
    const std::string& associated_lane_id = watch_vehicle.first;
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // remove duplicates (caused when same vehicle on mutiple lanes)
  watch_vehicle_ids.erase(
      unique(watch_vehicle_ids.begin(), watch_vehicle_ids.end()),
      watch_vehicle_ids.end());

  if (watch_vehicle_ids.empty()) {
    return FinishStage();
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  // check timeout while waiting for only one vehicle
  if (wait_time > scenario_config_.stop_timeout_sec() &&
      watch_vehicle_ids.size() <= 1) {
    return FinishStage();
  }

  const PathDecision& path_decision = reference_line_info.path_decision();
  RemoveWatchVehicle(path_decision, &watch_vehicles);

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus StopSignUnprotectedStageStop::FinishStage() {
  // update PlanningContext
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->set_done_stop_sign_overlap_id(
          GetContext()->current_stop_sign_overlap_id);
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->clear_wait_for_obstacle_id();

  GetContext()->creep_start_time = Clock::NowInSeconds();

  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_CREEP;
  return Stage::FINISHED;
}
```

# StopSignUnprotectedStageCreep

## Process
```c++
Stage::StageStatus StopSignUnprotectedStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageCreep planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  const double stop_sign_end_s = current_stop_sign_overlap->end_s;
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));

  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s =
      stop_sign_end_s + task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info, stop_sign_end_s,
                           wait_time, timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus StopSignUnprotectedStageCreep::FinishStage() {
  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}
```

# StopSignUnprotectedStageIntersectionCruise

## Process
```c++
Stage::StageStatus StopSignUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  bool stage_done =
      stage_impl_.CheckDone(*frame, ScenarioConfig::STOP_SIGN_UNPROTECTED,
                            config_, injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus StopSignUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}
```
