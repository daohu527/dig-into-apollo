# park_and_go

# 流程

# ParkAndGoScenario
```
stage_type: PARK_AND_GO_CHECK
stage_type: PARK_AND_GO_ADJUST
stage_type: PARK_AND_GO_PRE_CRUISE
stage_type: PARK_AND_GO_CRUISE

stage_config: {
  stage_type: PARK_AND_GO_CHECK
  enabled: true
  task_type: OPEN_SPACE_ROI_DECIDER
  task_type: OPEN_SPACE_TRAJECTORY_PROVIDER
  task_type: OPEN_SPACE_TRAJECTORY_PARTITION
  task_type: OPEN_SPACE_FALLBACK_DECIDER
}

stage_config:{
  stage_type: PARK_AND_GO_ADJUST
  enabled: true
  task_type: OPEN_SPACE_ROI_DECIDER
  task_type: OPEN_SPACE_TRAJECTORY_PROVIDER
  task_type: OPEN_SPACE_TRAJECTORY_PARTITION
  task_type: OPEN_SPACE_FALLBACK_DECIDER
}

stage_config:{
  stage_type: PARK_AND_GO_PRE_CRUISE
  enabled: true
  task_type: OPEN_SPACE_ROI_DECIDER
  task_type: OPEN_SPACE_TRAJECTORY_PROVIDER
  task_type: OPEN_SPACE_TRAJECTORY_PARTITION
  task_type: OPEN_SPACE_FALLBACK_DECIDER
}

stage_config:{
  stage_type: PARK_AND_GO_CRUISE
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
  task_type: PIECEWISE_JERK_SPEED_OPTIMIZER
  task_type: RSS_DECIDER
}
```

# ParkAndGoStageCheck

## Process
```c++
Stage::StageStatus ParkAndGoStageCheck::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);
  ADCInitStatus();
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageAdjust planning error";
    return StageStatus::ERROR;
  }

  bool ready_to_cruise = scenario::util::CheckADCReadyToCruise(
      injector_->vehicle_state(), frame, scenario_config_);
  return FinishStage(ready_to_cruise);
}
```

## FinishStage
```c++
Stage::StageStatus ParkAndGoStageCheck::FinishStage(const bool success) {
  if (success) {
    next_stage_ = ScenarioConfig::PARK_AND_GO_CRUISE;
  } else {
    next_stage_ = ScenarioConfig::PARK_AND_GO_ADJUST;
  }
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_park_and_go()
      ->set_in_check_stage(false);
  return Stage::FINISHED;
}
```

# ParkAndGoStageAdjust

## Process
```c++
Stage::StageStatus ParkAndGoStageAdjust::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageAdjust planning error";
    return StageStatus::ERROR;
  }
  const bool is_ready_to_cruise = scenario::util::CheckADCReadyToCruise(
      injector_->vehicle_state(), frame, scenario_config_);

  bool is_end_of_trajectory = false;
  const auto& history_frame = injector_->frame_history()->Latest();
  if (history_frame) {
    const auto& trajectory_points =
        history_frame->current_frame_planned_trajectory().trajectory_point();
    if (!trajectory_points.empty()) {
      is_end_of_trajectory =
          (trajectory_points.rbegin()->relative_time() < 0.0);
    }
  }

  if (!is_ready_to_cruise && !is_end_of_trajectory) {
    return StageStatus::RUNNING;
  }
  return FinishStage();
}
```

## FinishStage
```c++
Stage::StageStatus ParkAndGoStageAdjust::FinishStage() {
  const auto vehicle_status = injector_->vehicle_state();
  ADEBUG << vehicle_status->steering_percentage();
  if (std::fabs(vehicle_status->steering_percentage()) <
      scenario_config_.max_steering_percentage_when_cruise()) {
    next_stage_ = ScenarioConfig::PARK_AND_GO_CRUISE;
  } else {
    ResetInitPostion();
    next_stage_ = ScenarioConfig::PARK_AND_GO_PRE_CRUISE;
  }
  return Stage::FINISHED;
}
```

# ParkAndGoStagePreCruise

## Process
```c++
Stage::StageStatus ParkAndGoStagePreCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStagePreCruise planning error";
    return StageStatus::ERROR;
  }
  // const bool ready_to_cruise =
  //     scenario::util::CheckADCReadyToCruise(frame, scenario_config_);
  auto vehicle_status = injector_->vehicle_state();
  ADEBUG << vehicle_status->steering_percentage();

  if ((std::fabs(vehicle_status->steering_percentage()) <
       scenario_config_.max_steering_percentage_when_cruise()) &&
      scenario::util::CheckADCReadyToCruise(injector_->vehicle_state(), frame,
                                            scenario_config_)) {
    return FinishStage();
  }
  return StageStatus::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus ParkAndGoStagePreCruise::FinishStage() {
  next_stage_ = ScenarioConfig::PARK_AND_GO_CRUISE;
  return Stage::FINISHED;
}
```

# ParkAndGoStageCruise

## Process
```c++
Stage::StageStatus ParkAndGoStageCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageCruise planning error";
  }

  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  // check ADC status:
  // 1. At routing beginning: stage finished
  ParkAndGoStatus status =
      CheckADCParkAndGoCruiseCompleted(reference_line_info);

  if (status == CRUISE_COMPLETE) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus ParkAndGoStageCruise::FinishStage() {
  return FinishScenario();
}
```