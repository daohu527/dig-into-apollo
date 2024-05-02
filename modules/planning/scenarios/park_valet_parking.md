# park_valet_parking

# 流程

# ValetParkingScenario

```
stage_type: VALET_PARKING_APPROACHING_PARKING_SPOT
stage_type: VALET_PARKING_PARKING

stage_config: {
  stage_type: VALET_PARKING_APPROACHING_PARKING_SPOT
  enabled: true
  task_type: OPEN_SPACE_PRE_STOP_DECIDER
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
}

stage_config: {
  stage_type: VALET_PARKING_PARKING
  enabled: true
  task_type: OPEN_SPACE_ROI_DECIDER
  task_type: OPEN_SPACE_TRAJECTORY_PROVIDER
  task_type: OPEN_SPACE_TRAJECTORY_PARTITION
  task_type: OPEN_SPACE_FALLBACK_DECIDER
}
```

## IsTransferable


# StageApproachingParkingSpot

## Process
```c++
Stage::StageStatus StageApproachingParkingSpot::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  GetContext()->target_parking_spot_id.clear();
  if (frame->local_view().routing->routing_request().has_parking_info() &&
      frame->local_view()
          .routing->routing_request()
          .parking_info()
          .has_parking_space_id()) {
    GetContext()->target_parking_spot_id = frame->local_view()
                                               .routing->routing_request()
                                               .parking_info()
                                               .parking_space_id();
  } else {
    AERROR << "No parking space id from routing";
    return StageStatus::ERROR;
  }

  if (GetContext()->target_parking_spot_id.empty()) {
    return StageStatus::ERROR;
  }

  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      GetContext()->target_parking_spot_id;
  frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(
      GetContext()->pre_stop_rightaway_flag);
  *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
      GetContext()->pre_stop_rightaway_point;

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    return StageStatus::ERROR;
  }

  GetContext()->pre_stop_rightaway_flag =
      frame->open_space_info().pre_stop_rightaway_flag();
  GetContext()->pre_stop_rightaway_point =
      frame->open_space_info().pre_stop_rightaway_point();

  if (CheckADCStop(*frame)) {
    next_stage_ = ScenarioConfig::VALET_PARKING_PARKING;
    return Stage::FINISHED;
  }

  return Stage::RUNNING;
}
```

# StageParking
## Process
```c++
Stage::StageStatus StageParking::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "StageParking planning error";
    return StageStatus::ERROR;
  }
  return StageStatus::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus StageParking::FinishStage() { return Stage::FINISHED; }
```
