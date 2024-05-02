# dead_end
断头路掉头

# 流程

# DeadEndTurnAroundScenario

```
stage_type: DEADEND_TURNAROUND_APPROACHING_TURNING_POINT
stage_type: DEADEND_TURNAROUND_TURNING

stage_config: {
  stage_type: DEADEND_TURNAROUND_APPROACHING_TURNING_POINT
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
  stage_type: DEADEND_TURNAROUND_TURNING
  enabled: true
  task_type: OPEN_SPACE_ROI_DECIDER
  task_type: OPEN_SPACE_TRAJECTORY_PROVIDER
  task_type: OPEN_SPACE_TRAJECTORY_PARTITION
  task_type: OPEN_SPACE_FALLBACK_DECIDER
}
```

## IsTransferable


# StageApproachingTurningPoint

## Process
```c++
Stage::StageStatus StageApproachingTurningPoint::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  GetContext()->dead_end_id.clear();
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    return StageStatus::ERROR;
  }
  // stage change
  if (CheckADCStop(*frame)) {
    next_stage_ = ScenarioConfig::DEADEND_TURNAROUND_TURNING;
    return Stage::FINISHED;
  }

  return Stage::RUNNING;
}
```
# StageTurning

## Process
```c++
Stage::StageStatus StageTurning::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "StageTurning planning error";
    return StageStatus::ERROR;
  }
  return StageStatus::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus StageTurning::FinishStage() { return Stage::FINISHED; }
```
