# park_pull_over

# 流程

# PullOverScenario

```
stage_type: PULL_OVER_APPROACH
PULL_OVER_RETRY_APPROACH_PARKING
stage_type: PULL_OVER_RETRY_PARKING

stage_config: {
  stage_type: PULL_OVER_APPROACH
  enabled: true
  # task_type: PATH_LANE_BORROW_DECIDER
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

stage_config: {
  stage_type: PULL_OVER_RETRY_APPROACH_PARKING
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
```

# PullOverStageApproach

## Process
```c++
Stage::StageStatus PullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "PullOverStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  scenario::util::PullOverStatus status = scenario::util::CheckADCPullOver(
      injector_->vehicle_state(), reference_line_info, scenario_config_,
      injector_->planning_context());

  if (status == scenario::util::PASS_DESTINATION ||
      status == scenario::util::PARK_COMPLETE) {
    return FinishStage(true);
  } else if (status == scenario::util::PARK_FAIL) {
    return FinishStage(false);
  }

  // check path_data to fail sooner
  bool path_fail = false;
  const auto& candidate_path_data = reference_line_info.GetCandidatePathData();
  if (!candidate_path_data.empty()) {
    for (const auto& path_data : candidate_path_data) {
      if (path_data.path_label().find("pullover") == std::string::npos) {
        break;
      }

      for (size_t i = path_data.discretized_path().size(); i >= 1; --i) {
        if (path_data.frenet_frame_path().back().s() -
                path_data.frenet_frame_path()[i - 1].s() <
            kNumExtraTailBoundPoint * kPathBoundsDeciderResolution) {
          continue;
        }
        // check the last adc_position planned
        const auto& path_point = path_data.discretized_path()[i];
        scenario::util::PullOverStatus status =
            scenario::util::CheckADCPullOverPathPoint(
                reference_line_info, scenario_config_, path_point,
                injector_->planning_context());
        if (status == scenario::util::PARK_FAIL) {
          path_fail = true;
        }
        break;
      }
    }
  }

  // add a stop fence for adc to pause at a better position
  if (path_fail) {
    const auto& pull_over_status =
        injector_->planning_context()->planning_status().pull_over();
    if (pull_over_status.has_position() &&
        pull_over_status.position().has_x() &&
        pull_over_status.position().has_y()) {
      const auto& reference_line = reference_line_info.reference_line();
      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

      const double stop_line_s =
          pull_over_sl.s() -
          scenario_config_.s_distance_to_stop_for_open_space_parking();
      const std::string virtual_obstacle_id = "DEST_PULL_OVER_PREPARKING";
      const std::vector<std::string> wait_for_obstacle_ids;
      planning::util::BuildStopDecision(
          virtual_obstacle_id, stop_line_s, 1.0,
          StopReasonCode::STOP_REASON_PREPARKING, wait_for_obstacle_ids,
          "PULL-OVER-scenario", frame,
          &(frame->mutable_reference_line_info()->front()));

      ADEBUG << "Build a stop fence to pause ADC at a better position: id["
             << virtual_obstacle_id << "] s[" << stop_line_s << "]";

      const double adc_front_edge_s =
          reference_line_info.AdcSlBoundary().end_s();
      double distance = stop_line_s - adc_front_edge_s;
      static constexpr double kPreparkingStopDistance = 1.0;
      static constexpr double kPreparkingAngleDiff = 0.2;
      auto ref_point = reference_line.GetReferencePoint(adc_front_edge_s);
      double angle = common::math::AngleDiff(pull_over_status.theta(),
                                             ref_point.heading());
      if (distance <= kPreparkingStopDistance &&
          angle <= kPreparkingAngleDiff) {
        return FinishStage(false);
      }
    }
  }

  return StageStatus::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus PullOverStageApproach::FinishStage(const bool success) {
  if (success) {
    return FinishScenario();
  } else {
    next_stage_ = ScenarioConfig::PULL_OVER_RETRY_APPROACH_PARKING;
    return Stage::FINISHED;
  }
}
```

# PullOverStageRetryApproachParking

## Process
```c++
Stage::StageStatus PullOverStageRetryApproachParking::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: RetryApproachParking";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "PullOverStageRetryApproachParking planning error";
  }

  if (CheckADCStop(*frame)) {
    return FinishStage();
  }

  return StageStatus::RUNNING;
}
```

# PullOverStageRetryParking

## Process
```c++
Stage::StageStatus PullOverStageRetryParking::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "PullOverStageRetryParking planning error";
    return StageStatus::ERROR;
  }

  // set debug info in planning_data
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  auto* pull_over_debug = frame->mutable_open_space_info()
                              ->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_pull_over();
  pull_over_debug->mutable_position()->CopyFrom(pull_over_status.position());
  pull_over_debug->set_theta(pull_over_status.theta());
  pull_over_debug->set_length_front(pull_over_status.length_front());
  pull_over_debug->set_length_back(pull_over_status.length_back());
  pull_over_debug->set_width_left(pull_over_status.width_left());
  pull_over_debug->set_width_right(pull_over_status.width_right());
  frame->mutable_open_space_info()->sync_debug_instance();

  if (CheckADCPullOverOpenSpace()) {
    return FinishStage();
  }

  return StageStatus::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus PullOverStageRetryParking::FinishStage() {
  return FinishScenario();
}
```
