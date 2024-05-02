# bare_intersection

# 流程

# BareIntersectionUnprotectedScenario

```
stage_type: BARE_INTERSECTION_UNPROTECTED_APPROACH
stage_type: BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE

stage_config: {
  stage_type: BARE_INTERSECTION_UNPROTECTED_APPROACH
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
  stage_type: BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE
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

# BareIntersectionUnprotectedStageApproach

## Process
```c++
Stage::StageStatus BareIntersectionUnprotectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const std::string pnc_junction_overlap_id =
      GetContext()->current_pnc_junction_overlap_id;
  if (pnc_junction_overlap_id.empty()) {
    return FinishScenario();
  }

  // get overlap along reference line
  PathOverlap* current_pnc_junction = scenario::util::GetOverlapOnReferenceLine(
      reference_line_info, pnc_junction_overlap_id,
      ReferenceLineInfo::PNC_JUNCTION);
  if (!current_pnc_junction) {
    return FinishScenario();
  }

  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_pnc_junction =
      current_pnc_junction->start_s - adc_front_edge_s;
  ADEBUG << "pnc_junction_overlap_id[" << pnc_junction_overlap_id
         << "] start_s[" << current_pnc_junction->start_s
         << "] distance_adc_to_pnc_junction[" << distance_adc_to_pnc_junction
         << "]";
  if (distance_adc_to_pnc_junction < -kPassStopLineBuffer) {
    // passed stop line
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().SetCruiseSpeed(
      scenario_config_.approach_cruise_speed());

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(current_pnc_junction->start_s,
                                            false);

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  std::vector<std::string> wait_for_obstacle_ids;
  bool clear = CheckClear(reference_line_info, &wait_for_obstacle_ids);

  if (scenario_config_.enable_explicit_stop()) {
    bool stop = false;
    static constexpr double kCheckClearDistance = 5.0;  // meter
    static constexpr double kStartWatchDistance = 2.0;  // meter
    if (distance_adc_to_pnc_junction <= kCheckClearDistance &&
        distance_adc_to_pnc_junction >= kStartWatchDistance && !clear) {
      stop = true;
    } else if (distance_adc_to_pnc_junction < kStartWatchDistance) {
      // creeping area
      auto* bare_intersection_status = injector_->planning_context()
                                                ->mutable_planning_status()
                                                ->mutable_bare_intersection();
      int clear_counter = bare_intersection_status->clear_counter();
      clear_counter = clear ? clear_counter + 1 : 0;

      if (clear_counter >= 5) {
        clear_counter = 0;  // reset
      } else {
        stop = true;
      }
      // use PlanningContext instead of static counter for multi-ADC
      bare_intersection_status->set_clear_counter(clear_counter);
    }

    if (stop) {
      // build stop decision
      ADEBUG << "BuildStopDecision: bare pnc_junction["
             << pnc_junction_overlap_id << "] start_s["
             << current_pnc_junction->start_s << "]";
      const std::string virtual_obstacle_id =
          "PNC_JUNCTION_" + current_pnc_junction->object_id;
      planning::util::BuildStopDecision(
          virtual_obstacle_id, current_pnc_junction->start_s,
          scenario_config_.stop_distance(),
          StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
          "bare intersection", frame,
          &(frame->mutable_reference_line_info()->front()));
    }
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus BareIntersectionUnprotectedStageApproach::FinishStage(
    Frame* frame) {
  next_stage_ =
      ScenarioConfig::BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE;

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.SetCruiseSpeed(FLAGS_default_cruise_speed);

  return Stage::FINISHED;
}
```

# BareIntersectionUnprotectedStageIntersectionCruise

## Process
```c++
Stage::StageStatus BareIntersectionUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::BARE_INTERSECTION_UNPROTECTED, config_,
      injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus
BareIntersectionUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}
```
