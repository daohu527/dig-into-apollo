# traffic_light_unprotected_left_turn_config.pb

# 流程

# TrafficLightUnprotectedLeftTurnScenario
```
stage_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH
stage_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP
stage_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE

stage_config: {
  stage_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH
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
  stage_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP
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
  stage_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE
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

# TrafficLightUnprotectedLeftTurnStageApproach

## Process
```c++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().SetCruiseSpeed(
      scenario_config_.approach_cruise_speed());

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageApproach planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  PathOverlap* traffic_light = nullptr;
  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_traffic_light_overlap =
        scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                  traffic_light_overlap_id,
                                                  ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

    traffic_light = current_traffic_light_overlap;

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    if (distance_adc_to_stop_line < 0)
        return FinishStage(frame);
    // check on traffic light color and distance to stop line
    if (signal_color != TrafficLight::GREEN ||
        distance_adc_to_stop_line >=
            scenario_config_.max_valid_stop_distance()) {
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light == nullptr) {
    return FinishScenario();
  }

  if (traffic_light_all_done) {
    return FinishStage(frame);
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageApproach::FinishStage(
    Frame* frame) {
  // check speed at stop_stage
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > scenario_config_.max_adc_speed_before_creep()) {
    // skip creep
    next_stage_ = ScenarioConfig ::
        TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE;
  } else {
    // creep
    // update PlanningContext
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->mutable_done_traffic_light_overlap_id()
        ->Clear();
    for (const auto& traffic_light_overlap_id :
         GetContext()->current_traffic_light_overlap_ids) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
    }

    GetContext()->creep_start_time = Clock::NowInSeconds();
    next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP;
  }

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.SetCruiseSpeed(FLAGS_default_cruise_speed);

  return Stage::FINISHED;
}
```

# TrafficLightUnprotectedLeftTurnStageCreep

## Process
```c++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageCreep planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string traffic_light_overlap_id =
      GetContext()->current_traffic_light_overlap_ids[0];
  PathOverlap* current_traffic_light_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                traffic_light_overlap_id,
                                                ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);

  // creep
  // note: don't check traffic light color while creeping on right turn
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));
  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s = current_traffic_light_overlap->end_s +
                        task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info,
                           current_traffic_light_overlap->end_s, wait_time,
                           timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageCreep::FinishStage() {
  next_stage_ =
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}
```

# TrafficLightUnprotectedLeftTurnStageIntersectionCruise

## Process
```c++
Stage::StageStatus
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageIntersectionCruise "
           << "plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN, config_,
      injector_->planning_context(), true);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}
```