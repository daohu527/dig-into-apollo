# traffic_light_protected

# 流程

#
```
stage_type: TRAFFIC_LIGHT_PROTECTED_APPROACH
stage_type: TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE

stage_config: {
  stage_type: TRAFFIC_LIGHT_PROTECTED_APPROACH
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
  stage_type: TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE
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

# TrafficLightProtectedStageApproach

## Process
```c++
Stage::StageStatus TrafficLightProtectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightProtectedStageApproach planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

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

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    // check distance to stop line
    if (distance_adc_to_stop_line >
        scenario_config_.max_valid_stop_distance()) {
      traffic_light_all_done = false;
      break;
    }

    // check on traffic light color
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light_all_done) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus TrafficLightProtectedStageApproach::FinishStage() {
  auto* traffic_light = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_traffic_light();
  traffic_light->clear_done_traffic_light_overlap_id();
  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    traffic_light->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
  }

  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}
```

# TrafficLightProtectedStageIntersectionCruise

## Process
```c++
Stage::StageStatus TrafficLightProtectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightProtectedStageIntersectionCruise plan error";
  }

  bool stage_done =
      stage_impl_.CheckDone(*frame, ScenarioConfig::TRAFFIC_LIGHT_PROTECTED,
                            config_, injector_->planning_context(), true);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus TrafficLightProtectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}
```
