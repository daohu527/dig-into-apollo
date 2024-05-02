# traffic_light_unprotected_right_turn_config.pb

# 流程

#

```
stage_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP
stage_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP
stage_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE

stage_config: {
  stage_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP
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
  stage_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP
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
  stage_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE
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

# TrafficLightUnprotectedRightTurnStageStop

## Process
```c++
Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightRightTurnUnprotectedStop planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  bool traffic_light_all_stop = true;
  bool traffic_light_all_green = true;
  bool traffic_light_no_right_turn_on_red = false;
  PathOverlap* current_traffic_light_overlap = nullptr;

  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    current_traffic_light_overlap = scenario::util::GetOverlapOnReferenceLine(
        reference_line_info, traffic_light_overlap_id,
        ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

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

    // check distance to stop line
    if (distance_adc_to_stop_line >
        scenario_config_.max_valid_stop_distance()) {
      traffic_light_all_stop = false;
      break;
    }

    // check on traffic light color
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_green = false;
      traffic_light_no_right_turn_on_red =
          CheckTrafficLightNoRightTurnOnRed(traffic_light_overlap_id);
      break;
    }
  }

  if (traffic_light_all_stop && traffic_light_all_green) {
    return FinishStage(true);
  }

  if (!traffic_light_no_right_turn_on_red) {
    if (traffic_light_all_stop && !traffic_light_all_green) {
      // check distance pass stop line
      const double distance_adc_pass_stop_line =
          adc_front_edge_s - current_traffic_light_overlap->end_s;
      ADEBUG << "distance_adc_pass_stop_line[" << distance_adc_pass_stop_line
             << "]";
      if (distance_adc_pass_stop_line >
          scenario_config_.min_pass_s_distance()) {
        return FinishStage(false);
      }

      if (scenario_config_.enable_right_turn_on_red()) {
        // when right_turn_on_red is enabled
        // check on wait-time
        if (GetContext()->stop_start_time == 0.0) {
          GetContext()->stop_start_time = Clock::NowInSeconds();
        } else {
          auto start_time = GetContext()->stop_start_time;
          const double wait_time = Clock::NowInSeconds() - start_time;
          ADEBUG << "stop_start_time[" << start_time << "] wait_time["
                 << wait_time << "]";
          if (wait_time >
              scenario_config_.red_light_right_turn_stop_duration_sec()) {
            return FinishStage(false);
          }
        }
      }
    }
  }

  return Stage::RUNNING;
}
```

## FinishStage
```c++
Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::FinishStage(
    const bool protected_mode) {
  if (protected_mode) {
    // intersection_cruise
    next_stage_ = ScenarioConfig ::
        TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
  } else {
    // check speed at stop_stage
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    if (adc_speed > scenario_config_.max_adc_speed_before_creep()) {
      // skip creep
      next_stage_ = ScenarioConfig ::
          TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
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
      next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP;
    }
  }
  return Stage::FINISHED;
}
```

# TrafficLightUnprotectedRightTurnStageCreep

## Process
```c++
Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageCreep planning error";
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
Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::FinishStage() {
  next_stage_ =
      ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}
```

# TrafficLightUnprotectedRightTurnStageIntersectionCruise

## Process
```c++
Stage::StageStatus
TrafficLightUnprotectedRightTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageIntersectionCruise "
           << "plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN, config_,
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
TrafficLightUnprotectedRightTurnStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}
```
