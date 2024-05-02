# yield_sign
让行场景

# 流程


# YieldSignScenario
```
stage_type: YIELD_SIGN_APPROACH
stage_type: YIELD_SIGN_CREEP

stage_config: {
  stage_type: YIELD_SIGN_APPROACH
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
  stage_type: YIELD_SIGN_CREEP
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
```

# YieldSignStageApproach

## Process
```c++
Stage::StageStatus YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "YieldSignStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  if (GetContext()->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  for (const auto& yield_sign_overlap_id :
       GetContext()->current_yield_sign_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        scenario::util::GetOverlapOnReferenceLine(
            reference_line_info, yield_sign_overlap_id,
            ReferenceLineInfo::YIELD_SIGN);
    if (!current_yield_sign_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_yield_sign_overlap->start_s, false);

    static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_pass_stop_sign =
        adc_front_edge_s - current_yield_sign_overlap->start_s;
    if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
      // passed stop line
      return FinishStage();
    }

    const double distance_adc_to_stop_line =
        current_yield_sign_overlap->start_s - adc_front_edge_s;
    ADEBUG << "yield_sign_overlap_id[" << yield_sign_overlap_id << "] start_s["
           << current_yield_sign_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "]";
    bool yield_sign_done = false;
    if (distance_adc_to_stop_line <
        scenario_config_.max_valid_stop_distance()) {
      // close enough, check yield_sign clear
      yield_sign_done = true;
      const auto& path_decision = reference_line_info.path_decision();
      for (const auto* obstacle : path_decision.obstacles().Items()) {
        const std::string& obstacle_id = obstacle->Id();
        std::string obstacle_type_name =
            PerceptionObstacle_Type_Name(obstacle->Perception().type());
        ADEBUG << "yield_sign[" << yield_sign_overlap_id << "] obstacle_id["
               << obstacle_id << "] type[" << obstacle_type_name << "]";
        if (obstacle->IsVirtual()) {
          continue;
        }

        if (obstacle->reference_line_st_boundary().IsEmpty()) {
          continue;
        }

        static constexpr double kMinSTBoundaryT = 6.0;  // sec
        if (obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT) {
          continue;
        }
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        // max st_min_t(sec) to ignore
        static constexpr double kIgnoreMaxSTMinT = 0.1;
        // min st_min_s(m) to ignore
        static constexpr double kIgnoreMinSTMinS = 15.0;
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT &&
            obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) {
          continue;
        }

        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_yield_sign()
            ->add_wait_for_obstacle_id(obstacle->Id());

        yield_sign_done = false;
      }
    }

    if (yield_sign_done) {
      return FinishStage();
    }
  }

  return Stage::RUNNING;
}
```

## FinishStage
处理yeild确认
```c++
Stage::StageStatus YieldSignStageApproach::FinishStage() {
  // update PlanningContext
  auto* yield_sign_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_yield_sign();

  yield_sign_status->mutable_done_yield_sign_overlap_id()->Clear();
  for (const auto& yield_sign_overlap_id :
       GetContext()->current_yield_sign_overlap_ids) {
    yield_sign_status->add_done_yield_sign_overlap_id(yield_sign_overlap_id);
  }
  yield_sign_status->clear_wait_for_obstacle_id();

  GetContext()->creep_start_time = Clock::NowInSeconds();

  next_stage_ = ScenarioConfig::YIELD_SIGN_CREEP;
  return Stage::FINISHED;
}
```

# YieldSignStageCreep

## Process

```c++
Stage::StageStatus YieldSignStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "YieldSignStageCreep planning error";
  }

  if (GetContext()->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string yield_sign_overlap_id =
      GetContext()->current_yield_sign_overlap_ids[0];

  // get overlap along reference line
  PathOverlap* current_yield_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                yield_sign_overlap_id,
                                                ReferenceLineInfo::YIELD_SIGN);
  if (!current_yield_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double yield_sign_start_s = current_yield_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(yield_sign_start_s, false);

  const double yield_sign_end_s = current_yield_sign_overlap->end_s;
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));

  double creep_stop_s =
      yield_sign_end_s + task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info, yield_sign_end_s,
                           wait_time, timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

## FinishStage
结束YieldSignStageCreep阶段，实际上就是结束整个场景
```c++
Stage::StageStatus YieldSignStageCreep::FinishStage() {
  return FinishScenario();
}
```
