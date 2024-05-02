# learning_model_sample

# 流程

# LearningModelSampleScenario

```
stage_type: LEARNING_MODEL_RUN

stage_config: {
  stage_type: LEARNING_MODEL_RUN
  enabled: true
  task_type: LEARNING_MODEL_INFERENCE_TASK
  task_type: LEARNING_MODEL_INFERENCE_TRAJECTORY_TASK
}
```

# LearningModelSampleStageRun

## Process
```c++
Stage::StageStatus LearningModelSampleStageRun::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Run";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok =
      ExecuteTaskOnReferenceLineForOnlineLearning(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "LearningModelSampleStageRun planning error";
    return Stage::RUNNING;
  }

  return FinishStage();
}
```

## ExecuteTaskOnReferenceLineForOnlineLearning
```c++
bool Stage::ExecuteTaskOnReferenceLineForOnlineLearning(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  // online learning mode
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    reference_line_info.SetDrivable(false);
  }

  // FIXME(all): current only pick up the first reference line to use
  // learning model trajectory
  auto& picked_reference_line_info =
      frame->mutable_reference_line_info()->front();
  for (auto* task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    const auto ret = task->Execute(frame, &picked_reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "task[" << task->Name() << "] time spent: " << time_diff_ms
           << " ms.";
    RecordDebugInfo(&picked_reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
  }

  const std::vector<common::TrajectoryPoint>& adc_future_trajectory_points =
      picked_reference_line_info.trajectory();
  DiscretizedTrajectory trajectory;
  if (picked_reference_line_info.AdjustTrajectoryWhichStartsFromCurrentPos(
          planning_start_point, adc_future_trajectory_points, &trajectory)) {
    picked_reference_line_info.SetTrajectory(trajectory);
    picked_reference_line_info.SetDrivable(true);
    picked_reference_line_info.SetCost(0);
  }

  return true;
}
```

## FinishStage
```c++
Stage::StageStatus LearningModelSampleStageRun::FinishStage() {
  return FinishScenario();
}
```