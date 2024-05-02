# learning_model_inference_task

# LearningModelInferenceTask

## Execute
```c++
Status LearningModelInferenceTask::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);

  Task::Execute(frame, reference_line_info);
  return Process(frame);
}
```

## Process
```c++
Status LearningModelInferenceTask::Process(Frame* frame) {
  CHECK_NOTNULL(frame);
  const auto& config = config_.learning_model_inference_task_config();

  if (!injector_->learning_based_data() ||
      !injector_->learning_based_data()->GetLatestLearningDataFrame()) {
    const std::string msg = "learning_data_frame empty";
    AERROR << msg;
    // hybrid model will use rule based planning when learning based data or
    // learning data frame is empty
    if (config.allow_empty_learning_based_data()) {
      return Status::OK();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  LearningDataFrame learning_data_frame;
  learning_data_frame.CopyFrom(
      *(injector_->learning_based_data()->GetLatestLearningDataFrame()));

  ADEBUG << "LearningModelInferenceTask: frame_num["
         << learning_data_frame.frame_num() << "] adc_trajectory_point_size["
         << learning_data_frame.adc_trajectory_point_size() << "]";

  if (learning_data_frame.adc_trajectory_point_size() <= 0) {
    const std::string msg =
        absl::StrCat("learning_data adc_trajectory_point empty. frame_num[",
                     learning_data_frame.frame_num(), "]");
    AERROR << msg;
    // hybrid model will use rule based planning when learning model output is
    // not ready
    if (config.allow_empty_output_trajectory()) {
      return Status::OK();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double start_point_timestamp_sec =
      learning_data_frame
          .adc_trajectory_point(
              learning_data_frame.adc_trajectory_point_size() - 1)
          .timestamp_sec();

  ADEBUG << "start_point_timestamp_sec: " << start_point_timestamp_sec;

  TrajectoryEvaluator trajectory_evaluator;

  // evaluate adc trajectory
  trajectory_evaluator.EvaluateADCTrajectory(start_point_timestamp_sec,
                                             config.trajectory_delta_t(),
                                             &learning_data_frame);

  // evaluate obstacle trajectory
  trajectory_evaluator.EvaluateObstacleTrajectory(start_point_timestamp_sec,
                                                  config.trajectory_delta_t(),
                                                  &learning_data_frame);

  // evaluate obstacle prediction trajectory
  trajectory_evaluator.EvaluateObstaclePredictionTrajectory(
      start_point_timestamp_sec, config.trajectory_delta_t(),
      &learning_data_frame);

  if (!trajectory_imitation_inference_->LoadModel()) {
    const std::string msg = absl::StrCat(
        "TrajectoryImitationInference LoadModel() failed. frame_num[",
        learning_data_frame.frame_num(), "]");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (!trajectory_imitation_inference_->DoInference(&learning_data_frame)) {
    const std::string msg = absl::StrCat(
        "TrajectoryImitationLibtorchInference Inference failed. frame_num[",
        learning_data_frame.frame_num(), "]");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const int adc_future_trajectory_point_size =
      learning_data_frame.output().adc_future_trajectory_point_size();
  ADEBUG << "   adc_future_trajectory_point_size["
         << adc_future_trajectory_point_size << "]";
  if (adc_future_trajectory_point_size < 10) {
    const std::string msg = absl::StrCat(
        "too short adc_future_trajectory_point. frame_num[",
        learning_data_frame.frame_num(), "] adc_future_trajectory_point_size[",
        adc_future_trajectory_point_size, "]");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // evaluate adc future trajectory
  // TODO(all): move to conf
  constexpr double kADCFutureTrajectoryDeltaTime = 0.02;
  std::vector<TrajectoryPointFeature> future_trajectory;
  for (const auto& tp :
       learning_data_frame.output().adc_future_trajectory_point()) {
    future_trajectory.push_back(tp);
  }

  TrajectoryPointFeature tp;
  const int last = learning_data_frame.adc_trajectory_point_size() - 1;
  tp.set_timestamp_sec(
      learning_data_frame.adc_trajectory_point(last).timestamp_sec());
  tp.mutable_trajectory_point()->CopyFrom(
      learning_data_frame.adc_trajectory_point(last).trajectory_point());
  future_trajectory.insert(future_trajectory.begin(), tp);

  std::vector<TrajectoryPointFeature> evaluated_future_trajectory;
  trajectory_evaluator.EvaluateADCFutureTrajectory(
      learning_data_frame.frame_num(), future_trajectory,
      start_point_timestamp_sec, kADCFutureTrajectoryDeltaTime,
      &evaluated_future_trajectory);

  // convert to common::TrajectoryPoint
  std::vector<common::TrajectoryPoint> adc_future_trajectory;
  ConvertADCFutureTrajectory(evaluated_future_trajectory,
                             &adc_future_trajectory);
  ADEBUG << "adc_future_trajectory_size: " << adc_future_trajectory.size();

  injector_->learning_based_data()
      ->set_learning_data_adc_future_trajectory_points(adc_future_trajectory);

  return Status::OK();
}
```
