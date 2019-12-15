## 如何获取场景？？？
如何分析并且获取场景？？？？
```
void ScenarioManager::Run() {
  auto environment_features = FeatureExtractor::ExtractEnvironmentFeatures();

  auto ptr_scenario_features = ScenarioAnalyzer::Analyze(environment_features);

  current_scenario_ = ptr_scenario_features->scenario();

  // TODO(all) other functionalities including lane, junction filters
}
```

## 如何生成LaneGraph
```
Obstacle::BuildLaneGraph()
```
1. 障碍物如何生成Lane图，作用是什么？？？  

2. common目录的"RoadGraph"生成的图和上述的图有什么关系？？？




