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


## protobuf中mutable_的作用
可以看出，对于每个字段会生成一个has函数(has_number)、clear清除函数(clear_number)、set函数(set_number)、get函数(number和mutable_number)。这儿解释下get函数中的两个函数的区别，对于原型为const std::string &number() const的get函数而言，返回的是常量字段，不能对其值进行修改。但是在有一些情况下，对字段进行修改是必要的，所以提供了一个mutable版的get函数，通过获取字段变量的指针，从而达到改变其值的目的。  


## 计划
1. 分析下LaneGraph和RoadGraph是如何生成的？？？  
2. 分析LTSM和MLP的实现？？？  
3. 分析预测曲线的原理？？？  

4. planning模块是如何利用这些预测的障碍物信息的？？？  
