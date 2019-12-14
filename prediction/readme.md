# Dig into Apollo - Prediction ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 悟已往之不諫，知來者之可追


## Table of Contents
- [介绍](#introduction)
- [目录结构](#directory)


<a name="introduction" />

## 介绍
首先建议先阅读官方文档(readme.md)，里面说明了数据流向，也就是说预测模块是直接接收的感知模块给出的障碍物信息，这和CV领域的传统预测任务有区别，CV领域的预测任务不需要先识别物体，只需要根据物体的特征，对比前后2帧，然后得出物体的位置，也就说甚至不需要物体识别，业界之所以不这么做的原因是因为检测物体太耗时了。当然也有先检测物体再做跟踪的，也就是说目前apollo中的物体检测实际上是采用的第二种方法，这也可以理解，反正感知模块一定会工作，而且一定要检测物体，所以何不把这个信息直接拿过来用呢？这和人类似，逐帧跟踪指定特征的对象，就是物体的轨迹，然后再根据现有的轨迹预测物体讲来的轨迹。  


<a name="directory" />

## 目录结构
预测模块的目录结构如下：  
```
.
├── BUILD
├── common                    // common目录，公用类
├── conf                      // 启动配置
├── container                 // 1. 消息容器
├── dag                       // 启动文件dag
├── data                      // 模型文件路径
├── evaluator                 // 3. 评估者
├── images                    // 文档（图片）
├── launch                    // 启动，加载模块
├── network
├── pipeline                  // 工具
├── prediction_component.cc       // 预测模块主入口
├── prediction_component.h
├── prediction_component_test.cc
├── predictor                // 4. 预测器
├── proto                   // protobuf消息格式
├── README_cn.md            // 文档（中文介绍，建议直接看英文）
├── README.md              // 文档（英文介绍）
├── scenario               // 2. 场景
├── submodules             // 子模块???
├── testdata               // 测试数据
└── util                   // 工具类
```
可以看到预测模块主要是分为2大块功能，一是实时的预测执行过程，一是工具类（离线验证？）：
* **在线预测流程** - container -> scenario -> evaluator -> predictor
* **离线流程** - pipeline (util)提取bag包中的数据给离线测试用？

## 预测模块(PredictionComponent类)
预测模块和其它模块一样，都是在cyber中注册，具体的实现在"prediction_component.h"和"prediction_component.cc"中，我们知道cyber模块有2种消息触发模式，一种是定时器触发，一种是消息触发，而预测为消息触发模式。  
预测模块的**输入消息**为：
1. **perception::PerceptionObstacles** - 感知模块输出的障碍物信息
2. **planning::ADCTrajectory** - 规划模块输出的行驶路径
3. **localization::LocalizationEstimate** - 车辆当前的位置  

**输出消息**:  
1. **prediction::PredictionObstacles** - 预测模块输出的障碍物信息  

预测模块和所有其它模块一样，都实现了"cyber::Component"基类中的"Init()"和"Proc()"方法，分别进行初始化和消息触发调用，调用由框架自动执行，关于cyber如何调用和执行每个模块，可以参考cyber模块的介绍，下面我们主要介绍这2个方法。  

#### 初始化(Init())
预测模块的初始化在"PredictionComponent::Init()"中进行，主要是注册消息读取和发送控制器，用来读取和发送消息，**需要注意的是初始化过程中也对"MessageProcess"类进行了初始化，而"MessageProcess"实现了预测模块的整个消息处理流程**。  
```
bool PredictionComponent::Init() {
  component_start_time_ = Clock::NowInSeconds();

  // 预测模块消息处理流程初始化
  if (!MessageProcess::Init()) {
    return false;
  }
  
  // 规划模块消息读取者
  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic, nullptr);

  // 定位模块消息读取者
  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          FLAGS_localization_topic, nullptr);
  // 故事读取???
  storytelling_reader_ = node_->CreateReader<storytelling::Stories>(
      FLAGS_storytelling_topic, nullptr);
  // 预测消息发送者
  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);
  // 中间消息的发送者，这一块的目的是什么？？？
  container_writer_ =
      node_->CreateWriter<SubmoduleOutput>(FLAGS_container_topic_name);

  adc_container_writer_ = node_->CreateWriter<ADCTrajectoryContainer>(
      FLAGS_adccontainer_topic_name);

  perception_obstacles_writer_ = node_->CreateWriter<PerceptionObstacles>(
      FLAGS_perception_obstacles_topic_name);

  return true;
}
```

下面我们接着看消息回调执行函数  
#### 回调执行(Proc)  
回调执行函数会执行以下过程:
```
bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  // 1. 如果使用lego，则执行子过程
  if (FLAGS_use_lego) {
    return ContainerSubmoduleProcess(perception_obstacles);
  }
  // 2. 否则就执行端到端的过程
  return PredictionEndToEndProc(perception_obstacles);
}
```

下面我们分别看下这2个过程有什么差异？我们先看  
#### ContainerSubmoduleProcess
子过程的函数如下：  
```c++
bool PredictionComponent::ContainerSubmoduleProcess(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  constexpr static size_t kHistorySize = 10;
  const auto frame_start_time = absl::Now();
  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  // 读取定位信息，并且更新位置容器
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(*ptr_localization_msg);

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  // 读取规划路径，并且更新路径容器
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(*ptr_trajectory_msg);
  }

  // Read storytelling message and call OnStorytelling to update the
  // StoryTellingContainer
  // 读取故事消息，并且更新故事容器？？？
  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(*ptr_storytelling_msg);
  }

  MessageProcess::ContainerProcess(*perception_obstacles);
  // 障碍物容器指针
  auto obstacles_container_ptr =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container_ptr);
  // 路径规划容器指针
  auto adc_trajectory_container_ptr =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(adc_trajectory_container_ptr);
  // 输出障碍物信息
  SubmoduleOutput submodule_output =
      obstacles_container_ptr->GetSubmoduleOutput(kHistorySize,
                                                  frame_start_time);
  // 发布消息
  container_writer_->Write(submodule_output);
  adc_container_writer_->Write(*adc_trajectory_container_ptr);
  perception_obstacles_writer_->Write(*perception_obstacles);
  return true;
}
```
看起来上述函数只是计算中间过程，并且发布消息到订阅节点。具体的用途需要结合业务来分析（具体的业务场景是什么？？？）。  
 

#### PredictionEndToEndProc
端到端的过程函数如下：  
```
bool PredictionComponent::PredictionEndToEndProc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {

  // Update relative map if needed
  // 如果是导航模式，需要判断地图是否准备好
  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return false;
  }

  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  // 读取定位消息，并且处理消息
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  MessageProcess::OnLocalization(*ptr_localization_msg);

  // Read storytelling message and call OnStorytelling to update the
  // StoryTellingContainer
  // 读取并且处理故事消息？？？
  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(*ptr_storytelling_msg);
  }

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  // 读取并且处理规划消息
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(*ptr_trajectory_msg);
  }


  // Get all perception_obstacles of this frame and call OnPerception to
  // process them all.
  // 处理障碍物消息
  auto perception_msg = *perception_obstacles;
  PredictionObstacles prediction_obstacles;
  MessageProcess::OnPerception(perception_msg, &prediction_obstacles);


  // 填充发布预测的障碍物轨迹消息
  // Postprocess prediction obstacles message
  prediction_obstacles.set_start_timestamp(frame_start_time_);
  ...
  
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  prediction_writer_->Write(prediction_obstacles);
  return true;
}
```

## 消息处理(MessageProcess)  
可以看到上述过程都是在MessageProcess中处理完成的，那么我们先看下MessageProcess的执行过程。  

#### 初始化(Init)  
消息处理的初始化首先在"PredictionComponent::Init()"中调用，下面我们看下实现了哪些功能：  
```c++
bool MessageProcess::Init() {
  // 1. 初始化容器
  InitContainers();
  // 2. 初始化评估器
  InitEvaluators();
  // 3. 初始化预测器
  InitPredictors();

  // 如果为导航模式，则判断地图是否加载
  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Map cannot be loaded.";
    return false;
  }

  return true;
}
```
上述子过程的初始化就是从配置文件读取配置，并且初始化对应的类，结构相对比较简单，这里就不一一介绍了。  

#### 消息处理
定位，规划和故事的消息处理相对比较简单，主要是向对应的容器中插入数据（每个容器都实现了Insert()方法），下面着重介绍感知模块消息的处理过程，该过程也输出了最后的结果。  
```c++
void MessageProcess::OnPerception(
    const perception::PerceptionObstacles& perception_obstacles,
    PredictionObstacles* const prediction_obstacles) {
  // 1. 分析场景和处理容器中的数据  
  ContainerProcess(perception_obstacles);

  // 获取障碍物容器
  auto ptr_obstacles_container =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  // 获取规划曲线容器
  auto ptr_ego_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);

  // Insert features to FeatureOutput for offline_mode
  // 离线模式，保存障碍物曲线？？？
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpFeatureProto) {
    for (const int id :
         ptr_obstacles_container->curr_frame_movable_obstacle_ids()) {
      Obstacle* obstacle_ptr = ptr_obstacles_container->GetObstacle(id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Null obstacle found.";
        continue;
      }
      if (!obstacle_ptr->latest_feature().IsInitialized()) {
        AERROR << "Obstacle [" << id << "] has no latest feature.";
        continue;
      }
      // TODO(all): the adc trajectory should be part of features for learning
      //            algorithms rather than part of the feature.proto
      /*
      *obstacle_ptr->mutable_latest_feature()->mutable_adc_trajectory_point() =
          ptr_ego_trajectory_container->adc_trajectory().trajectory_point();
      */
      FeatureOutput::InsertFeatureProto(obstacle_ptr->latest_feature());
      ADEBUG << "Insert feature into feature output";
    }
    // Not doing evaluation on offline mode
    return;
  }

  // Make evaluations
  // 2. 进行评估  
  EvaluatorManager::Instance()->Run(ptr_obstacles_container);
  if (FLAGS_prediction_offline_mode ==
          PredictionConstants::kDumpDataForLearning ||
      FLAGS_prediction_offline_mode == PredictionConstants::kDumpFrameEnv) {
    return;
  }

  // Make predictions
  // 3. 进行预测  
  PredictorManager::Instance()->Run(perception_obstacles,
                                    ptr_ego_trajectory_container,
                                    ptr_obstacles_container);

  // Get predicted obstacles
  // 4. 输出预测结果  
  *prediction_obstacles = PredictorManager::Instance()->prediction_obstacles();
}
```
上面的消息处理过程实际上是整个预测的过程，分为以下几个步骤：  
![process](img/process.jpg)  


下面主要分析各个模块的输入是什么，输出是什么？ 以及它们的作用？   
## 容器(container)  
实际上消息处理在"MessageProcess"类中，该类在"common/message_process.cc"中，而其中会调用"ContainerManager"类，把消息放入对应的Container中。  

#### 

## 场景(scenario)
根据本车的位置，和高精度地图，解析当前车辆所在的场景。  

## 评估者(evaluator)
"Evaluator"类为基类，其它类继承至该类，而"EvaluatorManager"类做为管理类，负责管理三种评估者，分别为：自行车，行人，汽车。  


## 预测器(predictor)
"Predictor"类为基类，其它类继承至该类，而"PredictorManager"类作为管理类。最后通过预测器预测障碍物的轨迹。 


## Reference
[Apollo 5.0 障碍物行为预测技术](https://www.cnblogs.com/liuzubing/p/11388485.html)   
[Apollo自动驾驶入门课程第⑥讲—预测](https://cloud.tencent.com/developer/news/310036)  

