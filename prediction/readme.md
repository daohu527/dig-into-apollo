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
├── common                      // common目录，公用类
├── conf                        // 启动配置
├── container                   // 1. 消息容器
├── dag                         // 启动文件dag
├── data                        // 模型文件路径
├── evaluator                   // 3. 评估者
├── images                      // 文档（图片）
├── launch                      // 启动，加载模块
├── network
├── pipeline                    // 工具
├── prediction_component.cc           // 预测模块主入口
├── prediction_component.h             
├── prediction_component_test.cc
├── predictor                   // 4. 预测器
├── proto                       // protobuf消息格式
├── README_cn.md               // 文档（中文介绍，建议直接看英文）
├── README.md                  // 文档（英文介绍）
├── scenario                   // 2. 场景
├── testdata                   // 测试数据
├── util                       // 工具
└── visualization              // 可视化预测信息
```
可以看到预测模块主要是分为2大块功能，一是实时的预测执行过程，一是工具类（离线验证？）：
* **在线预测流程** - container -> scenario -> evaluator -> predictor
* **离线流程** - pipeline (util)提取bag包中的数据给离线测试用？


下面主要分析下预测模块的主流程，各个模块的输入是什么，输出是什么？  
## 容器(container)  
实际上消息处理在"MessageProcess"类中，该类在"common/message_process.cc"中，而其中会调用"ContainerManager"类，把消息放入对应的Container中。  

## 场景(scenario)
根据本车的位置，和高精度地图，解析当前车辆所在的场景。  

## 评估者(evaluator)
"Evaluator"类为基类，其它类继承至该类，而"EvaluatorManager"类做为管理类，负责管理三种评估者，分别为：自行车，行人，汽车。  


## 预测器(predictor)
"Predictor"类为基类，其它类继承至该类，而"PredictorManager"类作为管理类。最后通过预测器预测障碍物的轨迹。  


