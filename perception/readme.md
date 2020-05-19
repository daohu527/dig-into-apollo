# Dig into Apollo - Perception ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 温故而知新，可以为师矣


## Table of Contents
- [CNN](cnn)
    - [什么是CNN？](cnn#what_is_cnn)
    - [CNN的原理](cnn#cnn_principle)
        - [卷积层(Convolutional Layer)](cnn#convolutional)
        - [池化层(Max Pooling Layer)](cnn#max_pool)
        - [全连接层(Fully Connected Layer)](cnn#fully_connect)
    - [如何构建CNN](cnn#how_to)
    - [基本概念](cnn#base_concept)
    - [引用](cnn#reference)
- [Caffe2](caffe2)
    - [Caffe2环境准备](caffe2#env)
    - [安装显卡驱动](caffe2#drivers)
    - [安装CUDA](caffe2#cuda)
        - [选择CUDA版本](caffe2#cuda_version)
        - [安装CUDA](caffe2#cuda_install)
        - [设置环境变量](caffe2#cuda_env)
        - [检验安装](caffe2#cuda_check)
    - [安装cuDNN](caffe2#cudnn)
    - [安装Caffe2](caffe2#caffe2)
    - [参考](caffe2#reference)

<a name="introduction" />

## Perception模块简介

我们先看下perception的目录结构：  
```
.
├── BUILD                           // 编译testdata，用于测试
├── Perception_README_3_5.md        
├── README.md
├── base                           // 基础类
├── camera                         // 相机相关  --- 子模块流程
├── common                         // 公共目录
├── data                           
├── fusion                         // 融合
├── inference                      // 深度学习推理相关
├── lib                            // lib库
├── lidar                          // 雷达相关  --- 子模块流程
├── map                            // 地图
├── onboard                        // 各个子模块的入口     --- 子模块入口
├── production                     // Cyber加载模块入口    --- Cyber加载入口
├── proto                          // 数据格式，protobuf
├── radar                          // 毫米波  --- 子模块流程
├── testdata                       // 上述几个模块的测试数据，包括训练好的模型
└── tool                           // 离线测试工具
```
下面介绍几个重要的目录结构: 
* production/onboard目录 - **感知模块的入口在production目录，通过lanuch加载对应的dag，启动感知模块**。可以看到感知启动了多个子模块，来处理不同的传感器信息（Lidar,Radar,Camera）。各个子模块的入口在onboard目录中，各个子模块会订阅不同的传感器Topic，然后进行统一的流水线(Pipeline)作业。每个子模块的流水线作业分别在不同的文件夹中(camera, lidar, radar)。这就是感知模块总体的目录结构。
* inference目录 - 深度学习推理相关的库，**主要实现了加载创建caffe深度学习模型，TensorRT深度学习优化器等**。
* lib目录 - 提供注册类和线程池，（提供的线程池也就是说module可以再次启动线程？？？module）

整个模块的流程如图：  
![process](img/perception_process.jpg)  


## production
production中主要是存放**配置和lanuch和dag启动文件**。
```
.
├── conf
├── dag
├── data
└── launch
```
该文件中有多个lanuch文件，同时一个lanuch文件中包含多个dag文件，也就是说一个lanuch文件会启动多个模块，每个模块


## onboard

#### component
感知模块的各个功能模块在component中定义，根据传感器的不同划分，下面我们分别介绍  
实际上在BUILD文件中，下面几个模块编译为一个模块，最后的可执行文件为"libperception_component_lidar"。也就是说下面几个模块的配置可以在dag中查找libperception_component_lidar的配置
```
    name = "perception_component_inner_lidar",
    srcs = [
        "fusion_component.cc",
        "lidar_output_component.cc",
        "radar_detection_component.cc",
        "recognition_component.cc",
        "segmentation_component.cc",
        "detection_component.cc",
    ],
```

## 传感器
目前apollo中的传感器分为radar,lidar,camera3种，每种传感器分别都执行了目标识别和追踪的任务，最后通过fusion对传感器的数据做融合，每个传感器的执行代码分别在"perception/radar","perception/lidar","perception/camera"目录中，整体的执行流程如下图。  
![sensor](img/sensor.jpg)    


## radar
我们先从一个简单的模块开始看起，首先看下radar目录：  
```
.
├── app          // 每个模块都有一个app目录
├── common       // 公共目录
└── lib          // 库
```




radar模块被"RadarDetectionComponent"引用，perception的入口在onboard中。我们最后分析下"RadarDetectionComponent"模块。  


## camera
camera模块的结构和radar类似，目录如下：  
```
.
├── app        \\ 主程序
├── common     \\ 公共程序
├── lib         \\ 库，用来做红绿灯、障碍物检测等功能
├── test        \\ 测试用例
└── tools       \\ 工具，用来做车道线和红绿灯识别结果展示
```




## Reference
[A Beginner's Guide to Convolutional Neural Networks](https://skymind.ai/wiki/convolutional-network)  
[cnn](https://cs231n.github.io/convolutional-networks/)  
[traffic light dataset](https://hci.iwr.uni-heidelberg.de/node/6132/download/3d66608cfb112934ef40175e9a20c81f)  
[pytorch-tutorial](https://github.com/yunjey/pytorch-tutorial)  
[全连接层的作用是什么？](https://www.zhihu.com/question/41037974)  
[索伯算子](https://zh.wikipedia.org/wiki/%E7%B4%A2%E8%B2%9D%E7%88%BE%E7%AE%97%E5%AD%90)  
[卷积](https://zh.wikipedia.org/wiki/%E5%8D%B7%E7%A7%AF)  

> tensorRT
[TensorRT(1)-介绍-使用-安装](https://arleyzhang.github.io/articles/7f4b25ce/)  
[高性能深度学习支持引擎实战——TensorRT](https://zhuanlan.zhihu.com/p/35657027)  

