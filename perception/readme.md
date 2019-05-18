# Dig into Apollo - Perception ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 温故而知新，可以为师矣


## Table of Contents
- [Perception模块简介](#introduction)
- [基础知识](#base)


<a name="introduction" />

## Perception模块简介

我们先看下perception的目录结构：  
```
.
├── base         // 基础类
├── BUILD        // 编译testdata，用于测试
├── camera       // 摄像头
├── common       // 公共目录
├── data
├── fusion       // 融合
├── inference    // 推理
├── lib          // lib库
├── lidar        // 雷达
├── map          // 地图
├── onboard      // 消息处理
├── production   // 加载模块
├── proto        // 数据格式，protobuf
├── radar        // 毫米波
├── README.md
└── testdata    // 上述几个模块的测试数据，包括训练好的模型
```
apollo的感知模块没有开放训练模型，只是开放了testdata，下载训练好的模型之后来跑一个简单的Demo。  

## radar
我们先从一个简单的模块开始看起，首先看下radar目录：  
```
.
├── app          // 每个模块都有一个app目录
├── common       // 公共目录
└── lib          // 库
```




