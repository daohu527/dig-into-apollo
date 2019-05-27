# Dig into Apollo ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> Rome wasn't built in a day. - Li Proverbe au Vilain

此项目是对[apollo](https://github.com/ApolloAuto/apollo)拙劣的介绍，主要根据模块对apollo代码进行介绍，如果你需要了解自动驾驶更多的内容，请参考另外一个项目[awesome-self-driving-cars](https://github.com/daohu527/awesome-self-driving-cars)，我们相信在不久的将来，科技必将改变世界。  


## 目录
- [Perception](perception)
- [Routing](routing)
    - [Routing模块简介](routing#routing%E6%A8%A1%E5%9D%97%E7%AE%80%E4%BB%8B)
    - [基础知识](#base)
      - [Demo](#demo)
      - [地图](#map)
      - [最短距离](#shortest_path)
    - [Routing模块分析](#routing)
      - [创建Routing地图](#create_routing_map)
      - [Routing主流程](#routing_main)
    - [调试工具](#tools)
    - [问题](#question)
    - [OSM数据查找](#osm_find)
    - [Reference](#reference)
- [Planning](planning)
    - [Planning模块简介](#introduction)
      - [Planning输入输出](#planning_io)
      - [Planning整个流程](#planning_flow)
    - [Planning模块入口](#planning_entry)
      - [模块注册](#planning_register)
      - [模块初始化](#planning_init)
      - [模块运行](#planning_proc)
    - [OnLanePlanning](#onLanePlanning)
      - [初始化](#onLanePlanning_init)
      - [事件触发](#onLanePlanning_trigger)
    - [Planner](#planner)
      - [Planner注册场景](#planner_register)
      - [运行场景](#planner_plan)
    - [Scenario](#scenario)
      - [场景转换](#scenario_update)
      - [场景运行](#scenario_process)
    - [Task](#task)
      - [DP & QP](#dp_qp)
    - [Reference](#reference)
- [Cyber](cyber)
    - [How do you design cyber?](#how)
    - [需求分析](#requirements)
    - [系统设计](#design)
      - [随意的假设](#hypothesis)
      - [多节点](#multinode)
      - [通信方式](#communication)
      - [资源调度](#schedule)
      - [软件复用](#reuse)
      - [快速测试](#test)
    - [其他](#other)
      - [云平台](#cloud)
    - [Reference](#reference)
- [Drivers](drivers)
- [Performance](performance)
    - [线程调度](#schedule)
    - [Cgroups](#cgroups)
    - [CPU亲和性](#cpu)
    - [中断绑定](#interrupt)
    - [linux性能优化](#linux)
      - [Perf安装](#perf)
      - [火焰图](#flame_graph)
    - [Reference](#reference)
- [Toolbox](toolbox)



> 各个模块的介绍都在对应的子目录中，主流程放在readme.md中，一些细节的函数和疑问放在todo.md中，需要了解主流程直接看readme.md，如果要深挖整个代码可以参考todo.md。

## 鸣谢
我们非常欢迎对项目的贡献，同时我们也欢迎提交建议，如果你觉得项目很酷，请点击下star。我们推崇自由软件和极客精神。



## 参考
[apollo](https://github.com/ApolloAuto/apollo)  
[awesome-self-driving-cars](https://github.com/daohu527/awesome-self-driving-cars)  
