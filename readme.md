# Dig into Apollo ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> Rome wasn't built in a day.

此项目是对[apollo](https://github.com/ApolloAuto/apollo)拙劣的介绍，主要根据模块对apollo代码进行介绍，如果你需要了解自动驾驶更多的内容，请参考另外一个项目[awesome-self-driving-cars](https://github.com/daohu527/awesome-self-driving-cars)，我们相信在不久的将来，科技必将改变世界。  


## 目录
- [Perception](perception)
    - [CNN](perception/cnn)
        - [什么是CNN？](perception/cnn#what_is_cnn)
        - [CNN的原理](perception/cnn#cnn_principle)
        - [如何构建CNN](perception/cnn#how_to)
        - [基本概念](perception/cnn#base_concept)
        - [引用](perception/cnn#reference)
- [Routing](routing)
    - [Routing模块简介](routing#introduction)
    - [基础知识](routing#base)
      - [Demo](routing#demo)
      - [地图](routing#map)
      - [最短距离](routing#shortest_path)
    - [Routing模块分析](routing#routing)
      - [创建Routing地图](routing#create_routing_map)
      - [Routing主流程](routing#routing_main)
    - [调试工具](routing#tools)
    - [问题](routing#question)
    - [OSM数据查找](routing#osm_find)
    - [Reference](routing#reference)
- [Planning](planning)
    - [Planning模块简介](planning#introduction)
      - [Planning输入输出](planning#planning_io)
      - [Planning整个流程](planning#planning_flow)
    - [Planning模块入口](planning#planning_entry)
      - [模块注册](planning#planning_register)
      - [模块初始化](planning#planning_init)
      - [模块运行](planning#planning_proc)
    - [OnLanePlanning](planning#onLanePlanning)
      - [初始化](planning#onLanePlanning_init)
      - [事件触发](planning#onLanePlanning_trigger)
    - [Planner](planning#planner)
      - [Planner注册场景](planning#planner_register)
      - [运行场景](planning#planner_plan)
    - [Scenario](planning#scenario)
      - [场景转换](planning#scenario_update)
      - [场景运行](planning#scenario_process)
    - [Task](planning#task)
      - [DP & QP](planning#dp_qp)
    - [Reference](planning#reference)
- [Simulation](simulation)
- [Cyber](cyber)
    - [How do you design cyber?](cyber#how)
    - [需求分析](cyber#requirements)
    - [系统设计](cyber#design)
      - [随意的假设](cyber#hypothesis)
      - [多节点](cyber#multinode)
      - [通信方式](cyber#communication)
      - [资源调度](cyber#schedule)
      - [软件复用](cyber#reuse)
      - [快速测试](cyber#test)
    - [其他](cyber#other)
      - [云平台](cyber#cloud)
    - [Reference](cyber#reference)
- [Drivers](drivers)
- [Performance](performance)
    - [线程调度](performance#schedule)
    - [Cgroups](performance#cgroups)
    - [CPU亲和性](performance#cpu)
    - [中断绑定](performance#interrupt)
    - [linux性能优化](performance#linux)
      - [Perf安装](performance#perf)
      - [火焰图](performance#flame_graph)
    - [Reference](performance#reference)
- [Toolbox](toolbox)



> 各个模块的介绍都在对应的子目录中，主流程放在readme.md中，一些细节的函数和疑问放在todo.md中，需要了解主流程直接看readme.md，如果要深挖整个代码可以参考todo.md。

## 鸣谢
我们非常欢迎对项目的贡献，如果您觉得项目很酷✌️️，欢迎star❤️。



## 参考
[apollo](https://github.com/ApolloAuto/apollo)  
[awesome-self-driving-cars](https://github.com/daohu527/awesome-self-driving-cars)  
