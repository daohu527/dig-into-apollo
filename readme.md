# Dig into Apollo ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> Rome wasn't built in a day. - Li Proverbe au Vilain

本项目是对[apollo](https://github.com/ApolloAuto/apollo)拙劣的介绍，主要根据各个模块对apollo的代码进行了分析，如果你需要了解自动驾驶更多的内容，请参考另外一个项目[awesome-self-driving-cars](https://github.com/daohu527/awesome-self-driving-cars)，相信在不久的将来，科技必将改变世界。  


## 目录
- [Perception](perception)
- [Routing](routing)
    - [Routing模块简介](#introduction)
    - [基础知识](#base)
      - [Demo](#demo)
      - [地图](#map)
      - [最短距离](#shortest_path)
    - [Routing模块分析](#routing)
      - [创建Routing地图](#create_routing_map)
        - [建图流程](#create_map_main)
        - [创建节点](#create_node)
        - [创建边](#create_edge)
      - [Routing主流程](#routing_main)
        - [Routing类](#routing_class)
        - [导航](#navigator_class)
        - [子节点](#subnode)
        - [节点切分](#generate_subnode)
        - [生成子图](#generate_subgraph)
        - [Astar算法](#astar)
    - [调试工具](#tools)
    - [问题](#question)
    - [OSM数据查找](#osm_find)
    - [Reference](#reference)
- [Planning](planning)
- [Cyber](cyber)
- [Drivers](drivers)
- [Performance](performance)
- [Toolbox](toolbox)



> 各个模块的介绍都在对应的子目录中，主流程会放在readme.md中，一些细节的函数和疑问会放在todo.md中，需要了解主流程直接看readme.md，如果需要深挖整个代码可以参考todo.md。

## 鸣谢
我们非常欢迎对项目的贡献，欢迎提交issue和pull request，同时我们也欢迎提交建议，如果你觉得项目很酷，请点击下star。我们推崇自由软件和极客精神。



## 参考
[apollo](https://github.com/ApolloAuto/apollo)  
[awesome-self-driving-cars](https://github.com/daohu527/awesome-self-driving-cars)  
