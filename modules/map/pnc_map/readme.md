# Dig into Apollo - Pnc_map ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 事者，生于虑，成于务，失于傲。

## Table of Contents
- [Pnc_map地图简介](#introduction)


<a name="introduction" />

## Pnc_map地图简介
pnc地图主要是给规划模块来使用的模块，它的输入是自动驾驶车的当前位置和routing消息，它的输出是路径参考线。参考线的主要作用是给规划模块做参考，减少路径搜索的复杂度，因此pnc_map实际上并不是地图，而是为规划和控制模块提供的一个地图接口，方便根据routing路径结合地图生成参考线的一系列接口。

pnc_map的目录结构如下，其中`cuda_util`没有用到可以忽略。  
```
.
├── BUILD
├── cuda_util.cu
├── cuda_util.h
├── cuda_util_test.cc   // 没有用到
├── path.cc
├── path.h            // 路径等数据结构
├── path_test.cc
├── pnc_map.cc
├── pnc_map.h         // pnc_map地图接口
├── pnc_map_test.cc
├── route_segments.cc
├── route_segments.h    // 规划段数据结构
├── route_segments_test.cc
└── testdata            // 测试数据
    └── sample_sunnyvale_loop_routing.pb.txt
```

接下来我们从pnc_map地图接口开始分析。

## pnc_map地图
我们先分析pnc_map地图的接口。  
```c++
  // 构造函数，获取hdmap赋值给内部变量hdmap_
  explicit PncMap(const HDMap *hdmap);
  // 更新规划回复
  bool UpdateRoutingResponse(const routing::RoutingResponse &routing_response);
  // 向前查看的距离，要么是FLAGS_look_forward_long_distance，要么是FLAGS_look_forward_short_distance
  static double LookForwardDistance(const double velocity);
  // 获取规划段
  bool GetRouteSegments(const common::VehicleState &vehicle_state,
                        const double backward_length,
                        const double forward_length,
                        std::list<RouteSegments> *const route_segments);
  // 获取规划段 使用启发式前向长度和后向长度
  bool GetRouteSegments(const common::VehicleState &vehicle_state,
                        std::list<RouteSegments> *const route_segments);
  // 检查routing是否和PncMap中的一致
  bool IsNewRouting(const routing::RoutingResponse &routing_response) const;
  static bool IsNewRouting(const routing::RoutingResponse &prev,
                           const routing::RoutingResponse &routing_response);
  // 扩展规划段
  bool ExtendSegments(const RouteSegments &segments,
                      const common::PointENU &point, double look_forward,
                      double look_backward, RouteSegments *extended_segments);

  bool ExtendSegments(const RouteSegments &segments, double start_s,
                      double end_s,
                      RouteSegments *const truncated_segments) const;
  // 返回接下来的routing路径点
  std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const;
```
