# Dig into Apollo - Routing ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 青，取之于蓝而青于蓝；冰，水为之而寒于水。


## Table of Contents
- [Routing模块简介](#introduction)
- [Routing模块分析](#routing)
  - [Planning整个流程](#planning_flow)
  

<a name="introduction" />


## Routing模块简介
Routing类似于现在开车时用到的导航模块，通常考虑的是起点到终点的最优路径（通常是最短路径），和Planning的区别是Routing考虑的是起点到终点的最短路径，而Planning则是行驶过程中，当前一小段时间如何行驶，需要考虑当前路况，是否有障碍物。Routing模块则不需要考虑这些信息，只需要做一个长期的规划路径即可，过程如下：  

![introduction](https://github.com/daohu527/Dig-into-Apollo/blob/master/routing/introduction.png)  

这也和我们开车类似，上车之后，首先搜索目的地，打开导航（Routing所做的事情），而开始驾车之后，则会根据当前路况，行人车辆信息来适当调整直到到达目的地（Planning所做的事情）。
* **Routing** - 主要关注起点到终点的长期路径，根据起点到终点之间的道路，选择一条最优路径。  
* **Planning** - 主要关注几秒钟之内汽车的行驶路径，根据当前行驶过程中的交通规则，车辆行人等信息，规划一条短期路径。  

下面我们开始分析Apollo Routing模块的代码流程。

<a name="routing" />

## routing模块分析

首先我们从"routing_component.h"和"routing_component.cc"开始，apollo的功能被划分为各个模块，启动时候由cyber框架根据模块间的依赖顺序加载(每个模块的dag文件定义了依赖顺序)，所以开始查看一个模块时，都是从component文件开始。  
可以看到"RoutingComponent"继承至"cyber::Component"，并且申明为"public"继承方式，"cyber::Component"是一个模板类，它定义了"Initialize"和"Process"方法。而"Proc"为纯虚函数由子类实现。  
```
template <typename M0>
class Component<M0, NullType, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg);

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg) = 0;
};
```
// todo 模板方法中为虚函数，而继承类中为公有方法？为什么？


```
class RoutingComponent final
    : public ::apollo::cyber::Component<RoutingRequest> {
 public:
  // default用来控制默认构造函数的生成。显式地指示编译器生成该函数的默认版本。
  RoutingComponent() = default;
  ~RoutingComponent() = default;

 public:
  // 初始化 todo 从哪里override?
  bool Init() override;
  // 收到routing request的时候触发执行
  bool Proc(const std::shared_ptr<RoutingRequest>& request) override;

 private:
  // 申明routing请求发布
  std::shared_ptr<::apollo::cyber::Writer<RoutingResponse>> response_writer_ =
      nullptr;
  std::shared_ptr<::apollo::cyber::Writer<RoutingResponse>>
      response_history_writer_ = nullptr;
  // Routing类
  Routing routing_;
  std::shared_ptr<RoutingResponse> response_ = nullptr;
  // 定时器
  std::unique_ptr<::apollo::cyber::Timer> timer_;
  // 锁
  std::mutex mutex_;
};

// 在cyber框架中注册routing模块
CYBER_REGISTER_COMPONENT(RoutingComponent)
```
我们先看下"Init"函数:  
```
bool RoutingComponent::Init() {
  // 设置消息qos，控制流量，创建消息发布response_writer_
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_routing_response_topic);
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  response_writer_ = node_->CreateWriter<RoutingResponse>(attr);

  ...
  // 设置消息qos，创建历史消息发布，和response_writer_类似
  response_history_writer_ = node_->CreateWriter<RoutingResponse>(attr_history);
  
  // todo 启动定时器，发布历史消息，todo，为什么要赋值，并且保证锁？
  std::weak_ptr<RoutingComponent> self =
      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
  timer_.reset(new ::apollo::cyber::Timer(
      FLAGS_routing_response_history_interval_ms,
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          std::lock_guard<std::mutex> guard(this->mutex_);
          if (this->response_.get() != nullptr) {
            auto response = *response_;
            auto timestamp = apollo::common::time::Clock::NowInSeconds();
            response.mutable_header()->set_timestamp_sec(timestamp);
            this->response_history_writer_->Write(response);
          }
        }
      },
      false));
  timer_->Start();

  // routing模块初始化和启动是否成功，todo routing_在哪里实例化？
  return routing_.Init().ok() && routing_.Start().ok();
}
```

接下来看"Proc"实现了哪些功能:  
```
bool RoutingComponent::Proc(const std::shared_ptr<RoutingRequest>& request) {
  auto response = std::make_shared<RoutingResponse>();
  // 响应routing_请求
  if (!routing_.Process(request, response.get())) {
    return false;
  }
  // 填充响应头部信息，并且发布
  common::util::FillHeader(node_->Name(), response.get());
  response_writer_->Write(response);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    response_ = std::move(response);
  }
  return true;
}
```
从上面的分析可以看出，"RoutingComponent"模块实现的主要功能:  
1. 实现"Init"和"Proc"函数
2. 接收"RoutingRequest"消息，输出"RoutingResponse"响应。

接下来我们来看routing的具体实现。  
#### routing
"Routing"类的实现在"routing.h"和"routing.cc"中，首先看下"Routing"类引用的头文件：  
```
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/core/navigator.h"
#include "modules/routing/proto/routing_config.pb.h"
```
看代码之前先看下头文件是个很好的习惯。通过头文件，我们可以知道当前模块的依赖项，从而搞清楚各个模块之间的依赖关系。可以看到"Routing"模块是一个相对比较独立的模块，只依赖于地图。  
Routing类的实现：  
```
class Routing {
 public:
  Routing();
  // 初始化
  apollo::common::Status Init();
  // 启动
  apollo::common::Status Start();
  // 执行
  bool Process(const std::shared_ptr<RoutingRequest> &routing_request,
               RoutingResponse *const routing_response);

 private:
  // 导航
  std::unique_ptr<Navigator> navigator_ptr_;
  // routing模块配置
  RoutingConfig routing_conf_;
  // 高精度地图，用来获取高精度地图信息
  const hdmap::HDMap *hdmap_ = nullptr;
};
```
下面看下具体的实现"routing.cc":  
```

```




## routing for osm
https://wiki.openstreetmap.org/wiki/Routing

http://www.patrickklose.com/posts/parsing-osm-data-with-python/

城市道路分析：
https://geoffboeing.com/2016/11/osmnx-python-street-networks/
https://automating-gis-processes.github.io/2018/notebooks/L6/network-analysis.html

https://socialhub.technion.ac.il/wp-content/uploads/2017/08/revise_version-final.pdf

https://stackoverflow.com/questions/29639968/shortest-path-using-openstreetmap-datanodes-and-ways


## openstreetmap 查找节点
If it's a polygon, then it's a closed way in the OSM database. You can find ways by id as simple as this: http://www.openstreetmap.org/way/305293190

If a specific node (the building blocks of ways) is giving a problem, the link would be http://www.openstreetmap.org/node/305293190 .

If it is a multipolygon (for example a building with a hole in it), the link would be http://www.openstreetmap.org/relation/305293190


## 地图介绍
https://blog.csdn.net/scy411082514/article/details/7484497

## 地图下载
https://www.openstreetmap.org/export#map=15/22.5163/113.9380
