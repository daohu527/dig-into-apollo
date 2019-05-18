1. 为什么会出现找不到节点的情况？？？
节点没有连接，所以需要找到节点附近最近的路段。

2. Astar算法
http://www-cs-students.stanford.edu/~amitp/gameprog.html#Paths  

http://theory.stanford.edu/~amitp/GameProgramming/  

https://www.geeksforgeeks.org/a-search-algorithm/  

http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html  


3. "RoutingComponent"类继承至"cyber::Component"，并且申明为"public"方式，"cyber::Component"是一个模板类，它定义了"Initialize"和"Process"方法。  
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


## GetWayNodes
```
bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }
    way_nodes->push_back(cur_node);
    way_s->push_back(point.s());
  }
  return true;
}
```

## SearchRoute

```
bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  ...
  // 初始化规划点和起点
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    return false;
  }
  // 根据节点和起点，查找返回结果，注意这里返回的是一段范围
  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    return false;
  }
  if (result_nodes.empty()) {
    return false;
  }
  // 插入起点和终点
  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());
  // 生成通道区域
  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    return false;
  }
  ...
}
```

## FillLaneInfoIfMissing
```
RoutingRequest Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  RoutingRequest fixed_request(routing_request);
  // 遍历routing请求的点
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    const auto& lane_waypoint = routing_request.waypoint(i);
    // routing_request请求的点有lane_id，则表示在路上，不用查找
    if (lane_waypoint.has_id()) {
      continue;
    }
    auto point = common::util::MakePointENU(lane_waypoint.pose().x(),
                                            lane_waypoint.pose().y(),
                                            lane_waypoint.pose().z());

    double s = 0.0;
    double l = 0.0;
    hdmap::LaneInfoConstPtr lane;
    // FIXME(all): select one reasonable lane candidate for point=>lane
    // is one to many relationship.
    // 找到当前点最近的lane信息
    if (hdmap_->GetNearestLane(point, &lane, &s, &l) != 0) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return routing_request;
    }
    auto waypoint_info = fixed_request.mutable_waypoint(i);
    waypoint_info->set_id(lane->id().id());
    waypoint_info->set_s(s);
  }
  return fixed_request;
}
```


