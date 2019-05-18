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
BlackListRangeGenerator生成黑名单路段
通过RoutingRequest中的black_lane和black_road来生成黑名单路段，这里会设置一整段路都为黑名单。
```
void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  AddBlackMapFromLane(request, graph, range_manager);
  AddBlackMapFromRoad(request, graph, range_manager);
  range_manager->SortAndMerge();
}
```
通过Terminal来设置黑名单，应用场景是设置routing的起点和终点。这里的起点和终点都是一个点，功能是把lane切分为2个subNode
```
void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  double start_length = src_node->Length();
  double end_length = dest_node->Length();
  if (start_s < 0.0 || start_s > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }
  if (end_s < 0.0 || end_s > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  double start_cut_s = MoveSBackward(start_s, 0.0);
  range_manager->Add(src_node, start_cut_s, start_cut_s);
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);

  double end_cut_s = MoveSForward(end_s, end_length);
  range_manager->Add(dest_node, end_cut_s, end_cut_s);
  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  range_manager->SortAndMerge();
}
```
> TODO: 如果在dreamview里设置多个routing点的情况，那么会出现第一段的终点和第二段的起点有overlap的情况，排序之后，会出现2厘米的gap?目前看起来不会影响，因为会往前开，另外为什么要设置往后偏移1厘米，如果刚好停在边界点上，会如何处理？？？


TODO: 判断是否足够进行切换Lane, 
```
bool TopoNode::IsOutRangeEnough(const std::vector<NodeSRange>& range_vec,
                                double start_s, double end_s) {
  // 是否足够切换Lane
  if (!NodeSRange::IsEnoughForChangeLane(start_s, end_s)) {
    return false;
  }
  int start_index = BinarySearchForSLarger(range_vec, start_s);
  int end_index = BinarySearchForSSmaller(range_vec, end_s);

  int index_diff = end_index - start_index;
  if (start_index < 0 || end_index < 0) {
    return false;
  }
  if (index_diff > 1) {
    return true;
  }

  double pre_s_s = std::max(start_s, range_vec[start_index].StartS());
  double suc_e_s = std::min(end_s, range_vec[end_index].EndS());

  if (index_diff == 1) {
    double dlt = range_vec[start_index].EndS() - pre_s_s;
    dlt += suc_e_s - range_vec[end_index].StartS();
    return NodeSRange::IsEnoughForChangeLane(dlt);
  }
  if (index_diff == 0) {
    return NodeSRange::IsEnoughForChangeLane(pre_s_s, suc_e_s);
  }
  return false;
}
```


## Reference
城市道路分析：
https://geoffboeing.com/2016/11/osmnx-python-street-networks/
https://automating-gis-processes.github.io/2018/notebooks/L6/network-analysis.html
https://socialhub.technion.ac.il/wp-content/uploads/2017/08/revise_version-final.pdf
https://stackoverflow.com/questions/29639968/shortest-path-using-openstreetmap-datanodes-and-ways

