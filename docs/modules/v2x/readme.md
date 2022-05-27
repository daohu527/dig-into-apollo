# Dig into Apollo - V2X ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)  

> 业精于勤，荒于嬉；行成于思，毁于随。  

## Table of Contents
- [v2x目录结构](#introduction)  
- [v2x_proxy](#v2x_proxy)
  - [app](#app)
  - [TrafficLightTimer](#trafficlight_timer)
  - [OnV2xCarStatusTimer](#onv2xcar_timer)
- [OBU接口(ObuInterFaceGrpcImpl)](#obu_interface)
  - [远程调用服务(grpc_interface)](#grpc_interface)
- [系统接口(OsInterFace)](#os_interface)
  - [SendMsgToOs](#send_msg_to_os)
  - [GetMsgFromOs](#get_msg_from_os)




<a name="introduction" />

## v2x目录结构
v2x的目录结构如下。  
```
.
├── BUILD        // 编译
├── common       // 公共目录
├── conf         // 配置
├── launch       
├── proto        // protobuf文件
└── v2x_proxy    // v2x代理
```

主要的实现在"v2x_proxy"中，无人驾驶车主要是OBU单元，和路侧RSU单元进行交互。  


<a name="v2x_proxy" />

## v2x_proxy
v2x_proxy的目录结构如下。  
```
.
├── app
├── obu_interface
└── os_interface
```

<a name="app" />

#### app
v2x模块的入口函数在"app/main.cc"中，在主函数中读取参数并且初始化v2x proxy。在"v2x_proxy.h"和"v2x_proxy.cc"中实现了"V2xProxy"类。  

1. 初始化  
首先我们看"V2xProxy"的初始化过程。
```c++
V2xProxy::V2xProxy(std::shared_ptr<::apollo::hdmap::HDMap> hdmap)
    : node_(::apollo::cyber::CreateNode("v2x_proxy")), exit_(false) {
  internal_ = std::make_shared<InternalData>();
  // 1. 读取高精度地图
  hdmap_ = std::make_shared<::apollo::hdmap::HDMap>();
  const auto hdmap_file = apollo::hdmap::BaseMapFile();
  // 2. 
  ::apollo::cyber::TimerOption v2x_car_status_timer_option;
  v2x_car_status_timer_option.period =
      static_cast<uint32_t>((1000 + FLAGS_v2x_car_status_timer_frequency - 1) /
                            FLAGS_v2x_car_status_timer_frequency);
  v2x_car_status_timer_option.callback = [this]() {
    this->OnV2xCarStatusTimer();
  };
  v2x_car_status_timer_option.oneshot = false;
  v2x_car_status_timer_.reset(
      new ::apollo::cyber::Timer(v2x_car_status_timer_option));
  os_interface_.reset(new OsInterFace());
  obu_interface_.reset(new ObuInterFaceGrpcImpl());
  recv_thread_.reset(new std::thread([this]() {
    while (!exit_.load()) {
      this->RecvTrafficlight();
    }
  }));

  planning_thread_.reset(new std::thread([this]() {
    while (!exit_.load()) {
      this->RecvOsPlanning();
    }
  }));
  obs_thread_.reset(new std::thread([this]() {
    while (!exit_.load()) {
      std::shared_ptr<::apollo::v2x::V2XObstacles> obs = nullptr;
      this->obu_interface_->GetV2xObstaclesFromObu(&obs);  // Blocked
      this->os_interface_->SendV2xObstacles2Sys(obs);
    }
  }));
  v2x_car_status_timer_->Start();
  // 从文件获取获取RSU列表
  GetRsuListFromFile(FLAGS_rsu_whitelist_name, &rsu_list_);
  init_flag_ = true;
}
```
可以看到"V2xProxy"初始化了2个定时器，一个是RSU发送给车的红绿灯信息，一个是主动上报的车辆状态信息，另外还初始化了os接口和obu接口。  


Apollo 6.0中又增加了几种消息,并且通过定时器发布,下面我们来看V2xProxy中包含几种定时器.
1. v2x_car_status_timer_ - OnV2xCarStatusTimer
2. obu_status_timer_ - 
3. rsu_whitelist_timer_ - 

几个线程
1. recv_thread_    RecvTrafficlight
2. planning_thread_   RecvOsPlanning
3. rsi_thread_        目前没有使用
4. obs_thread_        GetV2xObstaclesFromObu  ->  SendV2xObstacles2Sys


## OnV2xCarStatusTimer
发送车的信息到OBU->RSU
GetRsuInfo -> SendCarStatusToObu


## RecvTrafficlight
接收OBU的红绿灯消息,并且进行处理
```c++
void V2xProxy::RecvTrafficlight() {
  // get traffic light from obu
  std::shared_ptr<ObuLight> x2v_traffic_light = nullptr;
  // 1. 从OBU获取红绿灯状态,并且发送给Apollo
  obu_interface_->GetV2xTrafficLightFromObu(&x2v_traffic_light);
  os_interface_->SendV2xObuTrafficLightToOs(x2v_traffic_light);
  auto os_light = std::make_shared<OSLight>();
  std::string junction_id = "";
  {
    std::lock_guard<std::mutex> lg(lock_hdmap_junction_id_);
    junction_id = hdmap_junction_id_;
  }
  bool res_success_ProcTrafficlight = internal_->ProcTrafficlight(
      hdmap_, x2v_traffic_light.get(), junction_id, u_turn_,
      FLAGS_traffic_light_distance, FLAGS_check_time, &os_light);
  if (!res_success_ProcTrafficlight) {
    return;
  }
  utils::UniqueOslight(os_light.get());
  // 3. 发送红绿灯消息到HMI???
  os_interface_->SendV2xTrafficLightToOs(os_light);
  // save for hmi
  std::lock_guard<std::mutex> lock(lock_last_os_light_);
  ts_last_os_light_ = ::apollo::cyber::Time::MonoTime().ToMicrosecond();
  last_os_light_ = os_light;
}
```

## RecvOsPlanning
获取planning路线,并且根据红绿灯的剩余时间来调整线路?
```c++
void V2xProxy::RecvOsPlanning() {
  auto adc_trajectory = std::make_shared<::apollo::planning::ADCTrajectory>();
  auto res_light =
      std::make_shared<::apollo::perception::TrafficLightDetection>();
  os_interface_->GetPlanningAdcFromOs(adc_trajectory);
  // OK get planning message
  std::shared_ptr<OSLight> last_os_light = nullptr;
  {
    std::lock_guard<std::mutex> lock(lock_last_os_light_);

    auto now_us = ::apollo::cyber::Time::MonoTime().ToMicrosecond();
    if (last_os_light_ == nullptr ||
        2000LL * 1000 * 1000 < now_us - ts_last_os_light_) {
      AWARN << "V2X Traffic Light is too old!";
      last_os_light_ = nullptr;
    } else {
      ADEBUG << "V2X Traffic Light is on time.";
      last_os_light = std::make_shared<OSLight>();
      last_os_light->CopyFrom(*last_os_light_);
    }
  }
  // proc planning message
  bool res_proc_planning_msg = internal_->ProcPlanningMessage(
      adc_trajectory.get(), last_os_light.get(), &res_light);
  if (!res_proc_planning_msg) {
    return;
  }
  os_interface_->SendV2xTrafficLight4Hmi2Sys(res_light);
}
```

## obs_thread_
获取OBU发布的障碍物信息,然后发送到OS



<a name="trafficlight_timer" />

#### TrafficLightTimer
交通灯的定时器会定时回调"OnX2vTrafficLightTimer"，下面我们看定时回调里面执行了什么？  
```c++
void V2xProxy::OnX2vTrafficLightTimer() {
  x2v_trafficlight_->Clear();
  // 1. 从obu接口中获取红绿灯的状态
  obu_interface_->GetV2xTrafficLightFromObu(x2v_trafficlight_);
  if (!x2v_trafficlight_->has_current_lane_trafficlight()) {
    AERROR << "Error:v2x trafficlight ignore, no traffic light contained.";
    return;
  }
  // 2. 当前红绿灯状态
  auto current_traff = x2v_trafficlight_->mutable_current_lane_trafficlight();
  if (current_traff->single_traffic_light().empty()) {
    AERROR << "Error:v2x trafficlight ignore, no traffic light contained.";
    return;
  }
  ADEBUG << x2v_trafficlight_->DebugString();
  // 3. 执行红绿灯逻辑
  if (!TrafficLightProc(current_traff)) {
    return;
  }
  // 4. 发送红绿灯状态到OS接口
  os_interface_->SendV2xTrafficLightToOs(x2v_trafficlight_);
}
```
下面是红绿灯的处理过程。先根据接收到的坐标信息查找前面一定距离的所有红绿灯，然后把当前范围内的所有红绿灯改为接收到的颜色。该过程可能较少的考虑到一些逻辑，估计后面会继续完善。  
```c++
bool V2xProxy::TrafficLightProc(CurrentLaneTrafficLight* msg) {
  // 1. 获取rsu发送的红绿灯坐标
  apollo::common::PointENU point;
  point.set_x(msg->gps_x_m());
  point.set_y(msg->gps_y_m());
  std::vector<apollo::hdmap::SignalInfoConstPtr> signals;
  // 2. 获取当前范围内所有的红绿灯
  if (hdmap_->GetForwardNearestSignalsOnLane(point, 1000.0, &signals) != 0) {
    AERROR << "Error::v2x trafficlight ignore, hdmap get no signals";
    AERROR << "traffic light size : " << signals.size();
    return false;
  }
  // 3. 如果只有一个信号灯，则设置id，并且返回
  if (signals.size() == 1) {
    auto single = msg->mutable_single_traffic_light(0);
    single->set_id(signals[0]->id().id());
    return true;
  }
  // 4. 如果有多个信号灯，则把所有的信号灯设置为发送的颜色
  auto color = msg->single_traffic_light(0).color();
  msg->clear_single_traffic_light();

  for (auto i = signals.begin(); i != signals.end(); i++) {
    auto single = msg->add_single_traffic_light();
    single->set_id((*i)->id().id());
    single->set_color(color);
  }
  return true;
}
```
疑问：  
1. 按照代码RSU只发送了一个信号灯状态，并且RSU没有高精度地图信息，不知道无人车中高精度地图的signal_id，所以一方面要填充signal_id信息，一方面找到多个红绿灯时候，需要把所有的红绿灯信息都修改为发送的状态。  
2. RSU应该发送多个红绿灯的状态，而不仅仅只发送一个，这样就需要RSU侧也需要高精度地图，并且和车的高精度信息要同步。  


<a name="onv2xcar_timer" />

#### OnV2xCarStatusTimer
发送本车的状态到RSU。  
```c++
void V2xProxy::OnV2xCarStatusTimer() {
  v2x_carstatus_->Clear();
  auto localization = std::make_shared<LocalizationEstimate>();
  // 1. 获取本车的位置信息
  os_interface_->GetLocalizationFromOs(localization);
  if (!localization || !localization->has_header() ||
      !localization->has_pose()) {
    AERROR << "Error:localization ignore, no pose or header in it.";
    return;
  }
  // 2. 发送当前的位置信息到OBU
  v2x_carstatus_->mutable_localization()->CopyFrom(*localization);
  obu_interface_->SendCarStatusToObu(v2x_carstatus_);
}
```

通过上述分析，我们可以清晰的了解到"V2xProxy"实际上相当于一个桥梁，通过"os_interface_"获取车的信息，通过"obu_interface_"发送消息。下面是流程图。  
![v2x_proccess](img/v2x_proccess.jpg)  


<a name="obu_interface" />

## OBU接口(ObuInterFaceGrpcImpl)
OBU实际上是车和RSU的桥梁，当前OBU可能和车是单独的设备通过网络连接的，所以这里通过grpc实现调用。 主要实现了从OBU发送和接收障碍物信息，红绿灯信息。ObuInterFaceBase是纯虚类，定义了和OBU通信的接口。  
```c++
class ObuInterFaceGrpcImpl : public ObuInterFaceBase {
 public:
  ObuInterFaceGrpcImpl();
  ~ObuInterFaceGrpcImpl();
  // 1. 初始化grpc服务端
  bool InitialServer() override;
  // 2. 初始化grpc客户端
  bool InitialClient() override;

  // 3. 从OBU获取障碍物信息
  void GetV2xObstaclesFromObu(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg)
      override;
  // 4. 从OBU获取红绿灯信息
  void GetV2xTrafficLightFromObu(
      const std::shared_ptr<IntersectionTrafficLightData> &msg) override;

  // 5. 发送车的状态到OBU
  void SendCarStatusToObu(const std::shared_ptr<CarStatus> &msg) override;

  // 6. 发送障碍物信息到OBU
  void SendObstaclesToObu(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg)
      override;
};
```

ObuInterFaceGrpcImpl中创建了一个grpc客户端和服务端，服务端监听OBU发送过来的消息，并且保存。grpc客户端则发送消息到OBU。  


<a name="grpc_interface" />

#### 远程调用服务(grpc_interface)
主要实现了grpc的客户端和服务端，后面看下grpc的介绍之后再详细介绍。  
1. 其中GrpcServerImpl提供rpc服务，当OBU发送请求获取障碍物信息时候，返回无人车感知到的障碍物信息，反之同理。（OBU提供请求）  
2. GrpcClientImpl向OBU发出请求，获取红绿灯和障碍物信息。（OBU提供grpc服务）  

上述过程中无人车客户端会启动2个定时器，通过rpc客户端去获取OBU提供的红绿灯和障碍物信息，这里又回到之前的问题，为什么需要红绿灯做同步？如果无人车和OBU的检测不一致，那么理论上应该听谁的？  



<a name="os_interface" />

## 系统接口(OsInterFace)
OsInterFace中实现了2个模板，分别接收和发布消息给apollo，下面我们主要看下发布和订阅函数。


<a name="send_msg_to_os" />

#### SendMsgToOs
发布函数非常简单，就是通过reader发送指定的topic，**需要注意一定要对RSU发布的消息做融合了之后才能输出**，如果不做融合，一个简单的例子如果RSU发布的障碍物apollo没有看到，当RSU发布之后，Apollo的感知如果不做融合，下次发送的是apollo自己的感知结果，会出现一帧的障碍物存在，而下一帧不存在的情况，因此要对结果做融合。 另一个疑问是如何保证融合的时间戳一致，因为RSU的频率可能和激光雷达的时间戳不一致。  
```c++
  template <typename MessageT>
  void SendMsgToOs(cyber::Writer<MessageT> *writer,
                   const std::shared_ptr<MessageT> &msg) {
    if (writer == nullptr) {
      AERROR << "Writer in not valid";
      return;
    }
    // 1. 发送消息
    if (writer->Write(msg) == true) {
      ADEBUG << "Write msg success to: " << writer->GetChannelName();
    } else {
      AERROR << "Write msg failed to: " << writer->GetChannelName();
    }
  }
```

<a name="get_msg_from_os" />

#### GetMsgFromOs
从Apollo系统接收消息，这部分的消息接收没有采用事件驱动的方式，而是采用定时发布的方式。  
```c++
  template <typename MessageT>
  void GetMsgFromOs(const cyber::Reader<MessageT> *reader,
                    const std::shared_ptr<MessageT> &msg) {
    node_->Observe();
    if (reader->Empty()) {
      AINFO_EVERY(100) << "Has not received any data from "
                       << reader->GetChannelName();
      return;
    }
    msg->CopyFrom(*(reader->GetLatestObserved()));
  }
```


## 感知模块
最后感知模块"trafficlights_perception_component.cc"会订阅"/apollo/v2x/traffic_light"这个TOPIC，然后把V2X获取到的结果放入buffer中再进行处理。  
```c++
int TrafficLightsPerceptionComponent::InitV2XListener() {
  typedef const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData>
      V2XTrafficLightsMsgType;
  std::function<void(const V2XTrafficLightsMsgType&)> sub_v2x_tl_callback =
      std::bind(&TrafficLightsPerceptionComponent::OnReceiveV2XMsg, this,
                std::placeholders::_1);
  auto sub_v2x_reader = node_->CreateReader(
      v2x_trafficlights_input_channel_name_, sub_v2x_tl_callback);
  return cyber::SUCC;
}
```


## fusion模块
Apollo6.0在v2x中新增加了fusion模块，fusion模块的输入是"/perception/vehicle/obstacles"，输出也是"/apollo/perception/obstacles".接收的是感知模块输出的感知信息,输出融合之后的障碍物信息.  
这个模块的启动也在感知模块的"dag_streaming_perception.dag"中,也就是说V2X模块的感知融合可能会合入感知模块中.

输入:
/perception/vehicle/obstacles
/apollo/v2x/obstacles
/apollo/localization/pose

输出:
/apollo/perception/obstacles

V2XFusionComponent模块的处理过程主要在"V2XMessageFusionProcess"中.
```c++
bool V2XFusionComponent::V2XMessageFusionProcess(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  // 1. 读取最新的位置
  localization_reader_->Observe();
  auto localization_msg = localization_reader_->GetLatestObserved();
  base::Object hv_obj;
  CarstatusPb2Object(*localization_msg, &hv_obj, "VEHICLE");
  
  v2x_obstacles_reader_->Observe();
  auto v2x_obstacles_msg = v2x_obstacles_reader_->GetLatestObserved();
  // 2. 读取v2x的感知信息,如果没有,则直接输出感知模块的结果
  if (v2x_obstacles_msg == nullptr) {
    AERROR << "V2X: cannot receive any v2x obstacles message.";
    perception_fusion_obstacles_writer_->Write(*perception_obstacles);
  } else {
    header_.CopyFrom(perception_obstacles->header());
    std::vector<Object> fused_objects;
    std::vector<Object> v2x_fused_objects;
    std::vector<std::vector<Object>> fusion_result;
    std::vector<Object> v2x_objects;
    // 3. 转换v2x感知消息为对象,转换感知模块消息为对象
    V2xPbs2Objects(*v2x_obstacles_msg, &v2x_objects, "V2X");
    std::vector<Object> perception_objects;
    Pbs2Objects(*perception_obstacles, &perception_objects, "VEHICLE");
    perception_objects.push_back(hv_obj);
    
    // 4. 合并新资源
    fusion_.CombineNewResource(perception_objects, &fused_objects,
                               &fusion_result);
    fusion_.CombineNewResource(v2x_objects, &fused_objects, &fusion_result);
    // 5. 获取v2x融合对象
    fusion_.GetV2xFusionObjects(fusion_result, &v2x_fused_objects);
    // 6. 发送消息
    auto output_msg = std::make_shared<PerceptionObstacles>();
    SerializeMsg(v2x_fused_objects, output_msg);
    perception_fusion_obstacles_writer_->Write(*output_msg);
  }
  return true;
}
```

## Fusion类
Fusion类的构造函数.
```c++
Fusion::Fusion() {
  ft_config_manager_ptr_ = FTConfigManager::Instance();
  // 读取score params, 参数在"fusion_params.pt"中
  score_params_ = ft_config_manager_ptr_->fusion_params_.params.score_params();
  switch (score_params_.confidence_level()) {
    case fusion::ConfidenceLevel::C90P:
      m_matched_dis_limit_ = std::sqrt(4.605);
      break;
    case fusion::ConfidenceLevel::C95P:
      m_matched_dis_limit_ = std::sqrt(5.991);
      break;
    case fusion::ConfidenceLevel::C975P:
      m_matched_dis_limit_ = std::sqrt(7.378);
      break;
    case fusion::ConfidenceLevel::C99P:
      m_matched_dis_limit_ = std::sqrt(9.210);
      break;
    default:
      break;
  }
}
```
fusion_params.pt中的参数是,可以看到"confidence_level"为C99P则m_matched_dis_limit_为"std::sqrt(9.210)".
```
score_params {
  prob_scale: 0.125
  max_match_distance: 10
  min_score: 0
  use_mahalanobis_distance: true
  check_type: false
  confidence_level: C99P
}
```

那么我们看下CombineNewResource的实现.
```c++
bool Fusion::CombineNewResource(
    const std::vector<base::Object> &new_objects,
    std::vector<base::Object> *fused_objects,
    std::vector<std::vector<base::Object>> *fusion_result) {
  // 1. 如果fused_objects为空,则直接添加
  if (fused_objects->size() < 1) {
    fused_objects->assign(new_objects.begin(), new_objects.end());
    for (unsigned int j = 0; j < new_objects.size(); ++j) {
      std::vector<base::Object> matched_objects;
      matched_objects.push_back(new_objects[j]);
      fusion_result->push_back(matched_objects);
    }
    return true;
  }
  int u_num = fused_objects->size();
  int v_num = new_objects.size();
  Eigen::MatrixXf association_mat(u_num, v_num);
  // 2. 计算关联矩阵
  ComputeAssociateMatrix(*fused_objects, new_objects, &association_mat);
  std::vector<std::pair<int, int>> match_cps;
  // 3. 采用km_matcher_进行匹配
  if (u_num > v_num) {
    km_matcher_.GetKMResult(association_mat.transpose(), &match_cps, true);
  } else {
    km_matcher_.GetKMResult(association_mat, &match_cps, false);
  }
  // 4. 融合结果
  for (auto it = match_cps.begin(); it != match_cps.end(); it++) {
    if (it->second != -1) {
      if (it->first == -1) {
        fused_objects->push_back(new_objects[it->second]);
        std::vector<base::Object> matched_objects;
        matched_objects.push_back(fused_objects->back());
        fusion_result->push_back(matched_objects);
      } else {
        (*fusion_result)[it->first].push_back(new_objects[it->second]);
      }
    }
  }
  return true;
}
```

计算关联矩阵,先计算距离分数,再计算类型分数
```c++
bool Fusion::ComputeAssociateMatrix(
    const std::vector<base::Object> &in1_objects,  // fused
    const std::vector<base::Object> &in2_objects,  // new
    Eigen::MatrixXf *association_mat) {
  for (unsigned int i = 0; i < in1_objects.size(); ++i) {
    for (unsigned int j = 0; j < in2_objects.size(); ++j) {
      const base::Object &obj1_ptr = in1_objects[i];
      const base::Object &obj2_ptr = in2_objects[j];
      double score = 0;
      // 1. 计算距离分数
      if (!CheckDisScore(obj1_ptr, obj2_ptr, &score)) {
        AERROR << "V2X Fusion: check dis score failed";
      }
      // 2. 计算类型分数, 采用距离分数乘以类型系数
      if (score_params_.check_type() &&
          !CheckTypeScore(obj1_ptr, obj2_ptr, &score)) {
        AERROR << "V2X Fusion: check type failed";
      }
      (*association_mat)(i, j) =
          (score >= score_params_.min_score()) ? score : 0;
    }
  }
  return true;
}
```

## KMkernal
KM匹配算法.


## trans_tools
"trans_tools.cc"和"trans_tools.h"中主要是一些工具类,用来转换对象到proto和从proto到对象.
 
```c++
void Objects2Pbs(const std::vector<base::Object> &objects,
                 std::shared_ptr<PerceptionObstacles> obstacles) {
  obstacles->mutable_perception_obstacle()->Clear();
  if (objects.size() < 1) {
    return;
  }
  // obstacles->mutable_header()->set_frame_id(objects[0].frame_id);
  for (const auto &object : objects) {
    if (object.v2x_type == base::V2xType::HOST_VEHICLE) {
      continue;
    }
    PerceptionObstacle obstacle;
    Object2Pb(object, &obstacle);
    obstacles->add_perception_obstacle()->CopyFrom(obstacle);
  }
}
```


## V2x消息
2017年9月中旬，中国智能网联汽车产业创新联盟正式发布《合作式智能交通系统 车用通信系统应用层及应用数据交互标准》.该标准是一个应用层的标准,下载[链接](http://www.sae-china.org/download/1745/%E5%90%88%E4%BD%9C%E5%BC%8F%E6%99%BA%E8%83%BD%E8%BF%90%E8%BE%93%E7%B3%BB%E7%BB%9F+%E8%BD%A6%E7%94%A8%E9%80%9A%E4%BF%A1%E7%B3%BB%E7%BB%9F%E5%BA%94%E7%94%A8%E5%B1%82%E5%8F%8A%E5%BA%94%E7%94%A8%E6%95%B0%E6%8D%AE%E4%BA%A4%E4%BA%92%E6%A0%87%E5%87%86.pdf).

标准中规定了5大类消息:
* BSM - basic safety message
* MAP - map data
* RSM - road side safety message
* SPAT - signal phase and timing message
* RSI - road side information

目前Apollo中实现了RSI,SPAT,MAP 3种消息格式, RSM和BSM消息没有定义.Apollo中的BSM可能可以对应到CarStatus消息.每种消息的格式以及意义消息中都进行了明确的定义,并且对一些应用应该发什么消息,消息的频率和交互流程也做了定义.因此可以参考完成一些应用.
由于网联车辆还是一个比较新的领域,里面的一些流程可能不一定能够完全照搬,所以应该参考消息的用意,而具体的流程可以适当做一些修改.  





