# Dig into Apollo - MSF Localization ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout) 

## 目录
由于依赖单一的定位都会出现失效的情况，因此MSF融合了GPS、IMU和激光雷达3者输出定位信息，因此结果会比较鲁棒，相关的论文可以参考《Robust and Precise Vehicle Localization Based on Multi-Sensor Fusion in Diverse City Scenes》。 单个的定位信息的获取我们已经了解过了，例如GPS和IMU的RTK定位、激光雷达和IMU的NDT定位，而MSF则是融合（一般采用卡尔曼滤波）了2者的结果。最后定位都会面临现有地图还是先定位的问题，因此如何验证地图的准确性是值得研究和讨论的方向。  

MSF模块的整体目录结构如下。主要分为地图、工具和定位器。其中MSF的部分代码是采用so文件来提供的，不过好在我们可以看到接口定义，这部分头文件在`third_party/localization_msf/x86_64/include`中。

```
.
├── BUILD
├── common
├── local_integ
├── local_map
├── local_pyramid_map
├── local_tool
├── msf_localization.cc
├── msf_localization_component.cc
├── msf_localization_component.h
├── msf_localization.h
├── msf_localization_test.cc
├── params
└── README.md
```

## local_integ
MSF定位的功能主要在以下模块中实现，它的*入口函数*在`localization_integ.h`和`localization_integ.cc`中，*输入*为激光雷达消息、IMU消息、GPS消息和汽车heading消息，*输出*为融合后的定位信息。  
```
.
├── BUILD
├── gnss_msg_transfer.cc
├── gnss_msg_transfer.h
├── lidar_msg_transfer.cc
├── lidar_msg_transfer.h
├── localization_gnss_process.cc
├── localization_gnss_process.h
├── localization_integ.cc
├── localization_integ.h
├── localization_integ_impl.cc
├── localization_integ_impl.h
├── localization_integ_process.cc
├── localization_integ_process.h
├── localization_lidar.cc
├── localization_lidar.h
├── localization_lidar_process.cc
├── localization_lidar_process.h
├── localization_params.h
├── measure_republish_process.cc
├── measure_republish_process.h
├── online_localization_expert.cc
└── online_localization_expert.h
```

接下来我们开始按照执行过程分析代码。

#### localization_integ
`LocalizationInteg`的函数接口简单明了，分别对应了激光雷达消息、IMU消息、GPS消息和汽车heading消息的处理。这里唯一需要注意的时IMU和GPS有2个接口。

我们先看IMU的接口。这2个接口分别代表了FLU（前-左-天空）坐标系和RFU（右-前-天空）坐标系，需要根据坐标系调用不同的接口。
```c++
  void RawImuProcessFlu(const drivers::gnss::Imu &imu_msg);
  void RawImuProcessRfu(const drivers::gnss::Imu &imu_msg);
```
在看GPS的接口，其中`RawObservationProcess`和`RawEphemerisProcess`组合使用，效果类似于`GnssBestPoseProcess`。由于组合导航的差异选择不同的接口，这里默认采用`GnssBestPoseProcess`。
```c++
  // Gnss Info process.
  void RawObservationProcess(
      const drivers::gnss::EpochObservation &raw_obs_msg);
  void RawEphemerisProcess(const drivers::gnss::GnssEphemeris &gnss_orbit_msg);
  // gnss best pose process
  void GnssBestPoseProcess(const drivers::gnss::GnssBestPose &bestgnsspos_msg);
```

localization_integ调用的是`localization_integ_impl`接口，也就是说具体的功能实现在`localization_integ_impl`中。

#### localization_integ_impl
`localization_integ_impl`分为3个部分：
1. 处理消息（LocalizationGnssProcess, LocalizationLidarProcess）
2. 重新发布（MeasureRepublishProcess）
3. 融合过程（LocalizationIntegProcess）
4. 完善状态（OnlineLocalizationExpert）
也就是说先分别处理激光雷达、GPS、IMU等的消息，然后重新发布，转换为一种类型的消息（MeasureData），然后交给`LocalizationIntegProcess`做融合，最后完善融合结果的状态。


#### localization_gnss_process
求解GNSS的结果，求解器在`GnssSolver`中实现，头文件在`localization_msf/gnss_solver.h`中，以库文件的方式提供。

#### localization_lidar_process
求解Lidar的定位结果，和NDT定位类似，加入了反射率信息，实现在`localization_lidar_process`和`localization_lidar`中，也用到了Lidar求解器`LidarLocator`，同样头文件在`localization_msf/lidar_locator.h`中，以库文件的方式提供。

#### localization_integ_process
最后，我们最关注的融合过程，求解过程在`localization_msf/sins.h`中。

1. 在`StartThreadLoop`中处理之前放入`measure_data_queue_`的消息。
2. RawImuProcess中接收IMU的消息，并且计算融合之后的Pose。
```c++
  // add imu msg and get current predict pose
  sins_->AddImu(imu_msg);
  sins_->GetPose(&ins_pva_, pva_covariance_);
  sins_->GetRemoveBiasImu(&corrected_imu_);
  sins_->GetEarthParameter(&earth_param_);
```

#### measure_republish_process
重新发布消息，实际上是转换不同的消息到`MeasureData`。


#### online_localization_expert
主要完善各个定位的状态信息



## local_map
```
.
├── base_map
├── lossless_map
├── lossy_map
├── ndt_map
└── test_data
```

## local_pyramid_map
```
.
├── base_map
├── ndt_map
└── pyramid_map
```

## local_tool
`local_tool`包含了3个工具：解压数据、可视化定位结果和创建地图。
```
.
├── data_extraction       // 解压数据
├── local_visualization   // 本地可视化
└── map_creation          // 创建地图
```

#### local_visualization
可视化定位结果分为在线工具和离线工具，实际上只是消息的获取方式不一样，实现的原理都是一样，采用opencv绘制不同的定位结果和历史轨迹，方便进行分析。个人认为这一部分还可以优化，例如记录历史轨迹的方差大小，以及点云匹配差异的可视化展示等。


## params
`params`目录主要是存放一些参数，用于坐标转换，例如激光雷达到IMU的转换矩阵（激光雷达外参，通过标定获取），世界坐标到IMU的转换关系等。由于比较简单，这里就不赘述了。  
```
.
├── BUILD
├── gnss_params
├── novatel_localization_extrinsics.yaml
├── vehicle_params
└── velodyne_params
```

