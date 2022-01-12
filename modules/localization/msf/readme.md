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
```
.
├── data_extraction
├── local_visualization
└── map_creation
```

## params
```
.
├── BUILD
├── gnss_params
├── novatel_localization_extrinsics.yaml
├── vehicle_params
└── velodyne_params
```

