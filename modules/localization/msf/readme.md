# Dig into Apollo - Localization ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout) 

## 目录
MSF模块的整体目录结构如下。
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

