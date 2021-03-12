# Dig into Apollo - Localization ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout) 

## map_creation
加载pcd pose文件
```c++
    apollo::localization::msf::velodyne::LoadPcdPoses(
        pose_files[i], &pcd_poses[i], &time_stamps[i], &pcd_indices[i]);
```


## 创建NDT地图
创建ndt地图
```c++
apollo::localization::msf::pyramid_map::NdtMap
apollo::localization::msf::pyramid_map::NdtMapNodePool
```

## 平面提取

```c++
apollo::localization::msf::FeatureXYPlane
```