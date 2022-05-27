<a name="lidar_module" />

## lidar

## app
app目录主要实现3个功能lidar_obstacle_detection，lidar_obstacle_segmentation，lidar_obstacle_tracking三个功能。  



## lib目录
整个激光雷达的处理流程是什么？？？ 先分割，找地面，然后找障碍物？？？

#### classifier

#### ground_detector


#### map_manager


#### object_builder


#### object_filter_bank


#### pointcloud_preprocessor


#### roi_filter
感兴趣区域过滤

#### scene_manager
场景管理？？？

#### segmentation
分割

#### tracker
追踪

## tools
tools目录主要有2个工具，一个是OfflineLidarObstaclePerception，另一个是msg_exporter_main，下面分别介绍这2个工具的作用。  

