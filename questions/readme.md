# Dig into Apollo - Questions ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 非淡漠无以明德，非宁静无以致远。

## 目录

- [build](#build)
- [map](#map)
- [simulation](#simulation)
- [planning](#planning)
- [misc](#misc)


<a name="build" />

## build

<a name="map" />

## map
* How to change map.bin to human-readable map.txt?
In apollo docker run below cmds.
```
source scripts/apollo_base.sh

protoc --decode apollo.hdmap.Map modules/map/proto/map.proto < modules/map/data/sunnyvale_loop/base_map.bin > base_map.txt
```


<a name="simulation" />

## simulation


<a name="planning" />

## Planning

* How to add decider or optimizer to a planning scenario ?

1. Add your own decider in "modules/planning/tasks/deciders"
2. Add config in "modules/planning/conf/scenario/lane_follow_config.pb.txt"
3. Add TaskType in "modules/planning/proto/planning_config.proto"
4. Register your task in "TaskFactory::Init"


<a name="misc" />

## misc

* What is the format of the config file in Apollo?  
The config file base by **protobuf** format, and read by `cyber::common::GetProtoFromFile()` method.

* How to open the debug log?

1. modify the "apollo/cyber/setup.bash"
```
 # for DEBUG log 
 export GLOG_v=4 
```  
2. Enable environment variables
```
source cyber/setup.bash
```

3. Export bag data to lidar,camera
```
./bazel-bin/modules/localization/msf/local_tool/data_extraction/cyber_record_parser --bag_file data/bag/20210305145950.record.00000 --out_folder data/  --cloud_topic=/apollo/sensor/lidar32/compensator/PointCloud2
```
