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
1. 编译提示下载依赖库失败，可以把下载下来的文件放到data/pkg，然后在`tools/bazel.rc`中指定bazel的查找路径，重新编译
```
build --distdir=data/pkg
```

<a name="map" />

## map
* How to change map.bin to human-readable map.txt?
In apollo docker run below cmds.
```
source scripts/apollo_base.sh

protoc --decode apollo.hdmap.Map modules/map/proto/map.proto < modules/map/data/sunnyvale_loop/base_map.bin > base_map.txt
```

* How to create new map with RTK
1. 解压制作好的地图
```
python modules/tools/map_gen/extract_path.py map_file data/bag/20210406112554.record.00000
```
查看当前轨迹
```
python modules/tools/map_gen/plot_path.py map_file
```

2. 生成地图
```
python modules/tools/map_gen/map_gen.py map_file
```

3. 查看地图
```
python modules/tools/mapshow/mapshow.py -m map_map_file.txt
```


生成地图
```
python modules/tools/create_map/convert_map_txt2bin.py -i map_map_file.txt -o /apollo/modules/map/data/your_map_dir/base_map.bin
```

生成sim_map
```
./bazel-bin/modules/map/tools/sim_map_generator -map_dir=/apollo/modules/map/data/your_map_dir -output_dir=/apollo/modules/map/data/your_map_dir
```

生成routing_map
```
/apollo/bazel-bin/modules/routing/topo_creator/topo_creator -map_dir=/apollo/modules/map/data/your_map_dir --flagfile=modules/routing/conf/routing.conf
```

测试之前需要修改"vi modules/common/data/global_flagfile.txt"，屏蔽选项"--log_dir/--use_navigation_mode"
```
--map_dir=/apollo/modules/map/data/your_map_dir
```
测试生成的routing_map是否可以联通
```
python modules/tools/routing/debug_topo.py
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
