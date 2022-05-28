# Tools

> 君子生非异也，善假于物也。

## Table of Contents

- [mapviewers](#mapviewers)



## mapviewers
mapviewers用来可视化生成好的高精度地图。

1. 首先编译文件，这里可以把python文件编译为可执行文件。
```
bazel build //modules/tools/mapviewers:hdmapviewer
bazel build //modules/tools/mapviewers:gmapviewer
```

2. 编译好之后，执行命令。
```
./bazel-bin/modules/tools/mapviewers/hdmapviewer -m modules/map/data/demo/base_map.txt
```

3. 生成好的可视化地图文件在当前目录的base_map.html。

#### todo
这里gmapviewer和hdmapviewer的区别是什么？




