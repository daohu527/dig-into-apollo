# Dig into Apollo - Questions ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 非淡漠无以明德，非宁静无以致远。

## 目录

- [build](#build)
- [map](#map)
- [simulation](#simulation)
- [planning](#planning)


<a name="build" />

## build

<a name="map" />

## map


<a name="simulation" />

## simulation


<a name="planning" />

## Planning

1. How to add decider or optimizer to a planning scenario ?

* Add your own decider in "modules/planning/tasks/deciders"
* Add config in "modules/planning/conf/scenario/lane_follow_config.pb.txt"
* Add TaskType in "modules/planning/proto/planning_config.proto"
* Register your task in "TaskFactory::Init"