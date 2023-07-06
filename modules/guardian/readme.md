# Dig into Apollo - Guardian ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)  

> 人不知而不愠，不亦君子乎。  

## Table of Contents



<a name="introduction" />
## 简介
Guardian模块的主要作用是监控自动驾驶系统状态，当出现模块为失败状态的时候，会主动切断控制命令输出，并且刹车。  

## 触发
guardian模块的触发条件主要有2个。  
1. 上报模块状态的消息间隔超过kSecondsTillTimeout（2.5秒）
2. 上报的状态消息中有safety_mode_trigger_time字段
这时候就会触发进入接管。  

## TriggerSafetyMode
安全模式的步骤分为2步骤，第一步状态消息中需要紧急刹车或者超声波检测到障碍物，如果检测到障碍物则说明车已经非常接近障碍物了，该检测被称为硬件触发的检测，因为已经发现模块故障，也非常接近障碍物所以刹车会加急。第二步是普通刹车，刹车没有那么急。 guardian为定时模块，所以该过程中会一直发送消息，直到车辆停车。  
当前版本代码屏蔽了上述超声波检测。  

## 问题
guardian模块的频率是10ms，因此最大会增加control命令的延时10ms。  
