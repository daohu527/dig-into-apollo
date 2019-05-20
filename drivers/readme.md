# Dig into Apollo - Perception ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

> 一叶遮目，不见泰山


## Table of Contents
- [Canbus](#canbus)
  - [Radar](#radar)
- [Camera](#camera)


## Canbus
由于汽车采用的总线是canbus总线，因此汽车采用大部分的器件都是canbus，其中Radar，线控系统采用的就是canbus总线，当然canbus总线也有缺点，就是带宽不够，所以Camera采用的就是USB总线，个人认为后面汽车可能会有新的总线出现，一满足目前的高带宽，二能满足可扩展；另外的一个发展方向就是各个模块的集成化发展，比如以后摄像头和Radar一样都有单独的硬件处理数据之后上报。  
那么我们在看下radar的驱动实际上依赖canbus，只要装好canbus的驱动，radar就可以收发数据了，radar的驱动只需要实现数据格式的转换就可以了，所以说雷达的驱动简单，主要是依赖canbus的驱动。  
![drivers](img/drivers.jpg)  
apollo的canbus驱动在"third_party/can_card_library"目录中，主要是引用linux的动态链接库"lib/libntcan.so.4"，我们从linux操作系统开始，把整个canbus的调用流程分析一遍。

## Linux Canbus
首先下载linux kernel(这里以linux-3.16.64举例)，Canbus的驱动主要在"drivers/net/can"目录中，编译内核的时候可以打开对应的开关来指定内核是否编译这些模块。Canbus驱动的目录结构如下：  
```
.
├── at91_can.c
├── bfin_can.c
├── cc770
├── c_can
├── dev.c
├── flexcan.c
├── grcan.c
├── janz-ican3.c
├── Kconfig
├── led.c
├── Makefile
├── mscan
├── pch_can.c
├── rcar_can.c
├── sja1000
├── slcan.c
├── softing
├── spi
├── ti_hecc.c
├── usb
├── vcan.c
└── xilinx_can.c
```


## Reference
[linux](https://www.kernel.org/)  



