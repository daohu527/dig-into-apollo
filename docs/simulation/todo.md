# lgsvl介绍
## apollo桥接
lgsvl通过socket实现了apollo和lgsvl的桥接，主要的原理如下：  
![]()  

#### 数据格式
需要重点关注的是数据格式的转换：  
![]()  



## 地图制作
lgsvl默认支持的地图为"San Francisco"，目前还不支持导入地图，如果需要自己制作地图可以按照下面的流程，在cityengine中制作城市的3维地图，然后导出模型到unity，用unity的地图编辑器编辑并且导出hd_map到apollo，之后在unity中创建场景，并且导入车的模型，整个地图的制作就完成了。

## lgsvl场景介绍
我们以"SimpleMap"场景来举例子，介绍场景是如何组成的，SimpleMap由以下几个场景组成:  
```
Directional light  // 平行光
EventSystem  // 事件系统
XE_Rigged-apollo  // 车模型
ProceduralRoad  // 地图中的房屋以及模型
MapSimple  // 道路，交通灯模型
HDMapTool  // apollo高精度地图制作工具
VectorMapTool  // autoware地图制作工具
PointCloudTool  // ??
PointCloudBounds  // ??
MapOrigin // 地图起点，用来确定gps坐标？？
spawn_transform  // 
spawn_transform(1)
```

EventSystem 事件系统检测游戏的事件，并且提供调用接口。例如键盘鼠标回调，射线检测。  
Spawn GameObject 特殊的创建游戏对象事件。[](https://docs.unity3d.com/Manual/UNetSpawning.html)  

sanfrancisco  
```
Terrains 地形
```

碰撞采用的是[网格碰撞](http://docs.manew.com/Components/class-MeshCollider.html)  




## Unity帮助文档
http://docs.manew.com/Manual/71.html  

#### 回放
https://gamedev.stackexchange.com/questions/141149/action-replay-in-unity-3d  
https://www.gamasutra.com/blogs/AustonMontville/20141105/229437/Implementing_a_replay_system_in_Unity_and_how_Id_do_it_differently_next_time.php  
https://forum.unity.com/threads/best-approach-to-replay-system.624517/  
