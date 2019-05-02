# Routing
Routing类似于现在开车时用到的导航模块，通常考虑的是起点到终点的最优路径（通常是最短路径），和Planning的区别是Routing考虑的是起点到终点的最短路径，而Planning则是行驶过程中，当前一小段时间如何行驶，需要考虑当前路况，是否有障碍物。Routing模块则不需要考虑这些信息，只需要做一个长期的规划路径即可，过程如下：  

![introduction]()  

这也和我们开车类似，上车之后，首先搜索目的地，打开导航（Routing所做的事情），而开始驾车之后，则会根据当前路况，行人车辆信息来适当调整直到到达目的地（Planning所做的事情）。
* **Routing** - 主要关注起点到终点的长期路径，根据起点到终点之间的道路，选择一条最优路径。  
* **Planning** - 主要关注几秒钟之内汽车的行驶路径，根据当前行驶过程中的交通规则，车辆行人等信息，然后规划一条短期路径。  


## routing for osm
https://wiki.openstreetmap.org/wiki/Routing

http://www.patrickklose.com/posts/parsing-osm-data-with-python/

城市道路分析：
https://geoffboeing.com/2016/11/osmnx-python-street-networks/
https://automating-gis-processes.github.io/2018/notebooks/L6/network-analysis.html

https://socialhub.technion.ac.il/wp-content/uploads/2017/08/revise_version-final.pdf

https://stackoverflow.com/questions/29639968/shortest-path-using-openstreetmap-datanodes-and-ways


## openstreetmap 查找节点
If it's a polygon, then it's a closed way in the OSM database. You can find ways by id as simple as this: http://www.openstreetmap.org/way/305293190

If a specific node (the building blocks of ways) is giving a problem, the link would be http://www.openstreetmap.org/node/305293190 .

If it is a multipolygon (for example a building with a hole in it), the link would be http://www.openstreetmap.org/relation/305293190


## 地图介绍
https://blog.csdn.net/scy411082514/article/details/7484497

## 地图下载
https://www.openstreetmap.org/export#map=15/22.5163/113.9380
