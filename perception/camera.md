## camera

## lib目录

#### traffic_light
红绿灯的识别分为3个阶段，先预处理，然后识别，然后跟踪？
preprocessor // 预处理
detector - detection     // 检测
         - recognition   // 识别，通过上面的detection检测？？？
tracker // 追踪红绿灯

#### obstacle
detector // 检测
postprocessor // 后处理？做了什么处理???
tracker // 追踪
transformer // 坐标转换

#### lane
detector // 检测
postprocessor // 后处理


**数据集**
[CULane Dataset](https://xingangpan.github.io/projects/CULane.html)  
[bdd](https://bdd-data.berkeley.edu/)  
[tusimple](https://github.com/TuSimple/tusimple-benchmark)  



**参考**
[awesome-lane-detection](https://github.com/amusi/awesome-lane-detection)  



#### calibration_service & calibrator



#### feature_extractor ???


