参考 https://github.com/lgsvl/apollo-3.5

## 容器内部python运行
1. 在apollo目录下
```
source cyber/setup.bash
```

2. 执行
```
python cyber/python/examples/listener.py
python cyber/python/examples/talker.py
```



## 容器外部执行
1. python 环境变量

```
export PYTHONPATH=$PYTHONPATH:/media/data/k8s/apollo
export PYTHONPATH=$PYTHONPATH:/media/data/k8s/apollo/bazel-bin/cyber/py_wrapper
```

3. 环境变量
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

2. 安装Fast-RTPS
直接拷贝so也可以
/usr/local/fast-rtps/lib/libfastcdr.so.1
/usr/local/fast-rtps/lib/libfastrtps.so.1

4. 拷贝so
在容器里面
cp /usr/local/lib/libglog.so.0 .
cp /usr/local/lib/libgflags.so.2.2 .
cp /usr/lib/libprotobuf.so.17 .
cp /usr/lib/libPocoFoundation.so.9 .



## 拷贝到另外的机器
1. 设置机器IP
A机器 在setup.bash中设置 export CYBER_IP=192.168.1.101
B机器 在setup.bash中设置 export CYBER_IP=192.168.1.102

2. 拷贝文件和依赖库
192.168.1.101
在目录 /home/k8s/apollo/PythonClient

3. 设置环境变量
source setup.bash

export PYTHONPATH=$PYTHONPATH:/home/k8s/apollo/bazel-bin/cyber/py_wrapper
export PYTHONPATH=$PYTHONPATH:/home/k8s/apollo/py_proto

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/k8s/apollo/bazel-bin

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/k8s/apollo/third_party/tf2/lib/


export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/k8s/apollo/bazel-bin/cyber/py_wrapper/_cyber_init.so.runfiles/apollo/_solib_k8/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/k8s/apollo/bazel-bin/cyber/py_wrapper/_cyber_node.so.runfiles/apollo/_solib_k8


export PYTHONPATH=$PYTHONPATH:/home/k8s/apollo/third_party

sudo pip install protobuf

## 运行并且测试 
#### A机器 
python client_example.py --host 192.168.1.102
#### B机器 
1. 启动carla  ./CarlaUE4.sh -carla-server
2. 容器内部执行 python listener.py
 

## TODO
适配carla和apollo的参数，并且对接发布出去。 


# dreamview

1. bag转record
```
source /your-path-to-apollo-install-dir/cyber/setup.bash
rosbag_to_record input.bag output.record
```

[参考](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md)  

2. record播放
```
./bazel-bin/modules/dreamview/backend/simulation_command/sim_cmd
cyber_recorder play -f out.record -s 10 -c /apollo/canbus/chassis /apollo/localization/pose /apollo/sensor/gnss/odometry
```

3. HMI status
HMI status一直保存在内存中？
