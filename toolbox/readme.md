# Toolbox
## 查找依赖库
由于c++中是允许重载，而函数名要唯一，所以需要名字修饰

```
ldd -r /media/data/k8s/apollo/bazel-bin/cyber/py_wrapper/../../_solib_k8/libcyber_Sclass_Uloader_Slibclass_Uloader.so

c++filt _ZN4Poco13SharedLibraryC1ERKSs
```

也可以用"nm"命令查询:  
```
nm -u /media/data/k8s/apollo/bazel-bin/cyber/py_wrapper/../../_solib_k8/libcyber_Sclass_Uloader_Slibclass_Uloader.so | grep _ZN4Poco13SharedLibraryC1ERKSs
```

[undefined symbol](https://blog.csdn.net/stpeace/article/details/76561814)  
[名字修饰](https://zh.wikipedia.org/wiki/%E5%90%8D%E5%AD%97%E4%BF%AE%E9%A5%B0)  


## CoreDump
https://www.jianshu.com/p/3dc143c53ca2  


## 编译
#### gdb调试
https://blog.csdn.net/davidhopper/article/details/82589722  
启动gdb调试
```
gdb -q bazel-bin/modules/map/relative_map/navigation_lane_test
```
进入GDB调试界面后，使用l命令查看源代码，使用b 138在源代码第138行（可根据需要修改为自己所需的代码位置 ）设置断点，使用r命令运行navigation_lane_test程序，进入断点暂停后，使用p navigation_lane_查看当前变量值（可根据需要修改为其他变量名），使用n单步调试一条语句，使用s单步调试进入函数内部，使用c继续执行后续程序。如果哪个部分测试通不过，调试信息会立刻告诉你具体原因，可使用bt查看当前调用堆栈。




#### bazel编译
编译清除  
```
bazel clean --expunge
```



