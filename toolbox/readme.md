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


## coredump
https://www.jianshu.com/p/3dc143c53ca2  


## bazel编译
编译清除  
```
bazel clean --expunge
```

