## ClassLoader
每个ClassLoader加载一个路径，每个路径代表一个so库？load_ref_count_代表加载library引用计数。

plugin_ref_count_代表创建类的引用计数？

## loadLibrary
```c++
void ClassLoader::loadLibrary()
{
  boost::recursive_mutex::scoped_lock lock(load_ref_count_mutex_);
  // 每次引用计数加1
  load_ref_count_ = load_ref_count_ + 1;
  // 加载库
  class_loader::impl::loadLibrary(getLibraryPath(), this);
}
```

## isLibraryLoaded
判断Library是否加载

```c++
bool ClassLoader::isLibraryLoaded()
{
  return class_loader::impl::isLibraryLoaded(getLibraryPath(), this);
}
```

## isLibraryLoadedByAnyClassloader
判断Library是否被其它的classloader加载

```c++
bool ClassLoader::isLibraryLoadedByAnyClassloader()
{
  return class_loader::impl::isLibraryLoadedByAnybody(getLibraryPath());
}
```

## systemLibraryFormat
获取库的名称？

```c++
std::string systemLibraryFormat(const std::string & library_name)
{
  return systemLibraryPrefix() + library_name + systemLibrarySuffix();
}
```

## unloadLibrary

卸载Library，先判断是否plugin_ref_count_，即是否有创建的类没有销毁，如果没有销毁则不能卸载，如果类已经销毁了，需要加载次数清零，保证加载和卸载次数相等。


## 创建类
```c++
  template<class Base>
  std::shared_ptr<Base> createSharedInstance(const std::string & derived_class_name)
  {
    return std::shared_ptr<Base>(
      createRawInstance<Base>(derived_class_name, true),
      boost::bind(&ClassLoader::onPluginDeletion<Base>, this, _1));
  }
```


## registerPlugin
申明静态变量`g_register_plugin_UniqueID`

注册类，这里是注册派生类到factoryMap，通过派生类可以找到对应的MetaObject<Derived, Base>，创建的时候创建派生类。
```c++
  // Create factory
  impl::AbstractMetaObject<Base> * new_factory =
    new impl::MetaObject<Derived, Base>(class_name, base_class_name);
  new_factory->addOwningClassLoader(getCurrentlyActiveClassLoader());
  new_factory->setAssociatedLibraryPath(getCurrentlyLoadingLibraryName());

  FactoryMap & factoryMap = getFactoryMapForBaseClass<Base>();
  factoryMap[class_name] = new_factory;
```


## 总结
1. 首先通过宏定义注册派生类和基类
```c++
CLASS_LOADER_REGISTER_CLASS(Derived, Base)
```
2. 注册的派生类和基类会注册到哈希表，key为派生类的名称，value为MetaObject<Derived, Base>，创建的时候会创建派生类。

AbstractMetaObjectBase 对象有一个ClassLoaderVector，是一个加载器数组。 AbstractMetaObjectBase抽象类可以对应多个classloader？？？


一个ClassLoader对应一个library_path，同时还有引用计数和实例化计数。创建了多少个对象则有多少个plugin_ref_count_，每个对象析构的时候会引用计数plugin_ref_count_减去1。
同一个类可能被不同的ClassLoader加载，加载的路径当然也不一样，然后把classloader放到一个数组中。


## 程序链接几种方式
1. 静态链接
2. 动态链接
3. 动态加载

前2种在编译阶段就要指定，后1种是程序运行过程中动态加载到内存。有插件的系统都采用动态加载的方式来设计。


## 工作原理

#### linux
dlopen dlsysm dlclose

#### ELF文件
文件格式
INIT FIN
ctor
符号表

通过nm命令查看
readelf

## 加载原理

#### extern c
这样符号表的函数名称不会变化。

#### 静态变量
静态变量加载的时候会动态初始化
1. elf文件中的init和ctor段，也可以用`__attribute__ `宏来实现启动的时候就执行，然后再把控制权交给main函数

全部类继承一个基类，实现了new object方法，然后再注册工厂类到全局变量，这样就可以创建变量了。


#### 注册
1. 注册通过宏定义实现
2. 加载的时候动态构造，并且注册到全局变量
3. 类实现了创建方法

#### 创建
1. 找到对应的类，然后调用创建方法
2. 每次销毁的时候引用计数减1

#### 加载
采用dlopen加载，可以反复加载，但是返回同一个指针，卸载的时候次数相等？classloader对原生重复加载做了拦截？

#### 卸载
1. 判断是否有引用计数，有对象存活则不能卸载
2. 没有对象存活，则卸载库，这里的引用计数看起来没什么用（采用bool型就可以解决）？


总之上述2种方法都是找一个绝对的物理地址去调用，函数在代码段，通过名称去调用，静态变量在数据段，然后去调用。


## 静态变量作用域
1. file scope只作用在单个文件，全局变量的作用域，需要加上extern
2. 函数中的static变量作用域只在函数可见，并且在函数调用的时候初始化
3. static变量在动态库卸载的时候会清零，再次加载的时候会重新初始化 （apollo中和ros中的区别，需要做一些实验去验证是否正确）为什么函数中的全局变量没有被卸载？？
4. static变量的生命周期，创建一直保存到程序结束，对动态加载程序创建的变量好像是卸载的时候删除？
5. static变量线程安全，c++11之后是线程安全的

