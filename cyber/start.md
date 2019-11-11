## Cyber实现的功能
cyber提供的功能概括起来包括2方面：
1. **消息队列** - 主要作用是接收和发送各个节点的消息，涉及到消息的发布、订阅以及消息的buffer缓存等。 
2. **实时调度** - 主要作用是调度处理上述消息的算法模块，保证算法模块能够实时调度处理消息。  

除了这2方面的工作，cyber还需要提供以下2部分的工作：
1. **用户接口** - 提供灵活的用户接口
2. **工具** - 提供一系列的工具，例如bag包播放，点云可视化，消息监控等

![cyber](img/arch.jpg)  
总结起来就是，cyber是一个分布式收发消息，和调度框架，同时对外提供一系列的工具和接口来辅助开发和定位问题。其中cyber对比ROS来说有很多优势，唯一的劣势是cyber相对ROS没有丰富的算法库支持。  

下面我们开始分析整个cyber的代码流程。
## cyber入口
cyber的入口在"cyber/mainboard"目录中：
```
├── mainboard.cc           // 主函数
├── module_argument.cc     // 模块输入参数
├── module_argument.h
├── module_controller.cc   // 模块加载，卸载
└── module_controller.h
```
mainboard中的文件比较少，也很好理解，我们先从"mainboard.cc"中开始分析：
```c++
int main(int argc, char **argv) {
  google::SetUsageMessage("we use this program to load dag and run user apps.");
  
  // 注册信号量，当出现系统错误时，打印堆栈信息
  signal(SIGSEGV, SigProc);
  signal(SIGABRT, SigProc);
  
  // parse the argument
  // 解析参数
  ModuleArgument module_args;
  module_args.ParseArgument(argc, argv);

  // initialize cyber
  // 初始化cyber
  apollo::cyber::Init(argv[0]);
  
  // start module
  // 加载模块
  ModuleController controller(module_args);
  if (!controller.Init()) {
    controller.Clear();
    AERROR << "module start error.";
    return -1;
  }
  
  // 等待cyber关闭
  apollo::cyber::WaitForShutdown();
  // 卸载模块
  controller.Clear();
  AINFO << "exit mainboard.";

  return 0;
}
```
上述是"mainboard.cc"的主函数，下面我们重点介绍下具体的过程。

#### 打印堆栈
在主函数中注册了信号量"SIGSEGV"和"SIGABRT"，当系统出现错误的时候（空指针，异常）等，这时候就会触发打印堆栈信息，也就是说系统报错的时候打印出错的堆栈，方便定位问题，[参考](https://www.runoob.com/cplusplus/cpp-signal-handling.html)。
```c++
  // 注册信号量，当出现系统错误时，打印堆栈信息
  signal(SIGSEGV, SigProc);
  signal(SIGABRT, SigProc);
```
打印堆栈的函数在"SigProc"中实现，而打印堆栈的实现是通过"backtrace"实现，[参考](https://www.gnu.org/software/libc/manual/html_node/Backtraces.html)。
```c++
// 打印堆栈信息
void ShowStack() {
  int i;
  void *buffer[STACK_BUF_LEN];
  int n = backtrace(buffer, STACK_BUF_LEN);
  char **symbols = backtrace_symbols(buffer, n);
  AINFO << "=============call stack begin:================";
  for (i = 0; i < n; i++) {
    AINFO << symbols[i];
  }
  AINFO << "=============call stack end:================";
}
```

#### 解析参数
解析参数是在"ModuleArgument"类中实现的，主要是解析加载DAG文件时候带的参数。
```c++
void ModuleArgument::ParseArgument(const int argc, char* const argv[]) {
  // 二进制模块名称
  binary_name_ = std::string(basename(argv[0]));
  // 解析参数
  GetOptions(argc, argv);

  // 如果没有process_group_和sched_name_，则赋值为默认值
  if (process_group_.empty()) {
    process_group_ = DEFAULT_process_group_;
  }

  if (sched_name_.empty()) {
    sched_name_ = DEFAULT_sched_name_;
  }

  // 如果有，则设置对应的参数
  GlobalData::Instance()->SetProcessGroup(process_group_);
  GlobalData::Instance()->SetSchedName(sched_name_);
  AINFO << "binary_name_ is " << binary_name_ << ", process_group_ is "
        << process_group_ << ", has " << dag_conf_list_.size() << " dag conf";

  // 打印dag_conf配置，这里的dag是否可以设置多个？？？
  for (std::string& dag : dag_conf_list_) {
    AINFO << "dag_conf: " << dag;
  }
}
```

#### 模块加载
在"ModuleController"实现cyber模块的加载，在"ModuleController::Init()"中调用"LoadAll()"来加载所有模块，我们接着看cyber是如何加载模块。  
1. 首先是找到模块的路径
```c++
    if (module_config.module_library().front() == '/') {
      load_path = module_config.module_library();
    } else {
      load_path =
          common::GetAbsolutePath(work_root, module_config.module_library());
    }
```
2. 通过"class_loader_manager_"加载模块，后面我们会接着分析"ClassLoaderManager"的具体实现，加载好对应的类之后在创建对应的对象，并且初始化对象（调用对象的Initialize()方法，也就是说所有的cyber模块都是通过Initialize()方法启动的，后面我们会接着分析Initialize具体干了什么）。  
这里的"classloader"其实类似java中的classloader，即java虚拟机在运行时加载对应的类，并且实例化对象。  
cyber中其实也是实现了类型通过动态加载并且实例化类的功能，好处是可以动态加载和关闭单个cyber模块(定位，感知，规划等)，也就是在dreamview中的模块开关按钮，实际上就是动态的加载和卸载对应的模块。
```c++
    // 通过类加载器加载load_path下的模块
    class_loader_manager_.LoadLibrary(load_path);

    // 加载模块
    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      // 创建对象
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      // 调用对象的Initialize方法
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }

    // 加载定时器模块
    for (auto& component : module_config.timer_components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
```

上述就是cyber mainboard的整个流程，cyber main函数中先解析dag参数，然后根据解析的参数，通过类加载器动态的加载对应的模块，然后调用Initialize方法初始化模块。

下面我们会接着分析ClassLoaderManager
