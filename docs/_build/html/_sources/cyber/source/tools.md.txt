## cyber组件启动
cyber_launch主要用来启动cyber模块，其中一个launch文件可以有一个或者多个module，每个module 包含一个dag文件，而一个dag文件则对应一个或者多个components。  
launch文件中有几个module则会启动几个*进程*，每个进程有单独的内存空间，比如静态变量等都不会共享。  
一个dag文件中可以有一个或者多个components，一个components对应一个协程。协程中的静态变量是共享的，并且全局唯一。  

理解了上述原理，我们再详细分析以下2种启动方式。  
1. cyber_launch
2. dreamview
实际上上述2种启动方式没有太大区别，都是通过`mainboard`来启动程序，而一个dag文件对应一个进程，接着根据dag文件中有几个components，生成几个协程，每个components对应一个协程。  


下面介绍下cyber_launch的文件结构。
## cyber_launch
```
<cyber>
    <module>
        <name>planning</name>   \\ module名称
        <dag_conf>/apollo/modules/planning/dag/planning.dag</dag_conf>   \\ module的dag文件
        <process_name>planning</process_name>   \\ 指定调度文件
    </module>
</cyber>
```
* module 用于区分模块
* name  模块名称，主要用来cyber_launch启动的时候显示名称
* dag_conf  module模块对应的dag文件
* process_name  指定module的调度文件，如果找不到则会提示


## dag
下面以`velodyne.dag`文件为例来进行说明。  
```
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/lidar/velodyne/driver/libvelodyne_driver_component.so"
    // 模块的so文件，用于加载到内存
    # 128
    components {  // 第1个组件
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_128_driver"   // 名称必须不一样
        // 配置文件
        config_file_path : "/apollo/modules/drivers/lidar/velodyne/conf/velodyne128_conf.pb.txt"
      }
    }
    # 16_front_up
    components {  // 第2个组件
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_16_front_up_driver"
        config_file_path : "/apollo/modules/drivers/lidar/velodyne/conf/velodyne16_front_up_conf.pb.txt"
      }
    }
}

module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/lidar/velodyne/parser/libvelodyne_convert_component.so"
    // 模块的so文件，用于加载到内存
    # 128
    components {
      class_name : "VelodyneConvertComponent"
      config {
        name : "velodyne_128_convert"   
        config_file_path : "/apollo/modules/drivers/lidar/velodyne/conf/velodyne128_conf.pb.txt"
        readers {
          channel: "/apollo/sensor/lidar128/Scan"
        }
      }
    }
    # 16_front_up_center
    components {
      class_name : "VelodyneConvertComponent"
      config {
        name : "velodyne_16_front_up_convert"
        config_file_path : "/apollo/modules/drivers/lidar/velodyne/conf/velodyne16_front_up_conf.pb.txt"
        readers {
          channel: "/apollo/sensor/lidar16/front/up/Scan"
        }
      }
    }
}
```
dag文件有一个或者多个module_config，而每个module_config中对应一个或者多个components。  


## cyber_launch.py分析

#### start
在start函数中会遍历module，然后通过
```python
for module in root.findall('module'):
    if process_name not in process_list:
        if process_type == 'binary':
            if len(process_name) == 0:
                logger.error(
                    'Start binary failed. Binary process_name is null.')
                continue
            pw = ProcessWrapper(
                process_name.split()[0], 0, [
                    ""], process_name, process_type,
                exception_handler)
        # Default is library
        else:
            pw = ProcessWrapper(
                g_binary_name, 0, dag_dict[
                    str(process_name)], process_name,
                process_type, sched_name, exception_handler)
        result = pw.start()
        if result != 0:
            logger.error(
                'Start manager [%s] failed. Stop all!' % process_name)
            stop()
        pmon.register(pw)
        process_list.append(process_name)	
```
而ProcessWrapper中通过`subprocess.Popen`启动新的进程。  
```python
	self.popen = subprocess.Popen(args_list, stdout=subprocess.PIPE,
	                              stderr=subprocess.STDOUT)
```


#### stop
stop实际上是通过找到对应的launch文件，并且杀掉对应的进程来实现。(kill对应的是PID，pkill对应的是command)
```python
def stop_launch(launch_file):
    """
    Stop the launch file
    """
    if not launch_file:
        cmd = 'pkill -INT cyber_launch'   # 杀掉对应的进程
    else:
        cmd = 'pkill -INT -f ' + launch_file  # 杀掉对应的进程

    os.system(cmd)
    time.sleep(3)
    logger.info('Stop cyber launch finished.')
    sys.exit(0)
```


## mainboard
mainboard通过LoadModule来加载模块，而`module_config`来指定加载的so文件，然后启动一个或者多个components，每个components的配置文件可以不一样。  
```c++
bool ModuleController::LoadModule(const DagConfig& dag_config) {
  const std::string work_root = common::WorkRoot();

  for (auto module_config : dag_config.module_config()) {
    std::string load_path;
    if (module_config.module_library().front() == '/') {
      load_path = module_config.module_library();
    } else {
      load_path =
          common::GetAbsolutePath(work_root, module_config.module_library());
    }

    if (!common::PathExists(load_path)) {
      AERROR << "Path does not exist: " << load_path;
      return false;
    }
    // 1. 加载模块的so文件
    class_loader_manager_.LoadLibrary(load_path);
    
    // 2.1 加载触发模块
    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }

    // 2.2 加载定时模块
    for (auto& component : module_config.timer_components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
  }
  return true;
}
```


## dreamview
dreamview通过界面上的滑动按钮打开和关闭模块，大概的流程是前端通过websocket发送消息到后端，后端通过调用命令行启动模块，后端主要的实现在"hmi_worker.cc"中。  

dreamview中的modules通过指定"start_command"和"stop_command"来启动和关闭。 

#### LoadMode
对于so文件，会通过LoadMode来进行生成命令，下面介绍下生成"start_command"和"stop_command"的过程。生成的命令和cyber_launch中调用的命令一致，只是前面增加了`nohup`。  
```c++
HMIMode HMIWorker::LoadMode(const std::string& mode_config_path) {
  HMIMode mode;
  ACHECK(cyber::common::GetProtoFromFile(mode_config_path, &mode))
      << "Unable to parse HMIMode from file " << mode_config_path;
  // Translate cyber_modules to regular modules.
  for (const auto& iter : mode.cyber_modules()) {
    const std::string& module_name = iter.first;
    const CyberModule& cyber_module = iter.second;
    // Each cyber module should have at least one dag file.
    ACHECK(!cyber_module.dag_files().empty())
        << "None dag file is provided for " << module_name << " module in "
        << mode_config_path;

    Module& module = LookupOrInsert(mode.mutable_modules(), module_name, {});
    module.set_required_for_safety(cyber_module.required_for_safety());

    // 1. Construct start_command:
    //     nohup mainboard -p <process_group> -d <dag> ... &
    module.set_start_command("nohup mainboard");
    const auto& process_group = cyber_module.process_group();
    if (!process_group.empty()) {
      absl::StrAppend(module.mutable_start_command(), " -p ", process_group);
    }
    for (const std::string& dag : cyber_module.dag_files()) {
      absl::StrAppend(module.mutable_start_command(), " -d ", dag);
    }
    absl::StrAppend(module.mutable_start_command(), " &");

    // 2. Construct stop_command: pkill -f '<dag[0]>'
    const std::string& first_dag = cyber_module.dag_files(0);
    module.set_stop_command(absl::StrCat("pkill -f \"", first_dag, "\""));
    // Construct process_monitor_config.
    module.mutable_process_monitor_config()->add_command_keywords("mainboard");
    module.mutable_process_monitor_config()->add_command_keywords(first_dag);
  }
  mode.clear_cyber_modules();
  return mode;
}
```
接着调用`void HMIWorker::StartModule(const std::string& module) const`来启动模块，启动调用的`std::system()`来启动命令行，也是一个dag文件对应一个进程。dag中的模块都对应协程。


## 用途
如何启动2个一模一样的模块
1. 如果你需要启动2个相同的模块，可以在launch文件中增加2个module，并且修改dag文件中模块的名称，这样就可以启动2个一模一样的模块了。上述这种情况在模块中有静态变量，并且你希望静态变量不相互影响的情况下可以采用。
2. 如果需要在同一个进程中启动2个模块，可以在dag中增加components，然后修改config中的名称和配置文件，这样就可以启动2个一模一样的模块了。上述这种情况静态变量会相互影响，A模块改了，B模块的静态变量值也改变了。

perception中的inner消息订阅不到？
perception模块中的inner消息只有在同一个进程中才会收到，也就是在同一个dag文件中启动的模块才会接收到，因为reader读取的时候会判断2个节点是否是在同一个进程，如果是同一个进程就直接传递对象，不进行序列化，如果不是同一个进程则先进行序列化，然后再通过共享内存通信。
