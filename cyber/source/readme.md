# Dig into Apollo - Cyber ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)


写在之前，之前的分析都是一些源码级别的分析，发现一开始就深入源码，很容易陷进去，特别是模块非常多的情况，需要看很多遍才能理解清楚。要写出更容易理解的文档，需要的不是事无巨细的分析代码，更主要的是能够把复杂的东西抽象出来，变为简单的东西。一个很简答的例子是画函数调用流程图很简单，但是要把流程图转换成框图却很难。  


## 数据处理流程
我们先看下cyber中整个的数据处理流程，通过理解数据流程中各个模块如何工作，来搞清楚每个模块的作用，然后我们再接着分析具体的模块。    
![cyber数据流程](../img/data_progress.jpg)  

如上图所示，cyber的数据流程可以分为6个过程。  
1. Node节点中的Writer往通道里面写数据。
2. 通道中的Transmitter发布消息，通道中的Receiver订阅消息。
3. Receiver接收到消息之后，触发回调，触发DataDispather进行消息分发。
4. DataDispather接收到消息后，把消息放入CacheBuffer，并且触发Notifier，通知对应的DataVisitor处理消息。
5. DataVisitor把数据从CacheBuffer中读出，并且进行融合，然后通过notifier_唤醒对应的协程。
6. 协程执行对应的注册回调函数，进行数据处理，处理完成之后接着进入睡眠状态。

对数据流程有整体的认识之后，下面我们在分析具体的每个模块，我们还是按照功能划分。  

## 整体介绍
首先我们对cyber中各个模块做一个简单的介绍，之后再接着分析。实际上我们只要搞清楚了下面一些概念之间的关系，就基本上理解清楚了整个Cyber的数据流程。     

#### 1.Component和Node的关系
Component是cyber中封装好的数据处理流程，对用户来说，对应自动驾驶中的Planning Component, Perception Component等，目的是帮助我们更方便的订阅和处理消息。实际上Component模块在加载之后会执行"Initialize()"函数，这是个隐藏的初始化过程，对用户不可见。在"Initialize"中，Component会创建一个Node节点，概念上对应ROS的节点，**每个Component模块只能有一个Node节点**，也就是说每个Component模块有且只能有一个节点，在Node节点中进行消息订阅和发布。  


#### 2.Node和Reader\Writer的关系
在Node节点中可以创建Reader订阅消息，也可以创建Writer发布消息，每个Node节点中可以创建多个Reader和Writer。  


#### 3.Reader和Receiver,Writer和Transmitter,Channel的关系
一个Channel对应一个Topic，概念上对应ROS的消息通道，每个Topic都是唯一的。而Channel中包括一个发送器(Transmitter)和接收器(Receiver)，通过Receiver接收消息，通过Transmitter发送消息。  
一个Reader只能订阅一个通道的消息，如果一个Node需要订阅多个通道的消息，需要创建多个Reader。同理一个Writer也只能发布一个通道的消息，如果需要发布多个消息，需要创建多个Writer。Reader中调用Receiver订阅消息，而Writer通过Transmitter发布消息。 


#### 4.Receiver, DataDispather和DataVisitor的关系
每一个Receiver接收到消息之后，都会触发回调，回调中触发DataDispather（消息分发器）发布消息，DataDispather是一个单例，所有的数据分发都在数据分发器中进行，DataDispather会把数据放到对应的缓存中，然后Notify(通知)对应的协程（实际上这里调用的是DataVisitor中注册的Notify）去处理消息。  
DataVisitor（消息访问器）是一个辅助的类，**一个数据处理过程对应一个DataVisitor，通过在DataVisitor中注册Notify（唤醒对应的协程，协程执行绑定的回调函数），并且注册对应的Buffer到DataDispather**，这样在DataDispather的时候会通知对应的DataVisitor去唤醒对应的协程。  
也就是说DataDispather（消息分发器）发布对应的消息到DataVisitor，DataVisitor（消息访问器）唤醒对应的协程，协程中执行绑定的数据处理回调函数。 


#### 5.DataVisitor和Croutine的关系
实际上DataVisitor中的Notify是通过唤醒协程（为了方便理解也可以理解为线程，可以理解为你有一个线程池，通过线程池绑定数据处理函数，数据到来之后就唤醒对应的线程去执行任务），每个协程绑定了一个数据处理函数和一个DataVisitor，数据到达之后，通过DataVisitor中的Notify唤醒对应的协程，执行数据处理回调，执行完成之后协程进入休眠状态。  


#### 6.Scheduler, Task和Croutine
通过上述分析，**数据处理的过程实际上就是通过协程完成的，每一个协程被称为一个Task，所有的Task(任务)都由Scheduler进行调度**。从这里我们可以分析得出实际上Cyber的实时调度由协程去保障，并且可以灵活的通过协程去设置对应的调度策略，当然协程依赖于进程，Apollo在linux中设置进程的优先级为实时轮转，先保障进程的优先级最高，然后内部再通过协程实现对应的调度策略。  
协程和线程的优缺点这里就不展开了，这里有一个疑问是协程不能被终止，除非协程主动退出，这里先留一个伏笔，后面我们再分析协程的调度问题。  


上述就是各个概念之间的关系，上述介绍对理解数据的流程非常有帮助，希望有时间的时候，大家可以画一下对应的数据流程图和关系。  


## Component介绍

我们首先需要清楚一点，component实际上是cyber为了帮助我们特意实现的对象，component加载的时候会自动帮我们创建一个node，通过node来订阅和发布对应的消息，每个component有且只能对应一个node。  
component对用户提供2个接口"Init()"和"Proc()"，用户在Init中进行初始化，在"Proc"中接收Topic执行具体的算法。对用户隐藏的部分包括component的"Initialize()"初始化，以及"Process()"调用执行。     
component还可以动态的加载和卸载，这也可以对应到在dreamviewer上动态的打开关系模块。下面我们先大致介绍下component的工作流程，然后再具体介绍各个模块。  


#### component工作流程
component的工作流程大致如下：  
1. 通过继承"cyber::Component"，用户自定义一个模块，并且实现"Init()"和"Proc()"函数。编译生成".so"文件。
2. 通过classloader加载component模块到内存，创建component对象，调用"Initialize()"初始化。（Initialize中会调用Init）  
3. 创建协程任务，并且注册"Process()"回调，当数据到来的时候，唤醒对象的协程任务执行"Process()"处理数据。（Process会调用Proc）
综上所述，component帮助用户把初始化和数据收发的流程进行了封装，减少了用户的工作量，component封装了整个数据的收发流程，component本身并不是单独的一个线程执行，模块的初始化都在主线程中执行，而具体的任务则是在协程池中执行。  


#### cyber入口
cyber的入口在"cyber/mainboard/mainboard.cc"中，主函数中先进行cyber的初始化，然后启动cyber模块，然后运行，一直等到系统结束。  
```c++
int main(int argc, char** argv) {
  // 1. 解析参数
  ModuleArgument module_args;
  module_args.ParseArgument(argc, argv);

  // 2. 初始化cyber
  apollo::cyber::Init(argv[0]);

  // 3. 启动cyber模块
  ModuleController controller(module_args);
  if (!controller.Init()) {
    controller.Clear();
    AERROR << "module start error.";
    return -1;
  }
  
  // 4. 等待直到程序退出
  apollo::cyber::WaitForShutdown();
  controller.Clear();
  return 0;
}
```


#### component动态加载

cyber主函数在"ModuleController::Init()"进行模块的加载，具体的加载过程在"ModuleController::LoadModule"中。  
```c++
bool ModuleController::LoadModule(const DagConfig& dag_config) {
  const std::string work_root = common::WorkRoot();

  for (auto module_config : dag_config.module_config()) {
    // 1. 加载动态库
    class_loader_manager_.LoadLibrary(load_path);
    
    // 2. 加载消息触发模块
    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      // 3. 创建对象
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      // 4. 调用对象的Initialize方法
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
    
    // 5. 加载定时触发模块
    for (auto& component : module_config.timer_components()) {
      // 6. 创建对象
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      // 7. 调用对象的Initialize方法
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
  }
  return true;
}
```
模块首先通过classloader加载到内存，然后创建对象，并且调用模块的初始化方法。component中每个模块都设计为可以动态加载和卸载，可以实时在线的开启和关闭模块，实现的方式是通过classloader来进行动态的加载动态库。  


#### component初始化
component一共有4个模板类，分别对应接收0-3个消息，（这里有疑问为什么没有4个消息的模板类，是漏掉了吗？）我们这里主要分析2个消息的情况，其它的可以类推。  
```c++
template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  // 1. 创建Node
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);

  // 2. 调用用户自定义初始化Init()
  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }

  bool is_reality_mode = GlobalData::Instance()->IsRealityMode();

  ReaderConfig reader_cfg;
  reader_cfg.channel_name = config.readers(1).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(1).qos_profile());
  reader_cfg.pending_queue_size = config.readers(1).pending_queue_size();
  
  // 3. 创建reader1
  auto reader1 = node_->template CreateReader<M1>(reader_cfg);
  ...
  // 4. 创建reader0
  if (cyber_likely(is_reality_mode)) {
    reader0 = node_->template CreateReader<M0>(reader_cfg);
  } else {
    ...
  }
  
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));


  auto sched = scheduler::Instance();
  // 5. 创建回调，回调执行Proc()
  std::weak_ptr<Component<M0, M1>> self =
      std::dynamic_pointer_cast<Component<M0, M1>>(shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg0,
                     const std::shared_ptr<M1>& msg1) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg0, msg1);
    } else {
      AERROR << "Component object has been destroyed.";
    }
  };

  std::vector<data::VisitorConfig> config_list;
  for (auto& reader : readers_) {
    config_list.emplace_back(reader->ChannelId(), reader->PendingQueueSize());
  }
  // 6. 创建数据访问器
  auto dv = std::make_shared<data::DataVisitor<M0, M1>>(config_list);
  // 7. 创建协程，协程绑定回调func（执行proc）。数据访问器dv在收到订阅数据之后，唤醒绑定的协程执行任务，任务执行完成之后继续休眠。
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}
```
总结以下component的流程。  
1. 创建node节点（1个component只能有1个node节点，之后用户可以用node_在init中自己创建reader或writer）。  
2. 调用用户自定义的初始化函数Init()（子类的Init方法）  
3. 创建reader，订阅几个消息就创建几个reader。  
4. 创建回调函数，实际上是执行用户定义算法Proc()函数
5. 创建数据访问器，数据访问器的用途为接收数据（融合多个通道的数据），唤醒对应的协程执行任务。
6. 创建协程任务绑定回调函数，并且绑定数据访问器到对应的协程任务，用于唤醒对应的任务。  


因为之前对cyber数据的收发流程有了一个简单的介绍，这里我们会分别介绍如何创建协程、如何在scheduler注册任务并且绑定Notify。也就是说，为了方便理解，你可以认为数据通过DataDispatcher已经分发到了对应的DataVisitor中，**接下来我们只分析如何从DataVisitor中取数据，并且触发对应的协程执行回调任务**。  
#### 创建协程
创建协程对应上述代码
```
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1>(func, dv);
```
接下来我们查看下如何创建协程呢？协程通过工厂模式方法创建，里面包含一个回调函数和一个dv（数据访问器）。  
```
template <typename M0, typename M1, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1>>& dv) {
  RoutineFactory factory;
  // 1. 工厂中设置DataVisitor
  factory.SetDataVisitor(dv);
  factory.create_routine = [=]() {
    return [=]() {
      std::shared_ptr<M0> msg0;
      std::shared_ptr<M1> msg1;
      for (;;) {
        CRoutine::GetCurrentRoutine()->set_state(RoutineState::DATA_WAIT);
        // 2. 从DataVisitor中获取数据
        if (dv->TryFetch(msg0, msg1)) {
          // 3. 执行回调函数
          f(msg0, msg1);
          // 4. 继续休眠
          CRoutine::Yield(RoutineState::READY);
        } else {
          CRoutine::Yield();
        }
      }
    };
  };
  return factory;
}
```
上述过程总结如下：
1. 工厂中设置DataVisitor  
2. 工厂中创建设置协程执行函数，回调包括3个步骤：从DataVisitor中获取数据，执行回调函数，继续休眠。


#### 创建调度任务
创建调度任务是在过程"Component::Initialize"中完成。  
```c++
sched->CreateTask(factory, node_->Name());
```
我们接着分析如何在Scheduler中创建任务。  
```c++
bool Scheduler::CreateTask(std::function<void()>&& func,
                           const std::string& name,
                           std::shared_ptr<DataVisitorBase> visitor) {
  // 1. 根据名称创建任务ID
  auto task_id = GlobalData::RegisterTaskName(name);
  
  auto cr = std::make_shared<CRoutine>(func);
  cr->set_id(task_id);
  cr->set_name(name);
  AINFO << "create croutine: " << name;
  // 2. 分发协程任务
  if (!DispatchTask(cr)) {
    return false;
  }

  // 3. 注册Notify唤醒任务
  if (visitor != nullptr) {
    visitor->RegisterNotifyCallback([this, task_id]() {
      if (cyber_unlikely(stop_.load())) {
        return;
      }
      this->NotifyProcessor(task_id);
    });
  }
  return true;
}
```

#### TimerComponent
实际上Component分为2类：一类是上面介绍的消息驱动的Component，第二类是定时调用的TimerComponent。定时调度模块没有绑定消息收发，需要用户自己创建reader来读取消息，如果需要读取多个消息，可以创建多个reader。  
```c++
bool TimerComponent::Initialize(const TimerComponentConfig& config) {
  // 1. 创建node
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);
  // 2. 调用用户自定义初始化函数
  if (!Init()) {
    return false;
  }

  std::shared_ptr<TimerComponent> self =
      std::dynamic_pointer_cast<TimerComponent>(shared_from_this());
  // 3. 创建定时器，定时调用"Proc()"函数
  auto func = [self]() { self->Proc(); };
  timer_.reset(new Timer(config.interval(), func, false));
  timer_->Start();
  return true;
}
```
总结一下TimerComponent的执行流程如下。
1. 创建Node
2. 调用用户自定义初始化函数
3. 创建定时器，定时调用"Proc()"函数


上述就是Component模块的调用流程。为了弄清楚消息的调用过程，下面我们分析"DataDispatcher"和"DataVisitor"。 

#### DataVisitor和DataDispatcher

## DataVisitor


## DataDispatcher




## CRoutine协程

## Scheduler调度




## Timer定时器
定时器提供在指定的时间触发执行的功能。定时器的应用非常普遍，比如定时触发秒杀活动、定时清理日志、定时发送心跳信息等。实现定时器的方法多种多样，古代有采用水漏或者沙漏的方式，近代有采用机械的方式（各种各样的时钟），数字脉冲，元素衰减等方式。  
在计算机领域有2种形式，**一种是硬件定时器，一种是软件定时器**。硬件定时器的原理是计算时钟脉冲，当规定的时钟脉冲之后由硬件触发中断程序执行，硬件定时器一般是芯片自带的，硬件定时器时间比较精准但是数量有限，因此人们又发明了软件定时器。软件定时器由软件统计计算机时钟个数，然后触发对应的任务执行，由于是纯软件实现，理论上可以创建很多个，下面我们主要看下软件定时器的实现。  

## 定时器的实现

#### 双向链表
首先我们想到的是把定时任务放入一个队列中，每隔固定的时间（一个tick）去检查队列中是否有超时的任务，如果有，则触发执行该任务。这样做的好处是实现简单，但是每次都需要轮询整个队列来找到谁需要被触发，当队列的长度很大时，每个固定时间都需要去轮询一次队列，时间开销比较大。当我们需要删除一个任务的时候，也需要轮询一遍队列找到需要删除的任务，实际上我们可以优化一下用双向链表去实现队列，这样删除的任务的时间复杂度就是O(1)了。总结一下就是采用双向链表实现队列，插入的时间复杂度是O(1)，删除的时间复杂度也是O(1)，但是查询的时间复杂度是O(n)。  
![linklist](../img/linklist.jpg)    

#### 最小堆
最小堆的实现方式是为了解决上述查找的时候需要遍历整个链表的问题，我们知道最小堆中堆顶的元素就是最小的元素，每次我们只需要检查堆顶的元素是否超时，超时则弹出执行，然后再检查新的堆顶元素是否超时，这样查找可执行任务的时间复杂度约等于O(1)，最小堆虽然提高了查找的时间，但是插入和删除任务的时间复杂度为O(log2n)。下面我们看一个例子。  
堆中节点的值存放的是任务到期的时间，每隔1分钟判断下是否有任务需要执行，比如任务A是19:01分触发，周期为5分钟，任务B是19:02分触发，周期为10分钟，那么第一次最小堆弹出19:01，执行之后，在堆中重新插入19:06分的任务A，这时候任务B到了堆顶，1分钟之后检测需要执行任务B，执行完成后，在堆中重新插入19:12分的任务B。然后循环执行上述过程。每执行一次任务都需要重新插入任务到堆中，当任务频繁执行的时候，插入任务的开销也不容忽略。  
![heap](../img/heap.jpg)  

#### 时间轮
最后，我们介绍一种插入、删除和触发执行都是O(1)的方法，由计算机科学家"George Varghese"等提出，在NetBSD(一种操作系统)上实现并替代了早期内核中的callout定时器实现。最原始的时间轮如下图。
![timing_wheel](../img/timing_wheel.jpg)  
一共有8个bucket，每个bucket代表tick的时间，类似于时钟，每个1秒钟走一格，我们可以定义tick的时间为1秒钟，那么bucket[1]就代表第1秒，而bucket[8]就代表第8秒，然后循环进行上述步骤。一个bucket中可能有多个任务，每个任务采用链表的方式连接起来。下面通过一个例子来说明如何添加、删除和查找任务。  
假设时间轮中有8个bucket，每个bucket占用一个tick时间，每个tick为1秒。当前有2个定时任务A、B，分别需要3秒、11秒执行一次。目前指针指在0的位置，3秒钟之后指针将指向bucket[3]的位置，因此我们把任务A放入bucket[3]中，接下来我们再看如何放置任务B，任务B是11秒之后执行，也就是说时间轮转1圈之后，再过3秒种，任务B才执行，那么如何标记任务的圈数呢？这里引入了round的概念，round为1就表示需要1圈，如果round为2则需要2圈，同理推广到其它圈数。我们把B任务也放入bucket[3]，但是设置它的round为1。
我们先看下任务A和任务B的执行过程，3秒钟之后时间轮转到bucket[3]，这时候检查bucket[3]中的任务，只执行round为0的任务，这里执行任务A，然后把bucket[3]中所有任务的round减1，这时候任务B的round数为0了，等到时间轮转一圈之后，就会执行任务B了。  
这里还有一个疑问就是任务A执行完成之后，下一次触发如何执行，其实在bucket[3]执行完成之后，会把任务A从bucket[3]中删除，然后从新计算3+3，放入bucket[6]中，等到bucket[6]执行完成之后，然后再放入（6+3）对8取余，放入bucket[1]中。也就是说每次任务执行完成之后需要重新计算任务在哪个bucket，然后放入对应的bucket中。  
![timing_wheel_progress](../img/timing_wheel_progress.jpg)  
可以看到时间轮算法的插入复杂度是O(1)，删除的复杂度也是O(1)，查找执行的复杂度也是O(1)，因此时间轮实现的定时器非常高效。  

## Cyber定时器实现

#### 用户接口
Timer对象是开放给用户的接口，主要实现了定时器的配置"TimerOption"，启动定时器和关闭定时器3个接口。我们首先看下定时器的配置。
```c++
  TimerOption(uint32_t period, std::function<void()> callback, bool oneshot)
      : period(period), callback(callback), oneshot(oneshot) {}
```
包括：定时器周期、回调函数、一次触发还是周期触发（默认为周期触发）。  

Timer对象主要的实现都在"Start()"中。  
```
void Timer::Start() {

  // 1. 首先判断定时器是否已经启动
  if (!started_.exchange(true)) {
    // 2. 初始化任务
    if (InitTimerTask()) {
      // 3. 在时间轮中增加任务
      timing_wheel_->AddTask(task_);
      AINFO << "start timer [" << task_->timer_id_ << "]";
    }
  }
}
``` 
Start中的步骤很简单:  
1. 判断定时器是否已经启动
2. 如果定时器没有启动，则初始化定时任务
3. 在时间轮中增加任务。

那么初始化任务中做了哪些事情呢？  
```c++
bool Timer::InitTimerTask() {

  // 1. 初始化定时任务
  task_.reset(new TimerTask(timer_id_));
  task_->interval_ms = timer_opt_.period;
  task_->next_fire_duration_ms = task_->interval_ms;
  // 2. 是否单次触发
  if (timer_opt_.oneshot) {
    std::weak_ptr<TimerTask> task_weak_ptr = task_;
    // 3. 注册任务回调
    task_->callback = [callback = this->timer_opt_.callback, task_weak_ptr]() {
      auto task = task_weak_ptr.lock();
      if (task) {
        std::lock_guard<std::mutex> lg(task->mutex);
        callback();
      }
    };
  } else {
    std::weak_ptr<TimerTask> task_weak_ptr = task_;
    // 注册任务回调
    task_->callback = [callback = this->timer_opt_.callback, task_weak_ptr]() {

      std::lock_guard<std::mutex> lg(task->mutex);
      auto start = Time::MonoTime().ToNanosecond();
      callback();
      auto end = Time::MonoTime().ToNanosecond();
      uint64_t execute_time_ns = end - start;

      if (task->last_execute_time_ns == 0) {
        task->last_execute_time_ns = start;
      } else {
        // start - task->last_execute_time_ns 为2次执行真实间隔时间，task->interval_ms是设定的间隔时间
        // 注意误差会修复补偿，因此这里用的是累计，2次误差会抵消，保持绝对误差为0
        task->accumulated_error_ns +=
            start - task->last_execute_time_ns - task->interval_ms * 1000000;
      }

      task->last_execute_time_ns = start;
      // 如果执行时间大于任务周期时间，则下一个tick马上执行
      if (execute_time_ms >= task->interval_ms) {
        task->next_fire_duration_ms = TIMER_RESOLUTION_MS;
      } else {
        int64_t accumulated_error_ms = ::llround(
            static_cast<double>(task->accumulated_error_ns) / 1e6);
        if (static_cast<int64_t>(task->interval_ms - execute_time_ms -
                                 TIMER_RESOLUTION_MS) >= accumulated_error_ms) {
          // 这里会补偿误差
          task->next_fire_duration_ms =
              task->interval_ms - execute_time_ms - accumulated_error_ms;
        } else {
          task->next_fire_duration_ms = TIMER_RESOLUTION_MS;
        }
 
      }
      TimingWheel::Instance()->AddTask(task);
    };
  }
  return true;
}
```
下面对Timer中初始化任务的过程做一些解释。  
1. 在Timer对象中创建Task任务并注册回调"task_->callback"，任务回调中首先会调用用户传入的"callback()"函数，然后把新的任务放入下一个时间轮bucket中，对应到代码就是"TimingWheel::Instance()->AddTask(task)"。  
2. task->next_fire_duration_ms是任务下一次执行的间隔，这个间隔是以task执行完成之后为起始时间的，因为每次插入新任务到时间轮都是在用户"callback"函数执行之后进行的，因此这里的时间起点也是以这个时间为准。
3. task->accumulated_error_ns是累计时间误差，注意这个误差是累计的，而且每次插入任务的时候都会修复这个误差，因此这个误差不会一直增大，也就是说假设你第一次执行的比较早，那么累计误差为负值，下次执行的时间间隔就会变长，如果第一次执行的时间比较晚，那么累计误差为正值，下次执行的时间间隔就会缩短。通过动态的调节，保持绝对的时间执行间隔一致。  
![timer_task](../img/timer_task.jpg)  


#### TimingWheel时间轮
接下来看时间轮TimingWheel的实现，TimingWheel时间轮的配置如下：
```
512个bucket
64个round    
tick 为2ms
``` 

TimingWheel是通过AddTask调用执行的，下面是具体过程。
```c++
void TimingWheel::AddTask(const std::shared_ptr<TimerTask>& task,
                          const uint64_t current_work_wheel_index) {
  // 1.不是运行状态则启动时间轮
  if (!running_) {
    // 2.启动Tick线程，并且加入scheduler调度。
    Start();
  }
  
  // 3. 计算一下轮bucket编号
  auto work_wheel_index = current_work_wheel_index +
                          task->next_fire_duration_ms / TIMER_RESOLUTION_MS;
  
  // 4. 如果超过最大的bucket数
  if (work_wheel_index >= WORK_WHEEL_SIZE) {
    auto real_work_wheel_index = GetWorkWheelIndex(work_wheel_index);
    task->remainder_interval_ms = real_work_wheel_index;
    auto assistant_ticks = work_wheel_index / WORK_WHEEL_SIZE;
    // 5.疑问，如果转了一圈之后，为什么直接加入剩余的bucket？？？
    if (assistant_ticks == 1 &&
        real_work_wheel_index != current_work_wheel_index_) {
      work_wheel_[real_work_wheel_index].AddTask(task);
      ADEBUG << "add task to work wheel. index :" << real_work_wheel_index;
    } else {
      auto assistant_wheel_index = 0;
      {
        // 6.如果超出，则放入上一级时间轮中
        std::lock_guard<std::mutex> lock(current_assistant_wheel_index_mutex_);
        assistant_wheel_index = GetAssistantWheelIndex(
            current_assistant_wheel_index_ + assistant_ticks);
        assistant_wheel_[assistant_wheel_index].AddTask(task);
      }
      ADEBUG << "add task to assistant wheel. index : "
             << assistant_wheel_index;
    }
  } else {
    // 7. 如果没有超过最大bucket数，则增加到对应的bucket中
    work_wheel_[work_wheel_index].AddTask(task);
    ADEBUG << "add task [" << task->timer_id_
           << "] to work wheel. index :" << work_wheel_index;
  }
}
```
1. 从上述过程可以看出Cyber的时间轮单独采用一个线程调度执行"std::thread([this]() { this->TickFunc(); })"，定时任务则放入协程池中去执行。也就是说主线程单独执行时间计数，而具体的定时任务开多个协程去执行，可以并发执行多个定时任务。定时任务中最好不要引入阻塞的操作，或者执行时间过长。   
2. Cyber定时器中引入了2级时间轮的方法（消息队列kafka中也是类似实现），类似时钟的小时指针和分钟指针，当一级时间轮触发完成之后，再移动到二级时间轮中执行。第二级时间轮不能超过一圈，因此定时器的最大定时时间为64*512*2ms，最大不超过约65s。 
![timing_wheel_multi](../img/timing_wheel_multi.jpg)     
 
#### Tick
接下来我们看下时间轮中的Tick是如何工作的。在上述"AddTask"中会调用"Start"函数启动一个线程，线程执行"TickFunc"。  
```c++
void TimingWheel::TickFunc() {
  Rate rate(TIMER_RESOLUTION_MS * 1000000);  // ms to ns
  // 1. 循环调用
  while (running_) {
    // 2. 执行bucket中的回调，并且删除当前bucket中的任务(回调中会增加新的任务到bucket)
    Tick();
    
    tick_count_++;
    // 3. 休眠一个Tick
    rate.Sleep();
    {
      std::lock_guard<std::mutex> lock(current_work_wheel_index_mutex_);
      // 4.获取当前bucket id，每次加1
      current_work_wheel_index_ =
          GetWorkWheelIndex(current_work_wheel_index_ + 1);
    }
    // 5.下一级时间轮已经转了一圈，上一级时间轮加1
    if (current_work_wheel_index_ == 0) {
      {
        // 6.上一级时间轮bucket id加1
        std::lock_guard<std::mutex> lock(current_assistant_wheel_index_mutex_);
        current_assistant_wheel_index_ =
            GetAssistantWheelIndex(current_assistant_wheel_index_ + 1);
      }
      // 7. 
      Cascade(current_assistant_wheel_index_);
    }
  }
}
```
这里需要注意假设二级时间轮中有一个任务的时间周期就为512，那么在当前bucket回调中又会在当前bucket中增加一个任务，那么这个任务会执行2次，如何解决这个问题呢？ Cyber中采用把这个任务放入上一级时间轮中，然后在触发一个周期之后，放到下一级的时间轮中触发。  


## 总结
经过上述分析，介绍了Cyber中定时器的实现原理，这里还有2个疑问。
1. 一是定时器是否为单线程，任务都是在单线程中的多个协程中执行？？？
2. 当"TimingWheel::AddTask"中"work_wheel_index >= WORK_WHEEL_SIZE"并且"assistant_ticks == 1"时，假设原始的current_work_wheel_index_mutex_ = 200，消息触发周期为600个tick，那么按照上述计算方法得到的work_wheel_index = 800，real_work_wheel_index = 288，assistant_ticks = 1，那么"work_wheel_[real_work_wheel_index].AddTask(task)"会往288增加任务，实际上这个任务在88个tick之后就触发了？？？




## data_fusion_
data_fusion_总是以第一个消息为基准，查找融合最新的消息。


## Transport
Transport 把消息通过 DataDispatcher 把消息放进buffer 并且触发 DataNotifier::Notify

notify之后会触发 协程执行 而协程会调用DataVisitor::TryFetch 去取数据

取到数据之后，调用process函数执行。

## ListenerHandler 和 RtpsDispatcher
是否为通知signal接收发送消息，对应一张线性表？？？

RtpsDispatcher 用来分发消息，同时触发ListenerHandler？？？


## Croutine调度
什么时候采用协程，用协程做了哪些工作？？？


#### scheduler, task和croutine
如果有一个新的任务需要处理，则调度器会创建一个任务，而任务又由协程去处理。
创建任务的时候DataVisitorBase在调度器中注册回调，这个回调触发调度器根据任务id进行NotifyProcessor

一个任务就是一个协程，协程负责调用reader enqueue读取消息，平时处于yeild状态，等到DataVisitor触发回调之后开始工作。


