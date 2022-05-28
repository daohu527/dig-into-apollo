# Data

> 业精于勤，荒于嬉；行成于思，毁于随。


## data目录结构
data目录的结构如下，
```
.
├── BUILD
├── README.md
├── conf
├── proto
├── channel_pool.cc
├── channel_pool.h           // 设置small_channels_
├── drive_event_trigger.cc
├── drive_event_trigger.h
├── emergency_mode_trigger.cc
├── emergency_mode_trigger.h
├── hard_brake_trigger.cc
├── hard_brake_trigger.h
├── interval_pool.cc
├── interval_pool.h
├── post_record_processor.cc
├── post_record_processor.h
├── realtime_record_processor.cc
├── realtime_record_processor.h
├── record_processor.cc
├── record_processor.h
├── regular_interval_trigger.cc
├── regular_interval_trigger.h
├── small_topics_trigger.cc
├── small_topics_trigger.h
├── smart_recorder.cc
├── smart_recorder_gflags.cc
├── smart_recorder_gflags.h
├── swerve_trigger.cc
├── swerve_trigger.h
├── trigger_base.cc
└── trigger_base.h
```

主要的实现分为2部分，一部分为trigger，一部分为record。
trigger主要的作用是生成对应的时间段，并且放到pool中。
record主要用来录制数据，分为在线和离线录制。主要是调用RecordWriter和RecordReader，这2个类实现实时读取。


## RecordViewer
RecordViewer实现了迭代器，并且可以读取文件和实时的流式数据？？
RecordViewer中包含RecordReader

```c++
reader->GetHeader()
```
以上的原理是什么？？如果是文件则从文件读取，如果不是文件应该如何？？

RecordViewer通过`Iterator`读取msg_buffer_中的消息，通过`FillBuffer`往msg_buffer_中写入消息，注意这里的"std::multimap"为什么要用map因为每次读取的时候是读取的1s内的reader消息，遍历多个reader的时候，那么时间顺序可能会打乱，因此这里采用红黑树的方式按照key进行排序，所以采用了multimap的结构。

每次在Update中更新消息，
```c++
bool RecordViewer::Update(RecordMessage* message) {
  bool find = false;
  do {
    // 1. 如果msg_buffer_为空，则填充buffer，填充1s内的消息
    if (msg_buffer_.empty() && !FillBuffer()) {
      break;
    }
    // 2. 找到buffer中的第一条消息，并且退出
    auto& msg = msg_buffer_.begin()->second;
    if (channels_.empty() || channels_.count(msg->channel_name) == 1) {
      *message = *msg;
      find = true;
    }
    msg_buffer_.erase(msg_buffer_.begin());
  } while (!find);

  return find;
}
```

下面我们看下Process的过程，也就是说实时过程先启动recorder_，然后再从文件中读取消息，然后再重新保存？？？
```c++
bool RealtimeRecordProcessor::Process() {
  // 保存到文件
  recorder_->Start();

  // Now fast reader follows and reacts for any events
  std::string record_path;
  do {
    if (!GetNextValidRecord(&record_path)) {
      break;
    }
    auto reader = std::make_shared<RecordReader>(record_path);
    RecordViewer viewer(reader, 0, std::numeric_limits<uint64_t>::max(),
                        ChannelPool::Instance()->GetAllChannels());

    if (restore_reader_time_ == 0) {
      restore_reader_time_ = viewer.begin_time();
      GetNextValidRecord(&restore_path_);
    }
    // 迭代器++调用RecordViewer::Update读取消息
    for (const auto& msg : viewer) {
      for (const auto& trigger : triggers_) {
        trigger->Pull(msg);
      }
      // 保存消息
      RestoreMessage(msg.time);
    }
  } while (!is_terminating_);  // 循环上述步骤
  // Try restore the rest of messages one last time
  RestoreMessage(std::numeric_limits<uint64_t>::max());
  if (monitor_thread && monitor_thread->joinable()) {
    monitor_thread->join();
    monitor_thread = nullptr;
  }
  return true;
}
```

Recorder开始Start
```c++
bool Recorder::Start() {
  // 1. 创建文件
  writer_.reset(new RecordWriter(header_));
  if (!writer_->Open(output_)) {
    return false;
  }
  std::string node_name = "cyber_recorder_record_" + std::to_string(getpid());
  node_ = ::apollo::cyber::CreateNode(node_name);
  // 2. 初始化reader
  if (!InitReadersImpl()) {
    return false;
  }
  message_count_ = 0;
  message_time_ = 0;
  is_started_ = true;

  // 3. 显示进度
  display_thread_ =
      std::make_shared<std::thread>([this]() { this->ShowProgress(); });

  return true;
}
```


InitReadersImpl
```c++
bool Recorder::InitReadersImpl() {
  std::shared_ptr<ChannelManager> channel_manager =
      TopologyManager::Instance()->channel_manager();

  // get historical writers
  std::vector<proto::RoleAttributes> role_attr_vec;
  channel_manager->GetWriters(&role_attr_vec);
  for (auto role_attr : role_attr_vec) {
    FindNewChannel(role_attr);
  }

  // listen new writers in future
  change_conn_ = channel_manager->AddChangeListener(
      std::bind(&Recorder::TopologyCallback, this, std::placeholders::_1));
  if (!change_conn_.IsConnected()) {
    AERROR << "change connection is not connected";
    return false;
  }
  return true;
}
```

InitReaderImpl
注册callback给reader，然后写入文件
```c++
bool Recorder::InitReaderImpl(const std::string& channel_name,
                              const std::string& message_type) {
  try {
    std::weak_ptr<Recorder> weak_this = shared_from_this();
    std::shared_ptr<ReaderBase> reader = nullptr;
    auto callback = [weak_this, channel_name](
                        const std::shared_ptr<RawMessage>& raw_message) {
      auto share_this = weak_this.lock();
      if (!share_this) {
        return;
      }
      share_this->ReaderCallback(raw_message, channel_name);
    };
    ReaderConfig config;
    config.channel_name = channel_name;
    config.pending_queue_size =
        gflags::Int32FromEnv("CYBER_PENDING_QUEUE_SIZE", 50);
    reader = node_->CreateReader<RawMessage>(config, callback);
    if (reader == nullptr) {
      AERROR << "Create reader failed.";
      return false;
    }
    channel_reader_map_[channel_name] = reader;
    return true;
  } catch (const std::bad_weak_ptr& e) {
    AERROR << e.what();
    return false;
  }
}
```




