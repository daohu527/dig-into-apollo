## todo 

tf_static的问题：  
1. 为什么把obs_sensor2novatel_tf2_frame_id改为localization了？？？  
2. 为什么发布的时候需要传入node，即多个node往一个topic发送，还是说多个线程访问同一个node（location和perception同时通过一个node发布topic）？  
3. 为什么需要根据child_frame_id做过滤，即使child_frame_id不能一样，否则会后面会覆盖前面的？（StaticTransformComponent::SendTransform）  
4. 订阅的时候是每个模块都有listener吗，即遵循tf的设计原则。即代码可以复用，但是实例是每个模块自己new一个。  
5. Buffer::SubscriptionCallbackImpl 中为什么时间戳比更新就清空整个List？
```
  if (now.ToNanosecond() < last_update_.ToNanosecond()) {
    AINFO << "Detected jump back in time. Clearing TF buffer.";
    clear();
    // cache static transform stamped again.
    for (auto& msg : static_msgs_) {
      setTransform(msg, authority, true);
    }
  }
```
6. Buffer是一个单例，"apollo::transform::Buffer"提供了查询接口，即每个模块共享这个单例，和tf的设计原则还是不相符合？ 注意线程安全？  
