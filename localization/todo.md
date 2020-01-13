## RTKLocalization
RTKLocalization类的主要功能是实现RTK定位，然后用IMU做插值。
1. 只是通过GPS得到了位置，但是IMU的融合还没有进行计算？？？
2. 姿态解算是如何进行的？？？


#### InitConfig
```
void RTKLocalization::InitConfig(const rtk_config::Config &config) {
  // imu队列最大长度
  imu_list_max_size_ = config.imu_list_max_size();
  // GPS和IMU最大时间间隔
  gps_imu_time_diff_threshold_ = config.gps_imu_time_diff_threshold();
  // 地图偏移
  map_offset_[0] = config.map_offset_x();
  map_offset_[1] = config.map_offset_y();
  map_offset_[2] = config.map_offset_z();
}
```

#### GpsCallback
```c++
void RTKLocalization::GpsCallback(
    const std::shared_ptr<localization::Gps> &gps_msg) {
  double time_delay =
      last_received_timestamp_sec_
          ? common::time::Clock::NowInSeconds() - last_received_timestamp_sec_
          : last_received_timestamp_sec_;
  // GPS超时
  if (time_delay > gps_time_delay_tolerance_) {
    std::stringstream ss;
    ss << "GPS message time interval: " << time_delay;
    monitor_logger_.WARN(ss.str());
  }

  {
    std::unique_lock<std::mutex> lock(imu_list_mutex_);
    // 判断IMU消息是否为空，为空则返回
    if (imu_list_.empty()) {
      AERROR << "IMU message buffer is empty.";
      if (service_started_) {
        monitor_logger_.ERROR("IMU message buffer is empty.");
      }
      return;
    }
  }

  {
    std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
    // 判断GPS状态是否为空，为空则返回
    if (gps_status_list_.empty()) {
      AERROR << "Gps status message buffer is empty.";
      if (service_started_) {
        monitor_logger_.ERROR("Gps status message buffer is empty.");
      }
      return;
    }
  }

  // publish localization messages
  // 主要的处理过程都在该函数中
  PrepareLocalizationMsg(*gps_msg, &last_localization_result_,
                         &last_localization_status_result_);

  // 开启定位服务，并且记录服务开始时间
  service_started_ = true;
  if (service_started_time == 0.0) {
    service_started_time = common::time::Clock::NowInSeconds();
  }

  // watch dog
  // 看门狗，主要根据时间来判断？？？
  RunWatchDog(gps_msg->header().timestamp_sec());

  // 最后一次消息接收时间
  last_received_timestamp_sec_ = common::time::Clock::NowInSeconds();
}
```

#### 开门狗
```
void RTKLocalization::RunWatchDog(double gps_timestamp) {
  if (!enable_watch_dog_) {
    return;
  }

  // 定位服务启动后，GPS超时则设置msg_delay为真
  // check GPS time stamp against system time
  double gps_delay_sec = common::time::Clock::NowInSeconds() - gps_timestamp;
  double gps_service_delay =
      common::time::Clock::NowInSeconds() - service_started_time;
  int64_t gps_delay_cycle_cnt =
      static_cast<int64_t>(gps_delay_sec * localization_publish_freq_);

  bool msg_delay = false;
  if (gps_delay_cycle_cnt > report_threshold_err_num_ &&
      static_cast<int>(gps_service_delay) > service_delay_threshold) {
    msg_delay = true;
    std::stringstream ss;
    ss << "Raw GPS Message Delay. GPS message is " << gps_delay_cycle_cnt
       << " cycle " << gps_delay_sec << " sec behind current time.";
    monitor_logger_.ERROR(ss.str());
  }
   
  // 定位服务启动之后，IMU超时则设置msg_delay为真
  // check IMU time stamp against system time
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_msg = imu_list_.back();
  lock.unlock();
  double imu_delay_sec =
      common::time::Clock::NowInSeconds() - imu_msg.header().timestamp_sec();
  int64_t imu_delay_cycle_cnt =
      static_cast<int64_t>(imu_delay_sec * localization_publish_freq_);
  if (imu_delay_cycle_cnt > report_threshold_err_num_ &&
      static_cast<int>(gps_service_delay) > service_delay_threshold) {
    msg_delay = true;
    std::stringstream ss;
    ss << "Raw IMU Message Delay. IMU message is " << imu_delay_cycle_cnt
       << " cycle " << imu_delay_sec << " sec behind current time.";
    monitor_logger_.ERROR(ss.str());
  }

  // 第一次触发之后，然后每隔1s触发一次
  // to prevent it from beeping continuously
  if (msg_delay && (last_reported_timestamp_sec_ < 1. ||
                    common::time::Clock::NowInSeconds() >
                        last_reported_timestamp_sec_ + 1.)) {
    AERROR << "gps/imu frame Delay!";
    last_reported_timestamp_sec_ = common::time::Clock::NowInSeconds();
  }
}
```

#### 定位主流程
```
void RTKLocalization::PrepareLocalizationMsg(
    const localization::Gps &gps_msg, LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {
  // 根据GPS时间寻找匹配的IMU消息
  // find the matching gps and imu message
  double gps_time_stamp = gps_msg.header().timestamp_sec();
  CorrectedImu imu_msg;
  FindMatchingIMU(gps_time_stamp, &imu_msg);

  // 构建定位消息
  ComposeLocalizationMsg(gps_msg, imu_msg, localization);


  drivers::gnss::InsStat gps_status;
  // 查找最近的GPS状态信息
  FindNearestGpsStatus(gps_time_stamp, &gps_status);
  // 填充定位状态信息
  FillLocalizationStatusMsg(gps_status, localization_status);
}
```

#### 找到正确的IMU消息
在队列中找到最匹配的IMU，其中区分了队列的第一个，最后一个，以及如果在中间则进行插值。插值的时候根据距离最近的原则进行反比例插值。

```
bool RTKLocalization::FindMatchingIMU(const double gps_timestamp_sec,
                                      CorrectedImu *imu_msg) {
  // 1. 判断消息是否为空
  if (imu_msg == nullptr) {
    AERROR << "imu_msg should NOT be nullptr.";
    return false;
  }
  // 加锁，这里有疑问，为什么换个变量就没有锁了呢？
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_list = imu_list_;
  lock.unlock();

  // IMU为空
  if (imu_list.empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty! GPS timestamp[" << gps_timestamp_sec
           << "]";
    return false;
  }

  // 
  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  auto imu_it = imu_list.begin();
  for (; imu_it != imu_list.end(); ++imu_it) {
    if ((*imu_it).header().timestamp_sec() - gps_timestamp_sec >
        std::numeric_limits<double>::min()) {
      break;
    }
  }

  if (imu_it != imu_list.end()) {  // found one
    if (imu_it == imu_list.begin()) {
      AERROR << "IMU queue too short or request too old. "
             << "Oldest timestamp[" << imu_list.front().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_list.back().header().timestamp_sec() << "], GPS timestamp["
             << gps_timestamp_sec << "]";
      *imu_msg = imu_list.front();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      if (!(*imu_it).has_header() || !(*imu_it_1).has_header()) {
        AERROR << "imu1 and imu_it_1 must both have header.";
        return false;
      }
      if (!InterpolateIMU(*imu_it_1, *imu_it, gps_timestamp_sec, imu_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // 如果没有找到，则取最新的20ms以内的消息
    // give the newest imu, without extrapolation
    *imu_msg = imu_list.back();
    if (imu_msg == nullptr) {
      AERROR << "Fail to get latest observed imu_msg.";
      return false;
    }

    if (!imu_msg->has_header()) {
      AERROR << "imu_msg must have header.";
      return false;
    }

    if (std::fabs(imu_msg->header().timestamp_sec() - gps_timestamp_sec) >
        gps_imu_time_diff_threshold_) {
      // 20ms threshold to report error
      AERROR << "Cannot find Matching IMU. IMU messages too old. "
             << "Newest timestamp[" << imu_list.back().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
    }
  }

  return true;
}
```

#### IMU插值
```c++
bool RTKLocalization::InterpolateIMU(const CorrectedImu &imu1,
                                     const CorrectedImu &imu2,
                                     const double timestamp_sec,
                                     CorrectedImu *imu_msg) {
  if (!(imu1.header().has_timestamp_sec() &&
        imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - imu1.header().timestamp_sec() <
      std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU1]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU2]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << imu2.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else {
    // 线性插值
    *imu_msg = imu1;
    imu_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        imu2.header().timestamp_sec() - imu1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;

      if (imu1.imu().has_angular_velocity() &&
          imu2.imu().has_angular_velocity()) {
        auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                  imu2.imu().angular_velocity(), frac1);
        imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
      }

      if (imu1.imu().has_linear_acceleration() &&
          imu2.imu().has_linear_acceleration()) {
        auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                  imu2.imu().linear_acceleration(), frac1);
        imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
      }

      if (imu1.imu().has_euler_angles() && imu2.imu().has_euler_angles()) {
        auto val = InterpolateXYZ(imu1.imu().euler_angles(),
                                  imu2.imu().euler_angles(), frac1);
        imu_msg->mutable_imu()->mutable_euler_angles()->CopyFrom(val);
      }
    }
  }
  return true;
}
```

#### 线性插值算法
根据距离插值，反比例，即frac1越小，则越靠近p1，frac1越大，则越靠近p2
```
template <class T>
T RTKLocalization::InterpolateXYZ(const T &p1, const T &p2,
                                  const double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}
```

#### 填充定位消息
找到最匹配的IMU消息后，和GPS消息做融合。IMU的角度是否不随着物体的旋转而改变？？？
涉及到姿态解算
https://blog.csdn.net/MOU_IT/article/details/80369043
https://zhuanlan.zhihu.com/p/79894982
https://zhuanlan.zhihu.com/p/20382236

位置，航向，线速度是GPS的
加速度，角速度，欧拉角是IMU的

```c++
void RTKLocalization::ComposeLocalizationMsg(
    const localization::Gps &gps_msg, const localization::CorrectedImu &imu_msg,
    LocalizationEstimate *localization) {
  localization->Clear();

  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(gps_msg.header().timestamp_sec());

  // combine gps and imu
  auto mutable_pose = localization->mutable_pose();
  // GPS消息包含位置信息
  if (gps_msg.has_localization()) {
    const auto &pose = gps_msg.localization();
    // 1. 获取位置
    if (pose.has_position()) {
      // position
      // world frame -> map frame
      mutable_pose->mutable_position()->set_x(pose.position().x() -
                                              map_offset_[0]);
      mutable_pose->mutable_position()->set_y(pose.position().y() -
                                              map_offset_[1]);
      mutable_pose->mutable_position()->set_z(pose.position().z() -
                                              map_offset_[2]);
    }
    // 2. 获取方向
    // orientation
    if (pose.has_orientation()) {
      mutable_pose->mutable_orientation()->CopyFrom(pose.orientation());
      double heading = common::math::QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz());
      mutable_pose->set_heading(heading);
    }
    // linear velocity
    // 获取线速度
    if (pose.has_linear_velocity()) {
      mutable_pose->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
    }
  }

  if (imu_msg.has_imu()) {
    const auto &imu = imu_msg.imu();
    // linear acceleration
    // 获取imu的线性
    if (imu.has_linear_acceleration()) {
      if (localization->pose().has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        // 为什么需要做旋转？？？转换为车当前方向的速度？？？
        Vector3d orig(imu.linear_acceleration().x(),
                      imu.linear_acceleration().y(),
                      imu.linear_acceleration().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_linear_acceleration()->set_x(vec[0]);
        mutable_pose->mutable_linear_acceleration()->set_y(vec[1]);
        mutable_pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vfr
        // 设置线性加速度
        mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    }

    //设置角速度，也需要根据航向转换
    // angular velocity
    if (imu.has_angular_velocity()) {
      if (localization->pose().has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.angular_velocity().x(), imu.angular_velocity().y(),
                      imu.angular_velocity().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_angular_velocity()->set_x(vec[0]);
        mutable_pose->mutable_angular_velocity()->set_y(vec[1]);
        mutable_pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vf
        mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
            imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: fail to convert angular_velocity";
      }
    }

    // 设置欧拉角
    // euler angle
    if (imu.has_euler_angles()) {
      mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
    }
  }
}
```

#### 设置localiztion消息头
填充消息头
```
void RTKLocalization::FillLocalizationMsgHeader(
    LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(static_cast<unsigned int>(++localization_seq_num_));
}
```

## 查找最新的GPS状态
GPS状态列表可能是乱序的吗？不是按照时间顺序排列的？？？

```
bool RTKLocalization::FindNearestGpsStatus(const double gps_timestamp_sec,
                                           drivers::gnss::InsStat *status) {
  CHECK_NOTNULL(status);

  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  auto gps_status_list = gps_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 1e8;
  auto nearest_itr = gps_status_list.end();
  for (auto itr = gps_status_list.begin(); itr != gps_status_list.end();
       ++itr) {
    double diff = std::abs(itr->header().timestamp_sec() - gps_timestamp_sec);
    if (diff < timestamp_diff_sec) {
      timestamp_diff_sec = diff;
      nearest_itr = itr;
    }
  }

  if (nearest_itr == gps_status_list.end()) {
    return false;
  }

  if (timestamp_diff_sec > gps_status_time_diff_threshold_) {
    return false;
  }

  *status = *nearest_itr;
  return true;
}
```

## 增加位置的状态信息
设置位置的状态，主要是为RTK的状态信息
```
void RTKLocalization::FillLocalizationStatusMsg(
    const drivers::gnss::InsStat &status,
    LocalizationStatus *localization_status) {
  apollo::common::Header *header = localization_status->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_timestamp_sec(timestamp);
  localization_status->set_measurement_time(status.header().timestamp_sec());
  
  // 如果没有pose type则返回错误
  if (!status.has_pos_type()) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Status Is Missing.");
    return;
  }

  auto pos_type = static_cast<drivers::gnss::SolutionType>(status.pos_type());
  switch (pos_type) {
    // RTK FIXED状态
    case drivers::gnss::SolutionType::INS_RTKFIXED:
      localization_status->set_fusion_status(MeasureState::OK);
      localization_status->set_state_message("");
      break;
    // RTK FLOAT状态
    case drivers::gnss::SolutionType::INS_RTKFLOAT:
      localization_status->set_fusion_status(MeasureState::WARNNING);
      localization_status->set_state_message(
          "Warning: Current Localization Is Unstable.");
      break;
    default:
      localization_status->set_fusion_status(MeasureState::ERROR);
      localization_status->set_state_message(
          "Error: Current Localization Is Very Unstable.");
      break;
  }
}
```

## PublishPoseBroadcastTopic
输出无人车的位置，为什么没有通过IMU解算真实的位置信息？？？
```
void RTKLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}
```

## PublishPoseBroadcastTF
输出坐标转换信息，主要发布位置、航向。
```
void RTKLocalizationComponent::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  apollo::transform::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_timestamp_sec(localization.measurement_time());
  mutable_head->set_frame_id(broadcast_tf_frame_id_);
  tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

  auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
  mutable_translation->set_x(localization.pose().position().x());
  mutable_translation->set_y(localization.pose().position().y());
  mutable_translation->set_z(localization.pose().position().z());

  auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
  mutable_rotation->set_qx(localization.pose().orientation().qx());
  mutable_rotation->set_qy(localization.pose().orientation().qy());
  mutable_rotation->set_qz(localization.pose().orientation().qz());
  mutable_rotation->set_qw(localization.pose().orientation().qw());

  tf2_broadcaster_->SendTransform(tf2_msg);
}
```

## PublishLocalizationStatus
发布位置状态信息
```
void RTKLocalizationComponent::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}
```


# NDT
1. NDT的原理？

NDT mapping 
NDT match

我们主要应用NDT（Normal Distributions Transform，正态分布变换）或者其他SLAM算法来完成稠密点云地图的构建。
http://xchu.net/2019/09/27/HDMAP%E5%BB%BA%E5%9B%BE%E6%B5%81%E7%A8%8B/  
http://xchu.net/2019/10/11/autoware%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%BB%BA%E5%9B%BE%E5%92%8C%E5%AE%9A%E4%BD%8D/  


## 
