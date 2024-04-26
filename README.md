# semubot_audio 

ROS2 wrapper for the ReSpeaker 4 Mic Array. Publishes audio and direction-of-arrival information 

Based on [previous](https://github.com/furushchev/respeaker_ros) [wrappers](https://github.com/machinekoder/respeaker) for ROS. <br/>

Prerequisites:
* Copy the udev rules from `respeaker_ros/config` into `/etc/udev/rules.d/`.

* Build [audio_common](https://github.com/ros-drivers/audio_common) from source until it gets released into ROS2, you need the ros2 branch. <br/>
  ```git clone -b ros2 https://github.com/ros-drivers/audio_common.git```

