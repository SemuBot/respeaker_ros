# semubot_audio 

ROS2 wrapper for the ReSpeaker 4 Mic Array. Publishes audio and direction-of-arrival information. 

Based on [previous](https://github.com/furushchev/respeaker_ros) [wrappers](https://github.com/machinekoder/respeaker) for ROS. <br/>

# Prerequisites:
* For ROS2 Humble installation refer to here: <br/>
https://docs.ros.org/en/humble/Installation.html

* Copy the udev rules from `respeaker_ros/config` into `/etc/udev/rules.d/`. 

* ReSpeaker setup:
1.
2.
3.


* Build [audio_common](https://github.com/ros-drivers/audio_common) from source until it gets released into ROS2, you need the ros2 branch: <br/>
1.  ```cd your_workspace```<br/>
2.  ```mkdir src```<br/>
3.  ```git clone -b ros2 https://github.com/ros-drivers/audio_common.git```  <br/>
4.  ```cd ..``` <br/>
5.  ```colcon build``` <br/>

* In case of an error on ROS2 Humble regarding `ament_python_install_package` refer to `https://github.com/ros2/rosidl_python/pull/187` (easy fix).

