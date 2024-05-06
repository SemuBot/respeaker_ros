# semubot_audio 

ROS2 wrapper for the ReSpeaker 4 Mic Array. Publishes audio and direction-of-arrival information. 

Based on [previous](https://github.com/furushchev/respeaker_ros) [wrappers](https://github.com/machinekoder/respeaker) for ROS. <br/>

# Prerequisites:
* For ROS2 Humble installation refer to here: <br/>
https://docs.ros.org/en/humble/Installation.html

* Copy the udev rules from `respeaker_ros/config/40-respeaker.rules` into `/etc/udev/rules.d/`. <br/>
`!!!` There are different files in the `/config` folder, try putting the `40-respeaker.rules` first, in case of an error, try the second config `60-respeaker.rules`. <br/>

* ```sudo systemctl restart udev``` <br/>
* Then reconnect your device. <br/>

* ReSpeaker setup:
1. sudo pip install -r requirements.txt
2. sudo apt install python3-pyaudio
3. git clone https://github.com/respeaker/usb_4_mic_array.git
4. cd usb_4_mic_array
5. sudo python dfu.py --download 6_channels_firmware.bin  # The 6 channels version 


* Build [audio_common](https://github.com/ros-drivers/audio_common) from source until it gets released into ROS2, you need the ros2 branch: <br/>
1.  ```cd your_workspace```<br/>
2.  ```mkdir src```<br/>
3.  ```cd src```<br/>
4.  ```git clone -b ros2 https://github.com/ros-drivers/audio_common.git```  <br/>



* Then: <br/>
5.  ```git clone https://github.com/SemuBot/semubot_audio.git``` <br/>
6.  ```cd ..``` <br/>
7.  ```rosdep install --from-paths src -y --ignore-src``` <br/>
8.  ```colcon build --packages-select audio_common_msgs audio_capture audio_play``` <br/>
9.  ```colcon build --packages-select respeaker_ros``` <br/>



* Finally, open another terminal to run the node: <br/>
10.  ```source install/local_setup.bash``` <br/>
11.  ```ros2 run respeaker_ros respeaker_node``` <br/>

* If you need `sound_play` from `audio_common`, then build your workspace with the `colcon build` command, but the sound_play module might cause errors. <br/>
* In case of an error on ROS2 Humble regarding `ament_python_install_package` refer to `https://github.com/ros2/rosidl_python/pull/187` (easy fix).

