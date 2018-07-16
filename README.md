# DJIM100-people-detect-track

A ros demo for people detection and tracking on DJI M100 drone

## Building project
### 1.Prerequisites
* [ROS](http://wiki.ros.org/ROS/Installation)
* [OnboardSDK](https://github.com/dji-sdk/Onboard-SDK-ROS/tree/3.2)
* [MobileSDK](https://github.com/dji-sdk/Mobile-SDK-Android)
* [people_detect pkg](https://github.com/FanKaii/ros_people_detect/tree/master/people_detect)
* [kcf_tracker pkg](https://github.com/FanKaii/ros_kcf)
* [msgs pkg](https://github.com/FanKaii/ros_people_detect/tree/master/msgs)

### 2.Create a workspace and compile
`mkdir -p ~/catkin_ws/src`<br>

next, copy all packages to `/catkin_ws/src` and<br>

`catkin_make`<br>

## Usage 
* Set your own image topic

  find [ros_people_detect.launch](https://github.com/FanKaii/ros_people_detect/blob/master/people_detect/launch/ros_people_detect.launch), [ros_kcf_node.launch](https://github.com/FanKaii/ros_kcf/blob/master/ros_kcf/launch/ros_kcf_node.launch), [dji_sdk_client.launch](https://github.com/FanKaii/DJIM100-people-detect-track/blob/master/dji_sdk_demo/launch/dji_sdk_client.launch), and replace `/image_pub/image` with your own image topic.

* Run dji_sdk_client and dji_sdk

  `roslaunch dji_sdk_demo dji_sdk_client.launch`
  `roslaunch dji_sdk sdk_manifold.launch`
  
* Run people_detect node

  `roslaunch people_detect ros_people_detect.launch`
  
* Run ros_kcf node

  `roslaunch ros_kcf ros_kcf_node.launch`
  
* Remote control

  Now, you can use your remote control with custom functions to control the stop and start of tracking and detection. The control interface is defined in [client.cpp](https://github.com/FanKaii/DJIM100-people-detect-track/blob/master/dji_sdk_demo/src/client.cpp).

  ```
  void StartMission1Callback(DJIDrone *drone)
  {
      drone->request_sdk_permission_control();
      sleep(1);

      ros::Rate loop_rate(50);

      while(ros::ok())
      {
          ros::spinOnce();
          drone->attitude_control(0x4B,forwardV,leftrV,heightV,yawV);
          cout<<fixed<<setprecision(2)<<"          "<<forwardV<<"  "<<leftrV<<"  "<<heightV<<"  "<<yawV<<endl;
          loop_rate.sleep();
      }
  }
  ```
  
  This callback function is used to start the task, that is to start the autonomous detection and tracking.

  ```
  void StartMission2Callback(DJIDrone *drone)
  {
      std_srvs::Empty srv;
      start_tracking.call(srv);
  }
  ```
  
  This callback function is used to start tracking, that is, people_detect node give a [target](https://github.com/FanKaii/ros_people_detect/blob/master/msgs/msg/Target.msg) to kcf_track node. In fact, this goal is the closest person to the center of the image, and if there is no people is detected, the tracking program will not run, but when any people is detected again, the tracking program will automatic run.

  ```
  void StartMission3Callback(DJIDrone *drone)
  {
      std_srvs::Empty srv;
      if(stop_detecting.call(srv))
      {
          sleep(1);
          detect_stop.call(srv);
      }
  }
  ```

  This callback function is used to start and stop people detecting.

  ```
  void StartMission4Callback(DJIDrone *drone)
  {
      std_srvs::Empty srv;
      if(stop_tracking.call(srv))
      {
          sleep(1);
          track_stop.call(srv);
      }
  }
  ```
  This callback function is used to stop kcf tracker.

## Show results

  **videos**:<br>
  [autonomous detection and tracking.mp4](https://v.youku.com/v_show/id_XMzcyNjUwNDkzMg==.html?spm=a2h0k.11417342.soresults.dposter)<br>

  ![img1 load error](https://github.com/FanKaii/ros_people_detect/blob/master/image/img1.png)
  ![img2 load error](https://github.com/FanKaii/ros_people_detect/blob/master/image/img2.png)
