/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */
#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <signal.h>
#include <dji_sdk_demo/velocity.h>
#include <std_srvs/Empty.h>

using namespace DJI::onboardSDK;

/*自定义手机回调函数*/
void StartMission1Callback(DJIDrone *drone);//101
void StartMission2Callback(DJIDrone *drone);//102
void StartMission3Callback(DJIDrone *drone);//103
void StartMission4Callback(DJIDrone *drone);//104
void StopMissionCallback(DJIDrone *drone);//107

DJIDrone* drone;

/*调试用*/
int mission =0;
bool DEBUG = false;

void velocity_Receive_CallBack(const dji_sdk_demo::velocity& msg);

ros::Subscriber sub;
double forwardV,leftrV,heightV,yawV;

ros::ServiceClient start_tracking;  //start tracking
ros::ServiceClient stop_detecting;
ros::ServiceClient stop_tracking;

ros::ServiceClient track_stop;  //to track node
ros::ServiceClient detect_stop;

void Stop(int signo)
{
    drone->landing();
    cout<<"landing";
    exit(0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sdk_client");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);

    uint8_t userData = 0;

    ros::param::get("~mission",mission);
    ros::param::get("~DEBUG",DEBUG);

    forwardV = leftrV = heightV = yawV = 0;

    sub = nh.subscribe("velocity",1,velocity_Receive_CallBack);
    start_tracking = nh.serviceClient<std_srvs::Empty>("start_tracking");
    stop_detecting = nh.serviceClient<std_srvs::Empty>("stop_detecting");
    stop_tracking = nh.serviceClient<std_srvs::Empty>("stop_tracking");

    track_stop = nh.serviceClient<std_srvs::Empty>("track_stop");
    detect_stop = nh.serviceClient<std_srvs::Empty>("detect_stop");

    ros::spinOnce();
    {
        //Mobile callback
        drone->setStartMission1Callback(StartMission1Callback, &userData);//mission1
        drone->setStartMission2Callback(StartMission2Callback, &userData);//mission2
        drone->setStartMission3Callback(StartMission3Callback, &userData);//mission3
        drone->setStartMission4Callback(StartMission4Callback, &userData);//mission4
        drone->setStopMissionCallback(StopMissionCallback, &userData);//shutdown
    }
    signal(SIGINT,Stop);

     if(DEBUG)
    {
        switch(mission)
        {
           case 1:
             StartMission1Callback(drone);
             break;
           case 2:
             StartMission2Callback(drone);
             break;
           case 3:
             StartMission3Callback(drone);
             break;
          case 4:
             StartMission4Callback(drone);
             break;
        }
    }
     ros::AsyncSpinner spinner(1);
     spinner.start();
     ros::waitForShutdown();
     return 0;
}

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

void StartMission2Callback(DJIDrone *drone)
{
    std_srvs::Empty srv;
    start_tracking.call(srv);
}

void StartMission3Callback(DJIDrone *drone)
{
    std_srvs::Empty srv;
    if(stop_detecting.call(srv))
    {
        sleep(1);
        detect_stop.call(srv);
    }
}

void StartMission4Callback(DJIDrone *drone)
{
    std_srvs::Empty srv;
    if(stop_tracking.call(srv))
    {
        sleep(1);
        track_stop.call(srv);
    }
}

void StopMissionCallback(DJIDrone *drone)
{
     system("sudo poweroff");
}

void velocity_Receive_CallBack(const dji_sdk_demo::velocity &msg)
{
    forwardV = msg.forwardV;
    leftrV = msg.leftrV;
    heightV = msg.heightV;
    yawV = msg.yawV;
}
