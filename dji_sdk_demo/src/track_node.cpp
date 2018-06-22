#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <msgs/BoundingBox.h>
#include <msgs/DetectedTargets.h>
#include <dji_sdk_demo/velocity.h>
#include <dji_sdk/LocalPosition.h>
#include <std_srvs/Empty.h>
#include <PeopleTrackControl.h>

class Track
{
public:
    Track()
    {
        ros::param::get("~max_v",max_v);
        ros::param::get("~fixed_height",fixed_height);
        ros::param::get("~gps_available",gps_available);

        if(!gps_available)
            oppositeHeight = fixed_height;

        sub1 = nh.subscribe("tracked_bb",1,&Track::trackBB_ReceiveCallBack,this);
        sub2 = nh.subscribe("local_position", 1,&Track::local_position_subscriber_callback,this);
        sub3 = nh.subscribe("image",1,&Track::image_Receive_CallBack,this);
        sub4 = nh.subscribe("detected_peoples",1,&Track::detected_peoples_ReceiveCallBack,this);

        track_stop_srv = nh.advertiseService("track_stop",&Track::track_stop_CB,this);
        detect_stop_srv = nh.advertiseService("detect_stop",&Track::detect_stop_CB,this);

        pub = nh.advertise<dji_sdk_demo::velocity>("velocity",1);
    }

    ~Track(){}

    void process(const double &cameraInfo, bool store_video = false,const std::string &fileAddr = "/home/ubuntu/Desktop/")
    {
        ros::Rate loop_rate(50);
        while(ros::ok())
        {
            ros::spinOnce();
            if(imageReceived&&!image.empty())
            {
                if(!inited)
                {
                    trackerInit(cameraInfo);
                    if(store_video)
                        videoInit(fileAddr);
                    inited = true;
                }

                sendVelocity();

                drawBoxsAndShow();

                if(store_video)
                    storeVideo();
            }
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;

    ros::ServiceServer track_stop_srv;
    ros::ServiceServer detect_stop_srv;

    double max_v = 0;
    double fixed_height = 0;
    bool gps_available = false;

    PeopleTrackControl tracker;

    double oppositeHeight = 0;

    bool inited = false;
    bool imageReceived = false;
    bool trackReady = false;

    cv::Mat image;
    std::vector<cv::Rect>rects;

    cv::VideoWriter writer;

    void sendVelocity()
    {
        const PeopleTrackControl::Vec4 &v = tracker.getVelocity(oppositeHeight);

        dji_sdk_demo::velocity velocity;
        velocity.forwardV = v[0];
        velocity.leftrV = v[1];
        velocity.heightV = v[2];
        velocity.yawV = v[3];

        //std::cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<std::endl;
        pub.publish(velocity);
    }
    void drawBoxsAndShow()
    {
        for(auto it = rects.cbegin();it!=rects.cend();++it)
            cv::rectangle(image, *it, cv::Scalar(0, 255, 0), 2, 8);
        if(trackReady)
            cv::rectangle(image, tracker.target, cv::Scalar(0, 0, 255), 2, 8);

        cv::imshow("图像",image);
        cv::waitKey(1);
    }

    void videoInit(const std::string &fileAddr)
    {
        int t = cv::getTickCount();
        std::string fileName_ = fileAddr+std::to_string(t);
        std::string fileName = fileName_+".avi";
        writer = cv::VideoWriter(fileName,CV_FOURCC('F','L','V','1'),25.0,cv::Size(image.cols,image.rows));
    }

    void trackerInit(const double &cameraInfo)
    {
        tracker.setParams(cameraInfo,max_v).setImgSize(image.cols,image.rows).setTarget(cv::Rect(cv::Point(image.cols/2,image.rows/2),cv::Point(image.cols/2+1,image.rows/2+1)));
    }

    void storeVideo()
    {
        writer<<image;
    }

    void trackBB_ReceiveCallBack(const msgs::BoundingBoxConstPtr &msg)
    {
        ROS_ASSERT(msg->width > 0);
        ROS_ASSERT(msg->height > 0);

        tracker.setTarget(cv::Rect(cv::Point(msg->x,msg->y),cv::Point(msg->x+msg->width,msg->y+msg->height)));
        trackReady = true;
    }
    void detected_peoples_ReceiveCallBack(const msgs::DetectedTargets &msg)
    {
        rects.clear();
        for(auto it=msg.BoundingBoxs.cbegin();it!=msg.BoundingBoxs.cend();++it)
        {
            cv::Rect rect(cv::Point(it->x,it->y),cv::Point(it->x+it->width,it->y+it->height));
            rects.push_back(rect);
        }
    }

    void local_position_subscriber_callback(const dji_sdk::LocalPosition &msg)
    {
        if(gps_available)
            oppositeHeight = msg.z;
    }
    void image_Receive_CallBack(const sensor_msgs::Image &msg)
    {
        cv_bridge::CvImagePtr img;
        img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img->image.copyTo(image);

        imageReceived = true;
    }

    bool track_stop_CB(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        trackReady = false;
        tracker.setTarget(cv::Rect(cv::Point(image.cols/2,image.rows/2),cv::Point(image.cols/2+1,image.rows/2+1)));

        return true;
    }

    bool detect_stop_CB(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        rects.clear();

        return true;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"track_node");

    Track *tracker = new Track();

    tracker->process(740.0,true);

    delete tracker;

    return 0;
}
