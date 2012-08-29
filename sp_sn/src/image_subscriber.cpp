#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

// Added by jschoi, 2012-08-13
#include <std_msgs/String.h>
#include <sstream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::CvBridge bridge;
    try
    {
        //cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
        cvShowImage("view", bridge.imgMsgToCv(msg, "passthrough"));
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cvNamedWindow("view");
    cvStartWindowThread();
    image_transport::ImageTransport it(nh);

/*
	//image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe("camera03/image", 1, imageCallback);
	//image_transport::Subscriber sub = it.subscribe("axis_camera", 1, imageCallback);
*/

//----------------------------------------------------------------
// Added by jschoi, 2012-08-13
	std::stringstream ss;
	std_msgs::String msg;
	if(argc==1)
		ss << "camera03/image";
	else
		ss << argv[1];	// To get the name of topic from the first arguement
	msg.data = ss.str();

	ROS_INFO("argc=%d, topic = %s", argc, msg.data.c_str());
//----------------------------------------------------------------
	image_transport::Subscriber sub = it.subscribe(msg.data.c_str(), 1, imageCallback);


    ros::spin();
    cvDestroyWindow("view");
}
