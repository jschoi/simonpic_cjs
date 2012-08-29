/*
 * File:          sn_env_camera03_ctr.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */



/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/servo.h> 
#include <webots/camera.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


//--------------------------------------------------------------------------
//      Added by jschoi for ROS, 2012-07-16
#if POSIX_TIMERS <= 0
  #include <std_msgs/String.h>	// for Linux
#else
  #include "std_msgs/String.h"	// for Mac
#endif
#include <sstream>
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
//      Added by jschoi for ROS w/ image_transport, 2012-07-17
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
//--------------------------------------------------------------------------


#ifndef PI
#define PI  3.141592
#endif


//static int time_step = 0;


sensor_msgs::ImagePtr Camera2ImgsMsg(WbDeviceTag Dev, int ImageWidth, int ImageHeight, int bColor)
{
    sensor_msgs::ImagePtr image_msg;
    const unsigned char *image_wb = wb_camera_get_image(Dev);
    int x,y,xy;

    IplImage *wimage = cvCreateImage(cvSize(ImageWidth, ImageHeight),8,3);
    wimage->width = ImageWidth;
    wimage->height = ImageHeight;
    if (bColor != 0) {
      wimage->widthStep = 3*ImageWidth;  // DO NOT FORGET  (rgb: 3*width)
    }
    else {
      wimage->widthStep = ImageWidth;  // DO NOT FORGET  (rgb: 1*width)    
    }

    for (y=0; y<ImageHeight; y++) {
      for(x=0; x<ImageWidth; x++) {
        xy = y*ImageWidth + x;
				
        wimage->imageData[3*xy+0] = wb_camera_image_get_blue(image_wb, ImageWidth, x, y);
        wimage->imageData[3*xy+1] = wb_camera_image_get_green(image_wb, ImageWidth, x, y);
        wimage->imageData[3*xy+2] = wb_camera_image_get_red(image_wb, ImageWidth, x, y);
      }
    }
    
    //ROS_INFO("[width,height]=[%d,%d], [x,y,xy]=[%d,%d,%d]",ImageWidth,ImageHeight,x,y,xy);

    //image.SetIpl(wimage);

    //image.SetIpl( cvLoadImage("KIST.png", CV_LOAD_IMAGE_COLOR) );
    //image_msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");
    image_msg = sensor_msgs::CvBridge::cvToImgMsg(wimage,"bgr8");
    
    cvReleaseImage(&wimage);  // This is very very IMPORTANT in order not to have a memory leakage!
    
    return image_msg;
}


/*
 * You should put some helper functions here
 */

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
    // get time step and robot's devices
  int time_step  = wb_robot_get_basic_time_step();
    
  /*
   * You should declare here DeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   
  // get the pan/tilt camera device, enable it, and store its width and height
  char dev_name[32], topic_name[32];
  sprintf(dev_name, "sn_env_camera03");
  sprintf(topic_name, "%s/image",dev_name);  
  
  //WbDeviceTag ptcam = wb_robot_get_device("sn_env_camera03");
  WbDeviceTag ptcam = wb_robot_get_device(dev_name);
  
  wb_camera_enable(ptcam, time_step);
  int ptcam_width = wb_camera_get_width(ptcam);
  int ptcam_height = wb_camera_get_height(ptcam);   
  
  //////////////////////////////////
  // initialize servos
  WbDeviceTag pan_servo, tilt_servo;
  pan_servo = wb_robot_get_device("sn_env_camera03_pan_servo");
  tilt_servo = wb_robot_get_device("sn_env_camera03_tilt_servo"); 
  // init speeds
  double pan_speed = 1.0*PI; // rad/s
  double tilt_speed = 2.0*PI; // rad/s
  double pan_Kp = 40.;
  double tilt_Kp = 40.;
  double pan_angle, tilt_angle;
  
  wb_servo_set_velocity(pan_servo, pan_speed);  
  wb_servo_set_control_p(pan_servo, pan_Kp);
  wb_servo_set_acceleration(pan_servo, -1); // No limit in acceleration

  wb_servo_set_velocity(tilt_servo, tilt_speed);
  wb_servo_set_control_p(tilt_servo, tilt_Kp);
  wb_servo_set_acceleration(tilt_servo, -1); // No limit in acceleration
  
  
  pan_angle = PI/2.;  tilt_angle = PI/2.;
  //pan_angle = -PI/2.;  tilt_angle = -PI/2.;
  wb_servo_set_position(pan_servo, pan_angle);
  wb_servo_set_position(tilt_servo, tilt_angle);  
  
  //wb_servo_set_position(pan_servo, INFINITY); // for velocity control  
  
  wb_servo_enable_position(pan_servo, time_step);
  wb_servo_enable_position(tilt_servo, time_step);  
  //////////////////////////////////
  
  
  
  //--------------------------------------------------------------------------
  //      Added by jschoi for ROS w/ image_transport, 2012-07-17
  ros::init(argc, argv, "sn_env_camera03_ctr");
  
  //int time_step  = wb_robot_get_basic_time_step();
/*  
  WbDeviceTag kinect     = wb_robot_get_device("sn_env_kinect03");
  wb_camera_enable(kinect, time_step);
  int kinectWidth = wb_camera_get_width(kinect);
  int kinectHeight = wb_camera_get_height(kinect);
  int halfWidth   = kinectWidth/2;
  int viewHeight  = kinectHeight/2 + 10;
  double maxRange = wb_camera_get_max_range(kinect);
  double rangeThreshold = 1.5;
  double invMaxRangeTimesWidth = 1.0 / (maxRange * kinectWidth);
*/  
  ros::NodeHandle nh;


  //ros::Publisher pub = nh.advertise<sensor_msgs::Image>("sn_env_camera03/image",1);
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic_name,1);  
  //  We need to replace the above line with the following two lines (however, curretly those show error regarding pointer initialization)
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("kinect03/image", 1);
/*
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("kinect03/image", 1);
*/ 
 
  //cv::WImageBuffer3_b image( cvLoadImage("KIST.png", CV_LOAD_IMAGE_COLOR) );
  //sensor_msgs::ImagePtr image_msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");
  sensor_msgs::ImagePtr image_msg;

  ros::Rate loop_rate(5);
  //--------------------------------------------------------------------------
  
  
  
  /* main loop */
  do {
//////

    //--------------------------------------------------------------------------
    //      Added by jschoi for ROS w/ image_transport, 2012-07-17
    int bColor = 1;
    image_msg = Camera2ImgsMsg(ptcam, ptcam_width, ptcam_height, bColor);

    pub.publish(image_msg);
    ros::spinOnce();
//    loop_rate.sleep();

//////  
/*
    if(wb_servo_get_position(pan_servo) >= 0.99*PI/2.){
      pan_angle = 0;  tilt_angle = 0;
      wb_servo_set_position(pan_servo, pan_angle);
      wb_servo_set_position(tilt_servo, tilt_angle);
    } else if(wb_servo_get_position(pan_servo) <= 0.01*PI/2.){
      pan_angle = PI/2.;  tilt_angle = PI/2.;
      wb_servo_set_position(pan_servo, pan_angle);
      wb_servo_set_position(tilt_servo, tilt_angle);
    }
*/
    if(wb_servo_get_position(pan_servo) >= 0.99*PI/4.){
      pan_angle = -PI/4.;  tilt_angle = PI/9.;
      wb_servo_set_position(pan_servo, pan_angle);
      wb_servo_set_position(tilt_servo, tilt_angle);
    } else if(wb_servo_get_position(pan_servo) <= -0.99*PI/4.){
      pan_angle = PI/4.;  tilt_angle = PI/9.;
      wb_servo_set_position(pan_servo, pan_angle);
      wb_servo_set_position(tilt_servo, tilt_angle);
    }
        

    //---------------------------------------------------------------------
    
    
    /* 
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    
    /* Process sensor data here */
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
    
    /* 
     * Perform a simulation step of 64 milliseconds
     * and leave the loop when the simulation is over
     */
  } while (wb_robot_step(time_step) != -1);
  
  /* Enter here exit cleanup code */
  
  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();
  
  return 0;
}
