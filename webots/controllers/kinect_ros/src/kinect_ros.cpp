/************************************************************************
 *
 * kinect_ros.cpp modifed from joystick.cpp
 *
 * Sample implementation of a Webots controller as a ROS node
 *
 * This Webots controller is a ROS node that simply publishes
 * kinect ROS node to broadcast kinect 3D image.
 *
 * Date:    Jul. 13th 2012
 * Authors: JongSuk Choi, KIST
 *
 ************************************************************************/

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <ros/ros.h>
#include <signal.h>
#include <errno.h>

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

//#include <cv.h>
//#include <highgui.h>
//#include <opencv2/core/types_c.h>
//--------------------------------------------------------------------------

//---------------------------------------------------------------------------------
//      Added by jschoi for ROS, 2012-07-13
#include <webots/camera.h>
#include <webots/servo.h> 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MAX_SPEED 10.0
#define CRUISING_SPEED 7.5
#define TOLERANCE -0.1
#define OBSTACLE_THRESHOLD 0.5
#define SLOWDOWN_FACTOR 0.5

#ifndef PI
#define PI  3.141592
#endif

static WbDeviceTag leftWheel, rightWheel;
//static WbDeviceTag kinect;
//static WbDeviceTag ptcam;
const float* kinectValues;
//static int time_step = 0;
//---------------------------------------------------------------------------------

#define SPEED 50

sensor_msgs::ImagePtr Camera2ImgsMsg(WbDeviceTag Dev, int ImageWidth, int ImageHeight, int bColor)
{
    sensor_msgs::ImagePtr image_msg;
    int x,y,xy;

//      IplImage *wimage = cvCreateImage(cvSize(ImageWidth, ImageHeight),8,3);    

    IplImage *wimage;

    if (bColor != 0) {
      wimage = cvCreateImage(cvSize(ImageWidth, ImageHeight),8,3);    

      wimage->widthStep = 3*ImageWidth;  // DO NOT FORGET  (rgb: 3*width)
      
      wimage->width = ImageWidth;
      wimage->height = ImageHeight;      
      
      const unsigned char *image_wb = wb_camera_get_image(Dev);
      
      for (y=0; y<ImageHeight; y++) {
        for(x=0; x<ImageWidth; x++) {
          xy = y*ImageWidth + x;
          
          wimage->imageData[3*xy+0] = wb_camera_image_get_blue(image_wb, ImageWidth, x, y);
          wimage->imageData[3*xy+1] = wb_camera_image_get_green(image_wb, ImageWidth, x, y);
          wimage->imageData[3*xy+2] = wb_camera_image_get_red(image_wb, ImageWidth, x, y);
        }
      }      
      image_msg = sensor_msgs::CvBridge::cvToImgMsg(wimage,"bgr8");      
    }
    else {
//      IplImage *wimage = cvCreateImage(cvSize(ImageWidth, ImageHeight),8,1);   
      wimage = cvCreateImage(cvSize(ImageWidth, ImageHeight),8,1);    
 
      wimage->widthStep = ImageWidth;  // DO NOT FORGET  (rgb: 1*width) 
      
      wimage->width = ImageWidth;
      wimage->height = ImageHeight;      
      
      const float *image_wb = wb_camera_get_range_image(Dev);
      double max_range = wb_camera_get_max_range(Dev);
      double range;
                  
      for (y=0; y<ImageHeight; y++) {
        for(x=0; x<ImageWidth; x++) {
          xy = y*ImageWidth + x;
          
          range = wb_camera_range_image_get_depth(image_wb, ImageWidth, x, y);
          wimage->imageData[xy] = char (255.*range/max_range);          
       }
      }
      image_msg = sensor_msgs::CvBridge::cvToImgMsg(wimage,"mono8");      
    }

    
    //ROS_INFO("[width,height]=[%d,%d], [x,y,xy]=[%d,%d,%d]",ImageWidth,ImageHeight,x,y,xy);

    //image.SetIpl(wimage);

    //image.SetIpl( cvLoadImage("KIST.png", CV_LOAD_IMAGE_COLOR) );
    //image_msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");

    
    cvReleaseImage(&wimage);  // This is very very IMPORTANT in order not to have a memory leakage!
    
    return image_msg;
}

int main(int argc, char **argv) {

  wb_robot_init();


  // check if roslaunch is properly installed
  if (system("which roslaunch > /dev/null")!=0) {
    fprintf(stderr,"Cannot find roslaunch in PATH. Please check that ROS is properly installed.\n");
    wb_robot_cleanup();
    return 0;
  }


  // launch the joy ROS node
  int roslaunch=fork();
  if (roslaunch==0) { // child process
//     execlp("roslaunch","roslaunch","mjpeg_server.launch",NULL);
//     execlp("rosrun","rosrun","mjpeg_server","mjpeg_server",NULL); // $rosrun mjpeg_server mjpeg_server
    return 0;
  }


  // You need to do: rosrun mjpeg_server mjpeg_server _port:=8080

  // From this point we assume the ROS_ROOT, ROS_MASTER_URI and
  // ROS_PACKAGE_PATH environment variables are set appropriately.
  // See ROS manuals to set them properly.  
  ros::init(argc, argv, "kinect_ros");


/*
  //--------------------------------------------------------------------------
  //      Added by jschoi for ROS, 2012-07-16
  ros::NodeHandle nh_pub;
  ros::Publisher chatter_pub=nh_pub.advertise<std_msgs::String>("chatter", 1000);
  int count=0;
  //--------------------------------------------------------------------------
*/


/*    
  double left_speed = 0;
  double right_speed = 0;
  
  ROS_INFO("Commands:");
  ROS_INFO("arrow up: move forward, speed up");
  ROS_INFO("arrow down: move backward, slow down");
  ROS_INFO("arrow left: turn left");
  ROS_INFO("arrow right: turn right");
*/
    
//---------------------------------------------------------------------------------
//      Added by jschoi for ROS, 2012-07-13
//           get time step and robot's devices
  char dev_name[32], topic_name[32];
  sprintf(dev_name, "rb01_kinect01");
  sprintf(topic_name, "%s/image",dev_name);
  
  int time_step = wb_robot_get_basic_time_step();
  //WbDeviceTag kinect = wb_robot_get_device("rb01_kinect01");
  WbDeviceTag kinect = wb_robot_get_device(dev_name);
  wb_camera_enable(kinect, time_step);
  int kinectWidth = wb_camera_get_width(kinect);
  int kinectHeight = wb_camera_get_height(kinect);
  int halfWidth   = kinectWidth/2;
  int viewHeight  = kinectHeight/2 + 10;
  double maxRange = wb_camera_get_max_range(kinect);
  double rangeThreshold = 1.5;
  double invMaxRangeTimesWidth = 1.0 / (maxRange * kinectWidth);

  //////////////////////////////////
  // get the pan/tilt camera device, enable it, and store its width and height
  WbDeviceTag ptcam = wb_robot_get_device("ptcam");
  wb_camera_enable(ptcam, time_step);
  //int ptcam_width = wb_camera_get_width(ptcam);
  //int ptcam_height = wb_camera_get_height(ptcam); 

  // initialize pan/tilt camera servos
  WbDeviceTag pan_servo, tilt_servo;
  pan_servo = wb_robot_get_device("ptcam_pan_servo");
  tilt_servo = wb_robot_get_device("ptcam_tilt_servo"); 

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
  pan_angle = 0;  tilt_angle = 0;
  wb_servo_set_position(pan_servo, pan_angle);
  wb_servo_set_position(tilt_servo, tilt_angle);  
  
  //wb_servo_set_position(pan_servo, INFINITY); // for velocity control
  //////////////////////////////////

  leftWheel  = wb_robot_get_device("leftWheel");
  rightWheel = wb_robot_get_device("rightWheel");

  // set servos' positions
  wb_servo_set_position(leftWheel,INFINITY);
  wb_servo_set_position(rightWheel,INFINITY);
  // set speeds 
  wb_servo_set_velocity(leftWheel, 0.0);
  wb_servo_set_velocity(rightWheel, 0.0);
  //perform one control loop
  wb_robot_step(time_step);
  
  // init dynamic variables
  double leftObstacle = 0.0, rightObstacle = 0.0, obstacle = 0.0;
  double deltaObstacle = 0.0;
  double leftSpeed = CRUISING_SPEED, rightSpeed = CRUISING_SPEED;
  double speedFactor  = 1.0;
  float value = 0.0;
  int i = 0;
/*
  while (1) {
    // get range-finder values
    kinectValues = (float*) wb_camera_get_range_image(kinect);
    
    for (i = 0; i < halfWidth; i++) {
    // record near obstacle sensed on the left side
      value = wb_camera_range_image_get_depth(kinectValues, kinectWidth, i, viewHeight);
      if(value < rangeThreshold) { // far obstacles are ignored
        leftObstacle += value;
      }
    // record near obstacle sensed on the right side
      value = wb_camera_range_image_get_depth(kinectValues, kinectWidth, kinectWidth - i, viewHeight);
      if(value < rangeThreshold) {
        rightObstacle += value;
      }
    }
    
    obstacle  =  leftObstacle + rightObstacle;
    
    // compute the speed according to the information on
    // possible left and right obstacles
    if(obstacle > 0.0){
      obstacle = 1.0 - obstacle * invMaxRangeTimesWidth;// compute the relevant overall quantity of obstacle
      speedFactor = (obstacle > OBSTACLE_THRESHOLD) ? 0.0 : SLOWDOWN_FACTOR;
      deltaObstacle = - (leftObstacle - rightObstacle) * invMaxRangeTimesWidth;
      if(deltaObstacle > TOLERANCE){
        leftSpeed    =               CRUISING_SPEED;
        rightSpeed   = speedFactor * CRUISING_SPEED;   
      }
      else {
        leftSpeed    = speedFactor *  CRUISING_SPEED;
        rightSpeed   =                CRUISING_SPEED;
      }
    }
    else {
       leftSpeed    = CRUISING_SPEED;
       rightSpeed   = CRUISING_SPEED;
    }

    // set speeds
    wb_servo_set_velocity(leftWheel,  leftSpeed);
    wb_servo_set_velocity(rightWheel, rightSpeed);
  
    // run simulation
    wb_robot_step(time_step);
    leftObstacle  = 0.0;
    rightObstacle = 0.0;
  }
*/
//---------------------------------------------------------------------------------



  //--------------------------------------------------------------------------
  //      Added by jschoi for ROS w/ image_transport, 2012-07-17
  ros::NodeHandle nh;


//  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("rb01_kinect01/image",1);
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic_name,1);
  
  sprintf(topic_name, "%s/image_d",dev_name);  
  ros::Publisher pub2 = nh.advertise<sensor_msgs::Image>(topic_name,1);
  
  //  We need to replace the above line with the following two lines (however, curretly those show error regarding pointer initialization)
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("camera03/image", 1);
/*
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera03/image", 1);
*/ 
 
  //cv::WImageBuffer3_b image( cvLoadImage("KIST.png", CV_LOAD_IMAGE_COLOR) );
  //sensor_msgs::ImagePtr image_msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");
  sensor_msgs::ImagePtr image_msg, image_msg2;





  ros::Rate loop_rate(5);

  //--------------------------------------------------------------------------



  // control loop: simulation steps of 32 ms
  while(wb_robot_step(32) != -1) {
  //while(wb_robot_step(time_step) != -1) {

    // get range-finder values
    kinectValues = (float*) wb_camera_get_range_image(kinect);
    
    for (i = 0; i < halfWidth; i++) {
    // record near obstacle sensed on the left side
      value = wb_camera_range_image_get_depth(kinectValues, kinectWidth, i, viewHeight);
      if(value < rangeThreshold) { // far obstacles are ignored
        leftObstacle += value;
      }
    // record near obstacle sensed on the right side
      value = wb_camera_range_image_get_depth(kinectValues, kinectWidth, kinectWidth - i, viewHeight);
      if(value < rangeThreshold) {
        rightObstacle += value;
      }
    }
    
    obstacle  =  leftObstacle + rightObstacle;
    
    // compute the speed according to the information on
    // possible left and right obstacles
    if(obstacle > 0.0){
      obstacle = 1.0 - obstacle * invMaxRangeTimesWidth;// compute the relevant overall quantity of obstacle
      speedFactor = (obstacle > OBSTACLE_THRESHOLD) ? 0.0 : SLOWDOWN_FACTOR;
      deltaObstacle = - (leftObstacle - rightObstacle) * invMaxRangeTimesWidth;
      if(deltaObstacle > TOLERANCE){
        leftSpeed    =               CRUISING_SPEED;
        rightSpeed   = speedFactor * CRUISING_SPEED;   
      }
      else {
        leftSpeed    = speedFactor *  CRUISING_SPEED;
        rightSpeed   =                CRUISING_SPEED;
      }
    }
    else {
       leftSpeed    = CRUISING_SPEED;
       rightSpeed   = CRUISING_SPEED;
    }
ROS_INFO("[leftSpeed,rightSpeed]=[%5.3lf,%5.3lf]",leftSpeed,rightSpeed);
    // set speeds
    wb_servo_set_velocity(leftWheel,  leftSpeed);
    wb_servo_set_velocity(rightWheel, rightSpeed);
  
    // run simulation
    //wb_robot_step(time_step);
    leftObstacle  = 0.0;
    rightObstacle = 0.0;


/*
    //---------------------------------------------------------------------
    //      Added by jschoi for ROS, 2012-07-13
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world" << count;
    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());
    chatter_pub.publish(msg);
    ++count;
    //---------------------------------------------------------------------
*/


    //---------------------------------------------------------------------
    //      Added by jschoi for ROS w/ image_transport, 2012-07-16


    int bColor;
    
    bColor = 1;
    image_msg = Camera2ImgsMsg(kinect, kinectWidth, kinectHeight, bColor);

    // get range-finder values
    //kinectValues = (float*) wb_camera_get_range_image(kinect);
/*
    const unsigned char *image_kinect = wb_camera_get_image(kinect);
    int x,y,xy;

    IplImage *wimage_kinect = cvCreateImage(cvSize(kinectWidth, kinectHeight),8,3);
    wimage_kinect->width = kinectWidth;
    wimage_kinect->height = kinectHeight;
    wimage_kinect->widthStep = 3*kinectWidth;  // DO NOT FORGET  (rgb: 3*width)

    for (y=0; y<kinectHeight; y++) {
      for(x=0; x<kinectWidth; x++) {
        xy = y*kinectWidth + x;
				
        wimage_kinect->imageData[3*xy+0] = wb_camera_image_get_blue(image_kinect, kinectWidth, x, y);
        wimage_kinect->imageData[3*xy+1] = wb_camera_image_get_green(image_kinect, kinectWidth, x, y);
        wimage_kinect->imageData[3*xy+2] = wb_camera_image_get_red(image_kinect, kinectWidth, x, y);
      }
    }
    
    
    
    //ROS_INFO("[width,height]=[%d,%d], [x,y,xy]=[%d,%d,%d]",kinectWidth,kinectHeight,x,y,xy);

    //image.SetIpl(wimage_kinect);

    //image.SetIpl( cvLoadImage("KIST.png", CV_LOAD_IMAGE_COLOR) );
    //image_msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");
    image_msg = sensor_msgs::CvBridge::cvToImgMsg(wimage_kinect,"bgr8");
*/ 


   
    pub.publish(image_msg);
    
    bColor = 0;
    image_msg2 = Camera2ImgsMsg(kinect, kinectWidth, kinectHeight, bColor);
    pub2.publish(image_msg2);
    
    ros::spinOnce();
//    loop_rate.sleep();
    
    //---------------------------------------------------------------------


/*
    // get callback called
    ros::spinOnce();

    switch(cmd) {

    case JOYPAD_UP:
      left_speed=SPEED;
      right_speed=SPEED;
      break;

    case JOYPAD_DOWN:
      left_speed=-SPEED;
      right_speed=-SPEED;
      break;
        
    case JOYPAD_LEFT:
      left_speed=-SPEED;
      right_speed=SPEED;
      break;
       
    case JOYPAD_RIGHT:
      left_speed=SPEED;
      right_speed=-SPEED;
      break;
      
    case JOYPAD_STOP:
      left_speed = 0;
      right_speed = 0;
      break;
    }

    // set motor speeds
    wb_differential_wheels_set_speed(left_speed, right_speed);
*/
  }

  ros::shutdown();

  //kill(roslaunch,SIGINT); // terminate roslaunch for joy ROS node as with Ctrl-C

  wb_robot_cleanup();

  return 0;
}
