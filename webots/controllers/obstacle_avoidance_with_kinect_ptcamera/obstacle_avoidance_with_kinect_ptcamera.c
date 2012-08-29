/*
 * File:         obstacle_avoidance_with_kinect.c
 * Date:         August 24th, 2011
 * Description:  A Braitenberg-like controller moving a Pioneer 3-DX equipped with a kinect.
 * Author:       Luc Guyot
 *
 * Copyright (c) 2011 Cyberbotics - www.cyberbotics.com
 */


#include <webots/robot.h>
#include <webots/servo.h> 
#include <webots/camera.h>
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
static WbDeviceTag kinect;
//static WbDeviceTag ptcam;
const float* kinectValues;
static int time_step = 0;

int main()
{
  //necessary to initialize Webots 
  wb_robot_init();
  // get time step and robot's devices
  time_step  = wb_robot_get_basic_time_step();
  leftWheel  = wb_robot_get_device("leftWheel");
  rightWheel = wb_robot_get_device("rightWheel");
  kinect     = wb_robot_get_device("kinect_robot");
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
  pan_angle = PI/4.;  tilt_angle = 0;
  wb_servo_set_position(pan_servo, pan_angle);
  wb_servo_set_position(tilt_servo, tilt_angle);  
  
  //wb_servo_set_position(pan_servo, INFINITY); // for velocity control
  //////////////////////////////////



    
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
}
