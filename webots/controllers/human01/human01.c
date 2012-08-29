/*
 * File:         human01.c
 * Description:  This is an empty robot controller, the robot does nothing. 
 * Author:       www.cyberbotics.com
 * Note:         !!! PLEASE DO NOT MODIFY THIS SOURCE FILE !!!
 *               This is a system file that Webots needs to work correctly.
 */
 

#include <stdio.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/camera.h>

#ifndef PI
#define PI  3.141592
#endif

#define MAX_SPEED 40.0
#define CRUISING_SPEED 25
#define TOLERANCE -0.1
#define OBSTACLE_THRESHOLD 0.5
#define SLOWDOWN_FACTOR 0.5

int main() {

  wb_robot_init();

//---------------------------------------------------------------------------------
//      Added by jschoi for ROS, 2012-08-10
//           get time step and robot's devices
  const float* kinectValues;
  
  int time_step = wb_robot_get_basic_time_step();
  //printf("time_step = %d\n",time_step);
  
  // Initialize Kinect
  WbDeviceTag kinect = wb_robot_get_device("human_kinect");
  wb_camera_enable(kinect, time_step);  
  int kinectWidth = wb_camera_get_width(kinect);
  int kinectHeight = wb_camera_get_height(kinect);
  int halfWidth   = kinectWidth/2;
  int viewHeight  = kinectHeight/2 + 10;
  double maxRange = wb_camera_get_max_range(kinect);
  double rangeThreshold = 1.5;
  double invMaxRangeTimesWidth = 1.0 / (maxRange * kinectWidth);
  // init dynamic variables
  double leftObstacle = 0.0, rightObstacle = 0.0, obstacle = 0.0;
  double deltaObstacle = 0.0;
  double leftSpeed = CRUISING_SPEED, rightSpeed = CRUISING_SPEED;
  double speedFactor  = 1.0;
  float value = 0.0;
  int i = 0;

  // Initialize DifferentialWheels
  WbDeviceTag human01_wheel;
  human01_wheel = wb_robot_get_device("human01_plate");



  wb_differential_wheels_enable_encoders(time_step);
  
  
  for (;;)
  {
  
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
    
 
    // run simulation
    leftObstacle  = 0.0;
    rightObstacle = 0.0;


    // set speeds  
    //leftSpeed = 40.;  // rad/s
    //rightSpeed = 40.; // rad/s
    wb_differential_wheels_set_speed(leftSpeed, rightSpeed);
    
    //printf("  left_encoder=%7.5f\n",wb_differential_wheels_get_left_encoder());
  
    wb_robot_step(32);
  }
//---------------------------------------------------------------------------------
  
  return 0;  
}
