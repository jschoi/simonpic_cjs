/*
 * File:         human02.c
 * Description:  This is an empty robot controller, the robot does nothing. 
 * Author:       www.cyberbotics.com
 * Note:         !!! PLEASE DO NOT MODIFY THIS SOURCE FILE !!!
 *               This is a system file that Webots needs to work correctly.
 */
 
#include <stdio.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>

#ifndef PI
#define PI  3.141592
#endif

int main() {

  double w_left, w_right;

  wb_robot_init();
  
  WbDeviceTag human02_wheel;
  human02_wheel = wb_robot_get_device("human02_plate");

  int time_step = wb_robot_get_basic_time_step();

  wb_differential_wheels_enable_encoders(time_step);
  
  for (;;)
  {
    w_left = 10.;
    w_right = 10.;
    wb_differential_wheels_set_speed(w_left, w_right);
    
    //printf("  left_encoder=%7.5f\n",wb_differential_wheels_get_left_encoder());
  
    wb_robot_step(32);
  }
  return 0;
}
