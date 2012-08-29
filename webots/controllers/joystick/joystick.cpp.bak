/************************************************************************
 *
 * joystick.cpp
 *
 * Sample implementation of a Webots controller as a ROS node
 *
 * This Webots controller is a ROS node that simply subscribes to the
 * Joy ROS node to get joystick input. Then, it uses this joystick
 * information to drive a robot.
 *
 * Date:    December 2010
 * Authors: Andreas Breitenmoser, ETHZ
 *          Olivier Michel, Cyberbotics Ltd.
 *
 ************************************************************************/

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <ros/ros.h>
#include <joy/Joy.h>
#include <signal.h>
#include <errno.h>

#define SPEED 50

enum{JOYPAD_LEFT = 1, JOYPAD_RIGHT, JOYPAD_UP, JOYPAD_DOWN, JOYPAD_STOP};

static int cmd;

void joy_callback(const joy::Joy::ConstPtr& joy) {
/*
  // display all the axes and buttons values from the joy node
  printf("Axes:");
  for(size_t i=0;i<joy->axes.size();i++) printf(" %g",joy->axes[i]);
  printf(" - Buttons:");
  for(size_t i=0;i<joy->buttons.size();i++) printf(" %d",joy->buttons[i]);
  printf("\n");
 */

  // depending on your joystick, you may want to set the
  // cmd variable according to different axes or buttons events.

  // find the biggest axis position
  float m=0;
  for(size_t i=0;i<2;i++) if (fabsf(joy->axes[i]) > m) m=fabsf(joy->axes[i]);
  if (m>0.1) {
         if ( joy->axes[0] == m) cmd=JOYPAD_LEFT;
    else if (-joy->axes[0] == m) cmd=JOYPAD_RIGHT;
    else if ( joy->axes[1] == m) cmd=JOYPAD_UP;
    else if (-joy->axes[1] == m) cmd=JOYPAD_DOWN;
  }
  // force stop if button 1 or button 2 is pressed
  if (joy->buttons[0] || joy->buttons[1]) cmd=JOYPAD_STOP;

/*
  printf("cmd=%d\n",cmd);
 */
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
    execlp("roslaunch","roslaunch","joy.launch",NULL);
    return 0;
  }

  // From this point we assume the ROS_ROOT, ROS_MASTER_URI and
  // ROS_PACKAGE_PATH environment variables are set appropriately.
  // See ROS manuals to set them properly.  
  ros::init(argc, argv, "joystick");
  ros::NodeHandle nh;
  ros::Subscriber sub=nh.subscribe("joy", 10, joy_callback);
//  ros::Subscriber sub=nh.subscribe<int32>("joy", 10, joy_callback);

  double left_speed = 0;
  double right_speed = 0;
  
  ROS_INFO("Joypad connected and running.");
  ROS_INFO("Commands:");
  ROS_INFO("arrow up: move forward, speed up");
  ROS_INFO("arrow down: move backward, slow down");
  ROS_INFO("arrow left: turn left");
  ROS_INFO("arrow right: turn right");
  
  // control loop: simulation steps of 32 ms
  while(wb_robot_step(32) != -1) {

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
  }

  ros::shutdown();

  kill(roslaunch,SIGINT); // terminate roslaunch for joy ROS node as with Ctrl-C

  wb_robot_cleanup();

  return 0;
}
