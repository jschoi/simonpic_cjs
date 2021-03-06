//---------------------------------------------------------------------------------
//  File:         nao_demo.c
//  Description:  Example C controller program for Nao robot.
//                This demonstrates how to access sensors and actuators
//  Authors:      Yvan Bourquin, Cyberbotics Ltd.
//  Date:         September 14, 2011
//---------------------------------------------------------------------------------


#include <webots/utils/motion.h>
#include <webots/robot.h>
#include <webots/servo.h> 
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/touch_sensor.h>
#include <webots/led.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _MSC_VER
#define snprintf sprintf_s
#endif

#define PHALANX_MAX 8

static int time_step = -1;

// simulated devices
static WbDeviceTag camera, camera_select;             // camera with high/low position
static WbDeviceTag us[4];                             // ultra sound sensors
static WbDeviceTag accelerometer, gyro;               // inertial unit
static WbDeviceTag fsr[2][4];                         // force sensitive resistors
static WbDeviceTag lfoot_lbumper, lfoot_rbumper;      // left foot bumpers
static WbDeviceTag rfoot_lbumper, rfoot_rbumper;      // right foot bumpers
static WbDeviceTag leds[7];                           // controllable led groupsstatic WbDeviceTag lphalanx[PHALANX_MAX];
static WbDeviceTag rphalanx[PHALANX_MAX];             // right hand motors
static WbDeviceTag lphalanx[PHALANX_MAX];             // left hand motors
static WbDeviceTag RShoulderPitch;
static WbDeviceTag LShoulderPitch;

// motion file handles
static WbMotionRef hand_wave, forwards, backwards, side_step_left, side_step_right, turn_left_60, turn_right_60;
static WbMotionRef currently_playing = NULL;

static void find_and_enable_devices() {

  // camera
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 4 * time_step);

  // camera selection (high/low)
  camera_select = wb_robot_get_device("CameraSelect");

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // gyro
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, time_step);

  // ultrasound sensors
  us[0] = wb_robot_get_device("US/TopRight");
  us[1] = wb_robot_get_device("US/BottomRight");
  us[2] = wb_robot_get_device("US/TopLeft");
  us[3] = wb_robot_get_device("US/BottomLeft");
  int i;
  for (i = 0; i < 4; i++)
    wb_distance_sensor_enable(us[i], time_step);

  // foot sensors
  fsr[0][0] = wb_robot_get_device("LFsrFL");
  fsr[0][1] = wb_robot_get_device("LFsrFR");
  fsr[0][2] = wb_robot_get_device("LFsrBR");
  fsr[0][3] = wb_robot_get_device("LFsrBL");
  fsr[1][0] = wb_robot_get_device("RFsrFL");
  fsr[1][1] = wb_robot_get_device("RFsrFR");
  fsr[1][2] = wb_robot_get_device("RFsrBR");
  fsr[1][3] = wb_robot_get_device("RFsrBL");
  for (i = 0; i < 4; i++) {
    wb_touch_sensor_enable(fsr[0][i], time_step);
    wb_touch_sensor_enable(fsr[1][i], time_step);
  }

  // foot bumpers
  lfoot_lbumper = wb_robot_get_device("LFoot/Bumper/Left");
  lfoot_rbumper = wb_robot_get_device("LFoot/Bumper/Right");
  rfoot_lbumper = wb_robot_get_device("RFoot/Bumper/Left");
  rfoot_rbumper = wb_robot_get_device("RFoot/Bumper/Left");
  wb_touch_sensor_enable(lfoot_lbumper, time_step);
  wb_touch_sensor_enable(lfoot_rbumper, time_step);
  wb_touch_sensor_enable(rfoot_lbumper, time_step);
  wb_touch_sensor_enable(rfoot_rbumper, time_step);

  // There are 7 controlable LED groups in Webots
  leds[0] = wb_robot_get_device("ChestBoard/Led");
  leds[1] = wb_robot_get_device("RFoot/Led");
  leds[2] = wb_robot_get_device("LFoot/Led");
  leds[3] = wb_robot_get_device("Face/Led/Right");
  leds[4] = wb_robot_get_device("Face/Led/Left");
  leds[5] = wb_robot_get_device("Ears/Led/Right");
  leds[6] = wb_robot_get_device("Ears/Led/Left");
  
  // get phalanx motor tags
  // the real Nao has only 2 motors for RHand/LHand
  // but in Webots we must implement RHand/LHand with 2x8 motors
  for (i = 0; i < PHALANX_MAX; i++) {
    char name[32];
    sprintf(name, "LPhalanx%d", i + 1);
    lphalanx[i] = wb_robot_get_device(name);
    sprintf(name, "RPhalanx%d", i + 1);
    rphalanx[i] = wb_robot_get_device(name);
  }
  
  // shoulder pitch motors
  RShoulderPitch = wb_robot_get_device("RShoulderPitch");
  LShoulderPitch = wb_robot_get_device("LShoulderPitch");

  // keyboard
  wb_robot_keyboard_enable(10 * time_step); 
}

// load motion files
static void load_motion_files() {
  hand_wave = wbu_motion_new("../../motions/HandWave.motion");
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  turn_left_60 = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right_60 = wbu_motion_new("../../motions/TurnRight60.motion");
}

static void start_motion(WbMotionRef motion) {
  
  // interrupt current motion
  if (currently_playing)
    wbu_motion_stop(currently_playing);
  
  // start new motion
  wbu_motion_play(motion);
  currently_playing = motion;
}

// the accelerometer axes are oriented as on the real robot
// however the sign of the returned values may be opposite
static void print_acceleration() {
  const double *acc = wb_accelerometer_get_values(accelerometer);
  printf("----------accelerometer----------\n");
  printf("acceleration: [ x y z ] = [%f %f %f]\n", acc[0], acc[1], acc[2]);
}

// the gyro axes are oriented as on the real robot
// however the sign of the returned values may be opposite
static void print_gyro() {
  const double *vel = wb_gyro_get_values(gyro);
  printf("----------gyro----------\n");
  printf("angular velocity: [ x y ] = [%f %f]\n", vel[0], vel[1]);
}

static void print_foot_sensors() {
  double newtons = 0.0;
  double fsv[2][4]; // force sensor values
  int i;
  for (i = 0; i < 4; i++) {
    fsv[0][i] = wb_touch_sensor_get_value(fsr[0][i]);
    fsv[1][i] = wb_touch_sensor_get_value(fsr[1][i]);
    newtons += fsv[0][i] + fsv[1][i];
  }
  
  printf("----------foot sensors----------\n");
  printf("   left       right\n");
  printf("+--------+ +--------+\n");
  printf("|%3.1f  %3.1f| |%3.1f  %3.1f|  front\n", fsv[0][0], fsv[0][1], fsv[1][0], fsv[1][1]);
  printf("|        | |        |\n");
  printf("|%3.1f  %3.1f| |%3.1f  %3.1f|  back\n", fsv[0][3], fsv[0][2], fsv[1][3], fsv[1][2]);
  printf("+--------+ +--------+\n");
  printf("total: %.1f Newtons, %.1f kilograms\n", newtons, newtons / 9.81);
}

static void print_foot_bumpers() {
  int ll = (int)wb_touch_sensor_get_value(lfoot_lbumper);
  int lr = (int)wb_touch_sensor_get_value(lfoot_rbumper);
  int rl = (int)wb_touch_sensor_get_value(rfoot_lbumper);
  int rr = (int)wb_touch_sensor_get_value(rfoot_rbumper);

  printf("----------foot bumpers----------\n");
  printf("   left       right\n");
  printf("+--------+ +--------+\n");
  printf("|%d      %d| |%d      %d|\n", ll, lr, rl, rr);
  printf("|        | |        |\n");
  printf("|        | |        |\n");
  printf("+--------+ +--------+\n");
}

static void print_ultrasound_sensors() {
  double dist[4];
  int i;
  for (i = 0; i < 4; i++)
    dist[i] = wb_distance_sensor_get_value(us[i]);

  printf("-----ultrasound sensors-----\n");
  printf("top:   left: %f m, right %f m\n", dist[2], dist[0]);
  printf("bottom left: %f m, right %f m\n", dist[3], dist[1]);
}

static void print_camera_image() {

  const int SCALED = 2;

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  // read rgb pixel values from the camera
  const unsigned char *image = wb_camera_get_image(camera);
  
  printf("----------camera image (grey levels)---------\n");
  printf("original resolution: %d x %d, scaled to %d x %d\n",
    width, height, width / SCALED, height / SCALED);
  
  int y, x;
  char *line = malloc(width / SCALED + 1);
  line[width / SCALED] = 0;  // add line termination
  for (y = 0; y < height; y += SCALED) {
    int count = 0;
    for (x = 0; x < width; x += SCALED) {
      unsigned char grey = wb_camera_image_get_grey(image, width, x, y);
      line[count++] = '0' + grey * 9 / 255;
    }
    line[count++] = 0;
    printf("%s\n", line);
  }
  free(line);
}

static void set_all_leds_color(int rgb) {

  // these leds take RGB values
  int i;
  for (i = 0; i < 5; i++)
    wb_led_set(leds[i], rgb);

  // ear leds are single color (blue)
  // and take values between 0 - 255
  wb_led_set(leds[5], rgb & 0xff);
  wb_led_set(leds[6], rgb & 0xff);
}

static void set_hands_angle(double angle) {
  // we must activate the 8 phalanx motors
  int j;
  for (j = 0; j < PHALANX_MAX; j++) {
    if (rphalanx[j])
      wb_servo_set_position(rphalanx[j], angle);
    if (lphalanx[j])
      wb_servo_set_position(lphalanx[j], angle);
  }
}

static void print_help() {
  printf("----------nao_demo----------\n");
  printf("Use the keyboard to control the robot\n");
  printf("(The 3D window need to be focused)\n");
  printf("[Up][Down]: move one step forward/backwards\n");
  printf("[<-][->]: side step left/right\n");
  printf("[Shift] + [<-][->]: turn left/right\n");
  printf("[U]: print ultrasound sensors\n");
  printf("[A]: print accelerometers\n");
  printf("[G]: print gyros\n");
  printf("[F]: print foot sensors\n");
  printf("[B]: print foot bumpers\n");
  printf("[C]: print scaled camera image\n");
  printf("[Home][End]: camera selection (high/low)\n");
  printf("[PageUp][PageDown]: open/close hands\n");
  printf("[7][8][9]: change all leds RGB color\n");
  printf("[0]: turn all leds off\n");
  printf("[H]: print this help message\n");
}

static void terminate() {
  // add you cleanup code here: write results, close files, free memory, etc.
  // ...

  wb_robot_cleanup();
}

static void simulation_step() {
  if (wb_robot_step(time_step) == -1)
    terminate();
}

static void run_command(int key) {

  switch (key) {
    case WB_ROBOT_KEYBOARD_LEFT:
      start_motion(side_step_left);
      break;
    case WB_ROBOT_KEYBOARD_RIGHT:
      start_motion(side_step_right);
      break;
    case WB_ROBOT_KEYBOARD_UP:
      start_motion(forwards);
      break;
    case WB_ROBOT_KEYBOARD_DOWN:
      start_motion(backwards);
      break;
    case WB_ROBOT_KEYBOARD_LEFT | WB_ROBOT_KEYBOARD_SHIFT:
      start_motion(turn_left_60);
      break;
    case WB_ROBOT_KEYBOARD_RIGHT | WB_ROBOT_KEYBOARD_SHIFT:
      start_motion(turn_right_60);
      break;
    case 'A':
      print_acceleration();
      break;
    case 'G':
      print_gyro();
      break;
    case 'F':
      print_foot_sensors();
      break;
    case 'B':
      print_foot_bumpers();
      break;
    case 'U':
      print_ultrasound_sensors();
      break;
    case 'C':
      print_camera_image();
      break;
    case WB_ROBOT_KEYBOARD_HOME:
      wb_servo_set_position(camera_select, 0);
      break;
    case WB_ROBOT_KEYBOARD_END:
      wb_servo_set_position(camera_select, 1);
      break;
    case WB_ROBOT_KEYBOARD_PAGEUP:
      set_hands_angle(1.0);
      break;
    case WB_ROBOT_KEYBOARD_PAGEDOWN:
      set_hands_angle(0.0);
      break;      
    case '7':
      set_all_leds_color(0xff0000); // red
      break;
    case '8':
      set_all_leds_color(0x00ff00); // green
      break;
    case '9':
      set_all_leds_color(0x0000ff); // blue
      break;
    case '0':
      set_all_leds_color(0x000000); // off
      break;
    case 'H':
      print_help();
      break;
  }
}

// main function
int main(int argc, const char *argv[]) {
  
  // call this before any other call to a Webots function
  wb_robot_init();

  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();
  
  // initialize stuff
  find_and_enable_devices();
  load_motion_files();
  
  // move shoulder motors down
  wb_servo_set_position(RShoulderPitch, 1.4);
  wb_servo_set_position(LShoulderPitch, 1.4);
  
  // print instructions
  print_help();

  // walk forwards until a key is pressed
  wbu_motion_set_loop(forwards, true);
  wbu_motion_play(forwards);
  int key = 0;
  do {
    simulation_step();
    key = wb_robot_keyboard_get_key();
  }
  while (! key);
  
  // stop looping this motion
  wbu_motion_set_loop(forwards, false);

  // read keyboard and execute user commands
  while (1) {
    if (key)
      run_command(key);
 
    simulation_step();
    key = wb_robot_keyboard_get_key();
  }
  
  return 0;
}
