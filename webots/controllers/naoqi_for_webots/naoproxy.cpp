/* 
    NAOQI_FOR_WEBOTS: Interface between NaoQI and Webots
    Copyright (C) 2010-2011  Aldebaran-Robotics & Cyberbotics Ltd.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    IMPORTANT
    =========
    Please see the instructions in the readme.html file.
*/

#include "naoproxy.h"
#include <naoqiclient/naoqiclient.h>
#include <naoqiclient/interface/alnaosimulation_interface.h>
#include <webots/robot.h>
#include <webots/servo.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/touch_sensor.h>
#include <webots/led.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define GRAVITY 9.81
#define PI_2 1.57079633

// number of milliseconds in Webots simulation cycle
#define TIME_STEP 40

// milliseconds between two camera updates
#define CAMERA_TIME_STEP 40

using namespace std;
using namespace AL;

ALNaoProxy::ALNaoProxy()
{
  // initialize communication with Webots
  wb_robot_init();
  
  cout << "Enabling devices\n";

  // camera
  camera = wb_robot_get_device("camera");
  cameraSelect = wb_robot_get_device("CameraSelect");
  wb_camera_enable(camera, CAMERA_TIME_STEP);
  alreadyFailedToSendImage = false;

  // inertial unit
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, TIME_STEP);
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, TIME_STEP);
  
  // GPS (not a real Nao device)
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);


  // sonar
  sonar[USTopRight]    = wb_robot_get_device("US/TopRight");
  sonar[USTopLeft]     = wb_robot_get_device("US/TopLeft");
  sonar[USBottomRight] = wb_robot_get_device("US/BottomRight");
  sonar[USBottomLeft]  = wb_robot_get_device("US/BottomLeft");
  for (int i = 0; i < US_MAX; i++)
    wb_distance_sensor_enable(sonar[i], TIME_STEP);
  
  // force sensitive resistors (pressure sensors)
  fsrs[LFsrFL] = wb_robot_get_device("LFsrFL");
  fsrs[LFsrFR] = wb_robot_get_device("LFsrFR");
  fsrs[LFsrBR] = wb_robot_get_device("LFsrBR");
  fsrs[LFsrBL] = wb_robot_get_device("LFsrBL");
  fsrs[RFsrFL] = wb_robot_get_device("RFsrFL");
  fsrs[RFsrFR] = wb_robot_get_device("RFsrFR");
  fsrs[RFsrBR] = wb_robot_get_device("RFsrBR");
  fsrs[RFsrBL] = wb_robot_get_device("RFsrBL");
  for (int i = 0; i < FSR_MAX; i++)
    wb_touch_sensor_enable(fsrs[i], TIME_STEP);

  // foot bumpers
  bumpers[RFootBumperRight] = wb_robot_get_device("RFoot/Bumper/Right");
  bumpers[RFootBumperLeft]  = wb_robot_get_device("RFoot/Bumper/Left");
  bumpers[LFootBumperRight] = wb_robot_get_device("LFoot/Bumper/Right");
  bumpers[LFootBumperLeft]  = wb_robot_get_device("LFoot/Bumper/Left");
  for (int i = 0; i < BUMPER_MAX; i++)
    wb_touch_sensor_enable(bumpers[i], TIME_STEP);

  // regular joints
  servos[HeadYaw]        = wb_robot_get_device("HeadYaw");
  servos[HeadPitch]      = wb_robot_get_device("HeadPitch");
  servos[LShoulderPitch] = wb_robot_get_device("LShoulderPitch");
  servos[LShoulderRoll]  = wb_robot_get_device("LShoulderRoll");
  servos[LElbowYaw]      = wb_robot_get_device("LElbowYaw");
  servos[LElbowRoll]     = wb_robot_get_device("LElbowRoll");
  servos[LWristYaw]      = wb_robot_get_device("LWristYaw");
  servos[LHand]          = 0;
  servos[LHipYawPitch]   = wb_robot_get_device("LHipYawPitch");
  servos[LHipRoll]       = wb_robot_get_device("LHipRoll");
  servos[LHipPitch]      = wb_robot_get_device("LHipPitch");
  servos[LKneePitch]     = wb_robot_get_device("LKneePitch");
  servos[LAnklePitch]    = wb_robot_get_device("LAnklePitch");
  servos[LAnkleRoll]     = wb_robot_get_device("LAnkleRoll");
  servos[RHipYawPitch]   = wb_robot_get_device("RHipYawPitch");
  servos[RHipRoll]       = wb_robot_get_device("RHipRoll");
  servos[RHipPitch]      = wb_robot_get_device("RHipPitch");
  servos[RKneePitch]     = wb_robot_get_device("RKneePitch");
  servos[RAnklePitch]    = wb_robot_get_device("RAnklePitch");
  servos[RAnkleRoll]     = wb_robot_get_device("RAnkleRoll");
  servos[RShoulderPitch] = wb_robot_get_device("RShoulderPitch");
  servos[RShoulderRoll]  = wb_robot_get_device("RShoulderRoll");
  servos[RElbowYaw]      = wb_robot_get_device("RElbowYaw");
  servos[RElbowRoll]     = wb_robot_get_device("RElbowRoll");
  servos[RWristYaw]      = wb_robot_get_device("RWristYaw");
  servos[RHand]          = 0;
  for (int i = 0; i < SERVO_MAX; i++) {
    if (servos[i]) { // avoid warning with the hands
      // enable reading servo feedback position from Webots
      wb_servo_enable_position(servos[i], TIME_STEP);
    }
  }

  // get phalanx motors
  // the real Nao has only 2 motors for RHand/LHand
  // but in Webots we must implement RHand/LHand with 2x8 motors
  for (int i = 0; i < PHALANX_MAX; i++) {
    char name[32];
    sprintf(name, "LPhalanx%d", i + 1);
    lphalanx[i] = wb_robot_get_device(name);
    sprintf(name, "RPhalanx%d", i + 1);
    rphalanx[i] = wb_robot_get_device(name);
  }

  // currently not supported by ALNaoSimulationInterface:
  //wb_receiver_enable(devices["receiver"], TIME_STEP);
  //devices["ChestBoard/Led"] = wb_robot_get_device("ChestBoard/Led");
  //devices["emitter"] = wb_robot_get_device("emitter");
  //devices["receiver"] = wb_robot_get_device("receiver");

  naoqi = new AL::ALNaoQiClient();
	
  cout << "..::: starting NAOQI_FOR_WEBOTS 1.8 rev 1 (with hands) :::..\n";
  cout << "NAOQI_FOR_WEBOTS  Copyright (C) 2010  Aldebaran-Robotics & Cyberbotics Ltd.\n";
  cout << "This program comes with ABSOLUTELY NO WARRANTY.\n";
  cout << "This is free software, and you are welcome to redistribute it under certain conditions.\n";

  const char *AL_DIR = getenv("AL_DIR");
  if (! AL_DIR) {
    cerr << "ERROR: you must define the AL_DIR environment variable !!!\n";
    exit(1);
  }

  naoqi->setALPath(AL_DIR);

  // the robot name is used to decide how to start NaoQi, see file:
  // $AL_DIR/preferences/simulator.xml
  if (! naoqi->runNaoqi(wb_robot_get_name()))
  {
    cerr << "ERROR: failed to launch naoqi !!!\n";
    exit(1);
  }

  naoqiSimulated = naoqi->getSimulationAPI();
  if (! naoqiSimulated)
  {
    cerr << "ERROR: failed to get simulation API !!!\n";
    delete naoqi;
    exit(1);
  }

  // read camera resolution from Nao simulation:
  // this make it flexible with different resolutions
  imageWidth = wb_camera_get_width(camera);
  imageHeight = wb_camera_get_height(camera);
  cout << "Updating image size to " << imageWidth << "x" << imageHeight << "\n";
  if (! naoqiSimulated->updateImageSize(imageWidth, imageHeight))
    cerr << "ALNaoSimulationInterface::updateImageSize() failed.\n";

  cout << "Initialization successful\n";
}

ALNaoProxy::~ALNaoProxy() {
  delete naoqi;
  delete naoqiSimulated;
}

void ALNaoProxy::run() {
  // while Webots simulation is running
  do {
    step();
  }
  while (wb_robot_step(TIME_STEP) != -1);
}

// set the motor position if the motor exists
void ALNaoProxy::setServoPosition(WbDeviceTag tag, double pos) {
  if (tag)
    wb_servo_set_position(tag, pos);
}

//
// Communicate with naoqiSimulated.
// This function is called by Webots every TIME_STEP ms.
//
void ALNaoProxy::step()
{	
	//__________
	// Sonar values
	//
	// do not use USBottomRight or USBottomLeft, as we only have one value for each side on a real robot
	// Webots returns values in meters
	double right = wb_distance_sensor_get_value(sonar[USTopRight]);
	double left = wb_distance_sensor_get_value(sonar[USTopLeft]);
	if (! naoqiSimulated->updateSonarValues((float)left, (float)right, true))
    cerr << "ALNaoSimulationInterface::updateSonarValues() failed.\n";
	
	// this is experimental:
	// DCM stores values in meters
	insertFloatInMemory("Device/SubDeviceList/US/Right/Sensor/Value", right);
	insertFloatInMemory("Device/SubDeviceList/US/Left/Sensor/Value", left);

	//_________
	// Accelerometer values
	//
	// Webots returns values in m/s^2
	const double *acc = wb_accelerometer_get_values(accelerometer);
	accelerometerValues.clear();
	accelerometerValues.push_back((float)acc[0]);
	accelerometerValues.push_back((float)acc[1]);
	accelerometerValues.push_back((float)acc[2]);
	if (! naoqiSimulated->updateAccelerometers(accelerometerValues))
    cerr << "ALNaoSimulationInterface::updateAccelerometers() failed.\n";

	//__________
	// Inertial values (computed from accelerometer values)
	//
	inertialValues.clear();
	inertialValues.push_back((float)(-PI_2 + acos(acc[1]/GRAVITY)));
	inertialValues.push_back((float)(PI_2 - acos(acc[0]/GRAVITY)));
	if (! naoqiSimulated->updateInertialValues(inertialValues))
    cerr << "ALNaoSimulationInterface::updateInertialValues() failed.\n";
	
	//__________
	// Gyro values
	//
	// Webots reurns angular velocity in rad/s
	const double *angular_vel = wb_gyro_get_values(gyro);
  
	// this is experimental:
	insertFloatInMemory("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value", angular_vel[0]);
	insertFloatInMemory("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value", angular_vel[1]);
	
	//____________
	// Com position:
	//
	// Motion and Webots do not have the same coordinate system
	const double *values = wb_gps_get_values(gps);
	naosPosition.clear();
	naosPosition.push_back((float)values[0]);
	naosPosition.push_back((float)-values[2]);
	naosPosition.push_back((float)values[1]);
	if (! naoqiSimulated->updatePositionEstimate(naosPosition))
    cerr << "ALNaoSimulationInterface::updatePositionEstimate() failed.\n";

	//__________
	// Get body angles (command) from Motion, and set servo positions,
	// then updates real body angles:
	bodyAngles = naoqiSimulated->getBodyAngles();
	if (bodyAngles.size() == 0)
    cerr << "ALNaoSimulationInterface::getBodyAngles() failed.\n";

	bodyAnglesUpdate.clear();
	for (int i = 0; i < SERVO_MAX; i++)
	{
    double angle = bodyAngles.at(i);

    switch (i) {
      case RHipYawPitch:
        // NAOqi yields always zero for this joint: ignore it ...
        break;
      case LHipYawPitch: {
        // ... but use the value from the left side instead
        setServoPosition(servos[LHipYawPitch], angle);
        setServoPosition(servos[RHipYawPitch], angle);
        break;
      }
      case RHand: {
        // we must activate the 8 phalanx motors
        for (int j = 0; j < PHALANX_MAX; j++)
          setServoPosition(rphalanx[j], angle);
        break;
      }
      case LHand: {
        // we must activate the 8 phalanx motors
        for (int j = 0; j < PHALANX_MAX; j++)
          setServoPosition(lphalanx[j], angle);
        break;
      }
      default:
        // regular joint
        setServoPosition(servos[i], angle);
        break;
    }
    
    if (servos[i])
      bodyAnglesUpdate.push_back((float)wb_servo_get_position(servos[i]));
    else
      bodyAnglesUpdate.push_back(0.0);
	}

	//___________
	// FSR values
	//
	fsrUpdate.clear();
	for (int i = 0; i < FSR_MAX; i++) {
    // Webots returns values in Newtons, NaoQi expects values in kg
		fsrUpdate.push_back((float)(wb_touch_sensor_get_value(fsrs[i]) / GRAVITY)); 
	}
	if (! naoqiSimulated->updateFSRValues(fsrUpdate))
    cerr << "ALNaoSimulationInterface::updateFSRValues() failed.\n";
	
	//_____________
	// Bumpers
	//
	// Webots and DCM use the same convention: 0.0 unpressed, 1.0 pressed
	insertFloatInMemory("Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value", wb_touch_sensor_get_value(bumpers[RFootBumperRight]));
	insertFloatInMemory("Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",  wb_touch_sensor_get_value(bumpers[RFootBumperLeft]));
	insertFloatInMemory("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value", wb_touch_sensor_get_value(bumpers[LFootBumperRight]));
	insertFloatInMemory("Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",  wb_touch_sensor_get_value(bumpers[LFootBumperLeft]));

	//_____________
	// Camera Image
	//
	if (!alreadyFailedToSendImage)
	{
		static int lastCamera = naoqiSimulated->getActiveCamera();
		
		int currentCamera = naoqiSimulated->getActiveCamera();
		
    // meaning the camera has switched position since last time !
		if (lastCamera != currentCamera)
		{
			switch (currentCamera)
			{
			case 0:
        // set high camera position in Webots
				wb_servo_set_position(cameraSelect, 0.0);
				break;
			case 1:
        // set low (mouth) camera position in Webots
				wb_servo_set_position(cameraSelect, 1.0);
				break;
			default:
			  cerr << "ALNaoSimulationInterface:getActiveCamera() returned unexpected value: " << currentCamera << "\n";
				break;
			}

			lastCamera = currentCamera;
		}
		if (! naoqiSimulated->updateImage((uint8*)wb_camera_get_image(camera), imageWidth, imageHeight))
		{
			cerr << "ALNaoSimulationInterface::updateImage() failed.\n";
			alreadyFailedToSendImage = true;
		}
	}
}

void ALNaoProxy::insertFloatInMemory(const char *name, double value) {
  if (! naoqiSimulated->insertFloatInMemory(name, (float)value))
    cerr << "failed to insertFloatInMemory: name=" << name << ", value=" << value << "\n";
}
