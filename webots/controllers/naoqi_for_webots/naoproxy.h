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

#ifndef AL_NAO_PROXY_H
#define AL_NAO_PROXY_H

#include <webots/types.h>
#include <vector>

namespace AL
{
class ALNaoQiClient;
class ALNaoSimulationInterface;
/**
 * ALNaoProxy enables communication between webots and the official Nao API.
 * This class will not exists as such in the real Nao.
 */
class ALNaoProxy
{
public :
  ALNaoProxy();
  virtual ~ALNaoProxy();
  void run();
  
  // do not modify: this is the order of the motor angles returned by ALNaoSimulationInterface::getBodyAngles()
  enum { HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand,
         LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll,
         RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll,
         RWristYaw, RHand, SERVO_MAX };
         
  enum { PHALANX_MAX = 8 };
  
  // do not modify: this is the order expected by ALNaoSimulationInterface::updateFSRValues()
  enum { LFsrFL, LFsrFR, LFsrBR, LFsrBL, RFsrFL, RFsrFR, RFsrBR, RFsrBL, FSR_MAX };
  
  enum { USTopRight, USTopLeft, USBottomRight, USBottomLeft, US_MAX };
  
  enum { RFootBumperRight, RFootBumperLeft, LFootBumperRight, LFootBumperLeft, BUMPER_MAX };

private :
  ALNaoQiClient *naoqi;
  ALNaoSimulationInterface *naoqiSimulated;
  
  WbDeviceTag camera, cameraSelect, accelerometer, gyro, gps;  
  WbDeviceTag servos[SERVO_MAX];
  WbDeviceTag lphalanx[PHALANX_MAX];
  WbDeviceTag rphalanx[PHALANX_MAX];
  WbDeviceTag fsrs[FSR_MAX];
  WbDeviceTag sonar[US_MAX];
  WbDeviceTag bumpers[BUMPER_MAX];
  
  std::vector<float> accelerometerValues;
  std::vector<float> inertialValues; 
  std::vector<float> naosPosition;
  std::vector<float> bodyAngles;
  std::vector<float> bodyAnglesUpdate;
  std::vector<float> fsrUpdate;

  int imageWidth;
  int imageHeight;
  bool alreadyFailedToSendImage;

  void step();
  void insertFloatInMemory(const char *name, double value);
  void setServoPosition(WbDeviceTag, double pos);
};
}
#endif
