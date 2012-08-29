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
#include <iostream>

int main(int argc, char *argv[])
{
  std::cout << "Starting NAOQI_FOR_WEBOTS\n";
  AL::ALNaoProxy *naoProxy = new AL::ALNaoProxy();

  // run Webots simulation
  naoProxy->run();

  // Webots has terminated this controller
  std::cout << "Exiting NAOQI_FOR_WEBOTS.\n";
  delete naoProxy;
}

