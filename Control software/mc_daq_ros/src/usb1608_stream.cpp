/******************************************************************************
# usb1608_stream.cpp:  ROS node for DAQ board MC USB1608                             #
# Copyright (c) 2022                                                          #
# Hasegawa Laboratory at Nagoya University                                    #
#                                                                             #
# Redistribution and use in source and binary forms, with or without          #
# modification, are permitted provided that the following conditions are met: #
#                                                                             #
#     - Redistributions of source code must retain the above copyright        #
#       notice, this list of conditions and the following disclaimer.         #
#     - Redistributions in binary form must reproduce the above copyright     #
#       notice, this list of conditions and the following disclaimer in the   #
#       documentation and/or other materials provided with the distribution.  #
#     - Neither the name of the Hasegawa Laboratory nor the                   #
#       names of its contributors may be used to endorse or promote products  #
#       derived from this software without specific prior written permission. #
#                                                                             #
# This program is free software: you can redistribute it and/or modify        #
# it under the terms of the GNU Lesser General Public License LGPL as         #
# published by the Free Software Foundation, either version 3 of the          #
# License, or (at your option) any later version.                             #
#                                                                             #
# This program is distributed in the hope that it will be useful,             #
# but WITHOUT ANY WARRANTY; without even the implied warranty of              #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                #
# GNU Lesser General Public License LGPL for more details.                    #
#                                                                             #
# You should have received a copy of the GNU Lesser General Public            #
# License LGPL along with this program.                                       #
# If not, see <http://www.gnu.org/licenses/>.                                 #
#                                                                             #
# #############################################################################
#                                                                             #
#   Author: Jacinto Colan, email: colan@robo.mein.nagoya-u.ac.jp              #
#                                                                             #
# ###########################################################################*/

// C++
#include <csignal>

// External
#include <ros/ros.h>

// Internal
#include <usb1608/usb1608.h>

bool kill_this_process = false;

void SigIntHandler(int sig)
{
  kill_this_process = true;
  ROS_INFO("Shutting down");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  int cycle_freq;

  // ROS Initialization
  ros::init(argc, argv, "usb1608_stream_node");
  ros::NodeHandle nh;
  signal(SIGINT, SigIntHandler);

  if (!nh.getParam("usb1608_stream_freq", cycle_freq))
    cycle_freq = 100;
  ROS_INFO("USB1608_stream_node: freq = %d Hz", cycle_freq);
  ros::Rate loop_rate(cycle_freq);

  USB1608 daq_ai(nh, "01F92A95");

  daq_ai.InitAI();
  daq_ai.InitDIO(4, 4);
  daq_ai.StartScanAI();

  // Control loop
  while (!kill_this_process)
  {
    daq_ai.UpdateScanStateAI();
    daq_ai.PublishStateAI();
    daq_ai.UpdateStateDI();
    daq_ai.PublishStateDI();
    loop_rate.sleep();
    ros::spinOnce();
  }

  daq_ai.StopScanAI();
  daq_ai.Quit();

  return 0;
}
