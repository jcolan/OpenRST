/******************************************************************************
# usbquad08_stream.cpp:  ROS node for DAQ board MC USBQUAD08                         #
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
#include <usbquad08/usbquad08.h>

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
  ros::init(argc, argv, "usbquad08_stream_node");
  ros::NodeHandle nh;
  signal(SIGINT, SigIntHandler);

  if (!nh.getParam("usbquad08_stream_freq", cycle_freq))
    cycle_freq = 100;
  ROS_INFO("USBQUAD08_stream_node: freq = %d Hz", cycle_freq);
  ros::Rate loop_rate(cycle_freq);

  USBQUAD08 daq_enc(nh, "1001633");

  daq_enc.InitENC();
  // daq_enc.set_offset(5000);
  daq_enc.StartScanENC();

  // Control loop
  while (!kill_this_process)
  {
    // daq_enc.UpdateStateENC();
    daq_enc.UpdateScanStateENC();
    daq_enc.PublishStateENC();

    loop_rate.sleep();
    ros::spinOnce();
  }

  daq_enc.Quit();

  return 0;
}
