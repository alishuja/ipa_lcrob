/*!
*****************************************************************
* \file
*		UARTDriver.cpp
* \note
* Copyright (c) 2013 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: ipa_lcrob
* \note
* ROS package name: ipa_odroidx_ultrasonic_interface
*
* \author
* Author: Ali Shuja Siddiqui <joshas@ipa.fraunhofer.de>
* \author
* Supervised by: Joshua Hampp <Joshua.Hampp@ipa.fraunhofer.de>
*
* \date Date of creation: 10 Jan 2013
*
* \brief
* Class implementation of UARTDriver Class.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

/*
 * UARTDriver.cpp
 *
 *  Created on: Jan 9, 2013
 *  	Author: josh-as
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "ipa_odroidx_ultrasonic_interface/UARTDriver.h"
UARTDriver::UARTDriver(const char * device_filename, int baud_rate)
{
	ROS_INFO("Opening device file: %s", device_filename);
	this->fd_ = open(device_filename, O_RDWR | O_NOCTTY | O_SYNC);
	if (this->fd_ == -1)
	{
		ROS_ERROR("%s", strerror(errno));
		exit(EXIT_FAILURE);
	}
	ROS_INFO("Checking if device file is a tty.");
	if(!isatty(this->fd_))
	{
		ROS_ERROR("%s", strerror(errno));
		exit(EXIT_FAILURE);
	}
	ROS_INFO("Saving device configuration.");
	if (tcgetattr(this->fd_, &(this->old_config_)) < 0)
	{
		ROS_ERROR("%s", strerror(errno));
		exit(EXIT_FAILURE);
	}
	//Making a config for raw binary data transfer.
	cfmakeraw(&(this->new_config_));
	cfsetspeed(&(this->new_config_), baud_rate);
	//Setting up the new configuration.
	tcsetattr(this->fd_, TCSANOW, &(this->new_config_));
	tcsetattr(this->fd_, TCSAFLUSH, &(this->new_config_));
	ROS_INFO("Sleeping for four seconds to ensure settings are applied.");
	sleep(4);
	tcflush(this->fd_, TCIFLUSH);
}

ssize_t UARTDriver::readBytes(void * buf, size_t count)
{
	return read(this->fd_, buf, count); //No extra work is required.
}
ssize_t UARTDriver::writeBytes(void * buf, size_t count)
{
	int return_val =  write(this->fd_, buf, count);
	if(return_val > 0)
	{
		tcdrain(this->fd_);
		usleep(250000);
	}
	return return_val;
}

UARTDriver::~UARTDriver()
{
	//Setting up original serial port configuration again.
//	ROS_INFO("Loading original serial port settings.");
//	tcsetattr(this->fd_, TCSANOW, &(this->old_config_));
	close(this->fd_);
}
