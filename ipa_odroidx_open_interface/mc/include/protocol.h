/*!
*****************************************************************
* \file
*		protocol.h
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
* ROS package name: ipa_odroidx_open_interface
*
* \author
* Author: Ali Shuja Siddiqui <joshas@ipa.fraunhofer.de>
* \author
* Supervised by: Joshua Hampp <Joshua.Hampp@ipa.fraunhofer.de>
*
* \date Date of creation: 03 Dec 2012
*
* \brief
* Function prototypes for handling iRobot's Create Open Protocol 
* Specification and initialization functions for the micro-controller.
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
#ifndef PROTOCOL_H
#define PROTOCOL_H

void init(void);
void parse(void);
void init_motors(void);
void parseSendSensorPacket(uint8_t);
void generateStreamResponse(void);
uint8_t is_stream_enabled(void);
void updatePosition(void);

extern volatile uint8_t		TIMER_OVERFLOW;
#endif
