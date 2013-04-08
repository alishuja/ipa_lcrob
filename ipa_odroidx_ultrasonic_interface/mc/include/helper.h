/*!
*****************************************************************
* \file
*		helper.h
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
* \date Date of creation: 07 Jan 2013
*
* \brief
* Function prototypes helper functions for timer related functions.
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
#ifndef __HELPER_H__
#define __HELPER_H__
#include "definitions.h"
void print_TIMER(); //Send the set timer values to the master, higher byte first.

/* This function is for populating the variable TIMER and sending it to the master
 * (Check doc/pin_mapping.ods for correct PORT_CONTROL, CHANNEL_POSITION, INPUT_POSITION and SENSOR_ADDRESS).
 * PORT_CONTROL: Is either set to PORTA_CONTROL or PORTD_CONTROL, based on the input pin to be checked. 
 * time_keeper: Pointer to the TIME_KEEPER structure the channel input pin is located in.(PORT[A-D]_TIMER_VALS)
 * INPUT_count: The PORT[A-D]_INPUT_count associated with time_keeper.
 * INPUT_POSITION: Position of the channel's input pin on the associated input port.
 * SENSOR_ADDRESS: Sensor address assigned to the sensor.
 */
void populateTIMER_SEND(uint8_t PORT_CONTROL, uint8_t CHANNEL_POSITION, volatile struct TIME_KEEPER * time_keeper, uint8_t INPUT_count, uint8_t INPUT_POSITION, uint8_t SENSOR_ADDRESS);

void printPORTA(); //Send timer values calculated on PORTA to the master.
void printPORTD(); ///Send timer values calculated on PORTD to the master.

#endif
