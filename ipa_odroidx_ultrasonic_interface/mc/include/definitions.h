/*!
*****************************************************************
* \file
*		definitions.h
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
* Global variable and struct definitions definitions.
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
#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#define MAX_VALUES 32 //Maximum number of values to be recorded per input pin
#define MAX_VALUES_SENT 15

#define MAX_INPUTS_PORTA 1 //Number of input pins on a port
#define MAX_INPUTS_PORTB 6
#define MAX_INPUTS_PORTC 6
#define MAX_INPUTS_PORTD 1

#define MAX_SENSOR_CONFIGS 16 //Maximum number of configurations that can be set at a time by the master.

struct TIME_KEEPER{ //struct for storing input port value and TIMER1 value in ISR associated with PCINTs
	uint8_t port_val;
	uint16_t time_reg_val;
};

extern volatile  uint8_t PING_STAGE; //For denoting different stages of a ping or listening pulse, from 0-3
/*
 *PORTA and PORTD are for output whereas PORTB and PORTC are for input
 *The board is programmed to use the external crystal of 18.432MHz and
 *the CLKDIV8 fuse is also programmed, causing effective frequency
 *equal to 2304000.
 *In PORTA_CONTROL and PORTD_CONTROL 1 denotes ping and 0 denotes listen,
 *whereas MSB is unused. Max value for a port should be 0x7F
 *NOTE:TIMER2 IN USE BY SOFTUART.
 */
//PORTA[6:0] and PORTD[6:0] are output pins connected to sensors, PORTA[7] and PORTD[7] are input pin.
extern volatile unsigned char PORTA_CONTROL; 
extern volatile unsigned char PORTD_CONTROL; 

extern volatile struct TIME_KEEPER PORTA_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTA];
extern volatile uint8_t PORTA_INPUT_count;// Counter for Input value for PORTA

extern volatile struct TIME_KEEPER PORTB_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTB];
extern volatile uint8_t PORTB_INPUT_count;// Counter for Input value for PORTB

extern volatile struct TIME_KEEPER PORTC_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTC];
extern volatile uint8_t PORTC_INPUT_count;// Counter for Input value for PORTC

extern volatile struct TIME_KEEPER PORTD_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTD];
extern volatile uint8_t PORTD_INPUT_count;// Counter for Input value for PORTD

extern volatile uint16_t TIMER[MAX_VALUES_SENT]; //For recording TIMER 1 due edge change.
extern volatile uint8_t TIMER_count;

extern volatile uint8_t CYCLE_COMPLETE;

extern volatile uint16_t SENSOR_CONFIG[MAX_SENSOR_CONFIGS]; //For storing sensor configurations sent by the master
extern volatile uint8_t TOTAL_SENSOR_CONFIGS;
extern volatile uint8_t CURRENT_SENSOR_CONFIG;
extern volatile uint8_t CONTROL_PORTS_SET;
#endif 
