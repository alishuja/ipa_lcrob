/*!
*****************************************************************
* \file
*		mcontroller.c
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
* Implementation of motor controller helper functions.
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
#include <stdlib.h>
#include "mcontroller.h"
#include "softuart.h"

void int2str(char *str, int16_t val){
	int16_t div=10000;
	if (val <0){
		*str = '-';
		str++;
		val=abs(val);
	}
	while(!(val/div) && div > 1) div/=10;
	while(div){
		*str = '0'+(val/div);
		str++;
		val %= div;
		div/=10;
	}
	*str = '\r';
	str++;
	*str = '\n';
	str++;
	*str = 0;
}

void motor_init() {
	softuart_broadcast(ENABLE);
	softuart_broadcast("HO\r\n"); // set current position to 0
}

void motor_stop() {
	softuart_broadcast(DISABLE);
}

void motor_setVel(int16_t rpm1, int16_t rpm2) {
	char buffer[8];
	buffer[0]='V';

	int2str(buffer+1, rpm1);
	softuart_puts(PORT_1, buffer);

	int2str(buffer+1, rpm2);
	softuart_puts(PORT_2, buffer);
}

void motor_reqPos(uint8_t motor){
	softuart_flush_input_buffer(motor);
	softuart_puts(motor, POSITION);
}
int32_t motor_getPos(uint8_t motor, uint8_t * valid_val) {
	if(softuart_kbhit(motor)){	
		char c;
		int32_t r=0;
		uint8_t is_negative= 0;
		while( (c=softuart_getchar(motor))!='\r' ) { // MUST IMPLEMENT WATCHDOG TIMER HERE
			if(c=='-') is_negative=1; // Assuming "-" is the first character.
			else if(c>='0'&&c<='9') {
				r*=10;
				r+=c-'0';
			}
		}
		if(is_negative)r*=-1;
		softuart_getchar(motor); // for '\n', MUST IMPLEMENT WATCHDOG TIMER HERE
		*valid_val = 1;
		return r;
	}
	else{
		*valid_val = 0;
		return 0;
	}
}
