/*!
*****************************************************************
* \file
*		helper.c
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
* Implementation of helper.h functions.
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
#include <stdint.h>
#include "iocompat.h"
#include "softuart.h"
#include "helper.h"
volatile uint8_t PING_STAGE = 0;
volatile unsigned char PORTA_CONTROL=0x00; 
volatile unsigned char PORTD_CONTROL=0x00; 

volatile struct TIME_KEEPER PORTA_INPUT_VALS[];
volatile uint8_t PORTA_INPUT_count=0;// Counter for Input value for PORTA

volatile struct TIME_KEEPER PORTB_INPUT_VALS[];
volatile uint8_t PORTB_INPUT_count=0;// Counter for Input value for PORTB

volatile struct TIME_KEEPER PORTC_INPUT_VALS[];
volatile uint8_t PORTC_INPUT_count=0;// Counter for Input value for PORTC

volatile struct TIME_KEEPER PORTD_INPUT_VALS[];
volatile uint8_t PORTD_INPUT_count=0;// Counter for Input value for PORTD

volatile uint16_t TIMER[]; //For recording TIMER 1 due edge change.
volatile uint8_t TIMER_count=0;

volatile uint8_t CYCLE_COMPLETE = 0;

volatile uint16_t SENSOR_CONFIG[]; //For storing sensor configurations sent by the master
volatile uint8_t TOTAL_SENSOR_CONFIGS=0;
volatile uint8_t CURRENT_SENSOR_CONFIG=0;
volatile uint8_t CONTROL_PORTS_SET=0;

void print_TIMER(){ 
	for (uint8_t count=0; count <TIMER_count; count++){
		softuart_putchar((uint8_t)((TIMER[count] & 0xFF00)>> 8));
		softuart_putchar((uint8_t)(TIMER[count] & 0x00FF));
	}
}
void populateTIMER_SEND(uint8_t PORT_CONTROL, uint8_t CHANNEL_POSITION, volatile struct TIME_KEEPER * time_keeper, uint8_t INPUT_count, uint8_t INPUT_POSITION, uint8_t SENSOR_ADDRESS){
	/* This function is for populating the variable TIMER and sending it to the master (Check doc/pin_mapping.ods for correct PORT_CONTROL, CHANNEL_POSITION, INPUT_POSITION and SENSOR_ADDRESS).
	 * PORT_CONTROL: Is either set to PORTA_CONTROL or PORTD_CONTROL, based on the input pin to be checked. 
	 * time_keeper: Pointer to the TIME_KEEPER structure the channel input pin is located in.(PORT[A-D]_TIMER_VALS)
	 * INPUT_count: The PORT[A-D]_INPUT_count associated with time_keeper.
	 * INPUT_POSITION: Position of the channel's input pin on the associated input port.
	 * SENSOR_ADDRESS: Sensor address assigned to the sensor.
	 */

	uint8_t last_value_temp=INPUT_POSITION; //Is used to check for falling edges.
	TIMER_count=0;
	for(uint8_t count=0; count < INPUT_count; count++){ //For all input values while making sure that we have a maximum of 16 timer values. More than that will corrupt the sensor address value.
		if(((INPUT_POSITION & time_keeper[count].port_val)==0x00) && (last_value_temp==INPUT_POSITION)){ //Checks for falling edge.
			if (TIMER_count <MAX_VALUES_SENT)
				TIMER[TIMER_count++] = time_keeper[count].time_reg_val;
		}
		last_value_temp = (INPUT_POSITION & time_keeper[count].port_val); //Assign value to last_value_temp before looping.
	}
	//Sending the data out
	softuart_putchar((SENSOR_ADDRESS<<4) | TIMER_count); //Address + TIMER value
	if (TIMER_count>0){ //If any timer value recorded.
		print_TIMER();
	}
}

void printPORTA(){
	//For PA6-PC7
	//	POSITION ON PORT=0x80;
	populateTIMER_SEND(PORTA_CONTROL, 0x40, PORTC_INPUT_VALS, PORTC_INPUT_count, 0x80, 0x06);

	//For PA5-PB6
	//POSITION ON PORT = 0x40;
	populateTIMER_SEND(PORTA_CONTROL, 0x20, PORTB_INPUT_VALS, PORTB_INPUT_count, 0x40, 0x05);

	//For PA[4:0]-PB[4:0]
	uint8_t sensor_address = 0x04;
	for(uint8_t COMMON_MASK = 0x10; COMMON_MASK >0; COMMON_MASK>>=1){
		//	POSITION ON PORT = COMMON_MASK;
		populateTIMER_SEND(PORTA_CONTROL, COMMON_MASK, PORTB_INPUT_VALS, PORTB_INPUT_count, COMMON_MASK, sensor_address);
		sensor_address--;
	}
}

void printPORTD(){
	//The last values are set assuming that the inputs were initially high

	//For PD6-PD7
	//POSITION ON PORT = 0x80;
	populateTIMER_SEND(PORTD_CONTROL, 0x40, PORTD_INPUT_VALS, PORTD_INPUT_count, 0x80, 0x0D);

	//For PD[5:1]-PC[2:6]
	uint8_t sensor_address = 0xC;
	for (uint8_t count=2; count<7 ;count++)
		populateTIMER_SEND(PORTD_CONTROL, (0x80>>count), PORTC_INPUT_VALS, PORTC_INPUT_count, (1 << count), sensor_address--);

	//For PD0-PA7
	//POSITION ON PORT = 0x80;
	populateTIMER_SEND(PORTD_CONTROL, 0x01, PORTA_INPUT_VALS, PORTA_INPUT_count, 0x80, 0x07);
}

