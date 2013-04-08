/*!
*****************************************************************
* \file
*		main.c
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
* \date Date of creation: 04 Dec 2012
*
* \brief
* Main loop implementation.
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "helper.h"
#include "softuart.h"
int main(void){
	MCUSR = 0; //Setting Reset Status Register to 0 (In case the watchdog timer restarts the controller).

	//Setting PORTA and PORTD as output
	DDRA = 0x7F;
	DDRD = 0x7F;

	//Setting PORTB and PORTC as input
	DDRB = 0x00;
	DDRC = 0x00;

	//Setting initial values for PORTA and PORTC
	PORTA = 0;
	PORTD = 0;

	PORTB = 0x00; //Disabling use of pull up resistors
	PORTC = 0x00;

	softuart_init();

	//prescalers
	TCCR0B |= (1 << CS01);
	TCCR1B |= (1 << CS11);

	// Place to enable PCINT
	//

	PCMSK0 |= 0x80;
	PCMSK1 |= 0x5F;
	PCMSK2 |= 0xFE;
	PCMSK3 |= 0x80;

	sei(); // Setting global interrupt

	for(;;){
		if(CURRENT_SENSOR_CONFIG < TOTAL_SENSOR_CONFIGS){ //Traversing through each sensor config sequentially.
			if(CONTROL_PORTS_SET==1){ // If current config has been loaded.
				if(CYCLE_COMPLETE == 1){ //After PORT[A-D]_TIMER_VALS have been populated.
					softuart_putchar(CURRENT_SENSOR_CONFIG);//Sending out current sensor configuration number.
					printPORTD();
					printPORTA();

					//After All Values have been sent out to the master.
					PORTA_INPUT_count=0;
					PORTB_INPUT_count=0;
					PORTC_INPUT_count=0;
					PORTD_INPUT_count=0;

					CONTROL_PORTS_SET =0; //To load the new configuration.
					CURRENT_SENSOR_CONFIG++;
				}
			}
			else{
				PORTA_CONTROL = (uint8_t)(SENSOR_CONFIG[CURRENT_SENSOR_CONFIG] & 0x00FF);
				PORTD_CONTROL = (uint8_t)(SENSOR_CONFIG[CURRENT_SENSOR_CONFIG] >> 8);
				CONTROL_PORTS_SET=1;

				CYCLE_COMPLETE = 0;

				TCNT0 = 1; //Loading random overflow value.
				TIFR0 |= (1 << TOV0); //Forced timer0 interrupt trigger.
				TIMSK0 |= (1 << TOIE0); // Enabling timer 1 overflow

			}
		}
		else{
			if(softuart_kbhit()){ //To check if any new configuration is received from the master.
				//Reading config from serial port
				uint8_t number_of_config = 0;
				number_of_config = softuart_getchar(); //Should be less than 16, however no error detection.
				if(number_of_config >0){
					uint16_t temp16 = 0; // 16bit variable to store configuration input from the master.
					uint8_t temp8 = 0;
					for (uint8_t count = 0; count < number_of_config; count++){
						wdt_enable(WDTO_4S); //Sets the watchdog timer for a timeout of 4 seconds. 
							temp16 = (softuart_getchar() << 8);
						wdt_disable(); //Disables the watchdog timer if the completes successfully.
						
						wdt_enable(WDTO_4S);
							temp8 = softuart_getchar();
						wdt_disable();
						SENSOR_CONFIG[count] = (temp16 | temp8) ;
					}
					TOTAL_SENSOR_CONFIGS = number_of_config;
					CURRENT_SENSOR_CONFIG = 0;
					softuart_putchar(0x12); //Sending acknowledgement back to the master.
					softuart_putchar(0x7f &(((PIND & 0x80) >> 1) | ((PINC & 0x04) <<3) | ((PINC & 0x08) <<1) | ((PINC & 0x10) >> 1) | ((PINC & 0x20) >> 3) | ((PINC & 0x40) >>5) | ((PINA & 0x80)>>7)));
					softuart_putchar(0x7f &(((PINC & 0x80) >> 1) | (((PINB & 0x40)>>1) |(PINB & 0x1f))));
				}
			}
			else if(TOTAL_SENSOR_CONFIGS > 0){
				CURRENT_SENSOR_CONFIG = 0; // Resetting the configuration
			}
		}
	}
}
ISR(TIMER0_OVF_vect){ // Timer 0 is dedicated for Pinging and listening.
	if(PING_STAGE == 0){
		PORTA = 0x7F;
		PORTD = 0x7F;
		TCNT0 = 160; // Setting for 330us
		PING_STAGE = 1;
	}
	else if(PING_STAGE == 1){
		PORTA = (~PORTA_CONTROL) & (0x7F);
		PORTD = (~PORTD_CONTROL) & (0x7F);
		TCNT0 = 154; // Setting for 350us -0.02% error
		PING_STAGE = 2;

		//Setting up timer1 for 100ms
		TCNT1 = 36735;
		TIMSK1 |= (1 << TOIE1);

		PCICR |= 0x0F; // Enabling input interrupts.
	}
	else if(PING_STAGE == 2){
		PORTA = 0x00;
		PORTD = 0x00;
		//Disabling timer0 to setup timer1 to enable data reading from sensors.
		TIMSK0 &= ~(1 << TOIE0);

//		softuart_disable(); // Not sure if works or not.

	}	
}
ISR(TIMER1_OVF_vect){
	//Disabling timer1
	TIMSK1 &= ~(1 << TOIE1);

	PING_STAGE = 0;

	//Trigger time send
	CYCLE_COMPLETE = 1;

	//Disbaling all PCINTs
	PCICR &= 0xF0;

//	softuart_enable(); //Not sure if works or not 
}

ISR(PCINT0_vect){
	if(PORTA_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTA)){
		PORTA_INPUT_VALS[PORTA_INPUT_count].port_val=PINA;
		PORTA_INPUT_VALS[PORTA_INPUT_count].time_reg_val = TCNT1 - 36735; // Subtracting the 100ms offset before recording the Timer1 value.
		PORTA_INPUT_count++;
	}
}

ISR(PCINT1_vect){
	if(PORTB_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTB)){
		PORTB_INPUT_VALS[PORTB_INPUT_count].port_val=PINB;
		PORTB_INPUT_VALS[PORTB_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTB_INPUT_count++;
	}
}

ISR(PCINT2_vect){
	if(PORTC_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTC)){
		PORTC_INPUT_VALS[PORTC_INPUT_count].port_val=PINC;
		PORTC_INPUT_VALS[PORTC_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTC_INPUT_count++;
	}
}

ISR(PCINT3_vect){
	if(PORTD_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTD)){
		PORTD_INPUT_VALS[PORTD_INPUT_count].port_val=PIND;
		PORTD_INPUT_VALS[PORTD_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTD_INPUT_count++;
	}
}

