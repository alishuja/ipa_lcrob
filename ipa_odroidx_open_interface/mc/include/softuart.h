/*!
*****************************************************************
* \file
*		softuart.h
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
* \date Date of creation: 19 Feb 2013
*
* \brief
* Rewrite of softaurt library available from: 
*	http://homepage.hispeed.ch/peterfleury/avr-software.html#libs. 
* This implementation extends a single soft UART implementation to provide
* two soft UARTs.
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
#ifndef __SOFTUART_H__
#define __SOFTUART_H__

#if !defined(F_CPU)
	#warning "F_CPU not defined in makefile - now defined in softuart.h"
	#define F_CPU 3686400UL
#endif

#define SOFTUART_PORT_1_BAUD_RATE 9600
#define SOFTUART_PORT_2_BAUD_RATE 9600

typedef enum {PORT_1=0, PORT_2=1} SOFTUART_PORT;

//PORT 1 Definitions
#define SOFTUART_PORT_1_RXPIN PIND
#define SOFTUART_PORT_1_RXDDR DDRD
#define SOFTUART_PORT_1_RXBIT PD2

#define SOFTUART_PORT_1_TXPORT PORTD
#define SOFTUART_PORT_1_TXDDR DDRD
#define SOFTUART_PORT_1_TXBIT PD4

#define SOFTUART_PORT_1_T_COMP_LABEL      TIMER0_COMPA_vect
#define SOFTUART_PORT_1_T_COMP_REG        OCR0A
#define SOFTUART_PORT_1_T_CONTR_REGA      TCCR0A
#define SOFTUART_PORT_1_T_CONTR_REGB      TCCR0B
#define SOFTUART_PORT_1_T_CNT_REG         TCNT0
#define SOFTUART_PORT_1_T_INTCTL_REG      TIMSK0
#define SOFTUART_PORT_1_CMPINT_EN_MASK    (1 << OCIE0A)
#define SOFTUART_PORT_1_CTC_MASKA         (1 << WGM01)
#define SOFTUART_PORT_1_CTC_MASKB         (0)

#define SOFTUART_PORT_1_PRESCALE (8)

#if (SOFTUART_PORT_1_PRESCALE == 8)
	#define SOFTUART_PORT_1_PRESC_MASKA         (0)
	#define SOFTUART_PORT_1_PRESC_MASKB         (1 << CS01)
#elif (SOFTUART_PORT_1_PRESCALE==1)
	#define SOFTUART_PORT_1_PRESC_MASKA         (0)
	#define SOFTUART_PORT_1_PRESC_MASKB         (1 << CS00)
#else 
	#error "prescale unsupported"
#endif

#define SOFTUART_PORT_1_TIMERTOP ( F_CPU/SOFTUART_PORT_1_PRESCALE/SOFTUART_PORT_1_BAUD_RATE/3 - 1)

//PORT 2 Definitions
#define SOFTUART_PORT_2_RXPIN PIND
#define SOFTUART_PORT_2_RXDDR DDRD
#define SOFTUART_PORT_2_RXBIT PD3

#define SOFTUART_PORT_2_TXPORT PORTD
#define SOFTUART_PORT_2_TXDDR DDRD
#define SOFTUART_PORT_2_TXBIT PD5

#define SOFTUART_PORT_2_T_COMP_LABEL      TIMER2_COMPA_vect
#define SOFTUART_PORT_2_T_COMP_REG        OCR2A
#define SOFTUART_PORT_2_T_CONTR_REGA      TCCR2A
#define SOFTUART_PORT_2_T_CONTR_REGB      TCCR2B
#define SOFTUART_PORT_2_T_CNT_REG         TCNT2
#define SOFTUART_PORT_2_T_INTCTL_REG      TIMSK2
#define SOFTUART_PORT_2_CMPINT_EN_MASK    (1 << OCIE2A)
#define SOFTUART_PORT_2_CTC_MASKA         (1 << WGM21)
#define SOFTUART_PORT_2_CTC_MASKB         (0)


#define SOFTUART_PORT_2_PRESCALE (8)

#if (SOFTUART_PORT_2_PRESCALE == 8)
	#define SOFTUART_PORT_2_PRESC_MASKA         (0)
	#define SOFTUART_PORT_2_PRESC_MASKB         (1 << CS21)
#elif (SOFTUART_PORT_2_PRESCALE==1)
	#define SOFTUART_PORT_2_PRESC_MASKA         (0)
	#define SOFTUART_PORT_2_PRESC_MASKB         (1 << CS20)
#else 
	#error "prescale unsupported"
#endif

#define SOFTUART_PORT_2_TIMERTOP ( F_CPU/SOFTUART_PORT_2_PRESCALE/SOFTUART_PORT_2_BAUD_RATE/3 - 1)

#define SOFTUART_PORT_1_IN_BUF_SIZE 32
#define SOFTUART_PORT_2_IN_BUF_SIZE 32

// Init the Software Uart
void softuart_init(SOFTUART_PORT);

// Clears the contents of the input buffer.
void softuart_flush_input_buffer(SOFTUART_PORT);

// Tests whether an input character has been received.
unsigned char softuart_kbhit(SOFTUART_PORT);

// Reads a character from the input buffer, waiting if necessary.
char softuart_getchar(SOFTUART_PORT);

// To check if transmitter is busy
unsigned char softuart_transmit_busy(SOFTUART_PORT);

// Writes a character to the serial port.
void softuart_putchar(SOFTUART_PORT, const char );

// Turns on the receive function.
void softuart_turn_rx_on(SOFTUART_PORT);

// Turns off the receive function.
void softuart_turn_rx_off(SOFTUART_PORT);

// Write a NULL-terminated string from RAM to the serial port
void softuart_puts(SOFTUART_PORT s_p, const char *s );

//Broadcast a single message to the all the port sumultaneously. 
void softuart_broadcast( const char *s);

#endif
