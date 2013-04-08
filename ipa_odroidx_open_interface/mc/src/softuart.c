/*!
*****************************************************************
* \file
*		softuart.c
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
* Implementation of softuart functions.
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

#include <softuart.h>

#define SU_TRUE 1
#define SU_FALSE 0

#define RX_NUM_OF_BITS (8)
volatile static char           inbuf_port_1[SOFTUART_PORT_1_IN_BUF_SIZE], inbuf_port_2[SOFTUART_PORT_2_IN_BUF_SIZE] ;
volatile static unsigned char  qin_port_1, qin_port_2;
static unsigned char           qout_port_1, qout_port_2;
volatile static unsigned char  flag_rx_off_port_1, flag_rx_off_port_2;
volatile static unsigned char  flag_rx_ready_port_1, flag_rx_ready_port_2;

// 1 Startbit, 8 Databits, 1 Stopbit = 10 Bits/Frame
#define TX_NUM_OF_BITS (10)
volatile static unsigned char  flag_tx_busy_port_1, flag_tx_busy_port_2;
volatile static unsigned char  timer_tx_ctr_port_1, timer_tx_ctr_port_2;
volatile static unsigned char  bits_left_in_tx_port_1, bits_left_in_tx_port_2;
volatile static unsigned short internal_tx_buffer_port_1, internal_tx_buffer_port_2; /* ! mt: was type uchar - this was wrong */

#define set_tx_pin_high_port_1()      ( SOFTUART_PORT_1_TXPORT |=  ( 1 << SOFTUART_PORT_1_TXBIT ) )
#define set_tx_pin_low_port_1()       ( SOFTUART_PORT_1_TXPORT &= ~( 1 << SOFTUART_PORT_1_TXBIT ) )
#define get_rx_pin_status_port_1()    ( SOFTUART_PORT_1_RXPIN  &   ( 1 << SOFTUART_PORT_1_RXBIT ) )

#define set_tx_pin_high_port_2()      ( SOFTUART_PORT_2_TXPORT |=  ( 1 << SOFTUART_PORT_2_TXBIT ) )
#define set_tx_pin_low_port_2()       ( SOFTUART_PORT_2_TXPORT &= ~( 1 << SOFTUART_PORT_2_TXBIT ) )
#define get_rx_pin_status_port_2()    ( SOFTUART_PORT_2_RXPIN  &   ( 1 << SOFTUART_PORT_2_RXBIT ) )

ISR(SOFTUART_PORT_1_T_COMP_LABEL)
{
  static unsigned char flag_rx_waiting_for_stop_bit = SU_FALSE;
  static unsigned char rx_mask;
  
  static unsigned char timer_rx_ctr;
  static unsigned char bits_left_in_rx;
  static unsigned char internal_rx_buffer;
  
  unsigned char start_bit, flag_in;
  unsigned char tmp;
  
  // Transmitter Section
  if ( flag_tx_busy_port_1 == SU_TRUE ) {
    tmp = timer_tx_ctr_port_1;
    if ( --tmp == 0 ) { // if ( --timer_tx_ctr <= 0 )
      if ( internal_tx_buffer_port_1 & 0x01 ) {
	set_tx_pin_high_port_1();
      }
      else {
	set_tx_pin_low_port_1();
      }
      internal_tx_buffer_port_1 >>= 1;
      tmp = 3; // timer_tx_ctr = 3;
      if ( --bits_left_in_tx_port_1 == 0 ) {
	flag_tx_busy_port_1 = SU_FALSE;
      }
    }
    timer_tx_ctr_port_1 = tmp;
  }
  
  // Receiver Section
  if ( flag_rx_off_port_1 == SU_FALSE ) {
    if ( flag_rx_waiting_for_stop_bit ) {
      if ( --timer_rx_ctr == 0 ) {
	flag_rx_waiting_for_stop_bit = SU_FALSE;
	flag_rx_ready_port_1 = SU_FALSE;
	inbuf_port_1[qin_port_1] = internal_rx_buffer;
	if ( ++qin_port_1 >= SOFTUART_PORT_1_IN_BUF_SIZE ) {
	  // overflow - reset inbuf-index
	  qin_port_1 = 0;
	}
      }
    }
    else {  // rx_test_busy
      if ( flag_rx_ready_port_1 == SU_FALSE ) {
	start_bit = get_rx_pin_status_port_1();
	// test for start bit
	if ( start_bit == 0 ) {
	  flag_rx_ready_port_1      = SU_TRUE;
	  internal_rx_buffer = 0;
	  timer_rx_ctr       = 4;
	  bits_left_in_rx    = RX_NUM_OF_BITS;
	  rx_mask            = 1;
	}
      }
      else {  // rx_busy
	tmp = timer_rx_ctr;
	if ( --tmp == 0 ) { // if ( --timer_rx_ctr == 0 ) {
	  // rcv
	  tmp = 3;
	  flag_in = get_rx_pin_status_port_1();
	  if ( flag_in ) {
	    internal_rx_buffer |= rx_mask;
	  }
	  rx_mask <<= 1;
	  if ( --bits_left_in_rx == 0 ) {
	    flag_rx_waiting_for_stop_bit = SU_TRUE;
	  }
	}
	timer_rx_ctr = tmp;
      }
    }
  }
}

ISR(SOFTUART_PORT_2_T_COMP_LABEL)
{
  static unsigned char flag_rx_waiting_for_stop_bit = SU_FALSE;
  static unsigned char rx_mask;
  
  static unsigned char timer_rx_ctr;
  static unsigned char bits_left_in_rx;
  static unsigned char internal_rx_buffer;
  
  unsigned char start_bit, flag_in;
  unsigned char tmp;
  
  // Transmitter Section
  if ( flag_tx_busy_port_2 == SU_TRUE ) {
    tmp = timer_tx_ctr_port_2;
    if ( --tmp == 0 ) { // if ( --timer_tx_ctr <= 0 )
      if ( internal_tx_buffer_port_2 & 0x01 ) {
	set_tx_pin_high_port_2();
      }
      else {
	set_tx_pin_low_port_2();
      }
      internal_tx_buffer_port_2 >>= 1;
      tmp = 3; // timer_tx_ctr = 3;
      if ( --bits_left_in_tx_port_2 == 0 ) {
	flag_tx_busy_port_2 = SU_FALSE;
      }
    }
    timer_tx_ctr_port_2 = tmp;
  }
  
  // Receiver Section
  if ( flag_rx_off_port_2 == SU_FALSE ) {
    if ( flag_rx_waiting_for_stop_bit ) {
      if ( --timer_rx_ctr == 0 ) {
	flag_rx_waiting_for_stop_bit = SU_FALSE;
	flag_rx_ready_port_2 = SU_FALSE;
	inbuf_port_2[qin_port_2] = internal_rx_buffer;
	if ( ++qin_port_2 >= SOFTUART_PORT_2_IN_BUF_SIZE ) {
	  // overflow - reset inbuf-index
	  qin_port_2 = 0;
	}
      }
    }
    else {  // rx_test_busy
      if ( flag_rx_ready_port_2 == SU_FALSE ) {
	start_bit = get_rx_pin_status_port_2();
	// test for start bit
	if ( start_bit == 0 ) {
	  flag_rx_ready_port_2      = SU_TRUE;
	  internal_rx_buffer = 0;
	  timer_rx_ctr       = 4;
	  bits_left_in_rx    = RX_NUM_OF_BITS;
	  rx_mask            = 1;
	}
      }
      else {  // rx_busy
	tmp = timer_rx_ctr;
	if ( --tmp == 0 ) { // if ( --timer_rx_ctr == 0 ) {
	  // rcv
	  tmp = 3;
	  flag_in = get_rx_pin_status_port_2();
	  if ( flag_in ) {
	    internal_rx_buffer |= rx_mask;
	  }
	  rx_mask <<= 1;
	  if ( --bits_left_in_rx == 0 ) {
	    flag_rx_waiting_for_stop_bit = SU_TRUE;
	  }
	}
	timer_rx_ctr = tmp;
      }
    }
  }
}


static void io_init(SOFTUART_PORT s_p)
{
  if (s_p == PORT_1)
    {
      // TX-Pin as output
      SOFTUART_PORT_1_TXDDR |=  ( 1 << SOFTUART_PORT_1_TXBIT );
      // RX-Pin as input
      SOFTUART_PORT_1_RXDDR &= ~( 1 << SOFTUART_PORT_1_RXBIT );
    }
  else if (s_p == PORT_2)
    {
      // TX-Pin as output
      SOFTUART_PORT_2_TXDDR |=  ( 1 << SOFTUART_PORT_2_TXBIT );
      // RX-Pin as input
      SOFTUART_PORT_2_RXDDR &= ~( 1 << SOFTUART_PORT_2_RXBIT );
    }
}

static void timer_init(SOFTUART_PORT s_p)
{
  unsigned char sreg_tmp;
  
  sreg_tmp = SREG;
  cli();
  if (s_p == PORT_1)
    {
      SOFTUART_PORT_1_T_COMP_REG = SOFTUART_PORT_1_TIMERTOP;     /* set top */
      
      SOFTUART_PORT_1_T_CONTR_REGA = SOFTUART_PORT_1_CTC_MASKA | SOFTUART_PORT_1_PRESC_MASKA;
      SOFTUART_PORT_1_T_CONTR_REGB = SOFTUART_PORT_1_CTC_MASKB | SOFTUART_PORT_1_PRESC_MASKB;
      
      SOFTUART_PORT_1_T_INTCTL_REG |= SOFTUART_PORT_1_CMPINT_EN_MASK;
      
      SOFTUART_PORT_1_T_CNT_REG = 0; /* reset counter */
    }
  else if (s_p == PORT_2)
    {
      SOFTUART_PORT_2_T_COMP_REG = SOFTUART_PORT_2_TIMERTOP;     /* set top */
      SOFTUART_PORT_2_T_CONTR_REGA = SOFTUART_PORT_2_CTC_MASKA | SOFTUART_PORT_2_PRESC_MASKA;
      SOFTUART_PORT_2_T_CONTR_REGB = SOFTUART_PORT_2_CTC_MASKB | SOFTUART_PORT_2_PRESC_MASKB;
      SOFTUART_PORT_2_T_INTCTL_REG |= SOFTUART_PORT_2_CMPINT_EN_MASK;
      SOFTUART_PORT_2_T_CNT_REG = 0; /* reset counter */
    }
  SREG = sreg_tmp;
}

void softuart_init( SOFTUART_PORT s_p )
{
  if (s_p == PORT_1)
    {
      flag_tx_busy_port_1  = SU_FALSE;
      flag_rx_ready_port_1 = SU_FALSE;
      flag_rx_off_port_1   = SU_FALSE;
      set_tx_pin_high_port_1(); /* mt: set to high to avoid garbage on init */
    }
  else if (s_p == PORT_2)
    {
      flag_tx_busy_port_2  = SU_FALSE;
      flag_rx_ready_port_2 = SU_FALSE;
      flag_rx_off_port_2   = SU_FALSE;
      set_tx_pin_high_port_2(); /* mt: set to high to avoid garbage on init */
    }
  io_init(s_p);
  timer_init(s_p);
}

static void idle(void)
{
  // timeout handling goes here 
  // - but there is a "softuart_kbhit" in this code...
  // add watchdog-reset here if needed
}

void softuart_turn_rx_on( SOFTUART_PORT s_p )
{
  if (s_p == PORT_1)
    flag_rx_off_port_1 = SU_FALSE;
  else if (s_p == PORT_2)
    flag_rx_off_port_2 = SU_FALSE;
}

void softuart_turn_rx_off( SOFTUART_PORT s_p )
{
  if (s_p == PORT_1)
    flag_rx_off_port_1 = SU_TRUE;
  else if (s_p == PORT_2)
    flag_rx_off_port_2 = SU_TRUE;
}

char softuart_getchar( SOFTUART_PORT s_p )
{
  char ch = 0;
  if (s_p == PORT_1)
    {
      while ( qout_port_1 == qin_port_1 ) {
	idle();
      }
      ch = inbuf_port_1[qout_port_1];
      if ( ++qout_port_1 >= SOFTUART_PORT_1_IN_BUF_SIZE ) {
	qout_port_1 = 0;
      }
    }
  else if (s_p == PORT_2)
    {
      while ( qout_port_2 == qin_port_2 ) {
	idle();
      }
      ch = inbuf_port_2[qout_port_2];
      if ( ++qout_port_2 >= SOFTUART_PORT_2_IN_BUF_SIZE ) {
	qout_port_2 = 0;
      }
    }
  return( ch );
}

unsigned char softuart_kbhit(SOFTUART_PORT s_p)
{
  if (s_p == PORT_1)
    return( qin_port_1 != qout_port_1 );
  else if (s_p == PORT_2)
    return (qin_port_2 != qout_port_2 );
}

void softuart_flush_input_buffer( SOFTUART_PORT s_p )
{
  if (s_p == PORT_1)
    {
      qin_port_1  = 0;
      qout_port_1 = 0;
    }
  else if (s_p == PORT_2)
    {
      qin_port_2  = 0;
      qout_port_2 = 0;
    }
}

unsigned char softuart_transmit_busy( SOFTUART_PORT s_p ) 
{
  if (s_p == PORT_1)
    return ( flag_tx_busy_port_1 == SU_TRUE ) ? 1 : 0;
  else if (s_p == PORT_2)
    return ( flag_tx_busy_port_2 == SU_TRUE ) ? 1 : 0;
}

void softuart_putchar( SOFTUART_PORT s_p, const char ch )
{
  if (s_p == PORT_1)
    {
      while ( flag_tx_busy_port_1 == SU_TRUE ) {
	; // wait for transmitter ready
	// add watchdog-reset here if needed;
      }
      
      // invoke_UART_transmit
      timer_tx_ctr_port_1       = 3;
      bits_left_in_tx_port_1    = TX_NUM_OF_BITS;
      internal_tx_buffer_port_1 = ( ch << 1 ) | 0x200;
      flag_tx_busy_port_1       = SU_TRUE;
    }
  else if (s_p == PORT_2)
    {
      while ( flag_tx_busy_port_2 == SU_TRUE ) {
	; // wait for transmitter ready
	// add watchdog-reset here if needed;
      }
      
      // invoke_UART_transmit
      timer_tx_ctr_port_2       = 3;
      bits_left_in_tx_port_2    = TX_NUM_OF_BITS;
      internal_tx_buffer_port_2 = ( ch << 1 ) | 0x200;
      flag_tx_busy_port_2       = SU_TRUE;
    }
}

void softuart_puts( SOFTUART_PORT s_p, const char *s )
{
  while ( *s ) {
    softuart_putchar(s_p, *s++ );
  }
}

void softuart_broadcast(const char *s)
{
	while(*s){
		softuart_putchar(PORT_1, *s);
		softuart_putchar(PORT_2, *s++);
	}
}
