//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, APril 2012

//Include Header file

#ifndef INCLUDE_H
#define INCLUDE_H

#include <MSP430.h>
#include <functions.h>
#include <Radio_Registers.h>
#include <string.h>
#include <stdio.h>
#include <uart.h>
#include <radio_interrupts.h>
#include <initialize_clk.h>
#include <initialize_timerA.h>

extern char TxBuffer[258];
extern char RxBuffer[258];
extern char message, radio;
extern int RxPktLength;
extern unsigned int index;
extern int j, Tx_Flag, TxBufferSize, data_length, buffer_position, packet_length;
extern char paTable_CC1101[];
extern char paTable_CC2500[];
extern char paTableLen;
extern unsigned int i, k;

#endif
