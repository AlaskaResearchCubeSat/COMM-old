//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, APril 2012

//Include Header file

#ifndef INCLUDE_H
#define INCLUDE_H
#define CC1101  1
#define CC2500  0

typedef int BOOL;
#define TRUE 1
#define FALSE 0

#include <MSP430.h>
#include <functions.h>
#include <Radio_Registers.h>
#include <string.h>
#include <stdio.h>
#include <UCA1_uart.h>


extern CTL_EVENT_SET_t radio_event_flags;

extern char TxBuffer[900];
extern char RxBuffer[900];
extern char uhf, temp_count1, temp_count2, button, RxBufferLen;
extern unsigned int data_length, count, state, k, i, TxID, TxNum;
extern unsigned int TxBufferLen, TxBufferPos, TxBytesRemaining, TxThrBytes;
extern unsigned int RxBufferPos, RxBytesRemaining, RxFIFOLen, RxThrBytes, small_packet,  PktLenUpper, PktLenLower, PktLen;
extern BOOL INFINITE;
extern char paTable_CC1101[];
extern char paTable_CC2500[];
extern char paTableLen;


#endif

