//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Inter-board Communication
//Include headers file

#ifndef INCLUDE_H
#define INCLUDE_H

#include <MSP430.h>
#include <functions.h>
#include <Radio_Registers.h>
#include <string.h>
#include <AX25_send.h>
#include <uart.h>

extern char data[64];
extern char message;
extern int length, PacketSize;
extern unsigned int index;
extern int j, b, k, count_flag, data_flag, txBufferSize, tx_flag;
extern char paTable_CC1101[];
extern char paTable_CC2500[];
extern char paTableLen;


// Global Variables
extern char TX_Data[AX25_MAX];             	// 4 Data + 20 Bytes
                                	// local storage of the entire frame including flags, header, fcs
                                	// Do not use this memory for external functions as the content may
                                	//    be modified due to bit stuffing!

extern char TX_Data_to_CRC[16 + 4];		// 16 bytes (Address + control bytes) FIXED (do not modify)
					// + 4 bytes Data (modify according to size of your data field)
					// The array stores the AX25 header + Data that are used as input for the FCS (CRC) calculation

extern unsigned short 	TX_crc;            	// Store the FCS
extern char 	        TX_oneCounter;     	// counts the number of 1's in a row.  When it gets to 5, it is time to insert a 0.
extern unsigned int 	TX_bytePointer;    	// pointer to the current byte
extern unsigned int 	TX_bytePointerMax;	// Length of the packet
extern char 		TX_bitPointer;      	// Current pointer on the bit
extern unsigned short AX_crc_ccitt_table[];

#endif
