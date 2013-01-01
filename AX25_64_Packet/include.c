//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Inter-board Communication
//Include headers file

#include <include.h>

char data[64];
char message;
int length;
unsigned int index = 0;
int j, k, b, txBufferSize, PacketSize;
int count_flag, data_flag, tx_flag = 0;
char paTable_CC1101[] = {0x51};
char paTable_CC2500[] = {0xFB};
char paTableLen = 1;
    

// Global Variables for AX.25 function
char TX_Data[AX25_MAX];             	// 4 Data + 20 Bytes
                                	// local storage of the entire frame including flags, header, fcs
                                	// Do not use this memory for external functions as the content may
                                	//    be modified due to bit stuffing!

char TX_Data_to_CRC[16 + 4];		// 16 bytes (Address + control bytes) FIXED (do not modify)
					// + 4 bytes Data (modify according to size of your data field)
					// The array stores the AX25 header + Data that are used as input for the FCS (CRC) calculation

unsigned short 	TX_crc;            	// Store the FCS
char 	        TX_oneCounter;     	// counts the number of 1's in a row.  When it gets to 5, it is time to insert a 0.
unsigned int 	TX_bytePointer;    	// pointer to the current byte
unsigned int 	TX_bytePointerMax;	// Length of the packet
char 		TX_bitPointer;      	// Current pointer on the bit
unsigned short AX_crc_ccitt_table[];
