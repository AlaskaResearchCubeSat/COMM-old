//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Inter-board Communication
//Include headers file

#include <include.h>

CTL_EVENT_SET_t radio_event_flags;

char TxBuffer[900];
char RxBuffer[900];
char uhf, temp_count1, temp_count2, button, RxBufferLen, radio;
unsigned int data_length, count, state, k, i, TxID, TxNum, end;
unsigned int TxBufferLen, TxBufferPos, TxBytesRemaining, TxThrBytes;
unsigned int RxBufferPos, RxBytesRemaining, RxFIFOLen, RxThrBytes, small_packet, PktLenUpper, PktLenLower, PktLen;
BOOL INFINITE;
char paTable_CC1101[] = {0x84};
char paTable_CC2500[] = {0xFB};
char paTableLen = 1;
