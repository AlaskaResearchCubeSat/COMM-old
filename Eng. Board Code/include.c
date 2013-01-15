//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Inter-board Communication
//Include headers file

#include <include.h>

char TxBuffer[258];
char RxBuffer[258];
int data_length, bytes_remaining, buffer_position, packet_length, Tx_Flag;
int j;
char paTable_CC1101[] = {0xC5};
char paTable_CC2500[] = {0xFE};
char paTableLen = 1;
int TxBufferSize;
