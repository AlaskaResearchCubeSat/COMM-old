//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Inter-board Communication
//Include headers file

#include <include.h>

char data[66];
char message;
int length;
unsigned int index = 0;
int j, k, b, txBufferSize;
int count_flag, data_flag, tx_flag, rx_flag = 0;
char paTable_CC1101[] = {0x51};
char paTable_CC2500[] = {0xFB};
char paTableLen = 1;

    
