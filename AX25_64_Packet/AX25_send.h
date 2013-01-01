#define AX25_MAX        250     //Max number of bytes for an ax25 frame.
#define DATA_MAX        230     //Max number of bytes for data field.

void TX_prepareFrame(char *, unsigned int);
unsigned short sysCRC(char *, unsigned short);
void compute_CRC(char *, unsigned short);
