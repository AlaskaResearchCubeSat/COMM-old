//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, April 2012
#ifndef FUNCTIONS
#define FUNCTIONS

#define CS_1101 BIT4
#define CS_2500 BIT5

#define IDLE         0
#define TX_START     1
#define TX_RUNNING   2
#define RX_START     3
#define RX_RUNNING   4
#define CC1101_GDO0  BIT0
#define CC1101_GDO2  BIT2
#define CC2500_GDO0  BIT3
#define CC2500_GDO2  BIT4
#define RF_SW1       BIT0
#define RF_SW2       BIT1

//Create event flags for the radios
enum{CC1101_EV_RX_SYNC=(1<<0),CC1101_EV_RX_THR=(1<<1),CC1101_EV_RX_END=(1<<2),CC1101_EV_TX_START=(1<<3),CC1101_EV_TX_THR=(1<<4),CC1101_EV_TX_END=(1<<5),CC2500_EV_RX_SYNC=(1<<6),CC2500_EV_RX_THR=(1<<7),CC2500_EV_TX_START=(1<<8),CC2500_EV_TX_THR=(1<<9)};
//All events for the radios
#define RADIO_EVENTS (CC1101_EV_RX_SYNC|CC1101_EV_RX_THR|CC1101_EV_RX_END|CC1101_EV_TX_START|CC1101_EV_TX_THR|CC1101_EV_TX_END|CC2500_EV_RX_SYNC|CC2500_EV_RX_THR|CC2500_EV_TX_START|CC2500_EV_TX_THR)    

void sub_events(void *);
void TXRX(void *);
void radio_interrupts(void);
void Build_Packet(int);
void TI_CC_Wait(unsigned int);
void SPI_Setup(void);
void Reset_Radio(char);
void Radio_Strobe(char, char);
void Radio_Write_Registers(char, char, char);
void Radio_Write_Burst_Registers(char, char *, int, char);
char Radio_Read_Registers(char, char);
void Radio_Read_Burst_Registers(char, char *, int, char);
char Radio_Read_Status(char, char);
char RF_Receive_Packet(char *, char *, char);
void RF_Send_Packet(char *, int, char);
void Write_RF_Settings(char);

#endif
