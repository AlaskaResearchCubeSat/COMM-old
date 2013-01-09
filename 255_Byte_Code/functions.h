//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, April 2012

#define CS_1101 BIT4
#define CS_2500 BIT5

#define IDLE         0
#define TX_START     1
#define TX_RUNNING   2
#define RX_START     3
#define RX_RUNNING   4
#define CC1101_GDO0  BIT0
#define CC1101_GDO2  BIT1
#define CC2500_GDO0  BIT3
#define CC2500_GDO2  BIT4
#define RF_SW1       BIT0
#define RF_SW2       BIT1

void TXRX(void *);
void radio_interrupts(void);
void Build_Packet(int);
void TI_CC_Wait(unsigned int);
void SPI_Setup(void);
void Reset_Radios(void);
void Radio_Strobe(char, char);
void Radio_Write_Registers(char, char, char);
void Radio_Write_Burst_Registers(char, char *, int, char);
char Radio_Read_Registers(char, char);
void Radio_Read_Burst_Registers(char, char *, int, char);
char Radio_Read_Status(char, char);
char RF_Receive_Packet(char *, char *, char);
void RF_Send_Packet(char *, int, char);
void Write_RF_Settings(void);
