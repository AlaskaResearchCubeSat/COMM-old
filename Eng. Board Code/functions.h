//Code for engineering model Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, August 2012

#define MOSI     BIT1
#define MISO     BIT2
#define CLK      BIT3
#define CS_2500  BIT4
#define CS_1101  BIT5
#define CS_TEMP1 BIT6
#define CS_TEMP2 BIT7

#define TX_START 1
#define TX_RUNNING 2
#define RX_START 3
#define RX_RUNNING 4

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
void Write_RF_Settings(void);
void SD_Card_Setup(void);
