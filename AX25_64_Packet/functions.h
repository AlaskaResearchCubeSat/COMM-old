//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Interboard COmmunication
//Header file for Master_SPI.c

char RF_Receive_Packet(char *, char *, char);
void RF_Send_Packet(char *, char, char);
void Image_SPI_Setup(void);
void Radio_SPI_Setup(void);
void Take_Picture_command(void);
void Receive_Data(void);
void Transmit_Data(void);
void Reset_Radios(void);
void Radio_Write_Registers(char,char, char);
void Radio_Write_Burst_Registers(char, char *, char, char);
void TI_CC_Wait(unsigned int);
void Radio_Write_Power_Settings(char, char *, char, char);
char Radio_Read_Registers(char, char);
void Radio_Read_Burst_Registers(char, char *, char, char);
char Radio_Read_Status(char, char);
void Radio_Strobe(char, char);
void Write_RF_Settings(void);
