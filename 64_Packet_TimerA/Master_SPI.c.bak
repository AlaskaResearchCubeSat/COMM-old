//EE 645: Embedded Systems 
//Class Project--Cubesat Communication System:  RF and Interboard COmmunication
//Functions for setting up SPI for inter-board communication with the Image System (as slave)

#include <Master_SPI.h>
#include <include.h>

#define CS_1101 BIT4
#define CS_2500 BIT5

void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}

//void Image_SPI_Setup(void)
//{
//  UCA3CTL1 =  UCSWRST;                                          //Disable UART state machine
//  UCA3CTL0 = UCMSB + UCMST + UCMODE_0 + UCSYNC;                 //SPI Master, 3-pin, 8-bit, MSB, synchronous mode
//  UCA3CTL1 |= UCSSEL_2;                                         //Selects SMCLK as source for the USCI BRCLK
//  UCA3BR0= 16;
//  UCA3BR1 = 0;                                                  //Prescaler divider for Bitclock; currently set to 1
//
//  P10SEL |= BIT0 + BIT4 + BIT5;                                 //Select special sources for Port 10: 10.0--CLK; 10.4--MOSI; 10.5--MISO;  Leave  10.3 as GPIO for STE
//  P10DIR |= BIT0 + BIT3 + BIT4;                                 //Set clock, MOSI and CS as outputs
//  P10OUT |= BIT3;                                               //Ensure CS is high (disabled)
//  P1OUT &= ~BIT1;                                               //Turn LED2 off
//  UCA3CTL1 &=  ~UCSWRST;                                        //Enable UART state machine
//  UCA3IE |= UCTXIE + UCRXIE;                                    //Enable Transmit and Receive interrupts
//  UCA3IFG &= ~(UCB0RXIFG);
//}

void Radio_SPI_Setup(void)
{
  UCB1CTL1 = UCSWRST;                                          
  UCB1CTL0 = UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;
  UCB1CTL1 |= UCSSEL_2;
  UCB1BR0 = 16;      //Set frequency divider so SPI runs at 16/16 = 1 MHz
  UCB1BR1 = 0;

  UC1IE |= UCB1TXIE + UCB1RXIE;

  P5SEL |= BIT1 + BIT2 + BIT3;          //Select special sources for Port 5.1--MOSI, 5.2--MISO, 5.3--CLK
  P5DIR |= BIT1 + BIT3;                 //Set outputs: MOSI and CLK
  P5DIR |= CS_1101;                     //Set output for CC1101 CS
  P5DIR |= CS_2500;                     //Set output for CC2500 CS

  P5OUT |= CS_1101;                     //Ensure CS for CC1101 is disabled
  P5OUT |= CS_2500;                     //Ensure CS for CC2500 is disabled 
  UCB1CTL1 &= ~UCSWRST;                 //Enable UART state machine
}

//Sends Image Capture command: "IC" to Image System
//void Take_Picture_Command(void)
//{
//  while (!(UCA3IFG & UCTXIFG));              // wait for TXBUF to be ready
//  P10OUT &= ~BIT3;                              //Enable CS
//  UCA3TXBUF = 'I';                         //Send character "I"
//  count_flag = 0;
//}

//Function to write a single byte to the radio registers
void Radio_Write_Registers(char addr, char value, char radio)
{
  if (radio)
    P5OUT &= ~CS_1101;                          // CS enable CC1101

  else
    P5OUT &= ~CS_2500;                          // CS enable CC2500

  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = addr;                         // Send address
  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = value;                        // Send data
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete

  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 
  
}

//Function to write multiple bytes to the radio registers
void Radio_Write_Burst_Registers(char addr, char *buffer, char size, char radio)
{
  unsigned int i;

  if (radio)
    P5OUT &= ~CS_1101;                          // CS enable CC1101

  else
    P5OUT &= ~CS_2500;                            // CS enable CC2500
  
  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
 UCB1TXBUF = addr | TI_CCxxx0_WRITE_BURST;     // Adding 0x40 to address tells radio to perform burst write rather than single byte write
  for (i = 0; i < size; i++)
  {
    while (!(UC1IFG & UCB1TXIFG));              // Wait for TXBUF ready
   UCB1TXBUF = buffer[i];                     // Send data
  }
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete
  
  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 
}

//Function to read a single byte from the radio registers
char Radio_Read_Registers(char addr, char radio)
{
  char x;
  
  if (radio) {
     P5OUT &= ~CS_1101;                          // CS enable CC1101
           }

  else {
     P5OUT &= ~CS_2500;                          // CS enable CC2500
       }

  while (!(UC1IFG & UCB1TXIFG));                  // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_SINGLE);  // Adding 0x80 to address tells the radio to read a single byte
  while (!(UC1IFG & UCB1TXIFG));                  // Wait for TXBUF ready
  UCB1TXBUF = 0;                               // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                   // Wait for TX to complete
  x = UCB1RXBUF;                               // Read data
  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 

  return x;
}

//Function to read a multiple bytes from the radio registers
void Radio_Read_Burst_Registers(char addr, char *buffer, char count, char radio)
{
  char i;
  if (radio)
     P5OUT &= ~CS_1101;                          // CS enable CC1101

  else
     P5OUT &= ~CS_2500;

  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Adding 0xC0 to address tells the radio to read multiple bytes
  while (UCB1STAT & UCBUSY);                  // Wait for TX to complete
  UCB1TXBUF = 0;                              // Dummy write to read 1st data byte
                                              // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  UC1IFG &= ~UCB1RXIFG;                        // Clear flag
  while (!(UC1IFG & UCB1RXIFG));                 // Wait for end of 1st data byte TX
                                              // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
   UCB1TXBUF = 0;                            //Initiate next data RX, meanwhile..
    buffer[i] = UCB1RXBUF;                    // Store data from last data RX
    while (!(UC1IFG & UCB1RXIFG));               // Wait for RX to finish
  }
  buffer[count-1] = UCB1RXBUF;                // Store last RX byte in buffer
  
  P5OUT |= CS_2500;                            // CS disable radios
  P5OUT |= CS_1101;
}

char Radio_Read_Status(char addr, char radio)
{
  char status;

  if (radio) {
     P5OUT &= ~CS_1101;                          // CS enable CC1101
           }

  else {
     P5OUT &= ~CS_2500;                          // CS enable CC2500
       }

  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
 UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
 UCB1TXBUF = 0;                              // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                  // Wait for TX to complete
  status = UCB1RXBUF;                         // Read data
  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 

  return status;
}

//Function that sends single address to radio initiating a state or mode change 
//(e.g. sending addr 0x34 writes to SRX register initiating radio in RX mode
void Radio_Strobe(char strobe, char radio)
{
  if (radio)
     P5OUT &= ~CS_1101;                           // CS enable CC1101

  
  else
     P5OUT &= ~CS_2500;                           // CS enable CC2500

  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
 UCB1TXBUF = strobe;                       // Send strobe
                                             // Strobe addr is now being TX'ed
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete


  P5OUT |= CS_2500;                            // CS disable C2500 
  P5OUT |= CS_1101;                            // CS disable C1101
}

void Reset_Radios(void)
{
  P5OUT |= CS_1101;                   //Toggle CS with delays to power up radio
  TI_CC_Wait(30);
  P5OUT &= ~CS_1101;
  TI_CC_Wait(30);
  P5OUT |= CS_1101;
  TI_CC_Wait(45);

  P5OUT &= ~CS_1101;                           // CS enable
  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = TI_CCxxx0_SRES;               // Send strobe
                                            // Strobe addr is now being TX'ed
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete
  P5OUT |= CS_1101;                           // CS disable
  
  P5OUT |= CS_2500;                   //Toggle CS with delays to power up radio
  TI_CC_Wait(30);
  P5OUT &= ~CS_2500;
  TI_CC_Wait(30);
  P5OUT |= CS_2500;
  TI_CC_Wait(45);

  P5OUT &= ~CS_2500;                           // CS enable
  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = TI_CCxxx0_SRES;               // Send strobe
                                            // Strobe addr is now being TX'ed
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete
  P5OUT |= CS_2500;                           // CS disable
}

void Write_RF_Settings(void)
{

// Deviation = 5.157471 
// Base frequency = 432.999817 
// Carrier frequency = 432.999817 
// Channel number = 0 
// Carrier frequency = 432.999817 
// Modulated = true 
// Modulation format = GFSK 
// Manchester enable = false 
// Sync word qualifier mode = 30/32 sync word bits detected 
// Preamble count = 4 
// Channel spacing = 199.951172 kHz
// Carrier frequency = 432.999817 MHz
// Data rate = 9.59587 kbps
// RX filter BW = 58.035714 
// Data format = Normal mode 
// Length config = Variable packet length mode. Packet length configured by the first byte after sync word 
// CRC enable = true 
// Packet length = 255 
// Device address = 0 
// Address config = No address check 
// CRC autoflush = false 
// PA ramping = false 
// TX power = 0
Radio_Write_Registers(TI_CCxxx0_IOCFG0,    0x29, 1);
Radio_Write_Registers(TI_CCxxx0_IOCFG2,    0x29, 1);
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1,  0x04, 1);      //Status bytes appended (RSSI and LQI values, CRC ok) but no address check
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0,  0x04, 1);     //  0x05 = Variable packet length mode; packet length set by first byte after sync word
Radio_Write_Registers(TI_CCxxx0_PKTLEN,    0x40, 1);      //Packet length set to 64
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,   0x06, 1);
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,   0x00, 1);
Radio_Write_Registers(TI_CCxxx0_FREQ2,     0x10, 1);      // 0x10, 0xC4, 0xEC is 436.000 MHz
Radio_Write_Registers(TI_CCxxx0_FREQ1,     0xC4, 1);
Radio_Write_Registers(TI_CCxxx0_FREQ0,     0xEC, 1);
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,   0xF8, 1);      // 0xF8 = 9600 bps; 0xFA = 38400 bps
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,   0x83, 1);
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,   0x03, 1);      // High byte: 0000 is 2-FSK and 0001 is GFSK; Low byte: 0000 no preamble/sync, 0001 15/16 sync words detected, 0011 30/32 sync words
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,   0x22, 1);      // High byte: 0010 is 4 bytes of preamble, 0100 is 8 bytes of preamble
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,   0xF8, 1);
Radio_Write_Registers(TI_CCxxx0_CHANNR,    0x00, 1);
Radio_Write_Registers(TI_CCxxx0_DEVIATN,   0x15, 1);
Radio_Write_Registers(TI_CCxxx0_FREND1,    0x56, 1);
Radio_Write_Registers(TI_CCxxx0_FREND0,    0x10, 1);
Radio_Write_Registers(TI_CCxxx0_MCSM1,     0x0F, 1);
Radio_Write_Registers(TI_CCxxx0_MCSM0,     0x18, 1);
Radio_Write_Registers(TI_CCxxx0_FOCCFG,    0x16, 1);
Radio_Write_Registers(TI_CCxxx0_BSCFG,     0x6C, 1);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2,  0x03, 1);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1,  0x40, 1);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0,  0x91, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL3,    0xE9, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL2,    0x2A, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL1,    0x00, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL0,    0x1F, 1);
Radio_Write_Registers(TI_CCxxx0_FSTEST,    0x59, 1);
Radio_Write_Registers(TI_CCxxx0_TEST2,     0x81, 1);
Radio_Write_Registers(TI_CCxxx0_TEST1,     0x35, 1);
Radio_Write_Registers(TI_CCxxx0_TEST0,     0x09, 1);
Radio_Write_Registers(TI_CCxxx0_FIFOTHR,   0x07, 1);
Radio_Write_Registers(TI_CCxxx0_ADDR,      0x00, 1);


// Write CC2500 register settings
Radio_Write_Registers(TI_CCxxx0_IOCFG0,    0x06, 0);  // GDO0 output pin config.
Radio_Write_Registers(TI_CCxxx0_IOCFG2,    0x06, 0);  // GDO2 output pin config.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1,  0x04, 0);  // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0,  0x05, 0);  // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_PKTLEN,    0x3D, 0);  // Packet length.
Radio_Write_Registers(TI_CCxxx0_ADDR,      0x01, 0);  // Device address.
Radio_Write_Registers(TI_CCxxx0_CHANNR,    0x00, 0); // Channel number.
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,   0x07, 0); // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,   0x00, 0); // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FREQ2,     0x5D, 0); // Freq control word, high byte
Radio_Write_Registers(TI_CCxxx0_FREQ1,     0x44, 0); // Freq control word, mid byte.
Radio_Write_Registers(TI_CCxxx0_FREQ0,     0xEC, 0); // Freq control word, low byte.
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,   0x28, 0); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,   0x3B, 0); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,   0x73, 0); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,   0x22, 0); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,   0xF8, 0); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_DEVIATN,   0x00, 0); // Modem dev (when FSK mod en)
Radio_Write_Registers(TI_CCxxx0_MCSM1 ,    0x3F, 0); //MainRadio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_MCSM0 ,    0x18, 0); //MainRadio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_FOCCFG,    0x1D, 0); // Freq Offset Compens. Config
Radio_Write_Registers(TI_CCxxx0_BSCFG,     0x1C, 0); //  Bit synchronization config.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2,  0xC7, 0); // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1,  0x00, 0); // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0,  0xB2, 0); // AGC control.
Radio_Write_Registers(TI_CCxxx0_FREND1,    0xB6, 0); // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FREND0,    0x10, 0); // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FSCAL3,    0xEA, 0); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL2,    0x0A, 0); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL1,    0x00, 0); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL0,    0x11, 0); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSTEST,    0x59, 0); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_TEST2,     0x88, 0); // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST1,     0x31, 0); // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST0,     0x0B, 0);  // Various test settings.    
}

char RF_Receive_Packet(char *rxBuffer, char *length, char radio)
{
  char status[2];
  char pktLen;

  Radio_Strobe(TI_CCxxx0_SRX, radio);          //Ensure state is Rx

  if ((Radio_Read_Status(TI_CCxxx0_RXBYTES, radio) & TI_CCxxx0_NUM_RXBYTES))
  {
    pktLen = Radio_Read_Registers(TI_CCxxx0_RXFIFO, radio); // Read length byte

    if (pktLen <= *length)                  // If pktLen size <= rxBuffer
    {
      Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, rxBuffer, pktLen, radio); // Pull data
      *length = pktLen;                     // Return the actual size
      Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, status, 2, radio);
                                            // Read appended status bytes
      return (char)(status[TI_CCxxx0_LQI_RX] & TI_CCxxx0_CRC_OK);
    }                                       // Return CRC_OK bit
    else
    {
      *length = pktLen;                     // Return the large size
      Radio_Strobe(TI_CCxxx0_SFRX, radio);    // Flush RXFIFO
      return 0;                             // Error
    }
  }
  else
      return 0;                             // Error
}

void RF_Send_Packet(char *txBuffer, char size, char radio)
{
Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, txBuffer, size, radio); // Write TX data
Radio_Strobe(TI_CCxxx0_STX, radio);                                   // Change state to TX, initiating data transfer

tx_flag = 1;
                                                                 
}


