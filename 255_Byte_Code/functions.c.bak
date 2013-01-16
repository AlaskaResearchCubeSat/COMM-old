//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, April 2012

#include <functions.h>
#include <include.h>
#include "ARCbus.h"

int SUB_parseCmd(unsigned char src, unsigned char cmd, unsigned char *data, unsigned short len)
{
  switch(cmd)
  {
  }
  return ERR_UNKNOWN_CMD;
}

//Create string of data and of specified length and load into RxBuffer
void Build_Packet(int data_length)
{
int i;
TxBufferPos = 0;
TxBuffer[0] = data_length;                  // First byte is length of packet
for (i = 1; i < data_length; i++)      // Fill up buffer with data
    TxBuffer[i] = 0x0F;
TxBuffer[data_length - 1] = 0xF0;
}

/*void TI_CC_Wait(unsigned int cycles)
{
  /*while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
  __delay_cycles(16*cycles);
}*/

#define TI_CC_Wait(c) (__delay_cycles(16*c))

void radio_interrupts(void)
{
//Enable interrupts on Port 2
//2.0 -- GDO0  CC1101
//2.1 -- GDO2  CC1101
//2.3 -- GDO0  CC2500
//2.4 -- GDO2  CC2500
P2DIR &= ~(CC1101_GDO0 + CC1101_GDO2 + CC2500_GDO0 + CC2500_GDO2);
P2IE |= CC1101_GDO0 + CC1101_GDO2 + CC2500_GDO0 + CC2500_GDO2;             // Enable ints  
P2IES &= ~(CC1101_GDO0 + CC1101_GDO2 + CC2500_GDO0 + CC2500_GDO2);         // Int on rising edge: GDO0 interrupts when receives sync word
P2IFG = 0;                                                                 // Clear flags

//Set up amplifier switches, RF_SW1 for 430 MHz amplifier, RF_SW2 for 2.4 GHz amplifier
//P6DIR |= RF_SW1 + RF_SW2;                                                  // Set amplifier switches as outputs
//P6OUT |= (RF_SW1 + RF_SW2);                                               // Disable both amplifiers 
}



//Set up the UCB1 SPI channel for writing to the radios
void SPI_Setup(void)
{
  UCB1CTL1 = UCSWRST;                                          
  UCB1CTL0 = UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;
  UCB1CTL1 |= UCSSEL_2;
  UCB1BR0 = 16;      //Set frequency divider so SPI runs at 16/16 = 1 MHz
  UCB1BR1 = 0;

  //UC1IE |= UCB1TXIE + UCB1RXIE;

  P5SEL |= BIT1 + BIT2 + BIT3;               //Select special sources for Port 5.1--MOSI, 5.2--MISO, 5.3--CLK
  P5DIR |= BIT1 + BIT3;                      //Set outputs: MOSI and CLK
  P5DIR |= CS_1101;                          //Set output for CC1101 CS
  P5DIR |= CS_2500;                          //Set output for CC2500 CS

  P5REN |= BIT2;                         // Enable pull-up resistor for MISO
  P5OUT |= BIT2;                         // Set as pull-up

  P5OUT |= CS_1101;                     //Ensure CS for CC1101 is disabled
  P5OUT |= CS_2500;                     //Ensure CS for CC2500 is disabled 
  UCB1CTL1 &= ~UCSWRST;                 //Enable UART state machine
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

  P2IFG = 0;                                // Clear flags that were set
}

//Function that sends single address to radio initiating a state or mode change 
//(e.g. sending addr 0x34 writes to SRX register initiating radio in RX mode
void Radio_Strobe(char strobe, char uhf)
{
  if (uhf)
     P5OUT &= ~CS_1101;                           // CS enable CC1101

  else
     P5OUT &= ~CS_2500;                           // CS enable CC2500

  while (!(UC1IFG & UCB1TXIFG));                  // Wait for TXBUF ready

  UCB1TXBUF = strobe;                             // Send strobe
                                                  // Strobe addr is now being TX'ed
//  while ((UC1IFG & UCB1RXIFG) == 0);
  while (UCB1STAT & UCBUSY);                      // Wait for TX to complete

  P5OUT |= CS_2500;                               // CS disable C2500 
  P5OUT |= CS_1101;                               // CS disable C1101
  //P2IFG = 0;
}

//Function to write a single byte to the radio registers
void Radio_Write_Registers(char addr, char value, char uhf)
{
  if (uhf)
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
void Radio_Write_Burst_Registers(char addr, char *buffer, int count, char uhf)
{
  int i;

  if (uhf)
    P5OUT &= ~CS_1101;                            // CS enable CC1101

  else
    P5OUT &= ~CS_2500;                           // CS enable CC2500
  
  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = addr | TI_CCxxx0_WRITE_BURST;      // Adding 0x40 to address tells radio to perform burst write rather than single byte write
  for (i = 0; i < count; i++)
  {
    while (!(UC1IFG & UCB1TXIFG));              // Wait for TXBUF ready
    UCB1TXBUF = buffer[i];                      // Send data
  }
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete
  
  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 
}

//Function to read a single byte from the radio registers
char Radio_Read_Registers(char addr, char uhf)
{
  char x;
  
  if (uhf) {
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
void Radio_Read_Burst_Registers(char addr, char *buffer, int count, char uhf)
{
  char i;
  if (uhf)
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
  
  P5OUT |= CS_2500;                            // CS disable C2500 
  P5OUT |= CS_1101;
}

char Radio_Read_Status(char addr, char uhf)
{
  char status;

  if (uhf) {
     P5OUT &= ~CS_1101;                          // CS enable CC1101
           }

  else {
     P5OUT &= ~CS_2500;                          // CS enable CC2500
       }

  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);     // Send address
  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = 0;                                 // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                     // Wait for TX to complete
  status = UCB1RXBUF;                            // Read data
  P5OUT |= CS_1101;                              // CS disable C1101
  P5OUT |= CS_2500;                              // CS disable C2500 

  return status;
}

void TXRX(void *p) __toplevel 
{
    unsigned int e;
  short first=0;
while(1)
  {  
  e = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&radio_event_flags,RADIO_EVENTS,CTL_TIMEOUT_NONE,0);
  //printf("e = %i\r\n ", e);    
  if(e & CC1101_EV_RX_SYNC)
  {
      //RxFIFOLen = (Radio_Read_Status(TI_CCxxx0_RXBYTES, CC1101) & TI_CCxxx0_NUM_RXBYTES);
       //printf("RX Sync %i \r\n",RxFIFOLen);
      //if (RxFIFOLen > 0){
       // Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, RxFIFOLen, CC1101);
        //RxBufferPos += RxFIFOLen;
        //RxBytesRemaining -= RxFIFOLen;
      //}
      PktLen = 0;
      P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
      P2IES |= (BIT0);                                                            // Change edge interrupt happens on
      P2IFG &= ~BIT0;                                                                         // Clear flags
      P2IE |= BIT0 + BIT1 + BIT2 + BIT3;                                                 // Enable Port 2 interrupts
  }
  
  if(e & CC1101_EV_RX_THR)
  {
           if (PktLen == 0)
           {
           PktLen = Radio_Read_Registers(TI_CCxxx0_RXFIFO, CC1101);       // Read length byte
           RxBytesRemaining = PktLen;                                    // Set number of bytes left to receive
           first=1;
           }
           else
           {
            first=0;
           }
           if (RxBytesRemaining > 0)
           {
               if (RxBytesRemaining > RxThrBytes)
               {
                   
                   count = RxThrBytes;
                   RxBytesRemaining = RxBytesRemaining - RxThrBytes;
                   temp_count1++;
               }
               else 
               {   
                   count = RxBytesRemaining;
                   RxBytesRemaining = 0;
               }
                 if(first){
                   count--;
                 }
               Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, count, CC1101);
               RxBufferPos += count;
           }
 }
 
   if(e & CC1101_EV_RX_END)
   {
   RxFIFOLen = (Radio_Read_Status(TI_CCxxx0_RXBYTES, CC1101) & TI_CCxxx0_NUM_RXBYTES);
   if (RxFIFOLen > 0){
   Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, RxFIFOLen, CC1101);
   }
   P7OUT ^= BIT1;
   printf("Receiving packet on CC1101\r\n");
   printf("%d \r\n", PktLen);
   for (k=0; k < PktLen-1; k++)
   {
      printf("%03d ", RxBuffer[k]);
      if (k % 20 == 19)
       printf("\r\n");
   }        
   printf("\r\n");
   state = IDLE;
   P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
   P2IES &= ~(BIT0);                                                            // Change edge interrupt happens on
   P2IFG &= ~BIT0;                                                                         // Clear flags
   P2IE |= BIT0 + BIT1 + BIT2 + BIT3;                                                 // Enable Port 2 interrupts
   }
 
 
   if(e & CC1101_EV_TX_START)
   {
          state = TX_START;
          TxBufferPos = 0;
          P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
          Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x02, CC1101);                             // Set GDO2 to interrupt on FIFO thresholds
          P2IES |= (BIT0 + BIT1 + BIT2 + BIT3);                                              // Change edge interrupt happens on
          P2IFG = 0;                                                                         // Clear flags
          P2IE |= BIT0 + BIT1 + BIT2 + BIT3;                                                 // Enable Port 2 interrupts
          //Build_Packet(data_length);
          if (TxBytesRemaining > 64)
          {
              count = 64;
              TxBytesRemaining = TxBytesRemaining - 64;
          }
          else
          {
              count = TxBytesRemaining;
              TxBytesRemaining = 0;
          }
          
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer+TxBufferPos, count, CC1101);     // Write TX data
          TxBufferPos += count;
          Radio_Strobe(TI_CCxxx0_STX, CC1101);                                                  // Set radio state to Tx 
   }
 
 if(e & CC1101_EV_TX_THR)
 {
          if (TxBytesRemaining > 0) 
          {
             if (TxBytesRemaining > TxThrBytes)
             {
                 count = TxThrBytes;
                 TxBytesRemaining = TxBytesRemaining - TxThrBytes;
             }
             else
             {
                 count = TxBytesRemaining;
                 TxBytesRemaining = 0;
             }
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer + TxBufferPos, count, CC1101);
             TxBufferPos += count;
             //printf("State: %x ", Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
          }
          if (TxBytesRemaining == 0)
          {
                  //P7OUT ^= BIT0;
                  //printf("%d", temp_count1);
                  //printf(" packet(s) sent \r\n");
                  //printf("CC1101 \r\n");
//                  printf("packet sent \r\n");
//                  for (k=0; k < TxBufferPos; k++)
//                 {
//                     printf("%d ", TxBuffer[k]);
//                     printf(" ");
//                 }
              
          }
 }
          
  if(e & CC1101_EV_TX_END)
  {
   // Increment packet counter         
   temp_count1++;

   // Toggle LED to indicate packet sent
   P7OUT ^= BIT0;
   
   // Print out useful information
   printf("%x packets sent\r\n", temp_count1);
   printf("Radio State: %x \n\r", Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
            
            
   P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
   Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x00, CC1101);                             // Set GDO2 to interrupt on FIFO thresholds
   P2IES &= ~(BIT0 + BIT1);                                                           // Change edge interrupt happens on
   P2IFG = 0;                                                                         // Clear flags        
   P2IE |= BIT0 + BIT1 + BIT2 + BIT3;           
   
   
   // Check for TX FIFO underflow, if yes then flush dat buffer
   if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x16)
   {
      Radio_Strobe(TI_CCxxx0_SFTX,CC1101);
      Radio_Strobe(TI_CCxxx0_SRX,CC1101);
      __delay_cycles(16000);
      printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101)); 
   }
  
            
  }
          
  
  if(e & CC2500_EV_RX_SYNC)
  {
      RxFIFOLen = (Radio_Read_Status(TI_CCxxx0_RXBYTES, CC2500) & TI_CCxxx0_NUM_RXBYTES);
      PktLen = 255; //Radio_Read_Registers(TI_CCxxx0_RXFIFO, CC1101);       // Read length byte
      RxBytesRemaining = PktLen;                                  // Set number of bytes left to receive
      Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, RxFIFOLen, CC2500);
      RxBufferPos += RxFIFOLen;
      RxBytesRemaining -= RxFIFOLen;
  }
  
  if(e & CC2500_EV_RX_THR)
  {
          if (RxBytesRemaining > 0)
           {
               if (RxBytesRemaining > RxThrBytes)
               {
                   count = RxThrBytes;
                   RxBytesRemaining = RxBytesRemaining - RxThrBytes;
                   temp_count1++;
               }
               else 
               {
                   count = RxBytesRemaining;
                   RxBytesRemaining = 0;
               }
               Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, count, CC2500);
               RxBufferPos += count;
           }
           if (RxBytesRemaining == 0)
           {

                     state = 0;
                     P7OUT ^= BIT5;
                     printf("receiving packet\r\n");
                     printf("\r\n");
                     for (k=0; k < PktLen; k++)
                     {
                         printf("%d ", RxBuffer[k]);
                         printf("\r\n");
                     }
           }
 }
 
   if(e & CC2500_EV_TX_START)
 {
          P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
          Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x02, CC2500);                                // Set GDO2 to interrupt on FIFO thresholds
          P2IES |= (BIT0 + BIT1 + BIT2 + BIT3);                                              // Change edge interrupt happens on
          P2IFG = 0;                                                                         // Clear flags
          P2IE |= BIT0 + BIT1 + BIT2 + BIT3;                                                 // Enable Port 2 interrupts
          Build_Packet(data_length);
          if (TxBytesRemaining > 64)
          {
              count = 64;
              TxBytesRemaining = TxBytesRemaining - 64;
          }
          else
          {
              count = TxBytesRemaining;
              TxBytesRemaining = 0;
          }
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer+TxBufferPos, count, CC2500);     // Write TX data
          TxBufferPos += count;
          Radio_Strobe(TI_CCxxx0_STX, CC2500);                                                  // Set radio state to Tx 
}
 
 if(e & CC2500_EV_TX_THR)
 {
          if (TxBytesRemaining > 0) 
          {
             if (TxBytesRemaining > 30)
             {
                 count = 30;
                 TxBytesRemaining = TxBytesRemaining - 30;
             }
             else
             {
                 count = TxBytesRemaining;
                 TxBytesRemaining = 0;
             }
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer + TxBufferPos, count, CC2500);
             TxBufferPos += count;
          }
          if (TxBytesRemaining == 0)
          {
              state = 0;
              P7OUT ^= BIT4;
              temp_count1++;
              printf("%d", temp_count1);
              printf(" packet(s) sent \r\n");
              printf("CC2500 \r\n");
//                  printf("packet sent \r\n");
//                  for (k=0; k < TxBufferPos; k++)
//                 {
//                     printf("%d ", TxBuffer[k]);
//                     printf(" ");
//                 } 

              P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
              Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x00, CC2500);                                // Set GDO2 to interrupt on FIFO thresholds
              P2IES &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Change edge interrupt happens on
              P2IFG = 0;                                                                         // Clear flags
              P2IE |= BIT0 + BIT1 + BIT2 + BIT3; 
          }
 }
  }
}

void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i;
  unsigned char buf[10],*ptr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL,CTL_TIMEOUT_NONE,0);
    if(e&SUB_EV_PWR_OFF){
      //print message
      puts("System Powering Down\r");
    }
    if(e&SUB_EV_PWR_ON){
      //print message
      puts("System Powering Up\r");
    }
    if(e&SUB_EV_SEND_STAT){
      //send status
      //puts("Sending status\r");
      //setup packet 
      //TODO: put actual command for subsystem response
      ptr=BUS_cmd_init(buf,20);
      //TODO: fill in telemitry data
      //send command
      BUS_cmd_tx(BUS_ADDR_CDH,buf,0,0,SEND_FOREGROUND);
    }
    if(e&SUB_EV_TIME_CHECK){
      printf("time ticker = %li\r\n",get_ticker_time());
    }
    if(e&SUB_EV_SPI_DAT){
      puts("SPI data recived:\r");
      //get length
      len=arcBus_stat.spi_stat.len;
      //print out data
      for(i=0;i<len;i++){
        //printf("0x%02X ",rx[i]);
        printf("%03i ",arcBus_stat.spi_stat.rx[i]);
      }
      printf("\r\n");
     
    TxBuffer[0] = len; 
     for (i=0; i < len; i++)
      {
        TxBuffer[i+1] = arcBus_stat.spi_stat.rx[i];
      }
      TxBytesRemaining = len+1;
      ctl_events_set_clear(&radio_event_flags,CC1101_EV_TX_START,0);
     }
    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
    }
  }
}

void Write_RF_Settings(void)
{
// CC1101
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
Radio_Write_Registers(TI_CCxxx0_IOCFG0,   0x06, 1);   // Asserts when sync word has been sent / received, and de-asserts at the end of the packet. 
Radio_Write_Registers(TI_CCxxx0_IOCFG2,   0x00, 1);   // Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold.
Radio_Write_Registers(TI_CCxxx0_FIFOTHR,  0x07, 1);   // FIFO Threshold
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,  0x00, 1);
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,  0x06, 1);
Radio_Write_Registers(TI_CCxxx0_FREQ2,    0x10, 1);
Radio_Write_Registers(TI_CCxxx0_FREQ1,    0xC4, 1);
Radio_Write_Registers(TI_CCxxx0_FREQ0,    0xEC, 1);
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,  0xF8, 1);
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,  0x83, 1);
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,  0x03, 1); // High byte: 0000 is 2-FSK and 0001 is GFSK; Low byte: 0000 no preamble/sync, 0001 15/16 sync words detected, 0011 30/32 sync words
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,  0x22, 1); // High byte: 0010 is 4 bytes of preamble, 0100 is 8 bytes of preamble
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,  0xF8, 1);
Radio_Write_Registers(TI_CCxxx0_CHANNR,   0x00, 1);
Radio_Write_Registers(TI_CCxxx0_DEVIATN,  0x15, 1);
Radio_Write_Registers(TI_CCxxx0_FREND1,   0x56, 1);
Radio_Write_Registers(TI_CCxxx0_FREND0,   0x10, 1);
Radio_Write_Registers(TI_CCxxx0_MCSM0,    0x18, 1);
Radio_Write_Registers(TI_CCxxx0_MCSM1,    0x0F, 1);
Radio_Write_Registers(TI_CCxxx0_FOCCFG,   0x16, 1);
Radio_Write_Registers(TI_CCxxx0_BSCFG,    0x6C, 1);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2, 0x03, 1);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1, 0x40, 1);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0, 0x91, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL3,   0xE9, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL2,   0x2A, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL1,   0x00, 1);
Radio_Write_Registers(TI_CCxxx0_FSCAL0,   0x1F, 1);
Radio_Write_Registers(TI_CCxxx0_FSTEST,   0x59, 1);
Radio_Write_Registers(TI_CCxxx0_TEST2,    0x81, 1);
Radio_Write_Registers(TI_CCxxx0_TEST1,    0x35, 1);
Radio_Write_Registers(TI_CCxxx0_TEST0,    0x09, 1);
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1, 0x00, 1);     
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x01, 1);     // Variable packet length mode; packet length set by first byte after sync word
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x00, 1);
Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0xFF, 1);      // Packet length set for variable packet length mode


// Write CC2500 register settings
Radio_Write_Registers(TI_CCxxx0_IOCFG0,   0x06, 0);  // GDO0 output pin config.
Radio_Write_Registers(TI_CCxxx0_IOCFG2,   0x00, 0);  // GDO2 output pin config.
Radio_Write_Registers(TI_CCxxx0_FIFOTHR,  0x0F, 0);  // FIFO Threshold: 1 byte in TX FIFO and 63 in RX FIFO
Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0x08, 0);  // Packet length.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1, 0x04, 0);  // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x05, 0);  // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x01, 0);  // Device address.
Radio_Write_Registers(TI_CCxxx0_CHANNR,   0x00, 0);  // Channel number.
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,  0x07, 0);  // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,  0x00, 0);  // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FREQ2,    0x5D, 0);  // Freq control word, high byte
Radio_Write_Registers(TI_CCxxx0_FREQ1,    0x44, 0);  // Freq control word, mid byte.
Radio_Write_Registers(TI_CCxxx0_FREQ0,    0xEC, 0);  // Freq control word, low byte.
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,  0x2D, 0);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,  0x3B, 0);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,  0x73, 0);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,  0x22, 0);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,  0xF8, 0);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_DEVIATN,  0x00, 0);  // Modem dev (when FSK mod en)
Radio_Write_Registers(TI_CCxxx0_MCSM1 ,   0x3F, 0);  // Main Radio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_MCSM0 ,   0x18, 0);  // Main Radio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_FOCCFG,   0x1D, 0);  // Freq Offset Compens. Config
Radio_Write_Registers(TI_CCxxx0_BSCFG,    0x1C, 0);  //  Bit synchronization config.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2, 0xC7, 0);  // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1, 0x00, 0);  // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0, 0xB2, 0);  // AGC control.
Radio_Write_Registers(TI_CCxxx0_FREND1,   0xB6, 0);  // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FREND0,   0x10, 0);  // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FSCAL3,   0xEA, 0);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL2,   0x0A, 0);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL1,   0x00, 0);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL0,   0x11, 0);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSTEST,   0x59, 0);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_TEST2,    0x88, 0);  // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST1,    0x31, 0);  // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST0,    0x0B, 0);  // Various test settings.    
}
