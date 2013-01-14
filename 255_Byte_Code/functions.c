<<<<<<< HEAD
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
while(1)
  {  
  e = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&radio_event_flags,RADIO_EVENTS,CTL_TIMEOUT_NONE,0);
      
  if(e & CC1101_EV_RX_SYNC)
  {
      //RxFIFOLen = (Radio_Read_Status(TI_CCxxx0_RXBYTES, CC1101) & TI_CCxxx0_NUM_RXBYTES);
       //printf("RX Sync %i \r\n",RxFIFOLen);
      PktLen = 255; //Radio_Read_Registers(TI_CCxxx0_RXFIFO, CC1101);       // Read length byte
      RxBytesRemaining = PktLen;    // Set number of bytes left to receive
      //if (RxFIFOLen > 0){
       // Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, RxFIFOLen, CC1101);
        //RxBufferPos += RxFIFOLen;
        //RxBytesRemaining -= RxFIFOLen;
      //}
      P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
      P2IES |= (BIT0);                                                            // Change edge interrupt happens on
      P2IFG &= ~BIT0;                                                                         // Clear flags
      P2IE |= BIT0 + BIT1 + BIT2 + BIT3;                                                 // Enable Port 2 interrupts
  }
  
  if(e & CC1101_EV_RX_THR)
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
               Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, count, CC1101);
               RxBufferPos += count;
           }
           if (RxBytesRemaining == 0)
           {
                     state = 0;
                     P7OUT ^= BIT1;
                     printf("receiving packet\r\n");
                     printf("\r\n");
                     for (k=0; k < PktLen; k++)
                     {
                         printf("%d ", RxBuffer[k]);
                      }
                     printf("\r\n");

           }
 }
 
   if(e & CC1101_EV_RX_END)
   {
   RxFIFOLen = (Radio_Read_Status(TI_CCxxx0_RXBYTES, CC1101) & TI_CCxxx0_NUM_RXBYTES);
   Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, RxFIFOLen, CC1101);
   P7OUT ^= BIT1;
   printf("Receiving packet on CC1101\r\n");
   printf("\r\n");
   for (k=0; k < PktLen; k++)
   {
      printf("%3d ", RxBuffer[k]);
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
          P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
          Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x02, CC1101);                                // Set GDO2 to interrupt on FIFO thresholds
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
             printf("State: %x ", Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
          }
          if (TxBytesRemaining == 0)
          {
              state = 0;

                  P7OUT ^= BIT0;
                  temp_count1++;
                  printf("%d", temp_count1);
                  printf(" packet(s) sent \r\n");
                  printf("CC1101 \r\n");
//                  printf("packet sent \r\n");
//                  for (k=0; k < TxBufferPos; k++)
//                 {
//                     printf("%d ", TxBuffer[k]);
//                     printf(" ");
//                 }
              
              P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
              Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x00, CC1101);                                // Set GDO2 to interrupt on FIFO thresholds
              P2IES &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Change edge interrupt happens on
              P2IFG = 0;                                                                         // Clear flags
              P2IE |= BIT0 + BIT1 + BIT2 + BIT3; 
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
     
    TxBuffer[0] = len+1; 
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
Radio_Write_Registers(TI_CCxxx0_MCSM1,    0x3F, 1);
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
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1, 0x04, 1);     //Status bytes appended (RSSI and LQI values, CRC ok) but no address check
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x05, 1);     // Variable packet length mode; packet length set by first byte after sync word
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x00, 1);
Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0xFF, 1);      // Packet length set for variable packet length mode (PKTCTL0 = 0x05) so this doesn't matter


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
=======
  


<!DOCTYPE html>
<html>
  <head prefix="og: http://ogp.me/ns# fb: http://ogp.me/ns/fb# githubog: http://ogp.me/ns/fb/githubog#">
    <meta charset='utf-8'>
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
        <title>COMM/255_Byte_Code/functions.c at 255_Byte_ARCLib · AlaskaResearchCubeSat/COMM</title>
    <link rel="search" type="application/opensearchdescription+xml" href="/opensearch.xml" title="GitHub" />
    <link rel="fluid-icon" href="https://github.com/fluidicon.png" title="GitHub" />
    <link rel="apple-touch-icon-precomposed" sizes="57x57" href="apple-touch-icon-114.png" />
    <link rel="apple-touch-icon-precomposed" sizes="114x114" href="apple-touch-icon-114.png" />
    <link rel="apple-touch-icon-precomposed" sizes="72x72" href="apple-touch-icon-144.png" />
    <link rel="apple-touch-icon-precomposed" sizes="144x144" href="apple-touch-icon-144.png" />
    <meta name="msapplication-TileImage" content="/windows-tile.png">
    <meta name="msapplication-TileColor" content="#ffffff">

    
    
    <link rel="icon" type="image/x-icon" href="/favicon.ico" />

    <meta content="authenticity_token" name="csrf-param" />
<meta content="hOuPGEWMiPJUnlawwHqnPgCvULlSBU6jJDwl1DAgerw=" name="csrf-token" />

    <link href="https://a248.e.akamai.net/assets.github.com/assets/github-7ef703df15129d3b898830277d50fc760ca91cbc.css" media="screen" rel="stylesheet" type="text/css" />
    <link href="https://a248.e.akamai.net/assets.github.com/assets/github2-204e28c07493b8ba6089c49791ed7ab61ecb6581.css" media="screen" rel="stylesheet" type="text/css" />
    


      <script src="https://a248.e.akamai.net/assets.github.com/assets/frameworks-cc4895cbb610429d2ce48e7c2392822c33db2dfe.js" type="text/javascript"></script>
      <script src="https://a248.e.akamai.net/assets.github.com/assets/github-e539dcf1e3c93f4acda64d064d3f30a39afabae0.js" type="text/javascript"></script>
      

        <link rel='permalink' href='/AlaskaResearchCubeSat/COMM/blob/d0809d2d15872b0d16694ca4b54203bb88c1a887/255_Byte_Code/functions.c'>
    <meta property="og:title" content="COMM"/>
    <meta property="og:type" content="githubog:gitrepository"/>
    <meta property="og:url" content="https://github.com/AlaskaResearchCubeSat/COMM"/>
    <meta property="og:image" content="https://secure.gravatar.com/avatar/1e490c637e8db2cc4a5dc939d72a459e?s=420&amp;d=https://a248.e.akamai.net/assets.github.com%2Fimages%2Fgravatars%2Fgravatar-user-420.png"/>
    <meta property="og:site_name" content="GitHub"/>
    <meta property="og:description" content="Communications System. Contribute to COMM development by creating an account on GitHub."/>

    <meta name="description" content="Communications System. Contribute to COMM development by creating an account on GitHub." />

  <link href="https://github.com/AlaskaResearchCubeSat/COMM/commits/255_Byte_ARCLib.atom" rel="alternate" title="Recent Commits to COMM:255_Byte_ARCLib" type="application/atom+xml" />

  </head>


  <body class="logged_in page-blob windows vis-public env-production ">
    <div id="wrapper">

      

      

      


        <div class="header header-logged-in true">
          <div class="container clearfix">

            <a class="header-logo-blacktocat" href="https://github.com/organizations/AlaskaResearchCubeSat">
  <span class="mega-icon mega-icon-blacktocat"></span>
</a>

            <div class="divider-vertical"></div>

            
  <a href="/notifications" class="notification-indicator tooltipped downwards" title="You have unread notifications">
    <span class="mail-status unread"></span>
  </a>
  <div class="divider-vertical"></div>


              
  <div class="topsearch command-bar-activated">
    <form accept-charset="UTF-8" action="/search" class="command_bar_form" id="top_search_form" method="get">
  <a href="/search/advanced" class="advanced-search tooltipped downwards command-bar-search" id="advanced_search" title="Advanced search"><span class="mini-icon mini-icon-advanced-search "></span></a>

  <input type="text" name="q" id="command-bar" placeholder="Search or type a command" tabindex="1" data-username="samuelvanderwaal" autocapitalize="off">

  <span class="mini-icon help tooltipped downwards" title="Show command bar help">
    <span class="mini-icon mini-icon-help"></span>
  </span>

  <input type="hidden" name="ref" value="commandbar">

  <div class="divider-vertical"></div>
</form>

    <ul class="top-nav">
        <li class="explore"><a href="https://github.com/explore">Explore</a></li>
        <li><a href="https://gist.github.com">Gist</a></li>
        <li><a href="/blog">Blog</a></li>
      <li><a href="http://help.github.com">Help</a></li>
    </ul>
  </div>


            

  
    <ul id="user-links">
      <li>
        <a href="https://github.com/samuelvanderwaal" class="name">
          <img height="20" src="https://secure.gravatar.com/avatar/cb06419b31efe234e765209f3d3b1230?s=140&amp;d=https://a248.e.akamai.net/assets.github.com%2Fimages%2Fgravatars%2Fgravatar-user-420.png" width="20" /> samuelvanderwaal
        </a>
      </li>
      <li>
        <a href="/new" id="new_repo" class="tooltipped downwards" title="Create a new repo">
          <span class="mini-icon mini-icon-create"></span>
        </a>
      </li>
      <li>
        <a href="/settings/profile" id="account_settings"
          class="tooltipped downwards"
          title="Account settings ">
          <span class="mini-icon mini-icon-account-settings"></span>
        </a>
      </li>
      <li>
          <a href="/logout" data-method="post" id="logout" class="tooltipped downwards" title="Sign out">
            <span class="mini-icon mini-icon-logout"></span>
          </a>
      </li>
    </ul>



            
          </div>
        </div>


      

      


            <div class="site hfeed" itemscope itemtype="http://schema.org/WebPage">
      <div class="hentry">
        
        <div class="pagehead repohead instapaper_ignore readability-menu">
          <div class="container">
            <div class="title-actions-bar">
              


                  <ul class="pagehead-actions">
          <li class="nspr">
            <a href="/AlaskaResearchCubeSat/COMM/pull/new/255_Byte_ARCLib" class="button minibutton btn-pull-request" icon_class="mini-icon-pull-request"><span class="mini-icon mini-icon-pull-request"></span>Pull Request</a>
          </li>

          <li class="subscription">
              <form accept-charset="UTF-8" action="/notifications/subscribe" data-autosubmit="true" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="authenticity_token" type="hidden" value="hOuPGEWMiPJUnlawwHqnPgCvULlSBU6jJDwl1DAgerw=" /></div>  <input id="repository_id" name="repository_id" type="hidden" value="7364947" />
  <div class="context-menu-container js-menu-container js-context-menu">
    <span class="minibutton switcher bigger js-menu-target">
      <span class="js-context-button">
          <span class="mini-icon mini-icon-unwatch"></span>Unwatch
      </span>
    </span>

    <div class="context-pane js-menu-content">
      <a href="#" class="close js-menu-close"><span class="mini-icon mini-icon-remove-close"></span></a>
      <div class="context-title">Notification status</div>

      <div class="context-body pane-selector">
        <ul class="js-navigation-container">
          <li class="selector-item js-navigation-item js-navigation-target ">
            <span class="mini-icon mini-icon-confirm"></span>
            <label>
              <input id="do_included" name="do" type="radio" value="included" />
              <h4>Not watching</h4>
              <p>You will only receive notifications when you participate or are mentioned.</p>
            </label>
            <span class="context-button-text js-context-button-text">
              <span class="mini-icon mini-icon-watching"></span>
              Watch
            </span>
          </li>
          <li class="selector-item js-navigation-item js-navigation-target selected">
            <span class="mini-icon mini-icon-confirm"></span>
            <label>
              <input checked="checked" id="do_subscribed" name="do" type="radio" value="subscribed" />
              <h4>Watching</h4>
              <p>You will receive all notifications for this repository.</p>
            </label>
            <span class="context-button-text js-context-button-text">
              <span class="mini-icon mini-icon-unwatch"></span>
              Unwatch
            </span>
          </li>
          <li class="selector-item js-navigation-item js-navigation-target ">
            <span class="mini-icon mini-icon-confirm"></span>
            <label>
              <input id="do_ignore" name="do" type="radio" value="ignore" />
              <h4>Ignored</h4>
              <p>You will not receive notifications for this repository.</p>
            </label>
            <span class="context-button-text js-context-button-text">
              <span class="mini-icon mini-icon-mute"></span>
              Stop ignoring
            </span>
          </li>
        </ul>
      </div>
    </div>
  </div>
</form>
          </li>

          <li class="js-toggler-container js-social-container starring-container ">
            <a href="/AlaskaResearchCubeSat/COMM/unstar" class="minibutton js-toggler-target starred" data-remote="true" data-method="post" rel="nofollow">
              <span class="mini-icon mini-icon-star"></span>Unstar
            </a><a href="/AlaskaResearchCubeSat/COMM/star" class="minibutton js-toggler-target unstarred" data-remote="true" data-method="post" rel="nofollow">
              <span class="mini-icon mini-icon-star"></span>Star
            </a><a class="social-count js-social-count" href="/AlaskaResearchCubeSat/COMM/stargazers">0</a>
          </li>

              <li>
                <a href="/AlaskaResearchCubeSat/COMM/fork_select" class="minibutton js-toggler-target lighter" rel="facebox nofollow"><span class="mini-icon mini-icon-fork"></span>Fork</a><a href="/AlaskaResearchCubeSat/COMM/network" class="social-count">0</a>
              </li>


    </ul>

              <h1 itemscope itemtype="http://data-vocabulary.org/Breadcrumb" class="entry-title public">
                <span class="repo-label"><span>public</span></span>
                <span class="mega-icon mega-icon-public-repo"></span>
                <span class="author vcard">
                  <a href="/AlaskaResearchCubeSat" class="url fn" itemprop="url" rel="author">
                  <span itemprop="title">AlaskaResearchCubeSat</span>
                  </a></span> /
                <strong><a href="/AlaskaResearchCubeSat/COMM" class="js-current-repository">COMM</a></strong>
              </h1>
            </div>

            

  <ul class="tabs">
    <li><a href="/AlaskaResearchCubeSat/COMM/tree/255_Byte_ARCLib" class="selected" highlight="repo_sourcerepo_downloadsrepo_commitsrepo_tagsrepo_branches">Code</a></li>
    <li><a href="/AlaskaResearchCubeSat/COMM/network" highlight="repo_network">Network</a></li>
    <li><a href="/AlaskaResearchCubeSat/COMM/pulls" highlight="repo_pulls">Pull Requests <span class='counter'>0</span></a></li>

      <li><a href="/AlaskaResearchCubeSat/COMM/issues" highlight="repo_issues">Issues <span class='counter'>0</span></a></li>

      <li><a href="/AlaskaResearchCubeSat/COMM/wiki" highlight="repo_wiki">Wiki</a></li>


    <li><a href="/AlaskaResearchCubeSat/COMM/graphs" highlight="repo_graphsrepo_contributors">Graphs</a></li>

      <li>
        <a href="/AlaskaResearchCubeSat/COMM/settings">Settings</a>
      </li>

  </ul>
  
<div class="tabnav">

  <span class="tabnav-right">
    <ul class="tabnav-tabs">
          <li><a href="/AlaskaResearchCubeSat/COMM/tags" class="tabnav-tab" highlight="repo_tags">Tags <span class="counter blank">0</span></a></li>
    </ul>
    
  </span>

  <div class="tabnav-widget scope">


    <div class="select-menu js-menu-container js-select-menu js-branch-menu">
      <a class="minibutton select-menu-button js-menu-target" data-hotkey="w" data-ref="255_Byte_ARCLib">
        <span class="mini-icon mini-icon-branch"></span>
        <i>branch:</i>
        <span class="js-select-button">255_Byte_ARCLib</span>
      </a>

      <div class="select-menu-modal-holder js-menu-content js-navigation-container js-select-menu-pane">

        <div class="select-menu-modal js-select-menu-pane">
          <div class="select-menu-header">
            <span class="select-menu-title">Switch branches/tags</span>
            <span class="mini-icon mini-icon-remove-close js-menu-close"></span>
          </div> <!-- /.select-menu-header -->

          <div class="select-menu-filters">
            <div class="select-menu-text-filter">
              <input type="text" id="commitish-filter-field" class="js-select-menu-text-filter js-filterable-field js-navigation-enable" placeholder="Find or create a branch…">
            </div> <!-- /.select-menu-text-filter -->
            <div class="select-menu-tabs">
              <ul>
                <li class="select-menu-tab">
                  <a href="#" data-filter="branches" class="js-select-menu-tab selected">Branches</a>
                </li>
                <li class="select-menu-tab">
                  <a href="#" data-filter="tags" class="js-select-menu-tab">Tags</a>
                </li>
              </ul>
            </div><!-- /.select-menu-tabs -->
          </div><!-- /.select-menu-filters -->

          <div class="select-menu-list js-filter-tab js-filter-branches" data-filterable-for="commitish-filter-field" data-filterable-type="substring">



              <div class="select-menu-item js-navigation-item js-navigation-target selected">
                <span class="select-menu-checkmark mini-icon mini-icon-confirm"></span>

                    <a href="/AlaskaResearchCubeSat/COMM/blob/255_Byte_ARCLib/255_Byte_Code/functions.c" class="js-navigation-open select-menu-item-text js-select-button-text" data-name="255_Byte_ARCLib" rel="nofollow">255_Byte_ARCLib</a>

              </div> <!-- /.select-menu-item -->



              <div class="select-menu-item js-navigation-item js-navigation-target ">
                <span class="select-menu-checkmark mini-icon mini-icon-confirm"></span>

                    <a href="/AlaskaResearchCubeSat/COMM/blob/master/255_Byte_Code/functions.c" class="js-navigation-open select-menu-item-text js-select-button-text" data-name="master" rel="nofollow">master</a>

              </div> <!-- /.select-menu-item -->


              <form accept-charset="UTF-8" action="/AlaskaResearchCubeSat/COMM/branches" class="js-create-branch select-menu-footer select-menu-item select-menu-new-item-form js-navigation-item js-navigation-target js-new-item-form" method="post"><div style="margin:0;padding:0;display:inline"><input name="authenticity_token" type="hidden" value="hOuPGEWMiPJUnlawwHqnPgCvULlSBU6jJDwl1DAgerw=" /></div>


                <span class="mini-icon mini-icon-branch-create"></span>
                <div class="select-menu-item-text">
                  <h4>Create branch: <span class="js-new-item-name"></span></h4>
                  <span class="description">from ‘255_Byte_ARCLib’</span>
                </div>
                <input type="hidden" name="name" id="name" class="js-new-item-submit" />
                <input type="hidden" name="branch" id="branch" value="255_Byte_ARCLib" />

              </form> <!-- /.select-menu-footer -->


          </div> <!-- /.select-menu-list -->


          <div class="select-menu-list js-filter-tab js-filter-tags" data-filterable-for="commitish-filter-field" data-filterable-type="substring" style="display:none;">


            <div class="select-menu-no-results js-not-filterable">Nothing to show</div>

          </div> <!-- /.select-menu-list -->

        </div> <!-- /.select-menu-modal -->
      </div> <!-- /.select-menu-modal-holder -->
    </div> <!-- /.select-menu -->

  </div> <!-- /.scope -->

  <ul class="tabnav-tabs">
    <li><a href="/AlaskaResearchCubeSat/COMM/tree/255_Byte_ARCLib" class="selected tabnav-tab" highlight="repo_source">Files</a></li>
    <li><a href="/AlaskaResearchCubeSat/COMM/commits/255_Byte_ARCLib" class="tabnav-tab" highlight="repo_commits">Commits</a></li>
    <li><a href="/AlaskaResearchCubeSat/COMM/branches" class="tabnav-tab" highlight="repo_branches" rel="nofollow">Branches <span class="counter ">2</span></a></li>
  </ul>

</div>

  
  
  


            
          </div>
        </div><!-- /.repohead -->

        <div id="js-repo-pjax-container" class="container context-loader-container" data-pjax-container>
          


<!-- blob contrib key: blob_contributors:v21:34946a518d7f9418d228748cc6442641 -->
<!-- blob contrib frag key: views10/v8/blob_contributors:v21:34946a518d7f9418d228748cc6442641 -->

<div id="slider">


    <div class="frame-meta">

      <p title="This is a placeholder element" class="js-history-link-replace hidden"></p>
      <div class="breadcrumb">
        <span class='bold'><span itemscope="" itemtype="http://data-vocabulary.org/Breadcrumb"><a href="/AlaskaResearchCubeSat/COMM/tree/255_Byte_ARCLib" class="js-slide-to" data-direction="back" itemscope="url"><span itemprop="title">COMM</span></a></span></span> / <span itemscope="" itemtype="http://data-vocabulary.org/Breadcrumb"><a href="/AlaskaResearchCubeSat/COMM/tree/255_Byte_ARCLib/255_Byte_Code" class="js-slide-to" data-direction="back" itemscope="url"><span itemprop="title">255_Byte_Code</span></a></span> / <strong class="final-path">functions.c</strong> <span class="js-zeroclipboard zeroclipboard-button" data-clipboard-text="255_Byte_Code/functions.c" data-copied-hint="copied!" title="copy to clipboard"><span class="mini-icon mini-icon-clipboard"></span></span>
      </div>

      <a href="/AlaskaResearchCubeSat/COMM/find/255_Byte_ARCLib" class="js-slide-to" data-hotkey="t" style="display:none">Show File Finder</a>

        
  <div class="commit file-history-tease">
    <img class="main-avatar" height="24" src="https://secure.gravatar.com/avatar/cb06419b31efe234e765209f3d3b1230?s=140&amp;d=https://a248.e.akamai.net/assets.github.com%2Fimages%2Fgravatars%2Fgravatar-user-420.png" width="24" />
    <span class="author"><a href="/samuelvanderwaal" rel="author">samuelvanderwaal</a></span>
    <time class="js-relative-date" datetime="2013-01-10T12:02:30-08:00" title="2013-01-10 12:02:30">January 10, 2013</time>
    <div class="commit-title">
        <a href="/AlaskaResearchCubeSat/COMM/commit/d0809d2d15872b0d16694ca4b54203bb88c1a887" class="message">Finished ARCLib integration and Tx/Rx code</a>
    </div>

    <div class="participation">
      <p class="quickstat"><a href="#blob_contributors_box" rel="facebox"><strong>1</strong> contributor</a></p>
      
    </div>
    <div id="blob_contributors_box" style="display:none">
      <h2>Users on GitHub who have contributed to this file</h2>
      <ul class="facebox-user-list">
        <li>
          <img height="24" src="https://secure.gravatar.com/avatar/cb06419b31efe234e765209f3d3b1230?s=140&amp;d=https://a248.e.akamai.net/assets.github.com%2Fimages%2Fgravatars%2Fgravatar-user-420.png" width="24" />
          <a href="/samuelvanderwaal">samuelvanderwaal</a>
        </li>
      </ul>
    </div>
  </div>


    </div><!-- ./.frame-meta -->

    <div class="frames">
      <div class="frame" data-permalink-url="/AlaskaResearchCubeSat/COMM/blob/d0809d2d15872b0d16694ca4b54203bb88c1a887/255_Byte_Code/functions.c" data-title="COMM/255_Byte_Code/functions.c at 255_Byte_ARCLib · AlaskaResearchCubeSat/COMM · GitHub" data-type="blob">

        <div id="files" class="bubble">
          <div class="file">
            <div class="meta">
              <div class="info">
                <span class="icon"><b class="mini-icon mini-icon-text-file"></b></span>
                <span class="mode" title="File Mode">file</span>
                  <span>664 lines (594 sloc)</span>
                <span>28.679 kb</span>
              </div>
              <ul class="button-group actions">
                  <li>
                        <a class="grouped-button minibutton bigger lighter"
                           href="/AlaskaResearchCubeSat/COMM/edit/255_Byte_ARCLib/255_Byte_Code/functions.c"
                           data-method="post" rel="nofollow" data-hotkey="e">Edit</a>
                  </li>
                <li><a href="/AlaskaResearchCubeSat/COMM/raw/255_Byte_ARCLib/255_Byte_Code/functions.c" class="button minibutton grouped-button bigger lighter" id="raw-url">Raw</a></li>
                  <li><a href="/AlaskaResearchCubeSat/COMM/blame/255_Byte_ARCLib/255_Byte_Code/functions.c" class="button minibutton grouped-button bigger lighter">Blame</a></li>
                <li><a href="/AlaskaResearchCubeSat/COMM/commits/255_Byte_ARCLib/255_Byte_Code/functions.c" class="button minibutton grouped-button bigger lighter" rel="nofollow">History</a></li>
              </ul>

            </div>
                <div class="data type-c js-blob-data">
      <table cellpadding="0" cellspacing="0" class="lines">
        <tr>
          <td>
            <pre class="line_numbers"><span id="L1" rel="#L1">1</span>
<span id="L2" rel="#L2">2</span>
<span id="L3" rel="#L3">3</span>
<span id="L4" rel="#L4">4</span>
<span id="L5" rel="#L5">5</span>
<span id="L6" rel="#L6">6</span>
<span id="L7" rel="#L7">7</span>
<span id="L8" rel="#L8">8</span>
<span id="L9" rel="#L9">9</span>
<span id="L10" rel="#L10">10</span>
<span id="L11" rel="#L11">11</span>
<span id="L12" rel="#L12">12</span>
<span id="L13" rel="#L13">13</span>
<span id="L14" rel="#L14">14</span>
<span id="L15" rel="#L15">15</span>
<span id="L16" rel="#L16">16</span>
<span id="L17" rel="#L17">17</span>
<span id="L18" rel="#L18">18</span>
<span id="L19" rel="#L19">19</span>
<span id="L20" rel="#L20">20</span>
<span id="L21" rel="#L21">21</span>
<span id="L22" rel="#L22">22</span>
<span id="L23" rel="#L23">23</span>
<span id="L24" rel="#L24">24</span>
<span id="L25" rel="#L25">25</span>
<span id="L26" rel="#L26">26</span>
<span id="L27" rel="#L27">27</span>
<span id="L28" rel="#L28">28</span>
<span id="L29" rel="#L29">29</span>
<span id="L30" rel="#L30">30</span>
<span id="L31" rel="#L31">31</span>
<span id="L32" rel="#L32">32</span>
<span id="L33" rel="#L33">33</span>
<span id="L34" rel="#L34">34</span>
<span id="L35" rel="#L35">35</span>
<span id="L36" rel="#L36">36</span>
<span id="L37" rel="#L37">37</span>
<span id="L38" rel="#L38">38</span>
<span id="L39" rel="#L39">39</span>
<span id="L40" rel="#L40">40</span>
<span id="L41" rel="#L41">41</span>
<span id="L42" rel="#L42">42</span>
<span id="L43" rel="#L43">43</span>
<span id="L44" rel="#L44">44</span>
<span id="L45" rel="#L45">45</span>
<span id="L46" rel="#L46">46</span>
<span id="L47" rel="#L47">47</span>
<span id="L48" rel="#L48">48</span>
<span id="L49" rel="#L49">49</span>
<span id="L50" rel="#L50">50</span>
<span id="L51" rel="#L51">51</span>
<span id="L52" rel="#L52">52</span>
<span id="L53" rel="#L53">53</span>
<span id="L54" rel="#L54">54</span>
<span id="L55" rel="#L55">55</span>
<span id="L56" rel="#L56">56</span>
<span id="L57" rel="#L57">57</span>
<span id="L58" rel="#L58">58</span>
<span id="L59" rel="#L59">59</span>
<span id="L60" rel="#L60">60</span>
<span id="L61" rel="#L61">61</span>
<span id="L62" rel="#L62">62</span>
<span id="L63" rel="#L63">63</span>
<span id="L64" rel="#L64">64</span>
<span id="L65" rel="#L65">65</span>
<span id="L66" rel="#L66">66</span>
<span id="L67" rel="#L67">67</span>
<span id="L68" rel="#L68">68</span>
<span id="L69" rel="#L69">69</span>
<span id="L70" rel="#L70">70</span>
<span id="L71" rel="#L71">71</span>
<span id="L72" rel="#L72">72</span>
<span id="L73" rel="#L73">73</span>
<span id="L74" rel="#L74">74</span>
<span id="L75" rel="#L75">75</span>
<span id="L76" rel="#L76">76</span>
<span id="L77" rel="#L77">77</span>
<span id="L78" rel="#L78">78</span>
<span id="L79" rel="#L79">79</span>
<span id="L80" rel="#L80">80</span>
<span id="L81" rel="#L81">81</span>
<span id="L82" rel="#L82">82</span>
<span id="L83" rel="#L83">83</span>
<span id="L84" rel="#L84">84</span>
<span id="L85" rel="#L85">85</span>
<span id="L86" rel="#L86">86</span>
<span id="L87" rel="#L87">87</span>
<span id="L88" rel="#L88">88</span>
<span id="L89" rel="#L89">89</span>
<span id="L90" rel="#L90">90</span>
<span id="L91" rel="#L91">91</span>
<span id="L92" rel="#L92">92</span>
<span id="L93" rel="#L93">93</span>
<span id="L94" rel="#L94">94</span>
<span id="L95" rel="#L95">95</span>
<span id="L96" rel="#L96">96</span>
<span id="L97" rel="#L97">97</span>
<span id="L98" rel="#L98">98</span>
<span id="L99" rel="#L99">99</span>
<span id="L100" rel="#L100">100</span>
<span id="L101" rel="#L101">101</span>
<span id="L102" rel="#L102">102</span>
<span id="L103" rel="#L103">103</span>
<span id="L104" rel="#L104">104</span>
<span id="L105" rel="#L105">105</span>
<span id="L106" rel="#L106">106</span>
<span id="L107" rel="#L107">107</span>
<span id="L108" rel="#L108">108</span>
<span id="L109" rel="#L109">109</span>
<span id="L110" rel="#L110">110</span>
<span id="L111" rel="#L111">111</span>
<span id="L112" rel="#L112">112</span>
<span id="L113" rel="#L113">113</span>
<span id="L114" rel="#L114">114</span>
<span id="L115" rel="#L115">115</span>
<span id="L116" rel="#L116">116</span>
<span id="L117" rel="#L117">117</span>
<span id="L118" rel="#L118">118</span>
<span id="L119" rel="#L119">119</span>
<span id="L120" rel="#L120">120</span>
<span id="L121" rel="#L121">121</span>
<span id="L122" rel="#L122">122</span>
<span id="L123" rel="#L123">123</span>
<span id="L124" rel="#L124">124</span>
<span id="L125" rel="#L125">125</span>
<span id="L126" rel="#L126">126</span>
<span id="L127" rel="#L127">127</span>
<span id="L128" rel="#L128">128</span>
<span id="L129" rel="#L129">129</span>
<span id="L130" rel="#L130">130</span>
<span id="L131" rel="#L131">131</span>
<span id="L132" rel="#L132">132</span>
<span id="L133" rel="#L133">133</span>
<span id="L134" rel="#L134">134</span>
<span id="L135" rel="#L135">135</span>
<span id="L136" rel="#L136">136</span>
<span id="L137" rel="#L137">137</span>
<span id="L138" rel="#L138">138</span>
<span id="L139" rel="#L139">139</span>
<span id="L140" rel="#L140">140</span>
<span id="L141" rel="#L141">141</span>
<span id="L142" rel="#L142">142</span>
<span id="L143" rel="#L143">143</span>
<span id="L144" rel="#L144">144</span>
<span id="L145" rel="#L145">145</span>
<span id="L146" rel="#L146">146</span>
<span id="L147" rel="#L147">147</span>
<span id="L148" rel="#L148">148</span>
<span id="L149" rel="#L149">149</span>
<span id="L150" rel="#L150">150</span>
<span id="L151" rel="#L151">151</span>
<span id="L152" rel="#L152">152</span>
<span id="L153" rel="#L153">153</span>
<span id="L154" rel="#L154">154</span>
<span id="L155" rel="#L155">155</span>
<span id="L156" rel="#L156">156</span>
<span id="L157" rel="#L157">157</span>
<span id="L158" rel="#L158">158</span>
<span id="L159" rel="#L159">159</span>
<span id="L160" rel="#L160">160</span>
<span id="L161" rel="#L161">161</span>
<span id="L162" rel="#L162">162</span>
<span id="L163" rel="#L163">163</span>
<span id="L164" rel="#L164">164</span>
<span id="L165" rel="#L165">165</span>
<span id="L166" rel="#L166">166</span>
<span id="L167" rel="#L167">167</span>
<span id="L168" rel="#L168">168</span>
<span id="L169" rel="#L169">169</span>
<span id="L170" rel="#L170">170</span>
<span id="L171" rel="#L171">171</span>
<span id="L172" rel="#L172">172</span>
<span id="L173" rel="#L173">173</span>
<span id="L174" rel="#L174">174</span>
<span id="L175" rel="#L175">175</span>
<span id="L176" rel="#L176">176</span>
<span id="L177" rel="#L177">177</span>
<span id="L178" rel="#L178">178</span>
<span id="L179" rel="#L179">179</span>
<span id="L180" rel="#L180">180</span>
<span id="L181" rel="#L181">181</span>
<span id="L182" rel="#L182">182</span>
<span id="L183" rel="#L183">183</span>
<span id="L184" rel="#L184">184</span>
<span id="L185" rel="#L185">185</span>
<span id="L186" rel="#L186">186</span>
<span id="L187" rel="#L187">187</span>
<span id="L188" rel="#L188">188</span>
<span id="L189" rel="#L189">189</span>
<span id="L190" rel="#L190">190</span>
<span id="L191" rel="#L191">191</span>
<span id="L192" rel="#L192">192</span>
<span id="L193" rel="#L193">193</span>
<span id="L194" rel="#L194">194</span>
<span id="L195" rel="#L195">195</span>
<span id="L196" rel="#L196">196</span>
<span id="L197" rel="#L197">197</span>
<span id="L198" rel="#L198">198</span>
<span id="L199" rel="#L199">199</span>
<span id="L200" rel="#L200">200</span>
<span id="L201" rel="#L201">201</span>
<span id="L202" rel="#L202">202</span>
<span id="L203" rel="#L203">203</span>
<span id="L204" rel="#L204">204</span>
<span id="L205" rel="#L205">205</span>
<span id="L206" rel="#L206">206</span>
<span id="L207" rel="#L207">207</span>
<span id="L208" rel="#L208">208</span>
<span id="L209" rel="#L209">209</span>
<span id="L210" rel="#L210">210</span>
<span id="L211" rel="#L211">211</span>
<span id="L212" rel="#L212">212</span>
<span id="L213" rel="#L213">213</span>
<span id="L214" rel="#L214">214</span>
<span id="L215" rel="#L215">215</span>
<span id="L216" rel="#L216">216</span>
<span id="L217" rel="#L217">217</span>
<span id="L218" rel="#L218">218</span>
<span id="L219" rel="#L219">219</span>
<span id="L220" rel="#L220">220</span>
<span id="L221" rel="#L221">221</span>
<span id="L222" rel="#L222">222</span>
<span id="L223" rel="#L223">223</span>
<span id="L224" rel="#L224">224</span>
<span id="L225" rel="#L225">225</span>
<span id="L226" rel="#L226">226</span>
<span id="L227" rel="#L227">227</span>
<span id="L228" rel="#L228">228</span>
<span id="L229" rel="#L229">229</span>
<span id="L230" rel="#L230">230</span>
<span id="L231" rel="#L231">231</span>
<span id="L232" rel="#L232">232</span>
<span id="L233" rel="#L233">233</span>
<span id="L234" rel="#L234">234</span>
<span id="L235" rel="#L235">235</span>
<span id="L236" rel="#L236">236</span>
<span id="L237" rel="#L237">237</span>
<span id="L238" rel="#L238">238</span>
<span id="L239" rel="#L239">239</span>
<span id="L240" rel="#L240">240</span>
<span id="L241" rel="#L241">241</span>
<span id="L242" rel="#L242">242</span>
<span id="L243" rel="#L243">243</span>
<span id="L244" rel="#L244">244</span>
<span id="L245" rel="#L245">245</span>
<span id="L246" rel="#L246">246</span>
<span id="L247" rel="#L247">247</span>
<span id="L248" rel="#L248">248</span>
<span id="L249" rel="#L249">249</span>
<span id="L250" rel="#L250">250</span>
<span id="L251" rel="#L251">251</span>
<span id="L252" rel="#L252">252</span>
<span id="L253" rel="#L253">253</span>
<span id="L254" rel="#L254">254</span>
<span id="L255" rel="#L255">255</span>
<span id="L256" rel="#L256">256</span>
<span id="L257" rel="#L257">257</span>
<span id="L258" rel="#L258">258</span>
<span id="L259" rel="#L259">259</span>
<span id="L260" rel="#L260">260</span>
<span id="L261" rel="#L261">261</span>
<span id="L262" rel="#L262">262</span>
<span id="L263" rel="#L263">263</span>
<span id="L264" rel="#L264">264</span>
<span id="L265" rel="#L265">265</span>
<span id="L266" rel="#L266">266</span>
<span id="L267" rel="#L267">267</span>
<span id="L268" rel="#L268">268</span>
<span id="L269" rel="#L269">269</span>
<span id="L270" rel="#L270">270</span>
<span id="L271" rel="#L271">271</span>
<span id="L272" rel="#L272">272</span>
<span id="L273" rel="#L273">273</span>
<span id="L274" rel="#L274">274</span>
<span id="L275" rel="#L275">275</span>
<span id="L276" rel="#L276">276</span>
<span id="L277" rel="#L277">277</span>
<span id="L278" rel="#L278">278</span>
<span id="L279" rel="#L279">279</span>
<span id="L280" rel="#L280">280</span>
<span id="L281" rel="#L281">281</span>
<span id="L282" rel="#L282">282</span>
<span id="L283" rel="#L283">283</span>
<span id="L284" rel="#L284">284</span>
<span id="L285" rel="#L285">285</span>
<span id="L286" rel="#L286">286</span>
<span id="L287" rel="#L287">287</span>
<span id="L288" rel="#L288">288</span>
<span id="L289" rel="#L289">289</span>
<span id="L290" rel="#L290">290</span>
<span id="L291" rel="#L291">291</span>
<span id="L292" rel="#L292">292</span>
<span id="L293" rel="#L293">293</span>
<span id="L294" rel="#L294">294</span>
<span id="L295" rel="#L295">295</span>
<span id="L296" rel="#L296">296</span>
<span id="L297" rel="#L297">297</span>
<span id="L298" rel="#L298">298</span>
<span id="L299" rel="#L299">299</span>
<span id="L300" rel="#L300">300</span>
<span id="L301" rel="#L301">301</span>
<span id="L302" rel="#L302">302</span>
<span id="L303" rel="#L303">303</span>
<span id="L304" rel="#L304">304</span>
<span id="L305" rel="#L305">305</span>
<span id="L306" rel="#L306">306</span>
<span id="L307" rel="#L307">307</span>
<span id="L308" rel="#L308">308</span>
<span id="L309" rel="#L309">309</span>
<span id="L310" rel="#L310">310</span>
<span id="L311" rel="#L311">311</span>
<span id="L312" rel="#L312">312</span>
<span id="L313" rel="#L313">313</span>
<span id="L314" rel="#L314">314</span>
<span id="L315" rel="#L315">315</span>
<span id="L316" rel="#L316">316</span>
<span id="L317" rel="#L317">317</span>
<span id="L318" rel="#L318">318</span>
<span id="L319" rel="#L319">319</span>
<span id="L320" rel="#L320">320</span>
<span id="L321" rel="#L321">321</span>
<span id="L322" rel="#L322">322</span>
<span id="L323" rel="#L323">323</span>
<span id="L324" rel="#L324">324</span>
<span id="L325" rel="#L325">325</span>
<span id="L326" rel="#L326">326</span>
<span id="L327" rel="#L327">327</span>
<span id="L328" rel="#L328">328</span>
<span id="L329" rel="#L329">329</span>
<span id="L330" rel="#L330">330</span>
<span id="L331" rel="#L331">331</span>
<span id="L332" rel="#L332">332</span>
<span id="L333" rel="#L333">333</span>
<span id="L334" rel="#L334">334</span>
<span id="L335" rel="#L335">335</span>
<span id="L336" rel="#L336">336</span>
<span id="L337" rel="#L337">337</span>
<span id="L338" rel="#L338">338</span>
<span id="L339" rel="#L339">339</span>
<span id="L340" rel="#L340">340</span>
<span id="L341" rel="#L341">341</span>
<span id="L342" rel="#L342">342</span>
<span id="L343" rel="#L343">343</span>
<span id="L344" rel="#L344">344</span>
<span id="L345" rel="#L345">345</span>
<span id="L346" rel="#L346">346</span>
<span id="L347" rel="#L347">347</span>
<span id="L348" rel="#L348">348</span>
<span id="L349" rel="#L349">349</span>
<span id="L350" rel="#L350">350</span>
<span id="L351" rel="#L351">351</span>
<span id="L352" rel="#L352">352</span>
<span id="L353" rel="#L353">353</span>
<span id="L354" rel="#L354">354</span>
<span id="L355" rel="#L355">355</span>
<span id="L356" rel="#L356">356</span>
<span id="L357" rel="#L357">357</span>
<span id="L358" rel="#L358">358</span>
<span id="L359" rel="#L359">359</span>
<span id="L360" rel="#L360">360</span>
<span id="L361" rel="#L361">361</span>
<span id="L362" rel="#L362">362</span>
<span id="L363" rel="#L363">363</span>
<span id="L364" rel="#L364">364</span>
<span id="L365" rel="#L365">365</span>
<span id="L366" rel="#L366">366</span>
<span id="L367" rel="#L367">367</span>
<span id="L368" rel="#L368">368</span>
<span id="L369" rel="#L369">369</span>
<span id="L370" rel="#L370">370</span>
<span id="L371" rel="#L371">371</span>
<span id="L372" rel="#L372">372</span>
<span id="L373" rel="#L373">373</span>
<span id="L374" rel="#L374">374</span>
<span id="L375" rel="#L375">375</span>
<span id="L376" rel="#L376">376</span>
<span id="L377" rel="#L377">377</span>
<span id="L378" rel="#L378">378</span>
<span id="L379" rel="#L379">379</span>
<span id="L380" rel="#L380">380</span>
<span id="L381" rel="#L381">381</span>
<span id="L382" rel="#L382">382</span>
<span id="L383" rel="#L383">383</span>
<span id="L384" rel="#L384">384</span>
<span id="L385" rel="#L385">385</span>
<span id="L386" rel="#L386">386</span>
<span id="L387" rel="#L387">387</span>
<span id="L388" rel="#L388">388</span>
<span id="L389" rel="#L389">389</span>
<span id="L390" rel="#L390">390</span>
<span id="L391" rel="#L391">391</span>
<span id="L392" rel="#L392">392</span>
<span id="L393" rel="#L393">393</span>
<span id="L394" rel="#L394">394</span>
<span id="L395" rel="#L395">395</span>
<span id="L396" rel="#L396">396</span>
<span id="L397" rel="#L397">397</span>
<span id="L398" rel="#L398">398</span>
<span id="L399" rel="#L399">399</span>
<span id="L400" rel="#L400">400</span>
<span id="L401" rel="#L401">401</span>
<span id="L402" rel="#L402">402</span>
<span id="L403" rel="#L403">403</span>
<span id="L404" rel="#L404">404</span>
<span id="L405" rel="#L405">405</span>
<span id="L406" rel="#L406">406</span>
<span id="L407" rel="#L407">407</span>
<span id="L408" rel="#L408">408</span>
<span id="L409" rel="#L409">409</span>
<span id="L410" rel="#L410">410</span>
<span id="L411" rel="#L411">411</span>
<span id="L412" rel="#L412">412</span>
<span id="L413" rel="#L413">413</span>
<span id="L414" rel="#L414">414</span>
<span id="L415" rel="#L415">415</span>
<span id="L416" rel="#L416">416</span>
<span id="L417" rel="#L417">417</span>
<span id="L418" rel="#L418">418</span>
<span id="L419" rel="#L419">419</span>
<span id="L420" rel="#L420">420</span>
<span id="L421" rel="#L421">421</span>
<span id="L422" rel="#L422">422</span>
<span id="L423" rel="#L423">423</span>
<span id="L424" rel="#L424">424</span>
<span id="L425" rel="#L425">425</span>
<span id="L426" rel="#L426">426</span>
<span id="L427" rel="#L427">427</span>
<span id="L428" rel="#L428">428</span>
<span id="L429" rel="#L429">429</span>
<span id="L430" rel="#L430">430</span>
<span id="L431" rel="#L431">431</span>
<span id="L432" rel="#L432">432</span>
<span id="L433" rel="#L433">433</span>
<span id="L434" rel="#L434">434</span>
<span id="L435" rel="#L435">435</span>
<span id="L436" rel="#L436">436</span>
<span id="L437" rel="#L437">437</span>
<span id="L438" rel="#L438">438</span>
<span id="L439" rel="#L439">439</span>
<span id="L440" rel="#L440">440</span>
<span id="L441" rel="#L441">441</span>
<span id="L442" rel="#L442">442</span>
<span id="L443" rel="#L443">443</span>
<span id="L444" rel="#L444">444</span>
<span id="L445" rel="#L445">445</span>
<span id="L446" rel="#L446">446</span>
<span id="L447" rel="#L447">447</span>
<span id="L448" rel="#L448">448</span>
<span id="L449" rel="#L449">449</span>
<span id="L450" rel="#L450">450</span>
<span id="L451" rel="#L451">451</span>
<span id="L452" rel="#L452">452</span>
<span id="L453" rel="#L453">453</span>
<span id="L454" rel="#L454">454</span>
<span id="L455" rel="#L455">455</span>
<span id="L456" rel="#L456">456</span>
<span id="L457" rel="#L457">457</span>
<span id="L458" rel="#L458">458</span>
<span id="L459" rel="#L459">459</span>
<span id="L460" rel="#L460">460</span>
<span id="L461" rel="#L461">461</span>
<span id="L462" rel="#L462">462</span>
<span id="L463" rel="#L463">463</span>
<span id="L464" rel="#L464">464</span>
<span id="L465" rel="#L465">465</span>
<span id="L466" rel="#L466">466</span>
<span id="L467" rel="#L467">467</span>
<span id="L468" rel="#L468">468</span>
<span id="L469" rel="#L469">469</span>
<span id="L470" rel="#L470">470</span>
<span id="L471" rel="#L471">471</span>
<span id="L472" rel="#L472">472</span>
<span id="L473" rel="#L473">473</span>
<span id="L474" rel="#L474">474</span>
<span id="L475" rel="#L475">475</span>
<span id="L476" rel="#L476">476</span>
<span id="L477" rel="#L477">477</span>
<span id="L478" rel="#L478">478</span>
<span id="L479" rel="#L479">479</span>
<span id="L480" rel="#L480">480</span>
<span id="L481" rel="#L481">481</span>
<span id="L482" rel="#L482">482</span>
<span id="L483" rel="#L483">483</span>
<span id="L484" rel="#L484">484</span>
<span id="L485" rel="#L485">485</span>
<span id="L486" rel="#L486">486</span>
<span id="L487" rel="#L487">487</span>
<span id="L488" rel="#L488">488</span>
<span id="L489" rel="#L489">489</span>
<span id="L490" rel="#L490">490</span>
<span id="L491" rel="#L491">491</span>
<span id="L492" rel="#L492">492</span>
<span id="L493" rel="#L493">493</span>
<span id="L494" rel="#L494">494</span>
<span id="L495" rel="#L495">495</span>
<span id="L496" rel="#L496">496</span>
<span id="L497" rel="#L497">497</span>
<span id="L498" rel="#L498">498</span>
<span id="L499" rel="#L499">499</span>
<span id="L500" rel="#L500">500</span>
<span id="L501" rel="#L501">501</span>
<span id="L502" rel="#L502">502</span>
<span id="L503" rel="#L503">503</span>
<span id="L504" rel="#L504">504</span>
<span id="L505" rel="#L505">505</span>
<span id="L506" rel="#L506">506</span>
<span id="L507" rel="#L507">507</span>
<span id="L508" rel="#L508">508</span>
<span id="L509" rel="#L509">509</span>
<span id="L510" rel="#L510">510</span>
<span id="L511" rel="#L511">511</span>
<span id="L512" rel="#L512">512</span>
<span id="L513" rel="#L513">513</span>
<span id="L514" rel="#L514">514</span>
<span id="L515" rel="#L515">515</span>
<span id="L516" rel="#L516">516</span>
<span id="L517" rel="#L517">517</span>
<span id="L518" rel="#L518">518</span>
<span id="L519" rel="#L519">519</span>
<span id="L520" rel="#L520">520</span>
<span id="L521" rel="#L521">521</span>
<span id="L522" rel="#L522">522</span>
<span id="L523" rel="#L523">523</span>
<span id="L524" rel="#L524">524</span>
<span id="L525" rel="#L525">525</span>
<span id="L526" rel="#L526">526</span>
<span id="L527" rel="#L527">527</span>
<span id="L528" rel="#L528">528</span>
<span id="L529" rel="#L529">529</span>
<span id="L530" rel="#L530">530</span>
<span id="L531" rel="#L531">531</span>
<span id="L532" rel="#L532">532</span>
<span id="L533" rel="#L533">533</span>
<span id="L534" rel="#L534">534</span>
<span id="L535" rel="#L535">535</span>
<span id="L536" rel="#L536">536</span>
<span id="L537" rel="#L537">537</span>
<span id="L538" rel="#L538">538</span>
<span id="L539" rel="#L539">539</span>
<span id="L540" rel="#L540">540</span>
<span id="L541" rel="#L541">541</span>
<span id="L542" rel="#L542">542</span>
<span id="L543" rel="#L543">543</span>
<span id="L544" rel="#L544">544</span>
<span id="L545" rel="#L545">545</span>
<span id="L546" rel="#L546">546</span>
<span id="L547" rel="#L547">547</span>
<span id="L548" rel="#L548">548</span>
<span id="L549" rel="#L549">549</span>
<span id="L550" rel="#L550">550</span>
<span id="L551" rel="#L551">551</span>
<span id="L552" rel="#L552">552</span>
<span id="L553" rel="#L553">553</span>
<span id="L554" rel="#L554">554</span>
<span id="L555" rel="#L555">555</span>
<span id="L556" rel="#L556">556</span>
<span id="L557" rel="#L557">557</span>
<span id="L558" rel="#L558">558</span>
<span id="L559" rel="#L559">559</span>
<span id="L560" rel="#L560">560</span>
<span id="L561" rel="#L561">561</span>
<span id="L562" rel="#L562">562</span>
<span id="L563" rel="#L563">563</span>
<span id="L564" rel="#L564">564</span>
<span id="L565" rel="#L565">565</span>
<span id="L566" rel="#L566">566</span>
<span id="L567" rel="#L567">567</span>
<span id="L568" rel="#L568">568</span>
<span id="L569" rel="#L569">569</span>
<span id="L570" rel="#L570">570</span>
<span id="L571" rel="#L571">571</span>
<span id="L572" rel="#L572">572</span>
<span id="L573" rel="#L573">573</span>
<span id="L574" rel="#L574">574</span>
<span id="L575" rel="#L575">575</span>
<span id="L576" rel="#L576">576</span>
<span id="L577" rel="#L577">577</span>
<span id="L578" rel="#L578">578</span>
<span id="L579" rel="#L579">579</span>
<span id="L580" rel="#L580">580</span>
<span id="L581" rel="#L581">581</span>
<span id="L582" rel="#L582">582</span>
<span id="L583" rel="#L583">583</span>
<span id="L584" rel="#L584">584</span>
<span id="L585" rel="#L585">585</span>
<span id="L586" rel="#L586">586</span>
<span id="L587" rel="#L587">587</span>
<span id="L588" rel="#L588">588</span>
<span id="L589" rel="#L589">589</span>
<span id="L590" rel="#L590">590</span>
<span id="L591" rel="#L591">591</span>
<span id="L592" rel="#L592">592</span>
<span id="L593" rel="#L593">593</span>
<span id="L594" rel="#L594">594</span>
<span id="L595" rel="#L595">595</span>
<span id="L596" rel="#L596">596</span>
<span id="L597" rel="#L597">597</span>
<span id="L598" rel="#L598">598</span>
<span id="L599" rel="#L599">599</span>
<span id="L600" rel="#L600">600</span>
<span id="L601" rel="#L601">601</span>
<span id="L602" rel="#L602">602</span>
<span id="L603" rel="#L603">603</span>
<span id="L604" rel="#L604">604</span>
<span id="L605" rel="#L605">605</span>
<span id="L606" rel="#L606">606</span>
<span id="L607" rel="#L607">607</span>
<span id="L608" rel="#L608">608</span>
<span id="L609" rel="#L609">609</span>
<span id="L610" rel="#L610">610</span>
<span id="L611" rel="#L611">611</span>
<span id="L612" rel="#L612">612</span>
<span id="L613" rel="#L613">613</span>
<span id="L614" rel="#L614">614</span>
<span id="L615" rel="#L615">615</span>
<span id="L616" rel="#L616">616</span>
<span id="L617" rel="#L617">617</span>
<span id="L618" rel="#L618">618</span>
<span id="L619" rel="#L619">619</span>
<span id="L620" rel="#L620">620</span>
<span id="L621" rel="#L621">621</span>
<span id="L622" rel="#L622">622</span>
<span id="L623" rel="#L623">623</span>
<span id="L624" rel="#L624">624</span>
<span id="L625" rel="#L625">625</span>
<span id="L626" rel="#L626">626</span>
<span id="L627" rel="#L627">627</span>
<span id="L628" rel="#L628">628</span>
<span id="L629" rel="#L629">629</span>
<span id="L630" rel="#L630">630</span>
<span id="L631" rel="#L631">631</span>
<span id="L632" rel="#L632">632</span>
<span id="L633" rel="#L633">633</span>
<span id="L634" rel="#L634">634</span>
<span id="L635" rel="#L635">635</span>
<span id="L636" rel="#L636">636</span>
<span id="L637" rel="#L637">637</span>
<span id="L638" rel="#L638">638</span>
<span id="L639" rel="#L639">639</span>
<span id="L640" rel="#L640">640</span>
<span id="L641" rel="#L641">641</span>
<span id="L642" rel="#L642">642</span>
<span id="L643" rel="#L643">643</span>
<span id="L644" rel="#L644">644</span>
<span id="L645" rel="#L645">645</span>
<span id="L646" rel="#L646">646</span>
<span id="L647" rel="#L647">647</span>
<span id="L648" rel="#L648">648</span>
<span id="L649" rel="#L649">649</span>
<span id="L650" rel="#L650">650</span>
<span id="L651" rel="#L651">651</span>
<span id="L652" rel="#L652">652</span>
<span id="L653" rel="#L653">653</span>
<span id="L654" rel="#L654">654</span>
<span id="L655" rel="#L655">655</span>
<span id="L656" rel="#L656">656</span>
<span id="L657" rel="#L657">657</span>
<span id="L658" rel="#L658">658</span>
<span id="L659" rel="#L659">659</span>
<span id="L660" rel="#L660">660</span>
<span id="L661" rel="#L661">661</span>
<span id="L662" rel="#L662">662</span>
<span id="L663" rel="#L663">663</span>
</pre>
          </td>
          <td width="100%">
                  <div class="highlight"><pre><div class='line' id='LC1'><span class="c1">//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program</span></div><div class='line' id='LC2'><span class="c1">//Samuel Vanderwaal, April 2012</span></div><div class='line' id='LC3'><br/></div><div class='line' id='LC4'><span class="cp">#include &lt;functions.h&gt;</span></div><div class='line' id='LC5'><span class="cp">#include &lt;include.h&gt;</span></div><div class='line' id='LC6'><span class="cp">#include &quot;ARCbus.h&quot;</span></div><div class='line' id='LC7'><br/></div><div class='line' id='LC8'><span class="kt">int</span> <span class="nf">SUB_parseCmd</span><span class="p">(</span><span class="kt">unsigned</span> <span class="kt">char</span> <span class="n">src</span><span class="p">,</span> <span class="kt">unsigned</span> <span class="kt">char</span> <span class="n">cmd</span><span class="p">,</span> <span class="kt">unsigned</span> <span class="kt">char</span> <span class="o">*</span><span class="n">data</span><span class="p">,</span> <span class="kt">unsigned</span> <span class="kt">short</span> <span class="n">len</span><span class="p">)</span></div><div class='line' id='LC9'><span class="p">{</span></div><div class='line' id='LC10'>&nbsp;&nbsp;<span class="k">switch</span><span class="p">(</span><span class="n">cmd</span><span class="p">)</span></div><div class='line' id='LC11'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC12'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC13'>&nbsp;&nbsp;<span class="k">return</span> <span class="n">ERR_UNKNOWN_CMD</span><span class="p">;</span></div><div class='line' id='LC14'><span class="p">}</span></div><div class='line' id='LC15'><br/></div><div class='line' id='LC16'><span class="c1">//Create string of data and of specified length and load into RxBuffer</span></div><div class='line' id='LC17'><span class="kt">void</span> <span class="nf">Build_Packet</span><span class="p">(</span><span class="kt">int</span> <span class="n">data_length</span><span class="p">)</span></div><div class='line' id='LC18'><span class="p">{</span></div><div class='line' id='LC19'><span class="kt">int</span> <span class="n">i</span><span class="p">;</span></div><div class='line' id='LC20'><span class="n">TxBufferPos</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC21'><span class="n">TxBuffer</span><span class="p">[</span><span class="mi">0</span><span class="p">]</span> <span class="o">=</span> <span class="n">data_length</span><span class="p">;</span>                  <span class="c1">// First byte is length of packet</span></div><div class='line' id='LC22'><span class="k">for</span> <span class="p">(</span><span class="n">i</span> <span class="o">=</span> <span class="mi">1</span><span class="p">;</span> <span class="n">i</span> <span class="o">&lt;</span> <span class="n">data_length</span><span class="p">;</span> <span class="n">i</span><span class="o">++</span><span class="p">)</span>      <span class="c1">// Fill up buffer with data</span></div><div class='line' id='LC23'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBuffer</span><span class="p">[</span><span class="n">i</span><span class="p">]</span> <span class="o">=</span> <span class="mh">0x0F</span><span class="p">;</span></div><div class='line' id='LC24'><span class="n">TxBuffer</span><span class="p">[</span><span class="n">data_length</span> <span class="o">-</span> <span class="mi">1</span><span class="p">]</span> <span class="o">=</span> <span class="mh">0xF0</span><span class="p">;</span></div><div class='line' id='LC25'><span class="p">}</span></div><div class='line' id='LC26'><br/></div><div class='line' id='LC27'><span class="cm">/*void TI_CC_Wait(unsigned int cycles)</span></div><div class='line' id='LC28'><span class="cm">{</span></div><div class='line' id='LC29'><span class="cm">  /*while(cycles&gt;15)                          // 15 cycles consumed by overhead</span></div><div class='line' id='LC30'><span class="cm">    cycles = cycles - 6;                    // 6 cycles consumed each iteration</span></div><div class='line' id='LC31'><span class="cm">  __delay_cycles(16*cycles);</span></div><div class='line' id='LC32'><span class="cm">}*/</span></div><div class='line' id='LC33'><br/></div><div class='line' id='LC34'><span class="cp">#define TI_CC_Wait(c) (__delay_cycles(16*c))</span></div><div class='line' id='LC35'><br/></div><div class='line' id='LC36'><span class="kt">void</span> <span class="nf">radio_interrupts</span><span class="p">(</span><span class="kt">void</span><span class="p">)</span></div><div class='line' id='LC37'><span class="p">{</span></div><div class='line' id='LC38'><span class="c1">//Enable interrupts on Port 2</span></div><div class='line' id='LC39'><span class="c1">//2.0 -- GDO0  CC1101</span></div><div class='line' id='LC40'><span class="c1">//2.1 -- GDO2  CC1101</span></div><div class='line' id='LC41'><span class="c1">//2.3 -- GDO0  CC2500</span></div><div class='line' id='LC42'><span class="c1">//2.4 -- GDO2  CC2500</span></div><div class='line' id='LC43'><span class="n">P2DIR</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">CC1101_GDO0</span> <span class="o">+</span> <span class="n">CC1101_GDO2</span> <span class="o">+</span> <span class="n">CC2500_GDO0</span> <span class="o">+</span> <span class="n">CC2500_GDO2</span><span class="p">);</span></div><div class='line' id='LC44'><span class="n">P2IE</span> <span class="o">|=</span> <span class="n">CC1101_GDO0</span> <span class="o">+</span> <span class="n">CC1101_GDO2</span> <span class="o">+</span> <span class="n">CC2500_GDO0</span> <span class="o">+</span> <span class="n">CC2500_GDO2</span><span class="p">;</span>             <span class="c1">// Enable ints  </span></div><div class='line' id='LC45'><span class="n">P2IES</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">CC1101_GDO0</span> <span class="o">+</span> <span class="n">CC1101_GDO2</span> <span class="o">+</span> <span class="n">CC2500_GDO0</span> <span class="o">+</span> <span class="n">CC2500_GDO2</span><span class="p">);</span>         <span class="c1">// Int on rising edge: GDO0 interrupts when receives sync word</span></div><div class='line' id='LC46'><span class="n">P2IFG</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                                                 <span class="c1">// Clear flags</span></div><div class='line' id='LC47'><br/></div><div class='line' id='LC48'><span class="c1">//Set up amplifier switches, RF_SW1 for 430 MHz amplifier, RF_SW2 for 2.4 GHz amplifier</span></div><div class='line' id='LC49'><span class="c1">//P6DIR |= RF_SW1 + RF_SW2;                                                  // Set amplifier switches as outputs</span></div><div class='line' id='LC50'><span class="c1">//P6OUT |= (RF_SW1 + RF_SW2);                                               // Disable both amplifiers </span></div><div class='line' id='LC51'><span class="p">}</span></div><div class='line' id='LC52'><br/></div><div class='line' id='LC53'><br/></div><div class='line' id='LC54'><br/></div><div class='line' id='LC55'><span class="c1">//Set up the UCB1 SPI channel for writing to the radios</span></div><div class='line' id='LC56'><span class="kt">void</span> <span class="nf">SPI_Setup</span><span class="p">(</span><span class="kt">void</span><span class="p">)</span></div><div class='line' id='LC57'><span class="p">{</span></div><div class='line' id='LC58'>&nbsp;&nbsp;<span class="n">UCB1CTL1</span> <span class="o">=</span> <span class="n">UCSWRST</span><span class="p">;</span>                                          </div><div class='line' id='LC59'>&nbsp;&nbsp;<span class="n">UCB1CTL0</span> <span class="o">=</span> <span class="n">UCCKPH</span> <span class="o">+</span> <span class="n">UCMSB</span> <span class="o">+</span> <span class="n">UCMST</span> <span class="o">+</span> <span class="n">UCMODE_0</span> <span class="o">+</span> <span class="n">UCSYNC</span><span class="p">;</span></div><div class='line' id='LC60'>&nbsp;&nbsp;<span class="n">UCB1CTL1</span> <span class="o">|=</span> <span class="n">UCSSEL_2</span><span class="p">;</span></div><div class='line' id='LC61'>&nbsp;&nbsp;<span class="n">UCB1BR0</span> <span class="o">=</span> <span class="mi">16</span><span class="p">;</span>      <span class="c1">//Set frequency divider so SPI runs at 16/16 = 1 MHz</span></div><div class='line' id='LC62'>&nbsp;&nbsp;<span class="n">UCB1BR1</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC63'><br/></div><div class='line' id='LC64'>&nbsp;&nbsp;<span class="c1">//UC1IE |= UCB1TXIE + UCB1RXIE;</span></div><div class='line' id='LC65'><br/></div><div class='line' id='LC66'>&nbsp;&nbsp;<span class="n">P5SEL</span> <span class="o">|=</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span>               <span class="c1">//Select special sources for Port 5.1--MOSI, 5.2--MISO, 5.3--CLK</span></div><div class='line' id='LC67'>&nbsp;&nbsp;<span class="n">P5DIR</span> <span class="o">|=</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span>                      <span class="c1">//Set outputs: MOSI and CLK</span></div><div class='line' id='LC68'>&nbsp;&nbsp;<span class="n">P5DIR</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                          <span class="c1">//Set output for CC1101 CS</span></div><div class='line' id='LC69'>&nbsp;&nbsp;<span class="n">P5DIR</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                          <span class="c1">//Set output for CC2500 CS</span></div><div class='line' id='LC70'><br/></div><div class='line' id='LC71'>&nbsp;&nbsp;<span class="n">P5REN</span> <span class="o">|=</span> <span class="n">BIT2</span><span class="p">;</span>                         <span class="c1">// Enable pull-up resistor for MISO</span></div><div class='line' id='LC72'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">BIT2</span><span class="p">;</span>                         <span class="c1">// Set as pull-up</span></div><div class='line' id='LC73'><br/></div><div class='line' id='LC74'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                     <span class="c1">//Ensure CS for CC1101 is disabled</span></div><div class='line' id='LC75'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                     <span class="c1">//Ensure CS for CC2500 is disabled </span></div><div class='line' id='LC76'>&nbsp;&nbsp;<span class="n">UCB1CTL1</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">UCSWRST</span><span class="p">;</span>                 <span class="c1">//Enable UART state machine</span></div><div class='line' id='LC77'><span class="p">}</span></div><div class='line' id='LC78'><br/></div><div class='line' id='LC79'><span class="kt">void</span> <span class="nf">Reset_Radios</span><span class="p">(</span><span class="kt">void</span><span class="p">)</span></div><div class='line' id='LC80'><span class="p">{</span></div><div class='line' id='LC81'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                   <span class="c1">//Toggle CS with delays to power up radio</span></div><div class='line' id='LC82'>&nbsp;&nbsp;<span class="n">TI_CC_Wait</span><span class="p">(</span><span class="mi">30</span><span class="p">);</span></div><div class='line' id='LC83'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span></div><div class='line' id='LC84'>&nbsp;&nbsp;<span class="n">TI_CC_Wait</span><span class="p">(</span><span class="mi">30</span><span class="p">);</span></div><div class='line' id='LC85'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span></div><div class='line' id='LC86'>&nbsp;&nbsp;<span class="n">TI_CC_Wait</span><span class="p">(</span><span class="mi">45</span><span class="p">);</span></div><div class='line' id='LC87'><br/></div><div class='line' id='LC88'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                           <span class="c1">// CS enable</span></div><div class='line' id='LC89'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>               <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC90'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">TI_CCxxx0_SRES</span><span class="p">;</span>               <span class="c1">// Send strobe</span></div><div class='line' id='LC91'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">// Strobe addr is now being TX&#39;ed</span></div><div class='line' id='LC92'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC93'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                           <span class="c1">// CS disable</span></div><div class='line' id='LC94'>&nbsp;&nbsp;</div><div class='line' id='LC95'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                   <span class="c1">//Toggle CS with delays to power up radio</span></div><div class='line' id='LC96'>&nbsp;&nbsp;<span class="n">TI_CC_Wait</span><span class="p">(</span><span class="mi">30</span><span class="p">);</span></div><div class='line' id='LC97'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span></div><div class='line' id='LC98'>&nbsp;&nbsp;<span class="n">TI_CC_Wait</span><span class="p">(</span><span class="mi">30</span><span class="p">);</span></div><div class='line' id='LC99'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span></div><div class='line' id='LC100'>&nbsp;&nbsp;<span class="n">TI_CC_Wait</span><span class="p">(</span><span class="mi">45</span><span class="p">);</span></div><div class='line' id='LC101'><br/></div><div class='line' id='LC102'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span>                           <span class="c1">// CS enable</span></div><div class='line' id='LC103'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>               <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC104'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">TI_CCxxx0_SRES</span><span class="p">;</span>               <span class="c1">// Send strobe</span></div><div class='line' id='LC105'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">// Strobe addr is now being TX&#39;ed</span></div><div class='line' id='LC106'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC107'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                           <span class="c1">// CS disable</span></div><div class='line' id='LC108'><br/></div><div class='line' id='LC109'>&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                <span class="c1">// Clear flags that were set</span></div><div class='line' id='LC110'><span class="p">}</span></div><div class='line' id='LC111'><br/></div><div class='line' id='LC112'><span class="c1">//Function that sends single address to radio initiating a state or mode change </span></div><div class='line' id='LC113'><span class="c1">//(e.g. sending addr 0x34 writes to SRX register initiating radio in RX mode</span></div><div class='line' id='LC114'><span class="kt">void</span> <span class="nf">Radio_Strobe</span><span class="p">(</span><span class="kt">char</span> <span class="n">strobe</span><span class="p">,</span> <span class="kt">char</span> <span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC115'><span class="p">{</span></div><div class='line' id='LC116'>&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC117'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                           <span class="c1">// CS enable CC1101</span></div><div class='line' id='LC118'><br/></div><div class='line' id='LC119'>&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC120'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span>                           <span class="c1">// CS enable CC2500</span></div><div class='line' id='LC121'><br/></div><div class='line' id='LC122'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                  <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC123'><br/></div><div class='line' id='LC124'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">strobe</span><span class="p">;</span>                             <span class="c1">// Send strobe</span></div><div class='line' id='LC125'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">// Strobe addr is now being TX&#39;ed</span></div><div class='line' id='LC126'><span class="c1">//  while ((UC1IFG &amp; UCB1RXIFG) == 0);</span></div><div class='line' id='LC127'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                      <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC128'><br/></div><div class='line' id='LC129'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                               <span class="c1">// CS disable C2500 </span></div><div class='line' id='LC130'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                               <span class="c1">// CS disable C1101</span></div><div class='line' id='LC131'>&nbsp;&nbsp;<span class="c1">//P2IFG = 0;</span></div><div class='line' id='LC132'><span class="p">}</span></div><div class='line' id='LC133'><br/></div><div class='line' id='LC134'><span class="c1">//Function to write a single byte to the radio registers</span></div><div class='line' id='LC135'><span class="kt">void</span> <span class="nf">Radio_Write_Registers</span><span class="p">(</span><span class="kt">char</span> <span class="n">addr</span><span class="p">,</span> <span class="kt">char</span> <span class="n">value</span><span class="p">,</span> <span class="kt">char</span> <span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC136'><span class="p">{</span></div><div class='line' id='LC137'>&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC138'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                          <span class="c1">// CS enable CC1101</span></div><div class='line' id='LC139'><br/></div><div class='line' id='LC140'>&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC141'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span>                          <span class="c1">// CS enable CC2500</span></div><div class='line' id='LC142'><br/></div><div class='line' id='LC143'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>               <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC144'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">addr</span><span class="p">;</span>                         <span class="c1">// Send address</span></div><div class='line' id='LC145'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>               <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC146'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">value</span><span class="p">;</span>                        <span class="c1">// Send data</span></div><div class='line' id='LC147'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC148'><br/></div><div class='line' id='LC149'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                            <span class="c1">// CS disable C1101</span></div><div class='line' id='LC150'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                            <span class="c1">// CS disable C2500 </span></div><div class='line' id='LC151'>&nbsp;&nbsp;</div><div class='line' id='LC152'><span class="p">}</span></div><div class='line' id='LC153'><br/></div><div class='line' id='LC154'><span class="c1">//Function to write multiple bytes to the radio registers</span></div><div class='line' id='LC155'><span class="kt">void</span> <span class="nf">Radio_Write_Burst_Registers</span><span class="p">(</span><span class="kt">char</span> <span class="n">addr</span><span class="p">,</span> <span class="kt">char</span> <span class="o">*</span><span class="n">buffer</span><span class="p">,</span> <span class="kt">int</span> <span class="n">count</span><span class="p">,</span> <span class="kt">char</span> <span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC156'><span class="p">{</span></div><div class='line' id='LC157'>&nbsp;&nbsp;<span class="kt">int</span> <span class="n">i</span><span class="p">;</span></div><div class='line' id='LC158'><br/></div><div class='line' id='LC159'>&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC160'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                            <span class="c1">// CS enable CC1101</span></div><div class='line' id='LC161'><br/></div><div class='line' id='LC162'>&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC163'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span>                           <span class="c1">// CS enable CC2500</span></div><div class='line' id='LC164'>&nbsp;&nbsp;</div><div class='line' id='LC165'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                 <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC166'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">addr</span> <span class="o">|</span> <span class="n">TI_CCxxx0_WRITE_BURST</span><span class="p">;</span>      <span class="c1">// Adding 0x40 to address tells radio to perform burst write rather than single byte write</span></div><div class='line' id='LC167'>&nbsp;&nbsp;<span class="k">for</span> <span class="p">(</span><span class="n">i</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span> <span class="n">i</span> <span class="o">&lt;</span> <span class="n">count</span><span class="p">;</span> <span class="n">i</span><span class="o">++</span><span class="p">)</span></div><div class='line' id='LC168'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC169'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>              <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC170'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="n">buffer</span><span class="p">[</span><span class="n">i</span><span class="p">];</span>                      <span class="c1">// Send data</span></div><div class='line' id='LC171'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC172'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC173'>&nbsp;&nbsp;</div><div class='line' id='LC174'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                            <span class="c1">// CS disable C1101</span></div><div class='line' id='LC175'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                            <span class="c1">// CS disable C2500 </span></div><div class='line' id='LC176'><span class="p">}</span></div><div class='line' id='LC177'><br/></div><div class='line' id='LC178'><span class="c1">//Function to read a single byte from the radio registers</span></div><div class='line' id='LC179'><span class="kt">char</span> <span class="nf">Radio_Read_Registers</span><span class="p">(</span><span class="kt">char</span> <span class="n">addr</span><span class="p">,</span> <span class="kt">char</span> <span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC180'><span class="p">{</span></div><div class='line' id='LC181'>&nbsp;&nbsp;<span class="kt">char</span> <span class="n">x</span><span class="p">;</span></div><div class='line' id='LC182'>&nbsp;&nbsp;</div><div class='line' id='LC183'>&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">uhf</span><span class="p">)</span> <span class="p">{</span></div><div class='line' id='LC184'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                          <span class="c1">// CS enable CC1101</span></div><div class='line' id='LC185'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC186'><br/></div><div class='line' id='LC187'>&nbsp;&nbsp;<span class="k">else</span> <span class="p">{</span></div><div class='line' id='LC188'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span>                          <span class="c1">// CS enable CC2500</span></div><div class='line' id='LC189'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC190'><br/></div><div class='line' id='LC191'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                  <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC192'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="p">(</span><span class="n">addr</span> <span class="o">|</span> <span class="n">TI_CCxxx0_READ_SINGLE</span><span class="p">);</span>  <span class="c1">// Adding 0x80 to address tells the radio to read a single byte</span></div><div class='line' id='LC193'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                  <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC194'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                               <span class="c1">// Dummy write so we can read data</span></div><div class='line' id='LC195'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                   <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC196'>&nbsp;&nbsp;<span class="n">x</span> <span class="o">=</span> <span class="n">UCB1RXBUF</span><span class="p">;</span>                               <span class="c1">// Read data</span></div><div class='line' id='LC197'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                            <span class="c1">// CS disable C1101</span></div><div class='line' id='LC198'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                            <span class="c1">// CS disable C2500 </span></div><div class='line' id='LC199'><br/></div><div class='line' id='LC200'>&nbsp;&nbsp;<span class="k">return</span> <span class="n">x</span><span class="p">;</span></div><div class='line' id='LC201'><span class="p">}</span></div><div class='line' id='LC202'><br/></div><div class='line' id='LC203'><span class="c1">//Function to read a multiple bytes from the radio registers</span></div><div class='line' id='LC204'><span class="kt">void</span> <span class="nf">Radio_Read_Burst_Registers</span><span class="p">(</span><span class="kt">char</span> <span class="n">addr</span><span class="p">,</span> <span class="kt">char</span> <span class="o">*</span><span class="n">buffer</span><span class="p">,</span> <span class="kt">int</span> <span class="n">count</span><span class="p">,</span> <span class="kt">char</span> <span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC205'><span class="p">{</span></div><div class='line' id='LC206'>&nbsp;&nbsp;<span class="kt">char</span> <span class="n">i</span><span class="p">;</span></div><div class='line' id='LC207'>&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC208'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                          <span class="c1">// CS enable CC1101</span></div><div class='line' id='LC209'><br/></div><div class='line' id='LC210'>&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC211'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span></div><div class='line' id='LC212'><br/></div><div class='line' id='LC213'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                 <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC214'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="p">(</span><span class="n">addr</span> <span class="o">|</span> <span class="n">TI_CCxxx0_READ_BURST</span><span class="p">);</span>  <span class="c1">// Adding 0xC0 to address tells the radio to read multiple bytes</span></div><div class='line' id='LC215'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                  <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC216'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                              <span class="c1">// Dummy write to read 1st data byte</span></div><div class='line' id='LC217'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">// Addr byte is now being TX&#39;ed, with dummy byte to follow immediately after</span></div><div class='line' id='LC218'>&nbsp;&nbsp;<span class="n">UC1IFG</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">UCB1RXIFG</span><span class="p">;</span>                        <span class="c1">// Clear flag</span></div><div class='line' id='LC219'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1RXIFG</span><span class="p">));</span>                 <span class="c1">// Wait for end of 1st data byte TX</span></div><div class='line' id='LC220'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">// First data byte now in RXBUF</span></div><div class='line' id='LC221'>&nbsp;&nbsp;<span class="k">for</span> <span class="p">(</span><span class="n">i</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span> <span class="n">i</span> <span class="o">&lt;</span> <span class="p">(</span><span class="n">count</span><span class="o">-</span><span class="mi">1</span><span class="p">);</span> <span class="n">i</span><span class="o">++</span><span class="p">)</span></div><div class='line' id='LC222'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC223'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                            <span class="c1">//Initiate next data RX, meanwhile..</span></div><div class='line' id='LC224'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">buffer</span><span class="p">[</span><span class="n">i</span><span class="p">]</span> <span class="o">=</span> <span class="n">UCB1RXBUF</span><span class="p">;</span>                    <span class="c1">// Store data from last data RX</span></div><div class='line' id='LC225'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1RXIFG</span><span class="p">));</span>               <span class="c1">// Wait for RX to finish</span></div><div class='line' id='LC226'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC227'>&nbsp;&nbsp;<span class="n">buffer</span><span class="p">[</span><span class="n">count</span><span class="o">-</span><span class="mi">1</span><span class="p">]</span> <span class="o">=</span> <span class="n">UCB1RXBUF</span><span class="p">;</span>                <span class="c1">// Store last RX byte in buffer</span></div><div class='line' id='LC228'>&nbsp;&nbsp;</div><div class='line' id='LC229'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                            <span class="c1">// CS disable C2500 </span></div><div class='line' id='LC230'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span></div><div class='line' id='LC231'><span class="p">}</span></div><div class='line' id='LC232'><br/></div><div class='line' id='LC233'><span class="kt">char</span> <span class="nf">Radio_Read_Status</span><span class="p">(</span><span class="kt">char</span> <span class="n">addr</span><span class="p">,</span> <span class="kt">char</span> <span class="n">uhf</span><span class="p">)</span></div><div class='line' id='LC234'><span class="p">{</span></div><div class='line' id='LC235'>&nbsp;&nbsp;<span class="kt">char</span> <span class="n">status</span><span class="p">;</span></div><div class='line' id='LC236'><br/></div><div class='line' id='LC237'>&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">uhf</span><span class="p">)</span> <span class="p">{</span></div><div class='line' id='LC238'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_1101</span><span class="p">;</span>                          <span class="c1">// CS enable CC1101</span></div><div class='line' id='LC239'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC240'><br/></div><div class='line' id='LC241'>&nbsp;&nbsp;<span class="k">else</span> <span class="p">{</span></div><div class='line' id='LC242'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">CS_2500</span><span class="p">;</span>                          <span class="c1">// CS enable CC2500</span></div><div class='line' id='LC243'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC244'><br/></div><div class='line' id='LC245'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                 <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC246'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="p">(</span><span class="n">addr</span> <span class="o">|</span> <span class="n">TI_CCxxx0_READ_BURST</span><span class="p">);</span>     <span class="c1">// Send address</span></div><div class='line' id='LC247'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="o">!</span><span class="p">(</span><span class="n">UC1IFG</span> <span class="o">&amp;</span> <span class="n">UCB1TXIFG</span><span class="p">));</span>                 <span class="c1">// Wait for TXBUF ready</span></div><div class='line' id='LC248'>&nbsp;&nbsp;<span class="n">UCB1TXBUF</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                 <span class="c1">// Dummy write so we can read data</span></div><div class='line' id='LC249'>&nbsp;&nbsp;<span class="k">while</span> <span class="p">(</span><span class="n">UCB1STAT</span> <span class="o">&amp;</span> <span class="n">UCBUSY</span><span class="p">);</span>                     <span class="c1">// Wait for TX to complete</span></div><div class='line' id='LC250'>&nbsp;&nbsp;<span class="n">status</span> <span class="o">=</span> <span class="n">UCB1RXBUF</span><span class="p">;</span>                            <span class="c1">// Read data</span></div><div class='line' id='LC251'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_1101</span><span class="p">;</span>                              <span class="c1">// CS disable C1101</span></div><div class='line' id='LC252'>&nbsp;&nbsp;<span class="n">P5OUT</span> <span class="o">|=</span> <span class="n">CS_2500</span><span class="p">;</span>                              <span class="c1">// CS disable C2500 </span></div><div class='line' id='LC253'><br/></div><div class='line' id='LC254'>&nbsp;&nbsp;<span class="k">return</span> <span class="n">status</span><span class="p">;</span></div><div class='line' id='LC255'><span class="p">}</span></div><div class='line' id='LC256'><br/></div><div class='line' id='LC257'><span class="kt">void</span> <span class="n">TXRX</span><span class="p">(</span><span class="kt">void</span> <span class="o">*</span><span class="n">p</span><span class="p">)</span> <span class="n">__toplevel</span> </div><div class='line' id='LC258'><span class="p">{</span></div><div class='line' id='LC259'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="kt">unsigned</span> <span class="kt">int</span> <span class="n">e</span><span class="p">;</span></div><div class='line' id='LC260'><span class="k">while</span><span class="p">(</span><span class="mi">1</span><span class="p">)</span></div><div class='line' id='LC261'>&nbsp;&nbsp;<span class="p">{</span>  </div><div class='line' id='LC262'>&nbsp;&nbsp;<span class="n">e</span> <span class="o">=</span> <span class="n">ctl_events_wait</span><span class="p">(</span><span class="n">CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR</span><span class="p">,</span><span class="o">&amp;</span><span class="n">radio_event_flags</span><span class="p">,</span><span class="n">RADIO_EVENTS</span><span class="p">,</span><span class="n">CTL_TIMEOUT_NONE</span><span class="p">,</span><span class="mi">0</span><span class="p">);</span></div><div class='line' id='LC263'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</div><div class='line' id='LC264'>&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC1101_EV_RX_SYNC</span><span class="p">)</span></div><div class='line' id='LC265'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC266'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//RxFIFOLen = (Radio_Read_Status(TI_CCxxx0_RXBYTES, CC1101) &amp; TI_CCxxx0_NUM_RXBYTES);</span></div><div class='line' id='LC267'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//printf(&quot;RX Sync %i \r\n&quot;,RxFIFOLen);</span></div><div class='line' id='LC268'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">PktLen</span> <span class="o">=</span> <span class="mi">255</span><span class="p">;</span> <span class="c1">//Radio_Read_Registers(TI_CCxxx0_RXFIFO, CC1101);       // Read length byte</span></div><div class='line' id='LC269'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">=</span> <span class="n">PktLen</span><span class="p">;</span>    <span class="c1">// Set number of bytes left to receive</span></div><div class='line' id='LC270'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//if (RxFIFOLen &gt; 0){</span></div><div class='line' id='LC271'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">// Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer+RxBufferPos, RxFIFOLen, CC1101);</span></div><div class='line' id='LC272'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//RxBufferPos += RxFIFOLen;</span></div><div class='line' id='LC273'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//RxBytesRemaining -= RxFIFOLen;</span></div><div class='line' id='LC274'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//}</span></div><div class='line' id='LC275'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Disable Port 2 interrupts</span></div><div class='line' id='LC276'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IES</span> <span class="o">|=</span> <span class="p">(</span><span class="n">BIT0</span><span class="p">);</span>                                                            <span class="c1">// Change edge interrupt happens on</span></div><div class='line' id='LC277'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">BIT0</span><span class="p">;</span>                                                                         <span class="c1">// Clear flags</span></div><div class='line' id='LC278'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">|=</span> <span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span>                                                 <span class="c1">// Enable Port 2 interrupts</span></div><div class='line' id='LC279'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC280'>&nbsp;&nbsp;</div><div class='line' id='LC281'>&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC1101_EV_RX_THR</span><span class="p">)</span></div><div class='line' id='LC282'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC283'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">RxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">0</span><span class="p">)</span></div><div class='line' id='LC284'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC285'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">RxBytesRemaining</span> <span class="o">&gt;</span> <span class="n">RxThrBytes</span><span class="p">)</span></div><div class='line' id='LC286'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC287'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</div><div class='line' id='LC288'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">RxThrBytes</span><span class="p">;</span></div><div class='line' id='LC289'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">=</span> <span class="n">RxBytesRemaining</span> <span class="o">-</span> <span class="n">RxThrBytes</span><span class="p">;</span></div><div class='line' id='LC290'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">temp_count1</span><span class="o">++</span><span class="p">;</span></div><div class='line' id='LC291'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC292'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">else</span> </div><div class='line' id='LC293'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span>   </div><div class='line' id='LC294'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">RxBytesRemaining</span><span class="p">;</span></div><div class='line' id='LC295'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC296'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC297'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Read_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_RXFIFO</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="o">+</span><span class="n">RxBufferPos</span><span class="p">,</span> <span class="n">count</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span></div><div class='line' id='LC298'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBufferPos</span> <span class="o">+=</span> <span class="n">count</span><span class="p">;</span></div><div class='line' id='LC299'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC300'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">RxBytesRemaining</span> <span class="o">==</span> <span class="mi">0</span><span class="p">)</span></div><div class='line' id='LC301'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC302'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">state</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC303'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P7OUT</span> <span class="o">^=</span> <span class="n">BIT1</span><span class="p">;</span></div><div class='line' id='LC304'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;receiving packet</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC305'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC306'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">for</span> <span class="p">(</span><span class="n">k</span><span class="o">=</span><span class="mi">0</span><span class="p">;</span> <span class="n">k</span> <span class="o">&lt;</span> <span class="n">PktLen</span><span class="p">;</span> <span class="n">k</span><span class="o">++</span><span class="p">)</span></div><div class='line' id='LC307'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC308'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;%d &quot;</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="p">[</span><span class="n">k</span><span class="p">]);</span></div><div class='line' id='LC309'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC310'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC311'><br/></div><div class='line' id='LC312'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC313'>&nbsp;<span class="p">}</span></div><div class='line' id='LC314'>&nbsp;</div><div class='line' id='LC315'>&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC1101_EV_RX_END</span><span class="p">)</span></div><div class='line' id='LC316'>&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC317'>&nbsp;&nbsp;&nbsp;<span class="n">RxFIFOLen</span> <span class="o">=</span> <span class="p">(</span><span class="n">Radio_Read_Status</span><span class="p">(</span><span class="n">TI_CCxxx0_RXBYTES</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">)</span> <span class="o">&amp;</span> <span class="n">TI_CCxxx0_NUM_RXBYTES</span><span class="p">);</span></div><div class='line' id='LC318'>&nbsp;&nbsp;&nbsp;<span class="n">Radio_Read_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_RXFIFO</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="o">+</span><span class="n">RxBufferPos</span><span class="p">,</span> <span class="n">RxFIFOLen</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span></div><div class='line' id='LC319'>&nbsp;&nbsp;&nbsp;<span class="n">P7OUT</span> <span class="o">^=</span> <span class="n">BIT1</span><span class="p">;</span></div><div class='line' id='LC320'>&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;Receiving packet on CC1101</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC321'>&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC322'>&nbsp;&nbsp;&nbsp;<span class="k">for</span> <span class="p">(</span><span class="n">k</span><span class="o">=</span><span class="mi">0</span><span class="p">;</span> <span class="n">k</span> <span class="o">&lt;</span> <span class="n">PktLen</span><span class="p">;</span> <span class="n">k</span><span class="o">++</span><span class="p">)</span></div><div class='line' id='LC323'>&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC324'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;%3d &quot;</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="p">[</span><span class="n">k</span><span class="p">]);</span></div><div class='line' id='LC325'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">k</span> <span class="o">%</span> <span class="mi">20</span> <span class="o">==</span> <span class="mi">19</span><span class="p">)</span></div><div class='line' id='LC326'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC327'>&nbsp;&nbsp;&nbsp;<span class="p">}</span>        </div><div class='line' id='LC328'>&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC329'>&nbsp;&nbsp;&nbsp;<span class="n">state</span> <span class="o">=</span> <span class="n">IDLE</span><span class="p">;</span></div><div class='line' id='LC330'>&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Disable Port 2 interrupts</span></div><div class='line' id='LC331'>&nbsp;&nbsp;&nbsp;<span class="n">P2IES</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span><span class="p">);</span>                                                            <span class="c1">// Change edge interrupt happens on</span></div><div class='line' id='LC332'>&nbsp;&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="n">BIT0</span><span class="p">;</span>                                                                         <span class="c1">// Clear flags</span></div><div class='line' id='LC333'>&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">|=</span> <span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span>                                                 <span class="c1">// Enable Port 2 interrupts</span></div><div class='line' id='LC334'>&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC335'>&nbsp;</div><div class='line' id='LC336'>&nbsp;</div><div class='line' id='LC337'>&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC1101_EV_TX_START</span><span class="p">)</span></div><div class='line' id='LC338'>&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC339'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">state</span> <span class="o">=</span> <span class="n">TX_START</span><span class="p">;</span></div><div class='line' id='LC340'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Disable Port 2 interrupts</span></div><div class='line' id='LC341'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG2</span><span class="p">,</span> <span class="mh">0x02</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span>                                <span class="c1">// Set GDO2 to interrupt on FIFO thresholds</span></div><div class='line' id='LC342'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IES</span> <span class="o">|=</span> <span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Change edge interrupt happens on</span></div><div class='line' id='LC343'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                                                         <span class="c1">// Clear flags</span></div><div class='line' id='LC344'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">|=</span> <span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span>                                                 <span class="c1">// Enable Port 2 interrupts</span></div><div class='line' id='LC345'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//Build_Packet(data_length);</span></div><div class='line' id='LC346'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">64</span><span class="p">)</span></div><div class='line' id='LC347'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC348'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="mi">64</span><span class="p">;</span></div><div class='line' id='LC349'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span> <span class="o">-</span> <span class="mi">64</span><span class="p">;</span></div><div class='line' id='LC350'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC351'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC352'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC353'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span><span class="p">;</span></div><div class='line' id='LC354'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC355'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC356'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TXFIFO</span><span class="p">,</span> <span class="n">TxBuffer</span><span class="o">+</span><span class="n">TxBufferPos</span><span class="p">,</span> <span class="n">count</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span>     <span class="c1">// Write TX data</span></div><div class='line' id='LC357'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBufferPos</span> <span class="o">+=</span> <span class="n">count</span><span class="p">;</span></div><div class='line' id='LC358'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Strobe</span><span class="p">(</span><span class="n">TI_CCxxx0_STX</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span>                                                  <span class="c1">// Set radio state to Tx </span></div><div class='line' id='LC359'>&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC360'>&nbsp;</div><div class='line' id='LC361'>&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC1101_EV_TX_THR</span><span class="p">)</span></div><div class='line' id='LC362'>&nbsp;<span class="p">{</span></div><div class='line' id='LC363'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">0</span><span class="p">)</span> </div><div class='line' id='LC364'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC365'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">&gt;</span> <span class="n">TxThrBytes</span><span class="p">)</span></div><div class='line' id='LC366'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC367'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">TxThrBytes</span><span class="p">;</span></div><div class='line' id='LC368'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span> <span class="o">-</span> <span class="n">TxThrBytes</span><span class="p">;</span></div><div class='line' id='LC369'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC370'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC371'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC372'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span><span class="p">;</span></div><div class='line' id='LC373'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC374'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC375'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TXFIFO</span><span class="p">,</span> <span class="n">TxBuffer</span> <span class="o">+</span> <span class="n">TxBufferPos</span><span class="p">,</span> <span class="n">count</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span></div><div class='line' id='LC376'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBufferPos</span> <span class="o">+=</span> <span class="n">count</span><span class="p">;</span></div><div class='line' id='LC377'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;State: %x &quot;</span><span class="p">,</span> <span class="n">Radio_Read_Status</span><span class="p">(</span><span class="n">TI_CCxxx0_MARCSTATE</span><span class="p">,</span><span class="n">CC1101</span><span class="p">));</span></div><div class='line' id='LC378'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC379'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">==</span> <span class="mi">0</span><span class="p">)</span></div><div class='line' id='LC380'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC381'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">state</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC382'><br/></div><div class='line' id='LC383'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P7OUT</span> <span class="o">^=</span> <span class="n">BIT0</span><span class="p">;</span></div><div class='line' id='LC384'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">temp_count1</span><span class="o">++</span><span class="p">;</span></div><div class='line' id='LC385'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;%d&quot;</span><span class="p">,</span> <span class="n">temp_count1</span><span class="p">);</span></div><div class='line' id='LC386'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot; packet(s) sent </span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC387'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;CC1101 </span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC388'><span class="c1">//                  printf(&quot;packet sent \r\n&quot;);</span></div><div class='line' id='LC389'><span class="c1">//                  for (k=0; k &lt; TxBufferPos; k++)</span></div><div class='line' id='LC390'><span class="c1">//                 {</span></div><div class='line' id='LC391'><span class="c1">//                     printf(&quot;%d &quot;, TxBuffer[k]);</span></div><div class='line' id='LC392'><span class="c1">//                     printf(&quot; &quot;);</span></div><div class='line' id='LC393'><span class="c1">//                 }</span></div><div class='line' id='LC394'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</div><div class='line' id='LC395'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Disable Port 2 interrupts</span></div><div class='line' id='LC396'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG2</span><span class="p">,</span> <span class="mh">0x00</span><span class="p">,</span> <span class="n">CC1101</span><span class="p">);</span>                                <span class="c1">// Set GDO2 to interrupt on FIFO thresholds</span></div><div class='line' id='LC397'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IES</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Change edge interrupt happens on</span></div><div class='line' id='LC398'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                                                         <span class="c1">// Clear flags</span></div><div class='line' id='LC399'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">|=</span> <span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span> </div><div class='line' id='LC400'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC401'>&nbsp;<span class="p">}</span></div><div class='line' id='LC402'>&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC2500_EV_RX_SYNC</span><span class="p">)</span></div><div class='line' id='LC403'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC404'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxFIFOLen</span> <span class="o">=</span> <span class="p">(</span><span class="n">Radio_Read_Status</span><span class="p">(</span><span class="n">TI_CCxxx0_RXBYTES</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">)</span> <span class="o">&amp;</span> <span class="n">TI_CCxxx0_NUM_RXBYTES</span><span class="p">);</span></div><div class='line' id='LC405'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">PktLen</span> <span class="o">=</span> <span class="mi">255</span><span class="p">;</span> <span class="c1">//Radio_Read_Registers(TI_CCxxx0_RXFIFO, CC1101);       // Read length byte</span></div><div class='line' id='LC406'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">=</span> <span class="n">PktLen</span><span class="p">;</span>                                  <span class="c1">// Set number of bytes left to receive</span></div><div class='line' id='LC407'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Read_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_RXFIFO</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="o">+</span><span class="n">RxBufferPos</span><span class="p">,</span> <span class="n">RxFIFOLen</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span></div><div class='line' id='LC408'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBufferPos</span> <span class="o">+=</span> <span class="n">RxFIFOLen</span><span class="p">;</span></div><div class='line' id='LC409'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">-=</span> <span class="n">RxFIFOLen</span><span class="p">;</span></div><div class='line' id='LC410'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC411'>&nbsp;&nbsp;</div><div class='line' id='LC412'>&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC2500_EV_RX_THR</span><span class="p">)</span></div><div class='line' id='LC413'>&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC414'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">RxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">0</span><span class="p">)</span></div><div class='line' id='LC415'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC416'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">RxBytesRemaining</span> <span class="o">&gt;</span> <span class="n">RxThrBytes</span><span class="p">)</span></div><div class='line' id='LC417'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC418'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">RxThrBytes</span><span class="p">;</span></div><div class='line' id='LC419'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">=</span> <span class="n">RxBytesRemaining</span> <span class="o">-</span> <span class="n">RxThrBytes</span><span class="p">;</span></div><div class='line' id='LC420'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">temp_count1</span><span class="o">++</span><span class="p">;</span></div><div class='line' id='LC421'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC422'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">else</span> </div><div class='line' id='LC423'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC424'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">RxBytesRemaining</span><span class="p">;</span></div><div class='line' id='LC425'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBytesRemaining</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC426'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC427'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Read_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_RXFIFO</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="o">+</span><span class="n">RxBufferPos</span><span class="p">,</span> <span class="n">count</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span></div><div class='line' id='LC428'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">RxBufferPos</span> <span class="o">+=</span> <span class="n">count</span><span class="p">;</span></div><div class='line' id='LC429'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC430'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">RxBytesRemaining</span> <span class="o">==</span> <span class="mi">0</span><span class="p">)</span></div><div class='line' id='LC431'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC432'><br/></div><div class='line' id='LC433'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">state</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC434'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P7OUT</span> <span class="o">^=</span> <span class="n">BIT5</span><span class="p">;</span></div><div class='line' id='LC435'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;receiving packet</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC436'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC437'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">for</span> <span class="p">(</span><span class="n">k</span><span class="o">=</span><span class="mi">0</span><span class="p">;</span> <span class="n">k</span> <span class="o">&lt;</span> <span class="n">PktLen</span><span class="p">;</span> <span class="n">k</span><span class="o">++</span><span class="p">)</span></div><div class='line' id='LC438'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC439'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;%d &quot;</span><span class="p">,</span> <span class="n">RxBuffer</span><span class="p">[</span><span class="n">k</span><span class="p">]);</span></div><div class='line' id='LC440'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC441'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC442'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC443'>&nbsp;<span class="p">}</span></div><div class='line' id='LC444'>&nbsp;</div><div class='line' id='LC445'>&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC2500_EV_TX_START</span><span class="p">)</span></div><div class='line' id='LC446'>&nbsp;<span class="p">{</span></div><div class='line' id='LC447'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Disable Port 2 interrupts</span></div><div class='line' id='LC448'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG2</span><span class="p">,</span> <span class="mh">0x02</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span>                                <span class="c1">// Set GDO2 to interrupt on FIFO thresholds</span></div><div class='line' id='LC449'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IES</span> <span class="o">|=</span> <span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Change edge interrupt happens on</span></div><div class='line' id='LC450'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                                                         <span class="c1">// Clear flags</span></div><div class='line' id='LC451'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">|=</span> <span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span>                                                 <span class="c1">// Enable Port 2 interrupts</span></div><div class='line' id='LC452'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Build_Packet</span><span class="p">(</span><span class="n">data_length</span><span class="p">);</span></div><div class='line' id='LC453'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">64</span><span class="p">)</span></div><div class='line' id='LC454'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC455'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="mi">64</span><span class="p">;</span></div><div class='line' id='LC456'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span> <span class="o">-</span> <span class="mi">64</span><span class="p">;</span></div><div class='line' id='LC457'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC458'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC459'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC460'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span><span class="p">;</span></div><div class='line' id='LC461'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC462'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC463'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TXFIFO</span><span class="p">,</span> <span class="n">TxBuffer</span><span class="o">+</span><span class="n">TxBufferPos</span><span class="p">,</span> <span class="n">count</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span>     <span class="c1">// Write TX data</span></div><div class='line' id='LC464'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBufferPos</span> <span class="o">+=</span> <span class="n">count</span><span class="p">;</span></div><div class='line' id='LC465'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Strobe</span><span class="p">(</span><span class="n">TI_CCxxx0_STX</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span>                                                  <span class="c1">// Set radio state to Tx </span></div><div class='line' id='LC466'><span class="p">}</span></div><div class='line' id='LC467'>&nbsp;</div><div class='line' id='LC468'>&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span> <span class="o">&amp;</span> <span class="n">CC2500_EV_TX_THR</span><span class="p">)</span></div><div class='line' id='LC469'>&nbsp;<span class="p">{</span></div><div class='line' id='LC470'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">0</span><span class="p">)</span> </div><div class='line' id='LC471'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC472'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">&gt;</span> <span class="mi">30</span><span class="p">)</span></div><div class='line' id='LC473'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC474'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="mi">30</span><span class="p">;</span></div><div class='line' id='LC475'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span> <span class="o">-</span> <span class="mi">30</span><span class="p">;</span></div><div class='line' id='LC476'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC477'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">else</span></div><div class='line' id='LC478'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC479'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">count</span> <span class="o">=</span> <span class="n">TxBytesRemaining</span><span class="p">;</span></div><div class='line' id='LC480'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC481'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC482'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Burst_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TXFIFO</span><span class="p">,</span> <span class="n">TxBuffer</span> <span class="o">+</span> <span class="n">TxBufferPos</span><span class="p">,</span> <span class="n">count</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span></div><div class='line' id='LC483'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBufferPos</span> <span class="o">+=</span> <span class="n">count</span><span class="p">;</span></div><div class='line' id='LC484'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC485'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span> <span class="p">(</span><span class="n">TxBytesRemaining</span> <span class="o">==</span> <span class="mi">0</span><span class="p">)</span></div><div class='line' id='LC486'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC487'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">state</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span></div><div class='line' id='LC488'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P7OUT</span> <span class="o">^=</span> <span class="n">BIT4</span><span class="p">;</span></div><div class='line' id='LC489'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">temp_count1</span><span class="o">++</span><span class="p">;</span></div><div class='line' id='LC490'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;%d&quot;</span><span class="p">,</span> <span class="n">temp_count1</span><span class="p">);</span></div><div class='line' id='LC491'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot; packet(s) sent </span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC492'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;CC2500 </span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC493'><span class="c1">//                  printf(&quot;packet sent \r\n&quot;);</span></div><div class='line' id='LC494'><span class="c1">//                  for (k=0; k &lt; TxBufferPos; k++)</span></div><div class='line' id='LC495'><span class="c1">//                 {</span></div><div class='line' id='LC496'><span class="c1">//                     printf(&quot;%d &quot;, TxBuffer[k]);</span></div><div class='line' id='LC497'><span class="c1">//                     printf(&quot; &quot;);</span></div><div class='line' id='LC498'><span class="c1">//                 } </span></div><div class='line' id='LC499'><br/></div><div class='line' id='LC500'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Disable Port 2 interrupts</span></div><div class='line' id='LC501'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG2</span><span class="p">,</span> <span class="mh">0x00</span><span class="p">,</span> <span class="n">CC2500</span><span class="p">);</span>                                <span class="c1">// Set GDO2 to interrupt on FIFO thresholds</span></div><div class='line' id='LC502'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IES</span> <span class="o">&amp;=</span> <span class="o">~</span><span class="p">(</span><span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">);</span>                                              <span class="c1">// Change edge interrupt happens on</span></div><div class='line' id='LC503'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IFG</span> <span class="o">=</span> <span class="mi">0</span><span class="p">;</span>                                                                         <span class="c1">// Clear flags</span></div><div class='line' id='LC504'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">P2IE</span> <span class="o">|=</span> <span class="n">BIT0</span> <span class="o">+</span> <span class="n">BIT1</span> <span class="o">+</span> <span class="n">BIT2</span> <span class="o">+</span> <span class="n">BIT3</span><span class="p">;</span> </div><div class='line' id='LC505'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC506'>&nbsp;<span class="p">}</span></div><div class='line' id='LC507'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC508'><span class="p">}</span></div><div class='line' id='LC509'><br/></div><div class='line' id='LC510'><span class="kt">void</span> <span class="n">sub_events</span><span class="p">(</span><span class="kt">void</span> <span class="o">*</span><span class="n">p</span><span class="p">)</span> <span class="n">__toplevel</span><span class="p">{</span></div><div class='line' id='LC511'>&nbsp;&nbsp;<span class="kt">unsigned</span> <span class="kt">int</span> <span class="n">e</span><span class="p">,</span><span class="n">len</span><span class="p">;</span></div><div class='line' id='LC512'>&nbsp;&nbsp;<span class="kt">int</span> <span class="n">i</span><span class="p">;</span></div><div class='line' id='LC513'>&nbsp;&nbsp;<span class="kt">unsigned</span> <span class="kt">char</span> <span class="n">buf</span><span class="p">[</span><span class="mi">10</span><span class="p">],</span><span class="o">*</span><span class="n">ptr</span><span class="p">;</span></div><div class='line' id='LC514'>&nbsp;&nbsp;<span class="k">for</span><span class="p">(;;){</span></div><div class='line' id='LC515'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">e</span><span class="o">=</span><span class="n">ctl_events_wait</span><span class="p">(</span><span class="n">CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR</span><span class="p">,</span><span class="o">&amp;</span><span class="n">SUB_events</span><span class="p">,</span><span class="n">SUB_EV_ALL</span><span class="p">,</span><span class="n">CTL_TIMEOUT_NONE</span><span class="p">,</span><span class="mi">0</span><span class="p">);</span></div><div class='line' id='LC516'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span><span class="o">&amp;</span><span class="n">SUB_EV_PWR_OFF</span><span class="p">){</span></div><div class='line' id='LC517'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//print message</span></div><div class='line' id='LC518'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">puts</span><span class="p">(</span><span class="s">&quot;System Powering Down</span><span class="se">\r</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC519'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC520'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span><span class="o">&amp;</span><span class="n">SUB_EV_PWR_ON</span><span class="p">){</span></div><div class='line' id='LC521'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//print message</span></div><div class='line' id='LC522'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">puts</span><span class="p">(</span><span class="s">&quot;System Powering Up</span><span class="se">\r</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC523'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC524'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span><span class="o">&amp;</span><span class="n">SUB_EV_SEND_STAT</span><span class="p">){</span></div><div class='line' id='LC525'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//send status</span></div><div class='line' id='LC526'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//puts(&quot;Sending status\r&quot;);</span></div><div class='line' id='LC527'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//setup packet </span></div><div class='line' id='LC528'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//TODO: put actual command for subsystem response</span></div><div class='line' id='LC529'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">ptr</span><span class="o">=</span><span class="n">BUS_cmd_init</span><span class="p">(</span><span class="n">buf</span><span class="p">,</span><span class="mi">20</span><span class="p">);</span></div><div class='line' id='LC530'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//TODO: fill in telemitry data</span></div><div class='line' id='LC531'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//send command</span></div><div class='line' id='LC532'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">BUS_cmd_tx</span><span class="p">(</span><span class="n">BUS_ADDR_CDH</span><span class="p">,</span><span class="n">buf</span><span class="p">,</span><span class="mi">0</span><span class="p">,</span><span class="mi">0</span><span class="p">,</span><span class="n">SEND_FOREGROUND</span><span class="p">);</span></div><div class='line' id='LC533'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC534'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span><span class="o">&amp;</span><span class="n">SUB_EV_TIME_CHECK</span><span class="p">){</span></div><div class='line' id='LC535'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;time ticker = %li</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">,</span><span class="n">get_ticker_time</span><span class="p">());</span></div><div class='line' id='LC536'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC537'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span><span class="o">&amp;</span><span class="n">SUB_EV_SPI_DAT</span><span class="p">){</span></div><div class='line' id='LC538'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">puts</span><span class="p">(</span><span class="s">&quot;SPI data recived:</span><span class="se">\r</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC539'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//get length</span></div><div class='line' id='LC540'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">len</span><span class="o">=</span><span class="n">arcBus_stat</span><span class="p">.</span><span class="n">spi_stat</span><span class="p">.</span><span class="n">len</span><span class="p">;</span></div><div class='line' id='LC541'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//print out data</span></div><div class='line' id='LC542'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">for</span><span class="p">(</span><span class="n">i</span><span class="o">=</span><span class="mi">0</span><span class="p">;</span><span class="n">i</span><span class="o">&lt;</span><span class="n">len</span><span class="p">;</span><span class="n">i</span><span class="o">++</span><span class="p">){</span></div><div class='line' id='LC543'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="c1">//printf(&quot;0x%02X &quot;,rx[i]);</span></div><div class='line' id='LC544'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;%03i &quot;</span><span class="p">,</span><span class="n">arcBus_stat</span><span class="p">.</span><span class="n">spi_stat</span><span class="p">.</span><span class="n">rx</span><span class="p">[</span><span class="n">i</span><span class="p">]);</span></div><div class='line' id='LC545'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC546'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">printf</span><span class="p">(</span><span class="s">&quot;</span><span class="se">\r\n</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC547'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</div><div class='line' id='LC548'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBuffer</span><span class="p">[</span><span class="mi">0</span><span class="p">]</span> <span class="o">=</span> <span class="n">len</span><span class="o">+</span><span class="mi">1</span><span class="p">;</span> </div><div class='line' id='LC549'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">for</span> <span class="p">(</span><span class="n">i</span><span class="o">=</span><span class="mi">0</span><span class="p">;</span> <span class="n">i</span> <span class="o">&lt;</span> <span class="n">len</span><span class="p">;</span> <span class="n">i</span><span class="o">++</span><span class="p">)</span></div><div class='line' id='LC550'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">{</span></div><div class='line' id='LC551'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBuffer</span><span class="p">[</span><span class="n">i</span><span class="o">+</span><span class="mi">1</span><span class="p">]</span> <span class="o">=</span> <span class="n">arcBus_stat</span><span class="p">.</span><span class="n">spi_stat</span><span class="p">.</span><span class="n">rx</span><span class="p">[</span><span class="n">i</span><span class="p">];</span></div><div class='line' id='LC552'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC553'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">TxBytesRemaining</span> <span class="o">=</span> <span class="n">len</span><span class="o">+</span><span class="mi">1</span><span class="p">;</span></div><div class='line' id='LC554'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">ctl_events_set_clear</span><span class="p">(</span><span class="o">&amp;</span><span class="n">radio_event_flags</span><span class="p">,</span><span class="n">CC1101_EV_TX_START</span><span class="p">,</span><span class="mi">0</span><span class="p">);</span></div><div class='line' id='LC555'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC556'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="k">if</span><span class="p">(</span><span class="n">e</span><span class="o">&amp;</span><span class="n">SUB_EV_SPI_ERR_CRC</span><span class="p">){</span></div><div class='line' id='LC557'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span class="n">puts</span><span class="p">(</span><span class="s">&quot;SPI bad CRC</span><span class="se">\r</span><span class="s">&quot;</span><span class="p">);</span></div><div class='line' id='LC558'>&nbsp;&nbsp;&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC559'>&nbsp;&nbsp;<span class="p">}</span></div><div class='line' id='LC560'><span class="p">}</span></div><div class='line' id='LC561'><br/></div><div class='line' id='LC562'><span class="kt">void</span> <span class="n">Write_RF_Settings</span><span class="p">(</span><span class="kt">void</span><span class="p">)</span></div><div class='line' id='LC563'><span class="p">{</span></div><div class='line' id='LC564'><span class="c1">// CC1101</span></div><div class='line' id='LC565'><span class="c1">// Deviation = 5.157471 </span></div><div class='line' id='LC566'><span class="c1">// Base frequency = 432.999817 </span></div><div class='line' id='LC567'><span class="c1">// Carrier frequency = 432.999817 </span></div><div class='line' id='LC568'><span class="c1">// Channel number = 0 </span></div><div class='line' id='LC569'><span class="c1">// Carrier frequency = 432.999817 </span></div><div class='line' id='LC570'><span class="c1">// Modulated = true </span></div><div class='line' id='LC571'><span class="c1">// Modulation format = GFSK </span></div><div class='line' id='LC572'><span class="c1">// Manchester enable = false </span></div><div class='line' id='LC573'><span class="c1">// Sync word qualifier mode = 30/32 sync word bits detected </span></div><div class='line' id='LC574'><span class="c1">// Preamble count = 4 </span></div><div class='line' id='LC575'><span class="c1">// Channel spacing = 199.951172 kHz</span></div><div class='line' id='LC576'><span class="c1">// Carrier frequency = 432.999817 MHz</span></div><div class='line' id='LC577'><span class="c1">// Data rate = 9.59587 kbps</span></div><div class='line' id='LC578'><span class="c1">// RX filter BW = 58.035714 </span></div><div class='line' id='LC579'><span class="c1">// Data format = Normal mode </span></div><div class='line' id='LC580'><span class="c1">// Length config = Variable packet length mode. Packet length configured by the first byte after sync word </span></div><div class='line' id='LC581'><span class="c1">// CRC enable = true </span></div><div class='line' id='LC582'><span class="c1">// Packet length = 255 </span></div><div class='line' id='LC583'><span class="c1">// Device address = 0 </span></div><div class='line' id='LC584'><span class="c1">// Address config = No address check </span></div><div class='line' id='LC585'><span class="c1">// CRC autoflush = false </span></div><div class='line' id='LC586'><span class="c1">// PA ramping = false </span></div><div class='line' id='LC587'><span class="c1">// TX power = 0</span></div><div class='line' id='LC588'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG0</span><span class="p">,</span>   <span class="mh">0x06</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span>   <span class="c1">// Asserts when sync word has been sent / received, and de-asserts at the end of the packet. </span></div><div class='line' id='LC589'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG2</span><span class="p">,</span>   <span class="mh">0x00</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span>   <span class="c1">// Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold.</span></div><div class='line' id='LC590'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FIFOTHR</span><span class="p">,</span>  <span class="mh">0x07</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span>   <span class="c1">// FIFO Threshold</span></div><div class='line' id='LC591'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCTRL0</span><span class="p">,</span>  <span class="mh">0x00</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC592'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCTRL1</span><span class="p">,</span>  <span class="mh">0x06</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC593'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREQ2</span><span class="p">,</span>    <span class="mh">0x10</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC594'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREQ1</span><span class="p">,</span>    <span class="mh">0xC4</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC595'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREQ0</span><span class="p">,</span>    <span class="mh">0xEC</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC596'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG4</span><span class="p">,</span>  <span class="mh">0xF8</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC597'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG3</span><span class="p">,</span>  <span class="mh">0x83</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC598'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG2</span><span class="p">,</span>  <span class="mh">0x03</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span> <span class="c1">// High byte: 0000 is 2-FSK and 0001 is GFSK; Low byte: 0000 no preamble/sync, 0001 15/16 sync words detected, 0011 30/32 sync words</span></div><div class='line' id='LC599'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG1</span><span class="p">,</span>  <span class="mh">0x22</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span> <span class="c1">// High byte: 0010 is 4 bytes of preamble, 0100 is 8 bytes of preamble</span></div><div class='line' id='LC600'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG0</span><span class="p">,</span>  <span class="mh">0xF8</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC601'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_CHANNR</span><span class="p">,</span>   <span class="mh">0x00</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC602'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_DEVIATN</span><span class="p">,</span>  <span class="mh">0x15</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC603'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREND1</span><span class="p">,</span>   <span class="mh">0x56</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC604'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREND0</span><span class="p">,</span>   <span class="mh">0x10</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC605'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MCSM0</span><span class="p">,</span>    <span class="mh">0x18</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC606'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MCSM1</span><span class="p">,</span>    <span class="mh">0x3F</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC607'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FOCCFG</span><span class="p">,</span>   <span class="mh">0x16</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC608'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_BSCFG</span><span class="p">,</span>    <span class="mh">0x6C</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC609'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_AGCCTRL2</span><span class="p">,</span> <span class="mh">0x03</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC610'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_AGCCTRL1</span><span class="p">,</span> <span class="mh">0x40</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC611'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_AGCCTRL0</span><span class="p">,</span> <span class="mh">0x91</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC612'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL3</span><span class="p">,</span>   <span class="mh">0xE9</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC613'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL2</span><span class="p">,</span>   <span class="mh">0x2A</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC614'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL1</span><span class="p">,</span>   <span class="mh">0x00</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC615'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL0</span><span class="p">,</span>   <span class="mh">0x1F</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC616'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSTEST</span><span class="p">,</span>   <span class="mh">0x59</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC617'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TEST2</span><span class="p">,</span>    <span class="mh">0x81</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC618'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TEST1</span><span class="p">,</span>    <span class="mh">0x35</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC619'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TEST0</span><span class="p">,</span>    <span class="mh">0x09</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC620'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_PKTCTRL1</span><span class="p">,</span> <span class="mh">0x04</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span>     <span class="c1">//Status bytes appended (RSSI and LQI values, CRC ok) but no address check</span></div><div class='line' id='LC621'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_PKTCTRL0</span><span class="p">,</span> <span class="mh">0x05</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span>     <span class="c1">// Variable packet length mode; packet length set by first byte after sync word</span></div><div class='line' id='LC622'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_ADDR</span><span class="p">,</span>     <span class="mh">0x00</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span></div><div class='line' id='LC623'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_PKTLEN</span><span class="p">,</span>   <span class="mh">0xFF</span><span class="p">,</span> <span class="mi">1</span><span class="p">);</span>      <span class="c1">// Packet length set for variable packet length mode (PKTCTL0 = 0x05) so this doesn&#39;t matter</span></div><div class='line' id='LC624'><br/></div><div class='line' id='LC625'><br/></div><div class='line' id='LC626'><span class="c1">// Write CC2500 register settings</span></div><div class='line' id='LC627'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG0</span><span class="p">,</span>   <span class="mh">0x06</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// GDO0 output pin config.</span></div><div class='line' id='LC628'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_IOCFG2</span><span class="p">,</span>   <span class="mh">0x00</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// GDO2 output pin config.</span></div><div class='line' id='LC629'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FIFOTHR</span><span class="p">,</span>  <span class="mh">0x0F</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// FIFO Threshold: 1 byte in TX FIFO and 63 in RX FIFO</span></div><div class='line' id='LC630'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_PKTLEN</span><span class="p">,</span>   <span class="mh">0x08</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Packet length.</span></div><div class='line' id='LC631'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_PKTCTRL1</span><span class="p">,</span> <span class="mh">0x04</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Packet automation control.</span></div><div class='line' id='LC632'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_PKTCTRL0</span><span class="p">,</span> <span class="mh">0x05</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Packet automation control.</span></div><div class='line' id='LC633'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_ADDR</span><span class="p">,</span>     <span class="mh">0x01</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Device address.</span></div><div class='line' id='LC634'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_CHANNR</span><span class="p">,</span>   <span class="mh">0x00</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Channel number.</span></div><div class='line' id='LC635'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCTRL1</span><span class="p">,</span>  <span class="mh">0x07</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Freq synthesizer control.</span></div><div class='line' id='LC636'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCTRL0</span><span class="p">,</span>  <span class="mh">0x00</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Freq synthesizer control.</span></div><div class='line' id='LC637'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREQ2</span><span class="p">,</span>    <span class="mh">0x5D</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Freq control word, high byte</span></div><div class='line' id='LC638'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREQ1</span><span class="p">,</span>    <span class="mh">0x44</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Freq control word, mid byte.</span></div><div class='line' id='LC639'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREQ0</span><span class="p">,</span>    <span class="mh">0xEC</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Freq control word, low byte.</span></div><div class='line' id='LC640'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG4</span><span class="p">,</span>  <span class="mh">0x2D</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Modem configuration.</span></div><div class='line' id='LC641'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG3</span><span class="p">,</span>  <span class="mh">0x3B</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Modem configuration.</span></div><div class='line' id='LC642'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG2</span><span class="p">,</span>  <span class="mh">0x73</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Modem configuration.</span></div><div class='line' id='LC643'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG1</span><span class="p">,</span>  <span class="mh">0x22</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Modem configuration.</span></div><div class='line' id='LC644'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MDMCFG0</span><span class="p">,</span>  <span class="mh">0xF8</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Modem configuration.</span></div><div class='line' id='LC645'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_DEVIATN</span><span class="p">,</span>  <span class="mh">0x00</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Modem dev (when FSK mod en)</span></div><div class='line' id='LC646'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MCSM1</span> <span class="p">,</span>   <span class="mh">0x3F</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Main Radio Cntrl State Machine</span></div><div class='line' id='LC647'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_MCSM0</span> <span class="p">,</span>   <span class="mh">0x18</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Main Radio Cntrl State Machine</span></div><div class='line' id='LC648'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FOCCFG</span><span class="p">,</span>   <span class="mh">0x1D</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Freq Offset Compens. Config</span></div><div class='line' id='LC649'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_BSCFG</span><span class="p">,</span>    <span class="mh">0x1C</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">//  Bit synchronization config.</span></div><div class='line' id='LC650'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_AGCCTRL2</span><span class="p">,</span> <span class="mh">0xC7</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// AGC control.</span></div><div class='line' id='LC651'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_AGCCTRL1</span><span class="p">,</span> <span class="mh">0x00</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// AGC control.</span></div><div class='line' id='LC652'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_AGCCTRL0</span><span class="p">,</span> <span class="mh">0xB2</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// AGC control.</span></div><div class='line' id='LC653'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREND1</span><span class="p">,</span>   <span class="mh">0xB6</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Front end RX configuration.</span></div><div class='line' id='LC654'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FREND0</span><span class="p">,</span>   <span class="mh">0x10</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Front end RX configuration.</span></div><div class='line' id='LC655'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL3</span><span class="p">,</span>   <span class="mh">0xEA</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Frequency synthesizer cal.</span></div><div class='line' id='LC656'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL2</span><span class="p">,</span>   <span class="mh">0x0A</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Frequency synthesizer cal.</span></div><div class='line' id='LC657'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL1</span><span class="p">,</span>   <span class="mh">0x00</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Frequency synthesizer cal.</span></div><div class='line' id='LC658'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSCAL0</span><span class="p">,</span>   <span class="mh">0x11</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Frequency synthesizer cal.</span></div><div class='line' id='LC659'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_FSTEST</span><span class="p">,</span>   <span class="mh">0x59</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Frequency synthesizer cal.</span></div><div class='line' id='LC660'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TEST2</span><span class="p">,</span>    <span class="mh">0x88</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Various test settings.</span></div><div class='line' id='LC661'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TEST1</span><span class="p">,</span>    <span class="mh">0x31</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Various test settings.</span></div><div class='line' id='LC662'><span class="n">Radio_Write_Registers</span><span class="p">(</span><span class="n">TI_CCxxx0_TEST0</span><span class="p">,</span>    <span class="mh">0x0B</span><span class="p">,</span> <span class="mi">0</span><span class="p">);</span>  <span class="c1">// Various test settings.    </span></div><div class='line' id='LC663'><span class="p">}</span></div></pre></div>
          </td>
        </tr>
      </table>
  </div>

          </div>
        </div>

        <a href="#jump-to-line" rel="facebox" data-hotkey="l" class="js-jump-to-line" style="display:none">Jump to Line</a>
        <div id="jump-to-line" style="display:none">
          <h2>Jump to Line</h2>
          <form accept-charset="UTF-8" class="js-jump-to-line-form">
            <input class="textfield js-jump-to-line-field" type="text">
            <div class="full-button">
              <button type="submit" class="button">Go</button>
            </div>
          </form>
        </div>

      </div>
    </div>
</div>

<div id="js-frame-loading-template" class="frame frame-loading large-loading-area" style="display:none;">
  <img class="js-frame-loading-spinner" src="https://a248.e.akamai.net/assets.github.com/images/spinners/octocat-spinner-128.gif?1347543527" height="64" width="64">
</div>


        </div>
      </div>
      <div class="context-overlay"></div>
    </div>

      <div id="footer-push"></div><!-- hack for sticky footer -->
    </div><!-- end of wrapper - hack for sticky footer -->

      <!-- footer -->
      <div id="footer">
  <div class="container clearfix">

      <dl class="footer_nav">
        <dt>GitHub</dt>
        <dd><a href="https://github.com/about">About us</a></dd>
        <dd><a href="https://github.com/blog">Blog</a></dd>
        <dd><a href="https://github.com/contact">Contact &amp; support</a></dd>
        <dd><a href="http://enterprise.github.com/">GitHub Enterprise</a></dd>
        <dd><a href="http://status.github.com/">Site status</a></dd>
      </dl>

      <dl class="footer_nav">
        <dt>Applications</dt>
        <dd><a href="http://mac.github.com/">GitHub for Mac</a></dd>
        <dd><a href="http://windows.github.com/">GitHub for Windows</a></dd>
        <dd><a href="http://eclipse.github.com/">GitHub for Eclipse</a></dd>
        <dd><a href="http://mobile.github.com/">GitHub mobile apps</a></dd>
      </dl>

      <dl class="footer_nav">
        <dt>Services</dt>
        <dd><a href="http://get.gaug.es/">Gauges: Web analytics</a></dd>
        <dd><a href="http://speakerdeck.com">Speaker Deck: Presentations</a></dd>
        <dd><a href="https://gist.github.com">Gist: Code snippets</a></dd>
        <dd><a href="http://jobs.github.com/">Job board</a></dd>
      </dl>

      <dl class="footer_nav">
        <dt>Documentation</dt>
        <dd><a href="http://help.github.com/">GitHub Help</a></dd>
        <dd><a href="http://developer.github.com/">Developer API</a></dd>
        <dd><a href="http://github.github.com/github-flavored-markdown/">GitHub Flavored Markdown</a></dd>
        <dd><a href="http://pages.github.com/">GitHub Pages</a></dd>
      </dl>

      <dl class="footer_nav">
        <dt>More</dt>
        <dd><a href="http://training.github.com/">Training</a></dd>
        <dd><a href="https://github.com/edu">Students &amp; teachers</a></dd>
        <dd><a href="http://shop.github.com">The Shop</a></dd>
        <dd><a href="/plans">Plans &amp; pricing</a></dd>
        <dd><a href="http://octodex.github.com/">The Octodex</a></dd>
      </dl>

      <hr class="footer-divider">


    <p class="right">&copy; 2013 <span title="0.11047s from fe18.rs.github.com">GitHub</span> Inc. All rights reserved.</p>
    <a class="left" href="https://github.com/">
      <span class="mega-icon mega-icon-invertocat"></span>
    </a>
    <ul id="legal">
        <li><a href="https://github.com/site/terms">Terms of Service</a></li>
        <li><a href="https://github.com/site/privacy">Privacy</a></li>
        <li><a href="https://github.com/security">Security</a></li>
    </ul>

  </div><!-- /.container -->

</div><!-- /.#footer -->


    

    

<div id="keyboard_shortcuts_pane" class="instapaper_ignore readability-extra" style="display:none">
  <h2>Keyboard Shortcuts <small><a href="#" class="js-see-all-keyboard-shortcuts">(see all)</a></small></h2>

  <div class="columns threecols">
    <div class="column first">
      <h3>Site wide shortcuts</h3>
      <dl class="keyboard-mappings">
        <dt>s</dt>
        <dd>Focus command bar</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>?</dt>
        <dd>Bring up this help dialog</dd>
      </dl>
    </div><!-- /.column.first -->

    <div class="column middle" style='display:none'>
      <h3>Commit list</h3>
      <dl class="keyboard-mappings">
        <dt>j</dt>
        <dd>Move selection down</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>k</dt>
        <dd>Move selection up</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>c <em>or</em> o <em>or</em> enter</dt>
        <dd>Open commit</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>y</dt>
        <dd>Expand URL to its canonical form</dd>
      </dl>
    </div><!-- /.column.first -->

    <div class="column last js-hidden-pane" style='display:none'>
      <h3>Pull request list</h3>
      <dl class="keyboard-mappings">
        <dt>j</dt>
        <dd>Move selection down</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>k</dt>
        <dd>Move selection up</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>o <em>or</em> enter</dt>
        <dd>Open issue</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt><span class="platform-mac">⌘</span><span class="platform-other">ctrl</span> <em>+</em> enter</dt>
        <dd>Submit comment</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt><span class="platform-mac">⌘</span><span class="platform-other">ctrl</span> <em>+</em> shift p</dt>
        <dd>Preview comment</dd>
      </dl>
    </div><!-- /.columns.last -->

  </div><!-- /.columns.equacols -->

  <div class="js-hidden-pane" style='display:none'>
    <div class="rule"></div>

    <h3>Issues</h3>

    <div class="columns threecols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt>j</dt>
          <dd>Move selection down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>k</dt>
          <dd>Move selection up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>x</dt>
          <dd>Toggle selection</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>o <em>or</em> enter</dt>
          <dd>Open issue</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="platform-mac">⌘</span><span class="platform-other">ctrl</span> <em>+</em> enter</dt>
          <dd>Submit comment</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="platform-mac">⌘</span><span class="platform-other">ctrl</span> <em>+</em> shift p</dt>
          <dd>Preview comment</dd>
        </dl>
      </div><!-- /.column.first -->
      <div class="column last">
        <dl class="keyboard-mappings">
          <dt>c</dt>
          <dd>Create issue</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>l</dt>
          <dd>Create label</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>i</dt>
          <dd>Back to inbox</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>u</dt>
          <dd>Back to issues</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>/</dt>
          <dd>Focus issues search</dd>
        </dl>
      </div>
    </div>
  </div>

  <div class="js-hidden-pane" style='display:none'>
    <div class="rule"></div>

    <h3>Issues Dashboard</h3>

    <div class="columns threecols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt>j</dt>
          <dd>Move selection down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>k</dt>
          <dd>Move selection up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>o <em>or</em> enter</dt>
          <dd>Open issue</dd>
        </dl>
      </div><!-- /.column.first -->
    </div>
  </div>

  <div class="js-hidden-pane" style='display:none'>
    <div class="rule"></div>

    <h3>Network Graph</h3>
    <div class="columns equacols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt><span class="badmono">←</span> <em>or</em> h</dt>
          <dd>Scroll left</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="badmono">→</span> <em>or</em> l</dt>
          <dd>Scroll right</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="badmono">↑</span> <em>or</em> k</dt>
          <dd>Scroll up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="badmono">↓</span> <em>or</em> j</dt>
          <dd>Scroll down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>t</dt>
          <dd>Toggle visibility of head labels</dd>
        </dl>
      </div><!-- /.column.first -->
      <div class="column last">
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">←</span> <em>or</em> shift h</dt>
          <dd>Scroll all the way left</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">→</span> <em>or</em> shift l</dt>
          <dd>Scroll all the way right</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">↑</span> <em>or</em> shift k</dt>
          <dd>Scroll all the way up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">↓</span> <em>or</em> shift j</dt>
          <dd>Scroll all the way down</dd>
        </dl>
      </div><!-- /.column.last -->
    </div>
  </div>

  <div class="js-hidden-pane" >
    <div class="rule"></div>
    <div class="columns threecols">
      <div class="column first js-hidden-pane" >
        <h3>Source Code Browsing</h3>
        <dl class="keyboard-mappings">
          <dt>t</dt>
          <dd>Activates the file finder</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>l</dt>
          <dd>Jump to line</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>w</dt>
          <dd>Switch branch/tag</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>y</dt>
          <dd>Expand URL to its canonical form</dd>
        </dl>
      </div>
    </div>
  </div>

  <div class="js-hidden-pane" style='display:none'>
    <div class="rule"></div>
    <div class="columns threecols">
      <div class="column first">
        <h3>Browsing Commits</h3>
        <dl class="keyboard-mappings">
          <dt><span class="platform-mac">⌘</span><span class="platform-other">ctrl</span> <em>+</em> enter</dt>
          <dd>Submit comment</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>escape</dt>
          <dd>Close form</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>p</dt>
          <dd>Parent commit</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>o</dt>
          <dd>Other parent commit</dd>
        </dl>
      </div>
    </div>
  </div>

  <div class="js-hidden-pane" style='display:none'>
    <div class="rule"></div>
    <h3>Notifications</h3>

    <div class="columns threecols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt>j</dt>
          <dd>Move selection down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>k</dt>
          <dd>Move selection up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>o <em>or</em> enter</dt>
          <dd>Open notification</dd>
        </dl>
      </div><!-- /.column.first -->

      <div class="column second">
        <dl class="keyboard-mappings">
          <dt>e <em>or</em> shift i <em>or</em> y</dt>
          <dd>Mark as read</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift m</dt>
          <dd>Mute thread</dd>
        </dl>
      </div><!-- /.column.first -->
    </div>
  </div>

</div>

    <div id="markdown-help" class="instapaper_ignore readability-extra">
  <h2>Markdown Cheat Sheet</h2>

  <div class="cheatsheet-content">

  <div class="mod">
    <div class="col">
      <h3>Format Text</h3>
      <p>Headers</p>
      <pre>
# This is an &lt;h1&gt; tag
## This is an &lt;h2&gt; tag
###### This is an &lt;h6&gt; tag</pre>
     <p>Text styles</p>
     <pre>
*This text will be italic*
_This will also be italic_
**This text will be bold**
__This will also be bold__

*You **can** combine them*
</pre>
    </div>
    <div class="col">
      <h3>Lists</h3>
      <p>Unordered</p>
      <pre>
* Item 1
* Item 2
  * Item 2a
  * Item 2b</pre>
     <p>Ordered</p>
     <pre>
1. Item 1
2. Item 2
3. Item 3
   * Item 3a
   * Item 3b</pre>
    </div>
    <div class="col">
      <h3>Miscellaneous</h3>
      <p>Images</p>
      <pre>
![GitHub Logo](/images/logo.png)
Format: ![Alt Text](url)
</pre>
     <p>Links</p>
     <pre>
http://github.com - automatic!
[GitHub](http://github.com)</pre>
<p>Blockquotes</p>
     <pre>
As Kanye West said:

> We're living the future so
> the present is our past.
</pre>
    </div>
  </div>
  <div class="rule"></div>

  <h3>Code Examples in Markdown</h3>
  <div class="col">
      <p>Syntax highlighting with <a href="http://github.github.com/github-flavored-markdown/" title="GitHub Flavored Markdown" target="_blank">GFM</a></p>
      <pre>
```javascript
function fancyAlert(arg) {
  if(arg) {
    $.facebox({div:'#foo'})
  }
}
```</pre>
    </div>
    <div class="col">
      <p>Or, indent your code 4 spaces</p>
      <pre>
Here is a Python code example
without syntax highlighting:

    def foo:
      if not bar:
        return true</pre>
    </div>
    <div class="col">
      <p>Inline code for comments</p>
      <pre>
I think you should use an
`&lt;addr&gt;` element here instead.</pre>
    </div>
  </div>

  </div>
</div>


    <div id="ajax-error-message" class="flash flash-error">
      <span class="mini-icon mini-icon-exclamation"></span>
      Something went wrong with that request. Please try again.
      <a href="#" class="mini-icon mini-icon-remove-close ajax-error-dismiss"></a>
    </div>

    
    
    <span id='server_response_time' data-time='0.11200' data-host='fe18'></span>
    
  </body>
</html>

>>>>>>> Computer code merger
