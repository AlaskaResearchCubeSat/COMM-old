//Code for engineering model Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, August 2012

#include <include.h>
#define CC1101  1
#define CC2500  0

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                                               // Stop watchdog timer
  
  initialize_clk();
  
  initialize_timerA();
  
  radio_interrupts();
  
  P5DIR |= BIT6;
  P5SEL |= BIT6;

  P7DIR |= BIT0 + BIT1 + BIT2 + BIT3;

  P7OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);

  SPI_Setup();                                                                     //Setup SPI for communication with CC1101 and CC2500 (Port 3)

  Reset_Radios();                                                                  // Power up both radios
  Write_RF_Settings();                                                             // Write RF settings to config reg
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, 1);   //Write PATABLE for CC1101
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC2500, paTableLen, 0);   //Write PATABLE for CC2500

  initUART();                                                                      //Initialize UART

  Radio_Strobe(TI_CCxxx0_SRX, CC1101);           // Initialize CC1101 in RX mode
  Radio_Strobe(TI_CCxxx0_SRX, CC2500);           // Initialize CC2500 in Rx mode

  data_length = 60;
  packet_length = data_length + 1;

  _EINT();

  while (1)
  {
  LPM0;
  }
}

//Interrupt service routine for Timer A
#pragma vector=TIMERA1_VECTOR
__interrupt void TA_ISR (void)
{
  int d, i;
  P7OUT ^= BIT1;
  Build_Packet(data_length);   //Generate fake data
   
  //RF_Send_Packet(TxBuffer, packet_length, CC1101);

/*
  printf("\r\n");
  printf("%x\r\n", Radio_Read_Status(TI_CCxxx0_MARCSTATE, CC1101));
  printf("\r\n");
  printf("Packet Sent: ");
  printf("\r\n");
  */
//  for (i=0; i<packet_length; i++)
//   {
//     printf("%d ", TxBuffer[i]);
//     printf(" "); 
//   }
//     printf("\r\n");
     d = TAIV;                   // Read TAIV to reset interrupt flags
}

#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR (void)
{
P7OUT ^= BIT2|BIT3;
}
