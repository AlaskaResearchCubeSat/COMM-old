//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, APril 2012

#include <include.h>

/*
void initCLK(void){
  //set XT1 load caps, do this first so XT1 starts up sooner
  BCSCTL3=XCAP_0;
  //stop watchdog
  WDTCTL = WDTPW|WDTHOLD;
  //setup clocks

  //set DCO to 16MHz from calibrated values
  DCOCTL=0;
  BCSCTL1=CALBC1_16MHZ;
  DCOCTL=CALDCO_16MHZ;
}
*/

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                                                // Stop watchdog timer

  initUART();                               // Initialize UART

  SPI_Setup();                                                                      //Setup SPI for communication with CC1101 and CC2500 (Port 3)

  Reset_Radios();                                                                   // Power up both radios
  Write_RF_Settings();                                                              // Write RF settings to config reg
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, 1);    // Write PATABLE for CC1101
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC2500, paTableLen, 0);    // Write PATABLE for CC2500

  DCOCTL = 0; 								  // Set DCO to lowest tap to protect it from getting set too high when RSELx is set
  BCSCTL1 = CALBC1_16MHZ;				                  // Pull calibration data for 16MHz and set RSELx to appropriate tap range
  DCOCTL = CALDCO_16MHZ;				                  // Pull calibration data for 
  BCSCTL2 |= SELM_0;							  // Selects DCO as source for MCLK and SMCLK
 
  P7DIR |= BIT0;              //Set up LEDs 1-8 for testing
  P7DIR |= BIT1;
  P7DIR |= BIT2;
  P7DIR |= BIT3;
  P7DIR |= BIT4;
  P7DIR |= BIT5;
  P7DIR |= BIT6;
  P7DIR |= BIT7;            

  P7OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6);
  P7OUT &= ~BIT7;
  
  //Set up ports 1.3, 1.5, and 1.7 as buttons

  P1DIR &= ~(BIT3 + BIT5 + BIT7);   //Set P1.3, 1.5, and 1.7 as Inputs
  P1REN |= BIT3 + BIT5 + BIT7;      //Enable Pull-up/pull-down Resistors for Port 1.3, 1.5, and 1.7
  P1OUT |= BIT3 + BIT5 + BIT7;      //Set Resistors as pull-up

  P1IES |= BIT3 + BIT5 + BIT7;      //Interrupts set on high-low transition
  P1IE |= BIT3 + BIT5 + BIT7;       //Enable interrupts for Port 1.3,5,7

  P1IFG &= ~(BIT3 + BIT5 + BIT7);   //Clear interrupt flags

  //Enable interrupts on Port 2.0 - 2.3:
  //2.0 -- GDO0  CC1101
  //2.1 -- GDO2  CC1101
  //2.2 -- GDO0  CC2500
  //2.3 -- GDO2  CC2500
  P2DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                      
  P2IES &= ~(BIT0 + BIT1 + BIT2 + BIT3);    // Low-to-high transition for interrupts
  P2IE |= BIT0 + BIT1 + BIT2 + BIT3;        // Enable ints
  P2IFG = 0;                                // Clear flags


  //Setup RF switch 1 and 2
 // P6DIR |= BIT0 + BIT1;

  //initCLK();
  
  Radio_Strobe(TI_CCxxx0_SRX, CC1101);           // Initialize CC1101 in RX mode
  Radio_Strobe(TI_CCxxx0_SRX, CC2500);           // Initialize CC2500 in Rx mode

  P7OUT = 0;

  data_length = 100;
 // RxBufferLen = sizeof(RxBuffer);          // Length of packet to be received  This does not work for some reason

  RxBufferLen = 255;
  temp_count1 = 0;
  temp_count2 = 0;
  P2IFG = 0;     // Clear flags
  _EINT();

  while (1){
  LPM0;

  switch (state){
      case TX_START:
          P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
          Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x02, uhf);                                // Set GDO2 to interrupt on FIFO thresholds
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
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer+TxBufferPos, count, uhf);     // Write TX data
          TxBufferPos += count;
          Radio_Strobe(TI_CCxxx0_STX, uhf);                                                  // Set radio state to Tx 
          break;

      case TX_RUNNING:
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
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer + TxBufferPos, count, uhf);
             TxBufferPos += count;
          }
          if (TxBytesRemaining == 0)
          {
              state = 0;
              if (uhf)
              {
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
              }
              if (!(uhf))
              {
                  P7OUT ^= BIT4;
                  temp_count2++;
                  printf("%d", temp_count2);
                  printf(" packet(s) sent \r\n");
                  printf("CC2500 \r\n");
//                  printf("packet sent \r\n");
//                  for (k=0; k < TxBufferPos; k++)
//                 {
//                     printf("%d ", TxBuffer[k]);
//                     printf(" ");
//                 }
              }
              P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Disable Port 2 interrupts
              Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x00, uhf);                                // Set GDO2 to interrupt on FIFO thresholds
              P2IES &= ~(BIT0 + BIT1 + BIT2 + BIT3);                                              // Change edge interrupt happens on
              P2IFG = 0;                                                                         // Clear flags
              P2IE |= BIT0 + BIT1 + BIT2 + BIT3; 
          }

          break;

      case RX_START:
          P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3);                       // Disable Port 2 interrupts
          Radio_Write_Registers(TI_CCxxx0_IOCFG2, 0x00, uhf);
          P2IES &= ~(BIT0 + BIT1 + BIT2 + BIT3);                      // Change edge interrupt happens on
          P2IFG = 0;                                                  // Clear flags
          P2IE |= BIT0 + BIT1 + BIT2 + BIT3;                          // Enable Port 2 interrupts
          if (RF_Receive_Packet(RxBuffer, &RxBufferLen, uhf))        // Get packet from CC1101
          RxBufferPos += RxFIFOLen;
          RxBytesRemaining -= RxFIFOLen;
//          if (RxBytesRemaining == 0)
//          {
//               state = 0;
//               P7OUT ^= BIT1;  
//               printf("receiving packet");
//               for (k=0; k < RxBufferLen; k++)
//               {
//                   printf("%d ", RxBuffer[k]);
//                   printf("\r\n");
//               }
//          }
          break;

      case RX_RUNNING:
             Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxBuffer + RxBufferPos, RxFIFOLen, uhf); // Pull data
             RxBufferPos += RxFIFOLen;
             RxBytesRemaining -= RxFIFOLen;
             if (RxBytesRemaining == 0)
             {
                 state = 0;
                 P7OUT ^= BIT1;  
//                 printf("receiving packet");
//                 for (k=0; k < RxBufferLen; k++)
//                 {
//                     printf("%d ", RxBuffer[k]);
//                     printf("\r\n");
//                 }
             }                         
          break;
      
      default:
          break;
  }
  }
}

#pragma vector=PORT1_VECTOR
__interrupt void Buttons (void)
{	                  
  TxBytesRemaining = data_length + 1;
  
  button = P1IFG;  //store the state of the P2 interrupt 
                   //register so we know which button
                   //was pressed
  P1IFG  = 0;      //delete the IFG register so no other 
                   //interrupt fires
  P1IE   = 0;      //disable button interrupts so none 
                   //of the bounces can refire the interrupt

  WDTCTL = WDT_ADLY_250;  //turn on the WDT and have it fire its
                          //interrupt after 250 ms.  You can change
                          //this to 64 and 32 ms if desired
  IE1 |= WDTIE;           //enable WDT interrupt
}                        
#pragma vector=WDT_VECTOR
__interrupt void WATCHDOG (void)
{

  LPM0_EXIT;
  IFG1 &= ~WDTIFG;  //clear WDT interrupt flag
  IE1 &= ~WDTIE;    //disable WDT interrupt
  WDTCTL = WDTPW + WDTHOLD;  //turn WDT off

  switch(button){   //button holds the info about which pin interrupted
    case BIT3:      //change the cases to support your hardware configuration
         uhf = 0;
         data_length = 10;
         Build_Packet(data_length);
         Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer, data_length, uhf);    // Write TX data
         Radio_Strobe(TI_CCxxx0_STX, uhf);                                      // Change state to TX, initiating data transfer
         P7OUT ^= BIT3;
         printf("Small packet sent. \r\n");
         printf("%x\r\n", Radio_Read_Status(TI_CCxxx0_MARCSTATE, CC2500));
    break;
    
    case BIT5:
         state = TX_START;
         uhf = 1;
         LPM0_EXIT;
    break;
    
    case BIT7:
         state = TX_START;
         uhf = 0;
         LPM0_EXIT;    
    break;
    
    default:
    break;
  }//end switch
 
  P1IE |= BIT3 + BIT5 + BIT7;
  P1IFG &= ~(BIT3 + BIT5 + BIT7);                    // Clear flags
  P2IFG = 0;                                         // Clear flags
}


//  if (P1IFG & BIT3)
//  {
//  uhf = 1;
//  data_length = 8;
//  Build_Packet(data_length);
//  RF_Send_Packet(TxBuffer, data_length, uhf);
////  Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer, data_length, uhf);    // Write TX data
////  Radio_Strobe(TI_CCxxx0_STX, uhf);                                      // Change state to TX, initiating data transfer
//  P7OUT ^= BIT2;
//  }
//
//  if (P1IFG & BIT5)
//  {
//  state = TX_START;
//  uhf = 1;
//  LPM0_EXIT;
//  }
//
//  if (P1IFG & BIT7)
//  {
//  state = TX_START;
//  uhf = 0;
//  LPM0_EXIT;
//  }
//  
//  P1IFG &= ~(BIT3 + BIT5 + BIT7);                           // Clear flags
//  P2IFG = 0;                                         // Clear flags
//}

#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR (void)
{
   if (P2IFG & BIT0)
    {
        if (state == TX_START)
        { 
            P2IFG &= ~BIT0;
        }
        else
        {
            state = RX_START;
            uhf = 1;
            P2IFG &= ~BIT0;
        }
    } 

    if (P2IFG & BIT1) 
    {
        if (state == TX_START)
            state = TX_RUNNING;

        if (state == RX_START)
            state = RX_RUNNING;

        uhf = 1;
        P2IFG &= ~BIT1;
    }
  
   if (P2IFG & BIT2)
    {
        if (state == TX_START)
        { 
            P2IFG &= ~BIT2;
        }
        else
        {
            state = RX_START;
            uhf = 0;
            P2IFG &= ~BIT2;                           // Clear flag
        }

    } 

    if (P2IFG & BIT3) 
    {
      if (state == TX_START)
          state = TX_RUNNING;

      if (state == RX_START)
          state = RX_RUNNING;

      uhf = 0;
      P2IFG &= ~BIT3;
    }
P2IFG = 0;
LPM0_EXIT;
}