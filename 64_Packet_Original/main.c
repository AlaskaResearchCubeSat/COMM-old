//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, 2012

#include <include.h>
#define CC1101  1
#define CC2500  0

char txBuffer[66];
char rxBuffer[66];
unsigned int i, uhf;

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                                                          // Stop watchdog timer
  
  initUART();                                                                       // Initialize UART

  Radio_SPI_Setup();                                                               //Setup SPI for communication with CC2500 (Port 3)

  Reset_Radios();                                                                  //Power up both radios
  Write_RF_Settings();                                                             //Write RF settings to config reg
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, 1);   //Write PATABLE for CC1101
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC2500, paTableLen, 0);   //Write PATABLE for CC2500

  DCOCTL = 0; 								  // Set DCO to lowest tap to protect it from getting set too high when RSELx is set
  BCSCTL1 = CALBC1_16MHZ;					          // Pull calibration data for 16MHz and set RSELx to appropriate tap range
  DCOCTL = CALDCO_16MHZ;					          // Pull calibration data for 
  BCSCTL2 |= SELM_0;							  // Selects DCO as source for MCLK and SMCLK

  //Enable interrupt on Port 2.0 and 2.2 for radios to indicate reception of packet; 2.0 is CC1101 and 2.2 is CC2500
  P2DIR &= ~(BIT1 + BIT3);
  P2IES &= ~(BIT1 + BIT3);                                        // Int on rising edge (sync sent or received)
  P2IE |= BIT1 + BIT3;                                         // Enable ints  

  P7DIR |= BIT0;                                               //Set up LEDs 1-8 for testing
  P7DIR |= BIT1;
  P7DIR |= BIT2;
  P7DIR |= BIT3;
  P7DIR |= BIT4;
  P7DIR |= BIT5;
  P7DIR |= BIT6;
  P7DIR |= BIT7;            

  P7OUT = 0;
  
  P1DIR &= ~(BIT5 + BIT7);                                    //Set P1.5 and 1.7 as Inputs
  P1REN |= BIT5 + BIT7;                                       //Enable Pull-up/pull-down Resistors for Port 1.5 and 1.7
  P1OUT |= BIT5 + BIT7;                                       //Set Resistors as pull-up

  P1IE |= BIT5 + BIT7;                                        //Enable interrupts for Port 1.5,7 (buttons)
  P1IES |= BIT5 + BIT7;                                       //Interrupts set on high-low transition
  
  //P1IFG &= ~(BIT5 + BIT7);
  //P2IFG &= ~(BIT0 + BIT2);     
  P1IFG = 0;                                                   // Clear flags
  P2IFG = 0;                  

  Radio_Strobe(TI_CCxxx0_SRX, CC1101);                              // Initialize CC1101 in RX mode
  Radio_Strobe(TI_CCxxx0_SRX, CC2500);                              // Initialize CC2500 in Rx mode

  length = 61;   	                                       // Len of pkt to be RXed (only addr
                                                               // plus data; size byte not incl b/c
                                                               // stripped away within RX function)
  for (i=i; i<=length; i++)
        data[i-1] = i;
  _EINT();
                                                               
  printf("Program ready");
  printf("\r\n");
   while (1) 
  {
      LPM0;
  }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
  txBufferSize = length + 1;

  P2IE &= ~(BIT1 + BIT3);                                      // Disable Port 2 interrupts
  P2IES |= BIT1 + BIT3;                                        // Change interrupts to falling edge (end of pkt)
  P2IE |= BIT1 + BIT3;                                         // Enable Port 2 interrupts
  P2IFG = 0;
  
  if (P1IFG & BIT5)
  {
      // Build Packet
      txBuffer[0] = length;
      for (i = 0; i < length; i++)
      {
          txBuffer[i+1] = data[i];                       // Load "data"
      }
      P1IFG &= ~BIT5;
      
      RF_Send_Packet(txBuffer, txBufferSize, 1);             // Send value over RF
  }

  if (P1IFG & BIT7)
  {
      // Build packet
      txBuffer[0] = length;
      for (i = 0; i < length; i++)
      {
          txBuffer[i+1] = data[i];                       // Load "data"
      }
      P1IFG &= ~BIT7;
      
      RF_Send_Packet(txBuffer, txBufferSize, 0);               // Send value over RF
  }
}

#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR (void)
{
  char len = length+1;                         // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function
  __delay_cycles(2000000);
  if (P2IFG & BIT1)
  {
    if (tx_flag)
    {
     P2IFG &= ~BIT1;
     Radio_Strobe(TI_CCxxx0_SRX, CC1101);         // Change state back to Rx
     P7OUT ^= BIT0;
     tx_flag = 0;                                           
    }

    if (RF_Receive_Packet(rxBuffer, &len, CC1101))       // Get packet from CC1101
        {
        P7OUT ^= BIT2;                             // Toggle LED to indicate packet received
        printf("70-cm packet received \r\n");       
        for (k=0; k < len; k++)
        {
            printf("%d ", rxBuffer[k]);
            printf(" "); 
        }
        printf("\r\n");   
        }
    P2IFG &= ~BIT1;                                    
  }
  
  if (P2IFG & BIT3)
  {
     if (tx_flag)
     {
     P2IFG &= ~BIT3;
     Radio_Strobe(TI_CCxxx0_SRX, CC2500);         // Change state back to Rx
     P7OUT ^= BIT4;
     tx_flag = 0;      
     }
    
    if (RF_Receive_Packet(rxBuffer, &len, CC2500))       // Get packet from CC2500
        {             
        P7OUT ^= BIT5;                            // Toggle LED to indicate packet received
        printf("2.4 GHz packet received \r\n");
        for (k=0; k < len; k++)
        {
            printf("%d ", rxBuffer[k]);
            printf(" ");
        }
        printf("\r\n");   
      }   
    P2IFG &= ~BIT3;                          // Clear flag
  }
}
