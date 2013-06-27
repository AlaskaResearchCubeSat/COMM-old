//Code for prototype Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal

#include <include.h>
#include <ctl_api.h>
#include "ARCbus.h"

//Create task structures
CTL_TASK_t tasks[2]; 

//Define stacks for tasks
unsigned stack1[1+256+1];
unsigned stack2[1+256+1];

void main(void)
{                                           

  // Sets up Timer A, clocks and pauses the watchdog timer
  ARC_setup();

  // Sets up peripherals for the ARCbus and initializes the tasking library. COMM I2C address as the argument
  initARCbus(BUS_ADDR_COMM);
  
  // Sets up the radios SPI 
  SPI_Setup();    

  // Set up the GDO interrupts for the radios and also the output pins for PA switches, use with engineering board
  radio_interrupts();

  Reset_Radio(CC1101);                                                                   // Power up radio
  __delay_cycles(8000);

    Radio_Strobe(TI_CCxxx0_SIDLE, CC1101;          // Initalize radio in Idle mode
  //Radio_Strobe(TI_CCxxx0_SRX, CC2500);           // Initialize CC2500 in Rx mode

  Write_RF_Settings(CC1101);                                                         // Write RF settings to configure registers
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, CC1101);    // Write PATABLE for CC1101
  //Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC2500, paTableLen, CC2500);    // Write PATABLE for CC2500

  // Ports used to control RF amplifiers
  P5SEL |= BIT6;
  P5DIR |= BIT6;
  
  //Set up LEDs 1-8 for testing
  P7DIR |= BIT0;             
  P7DIR |= BIT1;
  P7DIR |= BIT2;
  P7DIR |= BIT3;

  P7OUT = 0;
 
  //Initialize UART
  init_UCA1_UART();                           
     
  // Initalizes bus interface
  initARCbus(BUS_ADDR_COMM);

  //Initialize stacks
  memset(stack1,0xcd,sizeof(stack1));   //Write known values into stack
  stack1[0] = stack1[sizeof(stack1)/sizeof(stack1[0]) - 1] = 0xfeed; //Create marker values at beginning and end of stack
  
  //Initialize stacks
  memset(stack2,0xcd,sizeof(stack2));  //Write known values into stack
  stack2[0] = stack2[sizeof(stack2)/sizeof(stack2[0]) - 1] = 0xfeed; //Create marker values at beginning and end of stack
  
  //Create tasks
  ctl_task_run(&tasks[0],10,TXRX,NULL,"TXRX",sizeof(stack1)/sizeof(stack1[0]) - 2,stack1+1,0);
  ctl_task_run(&tasks[1],1,sub_events,NULL,"sub_events",sizeof(stack2)/sizeof(stack2[0]) - 2,stack2+1,0);
  
  data_length = 255;
  // RxBufferLen = sizeof(RxBuffer);          // Length of packet to be received  This does not work for some reason

  TxThrBytes = 30;
  RxThrBytes = 32;
  RxBufferLen = 258;
  temp_count1 = 0;
  temp_count2 = 0;
  state = 0;
  P2IFG = 0;     // Clear flags
  end = 0;
 
  //printf("Radio State: %x \n\r", Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
  printf("Ready\r\n");
                     
  mainLoop();
}

void Port2_ISR (void) __ctl_interrupt[PORT2_VECTOR]
{
   if (P2IFG & CC1101_GDO0)
    {
        switch(state)
        {
            case IDLE:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_RX_SYNC,0);
                 state = RX_START;
                 RxBufferPos = 0;
                 break;
            
            case RX_START:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_RX_END,0); 
                 small_packet = 1;
                 P2IFG &= ~CC1101_GDO0;
                 break;
            
            case RX_RUNNING:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_RX_END,0);
                 break;

            case TX_START:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_TX_END,0);
                 state = IDLE;
                 P2IFG &= ~CC1101_GDO0;
                 break;
            
            case TX_RUNNING:
                ctl_events_set_clear(&radio_event_flags,CC1101_EV_TX_END,0);
                state = IDLE;
                P2IFG &= ~CC1101_GDO0;
                break;

        }
    P2IFG &= ~(CC1101_GDO0);
    } 

    if (P2IFG & CC1101_GDO2) 
    {
        switch(state)
    
        {
            case IDLE:
                 break;

            case TX_START:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_TX_THR,0);
                 state = TX_RUNNING;
                 break;
            
            case TX_RUNNING:
                  ctl_events_set_clear(&radio_event_flags,CC1101_EV_TX_THR,0);
                  break;

            case RX_START:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_RX_THR,0);
                 state = RX_RUNNING;
                 break;
            
            case RX_RUNNING:
                 ctl_events_set_clear(&radio_event_flags,CC1101_EV_RX_THR,0);
                 state = RX_RUNNING;
                 break;

        }
                 
        uhf = 1;
        P2IFG &= ~CC1101_GDO2;
    }
  
   if (P2IFG & CC2500_GDO0)
    {
        switch(state){
            case IDLE:
                 ctl_events_set_clear(&radio_event_flags,CC2500_EV_RX_SYNC,0);
                 state = RX_START;
                 RxBufferPos = 0;
                 break;

            case TX_START:
                 P2IFG &= ~CC2500_GDO0;
                 break;

            case RX_START:
                 break;

                     }
    P2IFG &= ~CC2500_GDO0;
    } 

    if (P2IFG & CC2500_GDO2) 
    {
        switch(state)
        {
            case IDLE:
                 break;

            case TX_START:
                 ctl_events_set_clear(&radio_event_flags,CC2500_EV_TX_THR,0);
                 state = TX_RUNNING;
                 break;

            case RX_START:
                 ctl_events_set_clear(&radio_event_flags,CC2500_EV_RX_THR,0);
                 state = RX_RUNNING;
                 break;
        }

      uhf = 0;
      P2IFG &= ~CC2500_GDO2;
    }
//P2IFG = 0;
}
