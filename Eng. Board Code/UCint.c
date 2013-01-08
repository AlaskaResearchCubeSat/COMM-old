
#include <msp430.h>
#include "uart.h"

//buffers for UART
struct Tx TxBuf;
struct Rx RxBuf;

//UART TX ISR called to transmit UART data
void USB_TX(void) __interrupt[USCIAB0TX_VECTOR]{
  unsigned char flags=UC0IFG&(UC0IE);
  //process UART TXIFG
  if(flags&UCA0TXIFG){
    unsigned short t=TxBuf.out;
    //check if their are more chars
    if(TxBuf.in!=t){
      //more chars TX next
      UCA0TXBUF=TxBuf.buf[t++];
      TxBuf.out=(t)%TX_SIZE;
    }else{
      //buffer empty disable TX
      TxBuf.done=1;
      UC0IFG&=~UCA0TXIFG;
    }
    //more room in buffer, exit LPM
    LPM4_EXIT;
  }
}

// receive UART ISR
void USB_rx(void) __interrupt[USCIAB0RX_VECTOR]{
  unsigned char flags=UC0IFG&(UC0IE);
  //process UART RXIFG
  if(flags&UCA0RXIFG){
    unsigned int t;
    t=(RxBuf.in+1)%RX_SIZE;
    //check if there is room
    if(t!=RxBuf.out){
      //write char in buffer
      RxBuf.buf[RxBuf.in]=UCA0RXBUF;
      //advance index
      RxBuf.in=t;
      //new char ready, exit LPM
      LPM4_EXIT;
    }
    //if no room char is lost
  }
}