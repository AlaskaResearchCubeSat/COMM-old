
#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include "uart.h"

void initUART(void){
 //init structures
  memset(&TxBuf,0,sizeof(TxBuf));
  memset(&RxBuf,0,sizeof(RxBuf));
  TxBuf.done=0;
   //setup UCA0 for USB-UART operation
  UCA0CTL1=UCSWRST;
  UCA0CTL0=0;
  UCA0CTL1|=UCSSEL_1;
  //UCA0CTL1|=UCSSEL_2;
 //set baud rate to 9600
  UCA0BR0=3;
  UCA0BR1=0;
  UCA0MCTL=UCBRS_3;
 //set baud rate to 34400
//  UCA0BR0=26;
//  UCA0BR1=0;
//  UCA0MCTL=UCBRF_1|UCOS16;
  //setup pins
  P3SEL|=BIT4|BIT5;
  //take UCA0 out of reset mode
  UCA0CTL1&=~UCSWRST;
  //enable interupts
  UC0IE|=UCA0TXIE|UCA0RXIE;
}

//queue byte to get transmitted
int TxChar(unsigned char c){
	unsigned int t;
	int res=c;
        unsigned short state;
	//disable interrupt
	state=__disable_interrupt();
	//check if transmitting
	if(TxBuf.done){
		//bypass queue for first byte if not transmitting
		UCA0TXBUF=c;
		//clear done flag
		TxBuf.done=0;
	//queue byte
	}else{
		//get next input index
		t=(TxBuf.in+1)%TX_SIZE;
		//check if next input index equals output index
		if(t!=TxBuf.out){
			//not equal, room in buffer
			//put char in buffer
			TxBuf.buf[TxBuf.in]=c;
			//set next input index
			TxBuf.in=t;
		}else{
			//buffer full
			res=EOF;
		}
	}
	//enable interrupt
	__set_interrupt(state);
	//return result
	return res;
}

//get byte from buffer return if buffer is empty
int Getc(void){
	int c;
	unsigned int t;
        unsigned short state;
	//check for bytes
	if(RxBuf.in==RxBuf.out){
		//no bytes return EOF
		return EOF;
	}else{
		//disable interrupt while byte is retrieved
		state=__disable_interrupt();
		//get output index
		t=RxBuf.out;
		//get char from buffer
		c=(unsigned char)(RxBuf.buf[t++]);
		//set new output index
		RxBuf.out=(t)%RX_SIZE;
		//re enable interrupt
		__set_interrupt(state);
	}
	//return result
	return c;
}
      