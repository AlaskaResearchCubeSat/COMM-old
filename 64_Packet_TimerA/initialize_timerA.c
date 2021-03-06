//Code for engineering model Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, August 2012

#include <initialize_timerA.h>
#include <include.h>

void initialize_timerA(void)
{
TACTL = TASSEL_1 + ID_3 + MC_1 + TAIE;  //Setup Timer A with source as ACLK, clk divided by 8, in Up Mode and interrupts enabled
TACCR0 = 512;  //ACLK = 32768 Hz, /8 = 4096 Hz, 5 seconds = 20,480 Hz - 1
}
 
 // 5 seconds = 20479
 // 3 seconds = 12287
 // 1 second = 4095
 // 500 ms = 2047
 // 125 ms = 512
 // 100 ms = 409
 // 10 ms = 41