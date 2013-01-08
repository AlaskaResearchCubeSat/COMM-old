//Code for engineering model Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, August 2012

#include <initialize_timerA.h>
#include <include.h>

void initialize_timerA(void)
{
TACTL = TASSEL_1 + ID_3 + MC_1 + TAIE;  //Setup Timer A with source as ACLK, clk divided by 8, in Up Mode and interrupts enabled
TACCR0 = 20479;  //ACLK = 32768 Hz, /8 = 4096 Hz, 5 seconds = 20,480 Hz - 1
}
 