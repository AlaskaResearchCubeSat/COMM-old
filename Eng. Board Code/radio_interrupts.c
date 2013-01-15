//Code for engineering model Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, August 2012

#include <radio_interrupts.h>
#include <include.h>

void radio_interrupts(void)
{
//Enable interrupts on Port 2
//2.0 -- GDO0  CC1101
//2.1 -- GDO2  CC1101
//2.3 -- GDO0  CC2500
//2.4 -- GDO2  CC2500
//P2DIR &= ~(CC1101_GDO0 + CC1101_GDO2 + CC2500_GDO0 + CC2500_GDO2);
//P2IE |= CC1101_GDO0 + CC1101_GDO2 + CC2500_GDO0 + CC2500_GDO2;             // Enable ints  
//P2IES &= ~(CC1101_GDO0 + CC1101_GDO2 + CC2500_GDO0 + CC2500_GDO2);         // Int on rising edge: GDO0 interrupts when receives sync word
//P2IFG = 0;                                                                 // Clear flags

//Set up amplifier switches, RF_SW1 for 430 MHz amplifier, RF_SW2 for 2.4 GHz amplifier
P6DIR |= RF_SW1 + RF_SW2;                                                  // Set amplifier switches as outputs
P6OUT |= (RF_SW1 + RF_SW2);                                               // Disable both amplifiers 
}

