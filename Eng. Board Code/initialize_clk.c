//Code for engineering model Communication System for the Alaska Research CubeSat (ARC) as developed by the Alaska Space Grant Program
//Samuel Vanderwaal, August 2012

#include <initialize_clk.h>
#include <include.h>

void initialize_clk(void)
{
DCOCTL = 0; 								  // Set DCO to lowest tap to protect it from getting set too high when RSELx is set
BCSCTL1 = CALBC1_16MHZ;						          // Pull calibration data for 16MHz and set RSELx to appropriate tap range
DCOCTL = CALDCO_16MHZ;	                                                  // Pull calibration data for 
BCSCTL2 |= SELM_0;					         // Selects DCO as source for MCLK and SMCLK
BCSCTL3 |= XCAP_0;                                                       // Turn off internal capacitors for ACLK
}
