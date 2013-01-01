#include <functions.h>
#include <include.h>

// *****************************************************************
//                     K      F     7     O     F     W    SSID   K     L     2     T     U   Space SSID Control PID
char TX_Header[16] = {0x96, 0x8C, 0x6E, 0x9E, 0x8C, 0xAE, 0xE0, 0x96, 0x98, 0x64, 0xA8, 0xAA, 0x40, 0x61, 0x3E, 0xF0};

// Header:  A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14
//           Destination callsign  SSID Transmitter     Address     SSID 
  
// Destination Call Sign: KF70FW
// Symbol,   ASCII (Hex),  ASCII (BIN), Shifted (HEX), Shifted (BIN)
//   K         4B           0100 1011      96           1001 0110
//   F         46           0100 0110      8C           1000 1100
//   7         37           0011 0111      6E           0110 1110
//   O         4F           0100 1111      9E           1001 1110
//   F         46           0100 0110      8C           1000 1100
//   W         57           0101 0111      AE           1010 1110

// Transmitter Call Sign: KL2TU
// Symbol,   ASCII (Hex),  ASCII (BIN), Shifted (HEX), Shifted (BIN)
//   K         4B           0100 1011      96           1001 0110
//   L         4C           0100 1100      98           1001 1000
//   2         32           0011 0010      64           0110 0100
//   T         54           0101 0100      A8           1010 1000
//   U         55           0101 0101      AA           1010 1010
  
  
//char TX_Header[16] = {0x90, 0x84, 0x72, 0x8A, 0x8E, 0x40, 0x60, 0x90, 0x84, 0x72, 0x8A, 0x8E, 0x62, 0x61, 0x03, 0xF0};
// Symbol, Ascii (HEX), Ascii (BIN), Shifted (HEX)
// H, 48, 0100 1000, 90
// B, 42, 0100 0010, 84
// 9, 39, 0011 1001, 72
// E, 45, 0100 0101, 8A
// G, 47, 0100 0111, 8E
//  , 20, 0010 0000, 40

int size_header = sizeof(TX_Header);

// *****************************************************************
// Copy the data bytes into the internal memory
// length: length of the data bytes
void TX_prepareFrame(char *data, unsigned int length_data) {
 
  int i;
  unsigned short TX_Data_length = (unsigned short) (length_data + 20);
  
   _BIS_SR(GIE); 					// Allow interrupt nesting to allow RX!
  
  if (length_data>DATA_MAX) length_data=DATA_MAX;       // length limitation not required. char < 257
  
  TX_bytePointerMax = length_data+20;   		// frame size is data size + 20 bytes
  
  // Send a flag first
  TX_Data[0] = 0x7E;                                    // flag
  
  // Append the addresses and the control bytes
  for ( i = 0; i< size_header; i++)
  {
  	TX_Data[i+1] = TX_Header[i];
  }
  
  // Append the data
  for (i = 0; i<length_data; i++) {
    TX_Data[i+17] = *data++;       //data
  }
  
  // Calculate and append the FCS (low byte first)
  compute_CRC(TX_Data, TX_Data_length);
  
  // Append the final flag
  TX_Data[TX_bytePointerMax-1] = 0x7E;   // flag
}

// *****************************************************************
// Calculate the FCS
unsigned short sysCRC(char *buffer, unsigned short length)
{
  unsigned short crc = 0xffff;

  
  *buffer++;              //CRC does not take flags into account
  length = length - 4;    //No flags, and no bytes for CRC. Only data and headers
  
  while (length--)
  {
    crc = (crc >> 8) ^ AX_crc_ccitt_table[(crc ^ *buffer++) & 0xff];
  }
  return crc ^ 0xffff;
}

// *****************************************************************
// Calculate the FCS
void compute_CRC(char *frame, unsigned short size_frame){
    
  unsigned short temp;
    
  temp = sysCRC(frame, size_frame); 
  frame[size_frame - 3] =  (temp & 0xff);
  frame[size_frame - 2] = ((temp >> 8) & 0xff);
}
