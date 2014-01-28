/**
  *	Thanks Coto for this awesome piece of code.
  *
  */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#define POLY 0x1021 //0x1021 CRC16-CCITT polynomial
#define CRC16STARTBIT 0xffff //value to start comparing from (zero bits must be compared as well)
// This uses CRC16-CCITT with a mod so that the init value can be
// specified
/*unsigned char messs[12] ={
  0x15, 0x47, 0xC3, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF
};*/
 
unsigned short int crc_xmodem_update(uint16_t crc, uint8_t data)
{
  int i;
 
  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ POLY; //(polynomial = 0x1021)
    else
      crc <<= 1;
  }
  return crc;
}
 
uint16_t calc_crc(unsigned char *msg,int n,uint16_t init)
{
  uint16_t x = init;
 
  while(n--)
  {
    x=crc_xmodem_update(x, (uint16_t)*msg++);
  }
 
  return(x);
}