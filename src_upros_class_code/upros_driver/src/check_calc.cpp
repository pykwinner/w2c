#include "upros_driver/check_calc.h"
#include <iostream>
uint16_t crc16_calc(uint8_t *_pBuf, uint16_t _usLen)
{
    uint8_t ucCRCHi = 0xFF; 
    uint8_t ucCRCLo = 0xFF; 
    uint16_t usIndex; 
    while (_usLen--)
    {
        usIndex = ucCRCLo ^ *_pBuf++; 
        ucCRCLo = ucCRCHi ^ s_crchi[usIndex];
        ucCRCHi = s_crclo[usIndex];
    }
    uint16_t result = ((uint16_t)ucCRCHi << 8 | ucCRCLo);
    return result;
}