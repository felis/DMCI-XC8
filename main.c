/*****************************************************************************
 * RTDM for PIC16 Demo One
 */

#include "bsp.h"

uint8_t try = 8;

#pragma config FOSC=HS, IESO=ON, WDTE=OFF, MCLRE=OFF, BOREN=OFF, LVP=OFF



void main()
{
    BSP_init(); //board initialization

   while(1) {}
}
