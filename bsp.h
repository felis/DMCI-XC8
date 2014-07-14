/* 
 * File:   bsp.h
 * Author: felis
 *
 * Created on June 21, 2014, 7:18 PM
 *
 * RTDM for PIC16 Demo One Board Support Package header
 */

#ifndef BSP_H
#define	BSP_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define _XTAL_FREQ  16000000

/* RTDM */
#define RTDM_FCY    16000000UL  //This define has to be the system operating freq, this
                                //value is used to calculate the value of the BRG register

#define RTDM_BRG	(RTDM_FCY/(4*(RTDM_BAUDRATE+1)))

#define RTDM_BAUDRATE 256000    //non-standard baudrates are possible - check DMCI COM speed dropdown
                                //FTDI USB-to-serial converter supports them too

#define RTDM_RXBUF_SIZE    32  // This is the buffer size used by RTDM to handle messaages
                                //a message larger than buffer will be truncated and eventually
                                //duscarded

#define RTDM_MAX_XMIT_LEN   0x100   //This the size in bytes of the max num of bytes allowed in
                                    //the RTDM protocol frame, including header/trailer/crc

//macro to get array length at compile time
#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))

/* Pins */

// AD Converter
#define	ADCON0_INIT 0b11111101  // turn on ADC, chan = FVR
#define	ADCON1_INIT 0b11010000  //Left justified, FOSC/16

/* UART transmit queue */
typedef struct {        //UART queue entry
    uint8_t* dataptr;
    uint16_t datalen;
} UART_queue_entry;

#define UART_XMIT_QUEUE_SIZE 4  //we never transmit more than 3 chunks in a reply
                                //must be power of 2
#define UART_XMIT_QUEUE_MASK ( UART_XMIT_QUEUE_SIZE - 1 )
#if ( UART_XMIT_QUEUE_SIZE & UART_XMIT_QUEUE_MASK )
#error Transmit queue size is not a power of 2
#endif

UART_queue_entry UART_xmit_queue[UART_XMIT_QUEUE_SIZE];  //queue to feed UART transmitter
uint8_t UART_xmit_queue_head = 0;
uint8_t UART_xmit_queue_tail = 0;

void BSP_init( void );
int8_t RTDM_ProcessMsg(); //declared here so it can be used outside of ISR

#ifdef	__cplusplus
}
#endif




#endif	/* BSP_H */

