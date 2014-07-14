/*
* RTDM for PIC16 Demo One Board Support Package
*
*/

#include <htc.h>
#include "bsp.h"

/* Storage for Rx data */
uint8_t RTDM_RxBuf[RTDM_RXBUF_SIZE];
uint8_t* RTDM_RxBuf_Index = RTDM_RxBuf; //rx buffer index to use in ISR

#define RTDM_RXBUF_MAX (RTDM_RxBuf+RTDM_RXBUF_SIZE)

/*Structure enclosing the RTDM flags*/
struct {
    unsigned MessageReceived  :	1;
    unsigned TransmitNow      :	1;
    unsigned unused :   6;
} RTDMFlags;

/* RTDM protocol messages */
const uint8_t RTDM_Reply_Header[] =  { '+', '$' };
const uint8_t RTDM_Trailer[] = {'#'};
const uint8_t RTDM_Sanity_Reply[] = { 'R','T','D','M' };    //reply to 's' request
const uint8_t RTDM_MemWr_Reply[] = {'O','K'};               //Reply to 'm' request
const uint8_t RTDM_Limits_Reply[] = {(sizeof(RTDM_RxBuf) & 0x00FF),
                                    ((sizeof(RTDM_RxBuf) & 0xFF00) >> 8),
                                    (RTDM_MAX_XMIT_LEN & 0x00FF),
                                    ((RTDM_MAX_XMIT_LEN & 0xFF00) >> 8)};    //pre-generated reply to 'L' command
const uint8_t RTDM_Error_Reply[] = {'-','$','E',0x01};  //error message. Note different header

/* All buffers are expected to be smaller than 256 elements */
/* Easy to enforce given the platform max.memory size :-) */
#define AD_RAW_BUFSIZE 128
#define AD_RAW_BUFMASK ( AD_RAW_BUFSIZE-1 )
uint16_t  AD_Raw_Buf[AD_RAW_BUFSIZE];
uint8_t   AD_Raw_Idx = 0;

uint8_t sampling_delay = 50; //var attached to slider in DMCI

//Forward declarations
void UART_init();
void UART_send_data( uint8_t* dataptr, uint16_t datalen );
int8_t RTDM_Start();
uint16_t crc_calc( uint16_t prev_crc, uint8_t data );
void ad_conv( uint8_t delay );

/* Interrupt service routine for all possible interrupts */
void __interrupt pic16isr( void ) {

    if( ADIF )  { //A/D conversion complete.
                  //Sampling cap is connected to the input
        ad_conv( sampling_delay );  //start next delayed conversion
        RC0 = 1;        // raise the pin for external time measurement

        ADIF = 0;

        //Record converted value to the DMCI buffer
        AD_Raw_Buf[ AD_Raw_Idx++ ] = ( ADRESH << 8 | ADRESL );
        AD_Raw_Idx &= AD_RAW_BUFMASK;

        RC0 = 0;    //lower the pin for external time measurement
    }//if( ADIF )...

#define CRC_SEED_RX 0x5fff //pre-calculated CRC-16(var.Modbus) for the header symbol

    //RTDM processing
    if( RCIF ) {    //rx interrupt

    static uint16_t rx_crc = 0;
    static uint8_t tmpdata;         //copy of RCREG. Made static to save time
    static uint8_t rx_state = 0;    //rx state machine state

    tmpdata = RCREG;   //read the UART

    switch( rx_state ) {

        case 0:    //check for header
            
            if( tmpdata == '$' ) { //RTDM request header

                rx_crc = CRC_SEED_RX;

                RTDM_RxBuf_Index = RTDM_RxBuf; //point to the start of the buffer

                rx_state++;
            }//if( tmpdata == '$'...
            break;

        case 1: //get data

            if( tmpdata == '#' ) { //end of packet
                                   //buggy! - if '#' is xmitted in the middle of 'm'/'M' command
                                   //todo: see above
                rx_state++; //switch to receiving a checksum
            } else {    //receive data to Rx buffer

                rx_crc = crc_calc( rx_crc, tmpdata );   //accumulate crc

                *(RTDM_RxBuf_Index++) = tmpdata;                   //accumulate data

                if( RTDM_RxBuf_Index > RTDM_RXBUF_MAX ) { //we're past the buffer boundary

                    rx_state = 0;   //drop the packet
                }
            }//if( tmpdata == '#'...
            break;

        case 2: //check first byte of the checksum

            if( tmpdata && (rx_crc & 0xff)) {  //lower byte match

                rx_crc >>= 8;

                rx_state++; //go check the rest of the checksum
                            //and process the message
            } else {        //drop the packet

                rx_state = 0;
            }//if( tmpdata && (rx_crc & 0xff
            break;

        case 3: //process

            if( tmpdata && (rx_crc & 0xff)) {   //upper byte match

                /* This can be separated, if desired. For example, process
                 * messages outside of ISR to give way to other tasks.
                 * Note, however, that message processing is just placing
                 *  a couple of pointers into the tx queue, which will be read
                 * right below, in TXIF section of the ISR.
                 * Therefore, data will start going out in the same ISR call.
                 * Processing messages in other places could actually be slower */

                RTDMFlags.MessageReceived = 1;
                RTDM_ProcessMsg();
            }

            rx_state = 0;   //start over
            break;
    }//switch( rx_state...
}//if( RCIF )...

    if( TXIF && TXIE ) {

    static uint8_t tx_state = 0;
    static uint16_t tx_crc = 0xffff;    //seed for the very first reply
    static uint8_t* dataptr = NULL;     //pointer to data being transmitted via UART
    static int16_t bytes_left = 0;      //bytes left to transmit
    static UART_queue_entry qetmp;      //made static to save time

    switch( tx_state ) {   //all fallthroughs intentional

        case 0: //IDLE

            if( UART_xmit_queue_head != UART_xmit_queue_tail ) {    //new data present

                UART_xmit_queue_tail++; //Free the entry
                UART_xmit_queue_tail &= UART_XMIT_QUEUE_MASK;
                
                qetmp = UART_xmit_queue[ UART_xmit_queue_tail]; //get data
                dataptr = qetmp.dataptr;
                bytes_left = qetmp.datalen;

                tx_state++;     //proceed to transmitting data
            } else {            //stop servicing this interrupt

                TXIE = 0;   
                break;
            }
        case 1: //xmit data

            //todo - how to do it with less checking?
            if( bytes_left ) { //previous send incomplete
 
                tx_crc = crc_calc( tx_crc, *dataptr ); //accumulate crc

                TXREG = *dataptr++; //send out byte

                if( --bytes_left == 0 ) {   //end of data chunk
                    if( TXREG == '#')  {    //end of packet

                        dataptr = (uint8_t*)&tx_crc; //prepare to send checksum

                        bytes_left = 2;

                        tx_state++;    //will go out in the next interrupt
                    }//if( TXREG == '#...
                    else {
                        
                        tx_state = 0;  //get next chunk
                    }
                }//if( --bytes_left == 0...
            }//if( bytes_left
            break;

        case 2: //send checksum

            if( bytes_left-- ) { //more to send

                TXREG = *dataptr++;
            } else {

                    tx_state = 0;  //get next chunk

                    tx_crc = 0xffff; //seed CRC
            }//if( --bytes_left == 0...
        }//switch( tx_state...
    }//if( TXIF && TXIE ...
}//end of ISR

/* Board Initialization */
void BSP_init( void ) {

    /* Pin setup */
    /* All unused pins switched to digital out */
    PORTA = 0;
    TRISA = 0;
    ANSELA = 0;

    PORTB = 0;
    TRISB = 0b00100000; //RB5 UART input
    ANSELB = 0;

    PORTC = 0;
    TRISC = 0;
    ANSELC = 0;

    /* Fixed Voltage Reference setup */
    FVRCON = 	0b10001111;     //Enable reference, set comparator gain to 4x,
                                // ADC gain to 4x

    /* ADC setup */
    ADCON0 = 	ADCON0_INIT;
    ADCON1 = 	ADCON1_INIT;
    ADCON2 = 0b01010000;

    ADIF = 0;
    ADIE = 1;
    
    /* Timer 2 is used to send delayed A/D Go */ 
    T2CON = 0x01;   //prescaler 4, 1us step

    ad_conv( 100 ); //start A/D conversion 100us from now

    UART_init();
    RTDM_Start();   //initialize RTDM and enable UART

    /* Enable interrupts */
    PEIE = 1;
    GIE = 1;
}

void UART_init( void ) {
    /* UART Configuration */
    BRGH = 1;  
    BRG16 = 1;
    SPBRGL = RTDM_BRG;
    //SPBRGH is unlikely ever needs to be other than zero.

    SYNC = 0;
    TXEN = 1;   //this will set TXIF
    CREN = 1;
    RCIE = 1;   //enable Rx Interrupt
    SPEN = 1;
}

void UART_send_data( uint8_t* dataptr, uint16_t datalen ) {

 uint8_t tmphead = UART_xmit_queue_head + 1;
 UART_queue_entry qe;

    qe.dataptr = dataptr;
    qe.datalen = datalen;

    tmphead &= UART_XMIT_QUEUE_MASK;

    while( tmphead == UART_xmit_queue_tail ) {}     //this line blocks!

    UART_xmit_queue[ tmphead ] = qe;    //place an element into the queue

    UART_xmit_queue_head = tmphead;

    TXIE = 1;   //let the ISR know
}

/* Calculates CRC using lookup table */
/* params:
 *          prev_crc - previous value of crc
 *          data - a databyte for which new crc will be calculated
 * */
uint16_t crc_calc( uint16_t prev_crc, uint8_t data ) {

const uint16_t crc16_table[] = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
  0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
  0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
  0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
  0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
  0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
  0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
  0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
  0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
  0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
  0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
  0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
  0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
  0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
  0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
  0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
  0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
  0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
  0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
  0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
  0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
  0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
  0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
  0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
  0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
  0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
  0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
  0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
  0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

    return( (prev_crc  >> 8) ^ crc16_table[(prev_crc ^ data) & 0xff]);
}

int8_t RTDM_Start()
{

    /* Set ready for the next message */
    RTDMFlags.MessageReceived = 0;
    RTDM_RxBuf_Index = RTDM_RxBuf;

    return 0;
}

/* Process a message received from serial port */
/* Supports memory read/writes, as well as Sanity check, Buffer limits
 * and Unsupported command replies */
/* Much can be improved here */
int8_t RTDM_ProcessMsg()
{
    //Local pointer management variables
    uint32_t* u32tmp_p; //temporary 32 bit pointer
    uint8_t* RxBuf_Idx = RTDM_RxBuf + 1;    //rx buffer index
                                            //initialized past command byte
                                            //only used in 'm'/'M' commands

    static uint8_t* data_p;    //pointer to data to send back/write
    static uint16_t data_l;    // length of the data to send back/write

    if (!RTDMFlags.MessageReceived) {   //check if new data is available
		return -1;
    }

    switch( *RTDM_RxBuf ) { //command at RTDM_RxBuf[0]

        case 'm': //read memory area

//#if( 0 )  todo: get rid of u32tmp_p

            /*************** Extract Address **************/
            //Capture address as 32 bit quantity to match protocol definition.
            u32tmp_p = (uint32_t*)RxBuf_Idx;  //get memory address from the command

            //Increment receive buffer pointer to length field.
            
            RxBuf_Idx += 4;  //point to num.bytes
                        
            //Init a byte pointer
            data_p = (uint8_t*)(*u32tmp_p);

//#endif

            //data_p = (uint8_t*)(*((uint16_t*)RxBuf_Idx));
            //data_p = (uint8_t*)RxBuf_Idx;
            //data_p = (uint8_t*)((uint32_t*)RxBuf_Idx);
            //data_p = *RxBuf_Idx;

            //RxBuf_Idx += 4;  //point to num.bytes

            /********* Extract Number of Bytes ************/
            data_l = *((uint16_t*)RxBuf_Idx);

            UART_send_data((uint8_t*)RTDM_Reply_Header, ARRAY_LEN(RTDM_Reply_Header));  //send the header
            UART_send_data( data_p, data_l );
            
            break;  //case 'm'...

		  case 'M': //write memory area
			
			/*************** Extract Address **************/
		    //Capture address as 32 bit quantity to match protocol definition.
		    u32tmp_p = (uint32_t *)RxBuf_Idx;

		    //Increment receive buffer pointer to length field.
		    RxBuf_Idx += sizeof(uint32_t);

                    // Init a byte oriented address pointer for use in incrementing
                    data_p = (uint8_t*)(*u32tmp_p);

                    data_l = *((uint16_t*)RxBuf_Idx);


                    RxBuf_Idx += sizeof(uint16_t);  //point to payload

                    /********** Extract Data ************/
                    while( data_l-- ) { //copy rx buffer data to memory
                        *data_p++ = *RxBuf_Idx++;
                    }

                    UART_send_data((uint8_t*)RTDM_Reply_Header, ARRAY_LEN(RTDM_Reply_Header));
                    UART_send_data((uint8_t*)RTDM_MemWr_Reply, ARRAY_LEN(RTDM_MemWr_Reply));
                    break;

		  case 's': //sanity check
                        UART_send_data((uint8_t*)RTDM_Reply_Header, ARRAY_LEN(RTDM_Reply_Header));
                        UART_send_data( (uint8_t*)RTDM_Sanity_Reply, ARRAY_LEN(RTDM_Sanity_Reply));
                        break;
			
		  case 'L': //report platform limits
                        UART_send_data((uint8_t*)RTDM_Reply_Header, ARRAY_LEN(RTDM_Reply_Header));
                        UART_send_data((uint8_t*)RTDM_Limits_Reply, ARRAY_LEN(RTDM_Limits_Reply));
			break;

		  default:  //Unsupported command - send error message
                        UART_send_data((uint8_t*)RTDM_Error_Reply, ARRAY_LEN(RTDM_Error_Reply));
			break;
		  }//switch( *RTDM_RxBuf..

                  UART_send_data((uint8_t*)RTDM_Trailer, ARRAY_LEN(RTDM_Trailer));
	  
          RTDM_Start();        
	  return 0;
}

//this function starts A/D conversion after delay specified by 'delay' parameter.
//Timer 2 match event is used to trigger the conversion, therefore the function returns
//immediately after setting up timer. Delay unit is 1us (can be changed via prescaler setting,
//delays longer than full scale minus A/D conversion time are risky.
void ad_conv( uint8_t delay ) {

    TMR2ON = 0; //stop timer 2
    TMR2 = 0;
    PR2 = delay;
    TMR2ON = 1; //start timer. A/D conversion will start on PR2 match

}


