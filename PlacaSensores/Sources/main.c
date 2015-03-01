/*
 * Copyright (c) 2014 Ovcharenko Alexander <ferrocentos[dot]gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define F_CPU 8000000
#define DEFAULT_RADIO_CHANNEL (17)
#define MAX_FRAME_SIZE (127)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "radio.h"
#include "wuart.h"
#include "ioutil.h"
#include "transceiver.h"
#include "hif.h"
#include "common.h"
#include "uart.h"
#include "twi.h"
#include "ml.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MPU_DEBUG				// print some debug messages
#define MPU9150
#define TIMER
#define DEFAULT_MPU_HZ  (100)	// starting sampling rate
#define F_CPU	8000000

uint8_t tuvieja,aux;

// packet structure for InvenSense teapot demo
int8_t teapotPacket[24] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n'};

bool mpuInterrupt;	// indicates whether MPU interrupt pin has gone high

FILE uart_stream = FDEV_SETUP_STREAM(UartPutchar, NULL, _FDEV_SETUP_RW);

uint16_t dmp_features;


/* === Macros ======================================== */
#define SW_VERSION (0x03)
/* === Types ========================================= */


/* === globals ====================================== */

const node_config_t PROGMEM nc_flash = { .short_addr = 0xFECA,
	.pan_id = 0x3412,
	.channel = DEFAULT_RADIO_CHANNEL,
};
node_config_t NodeConfig;

volatile uint8_t RxbufIdx = 0;
static wuart_buffer_t RxBuf[2];

static uint16_t PeerAddress;
static wuart_buffer_t TxBuf;
volatile bool TxPending;

volatile wuart_state_t WuartState;
volatile int16_t EscapeTmoCounter;
volatile int8_t LastTransmitCounter;
volatile uint16_t SendPingReply = 0xffff;

static const p2p_ping_cnf_t PROGMEM pingrep = {
	.hdr.cmd= P2P_PING_CNF,
	.hdr.fcf = 0x8841, 				/* short addressing, frame type: data, no ACK requested */
	.version = SW_VERSION,
	.appname = "wuart\0",
};



/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static int8_t gyro_orientation[9] = {1, 0, 0,
									 0, 1, 0,
									 0, 0, 1};

ISR(INT2_vect)
{
	if(bit_is_clear(PIND,PD2))
	{
		mpuInterrupt = true;
	}
}

void EnableInterrupt(void)
{
	PORTD |= _BV(PD2);	 // Levanta el Pin2 del puerto D
	EICRA  = _BV(ISC21);	// Allow interrupts on INT2, triggered on falling edge ->OK
	EIMSK  = _BV(INT2);	 // Enable only on INT2	->OK
}


int main(void)
{
	
	uint8_t accel_fsr;
	uint16_t gyro_rate, gyro_fsr;
	
	union qua {
			uint8_t qb[16];
			long quatf[4];
		} q;
		
	union acc {
		int acelint[3];
		char acelb [6];
		}a;
	
	union gyro {
		int gyroint[3];
		char gyrob [6];
		}g;
		
	union mag {
		int magint[3];
		char magb [6];
	}m;
	
	char i=0;

	int16_t gyro[3], accel[3], sensors;
	int32_t quat[4];
	int16_t compass[3];
	uint8_t more=0;
	
	float quatf[4];
	
	char temp_out[4];
	int32_t temp_in;
	
	int inchar;
	uint8_t pluscnt = 0;
	bool do_send;
	wuart_buffer_t *prx;

	
	stdout = &uart_stream;

	UartInit();
	TwiInit();

	sei();

	tuvieja = mpu_init(NULL);

	tuvieja = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); //Habilito todos los sensores
	
	tuvieja = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	
	tuvieja = mpu_set_sample_rate(50);
	
	tuvieja= mpu_set_compass_sample_rate(50);
	
	tuvieja = dmp_load_motion_driver_firmware();

	tuvieja = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	
	tuvieja = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |DMP_FEATURE_SEND_RAW_GYRO);
	

	tuvieja = dmp_set_fifo_rate(50);

	tuvieja = mpu_set_dmp_state(1);
	
	tuvieja = dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);

	
#if defined MPU_DEBUG
	printf_P(PSTR("Result of load firmware %d\r\n"), tuvieja);
#endif
	
	/* Enable system interrupt for set "DMP Ready flag" */
	EnableInterrupt(); // esto es por parte del micro
	
	_delay_ms(500);
    wuart_init();
	WuartState = DATA_MODE;
  do 
  {
		    //inchar = hif_getc();
            inchar = tuvieja; 
		    /* state machine to detect Hayes '302 break condition */
		    switch (WuartState)
		    {
			    case DATA_MODE:
			    if (EOF == inchar)
			    {
				    EscapeTmoCounter = 255;
				    WuartState = WAIT_GUARDTIME;
			    }
			    break;

			    case WAIT_GUARDTIME:
			    if (EOF != inchar)
			    {
				    EscapeTmoCounter = 0;
				    WuartState = DATA_MODE;
			    }
			    else if (EscapeTmoCounter == 0)
			    {
				    if (pluscnt != 3)
				    {
					    pluscnt = 0;
					    WuartState = WAIT_PLUS;
				    }
				    else
				    {
					    pluscnt = 0;
					    WuartState = DO_CONFIGURE;
				    }
			    }
			    break;

			    case WAIT_PLUS:
			    if (inchar == '+')
			    {
				    pluscnt ++;
				    if (pluscnt == 3)
				    {
					    WuartState = WAIT_GUARDTIME;
				    }
			    }
			    else if (EOF != inchar)
			    {
				    WuartState = DATA_MODE;
			    }
			    break;

			    case DO_CONFIGURE:
			    radio_set_state(STATE_OFF);
			    do_configure_dialog();
			    configure_radio();
			    WuartState = DATA_MODE;
			    break;
		    }

		    if (WuartState != DO_CONFIGURE)
		    {
			    /* handle data "coming from air" */
			    prx = &RxBuf[RxbufIdx^1];
			    while(prx->start <= prx->end)
			    {
//				    hif_putc(prx->data.buf[prx->start]);
			        prx->start++;
				}
			    cli();
			    prx->end = 0;
			    sei();

			    /* handle data "going to air" */
			    do_send = false;
			    if ((TxBuf.start + pluscnt) >= TxBuf.end)
			    {
				    do_send = true;
			    }
			    else if ((LastTransmitCounter == 0) &&
			    (TxBuf.start > sizeof(p2p_wuart_data_t)))
			    {
				    do_send = true;
			    }

			    if (do_send == true)
			    {
				    /* wait here, while TX is busy */
				    while(TxPending == true)
				    ;
				    radio_set_state(STATE_TXAUTO);
				    TxPending = true;
				    TxBuf.data.hdr.hdr.seq ++;
				    radio_send_frame(TxBuf.start + CRC_SIZE, TxBuf.data.buf, 0);
				    LastTransmitCounter = 100;
				    TxBuf.start = sizeof(p2p_wuart_data_t);
				    TxBuf.end = UART_FRAME_SIZE - CRC_SIZE;
				    LED_SET(0);
			    }
		    }

		    if (WuartState == DATA_MODE)
		    {
			    /* handle data coming from hif/uart */
			    while (pluscnt)
			    {
				    //hif_putc('+');
				    --pluscnt;
				    TxBuf.data.buf[TxBuf.start++] = '+';
			    }
			    if (EOF != inchar)
			    {
				    /* fill in new bytes */
				    //hif_putc(inchar);
				    TxBuf.data.buf[TxBuf.start++] = (uint8_t)inchar;
			    }
		    }

		    /* always respond to a ping request */
		    //if ((SendPingReply != 0xffff) && (TxPending == false))
		    //{
			    //send_ping_reply();
		    //}
	  
 } while (1);
 
}

static void wuart_init(void)
{
char cfg_location = '?';

    /* setup peripherals */
    LED_INIT();
    LED_SET_VALUE(0);
    KEY_INIT();
//    hif_init(HIF_DEFAULT_BAUDRATE);
    TIMER_INIT();

    /* get configuration data */

    /* 1st trial: read from internal EEPROM */
    if (get_node_config_eeprom(&NodeConfig, 0) == 0)
    {
        /* using EEPROM config */;
        cfg_location = 'E';
    }
    /* 2nd trial: read from FLASHEND */
    else if (get_node_config(&NodeConfig) == 0)
    {
        /* using FLASHEND config */;
        cfg_location = 'F';
    }
    /* 3rd trial: read default values compiled into the application */
    else
    {
        /* using application default config */;
        memcpy_P(&NodeConfig, &nc_flash, sizeof(node_config_t) );
        cfg_location = 'D';
    }

    if (((1UL<<NodeConfig.channel) & TRX_SUPPORTED_CHANNELS) == 0)
    {
        /* fall back to DEFAULT_CHANNEL,*/
        //PRINTF("Invalid channel: %d use now: %d",
                //NodeConfig.channel, DEFAULT_RADIO_CHANNEL);
        //NodeConfig.channel = DEFAULT_RADIO_CHANNEL;

    }

    RxbufIdx = 0;
    memset(RxBuf, 0, sizeof(RxBuf));
    memset(&TxBuf, 0, sizeof(TxBuf));

    /* radio setup */
    radio_init(RxBuf[RxbufIdx].data.buf, UART_FRAME_SIZE);
    configure_radio();
    sei();

    //PRINTF("Wuart %d.%d chan=%d radio %02x.%02x cfg %c"EOL,
            //(SW_VERSION >> 4), (SW_VERSION &0x0f),
            //NodeConfig.channel,
            //trx_reg_read(RG_PART_NUM),
            //trx_reg_read(RG_VERSION_NUM),
            //cfg_location);


    /*Don't care about the revision, verify if PART_NUM does match. */

    //if (trx_identify() & INVALID_PART_NUM)
    //{
        //PRINTF("radio does not match: expected %d.%d got %d.%d"EOL,
              //RADIO_PART_NUM, RADIO_VERSION_NUM,
              //trx_reg_read(RG_PART_NUM),
              //trx_reg_read(RG_VERSION_NUM)
              //);
    //}
}

static void configure_radio(void)
{

    cli();
    /* initialization after Update */
    PeerAddress = CALC_PEER_ADDRESS(NodeConfig.short_addr);

    TxBuf.start = sizeof(p2p_wuart_data_t);
    TxBuf.end = UART_FRAME_SIZE - CRC_SIZE;

    FILL_P2P_HEADER_NOACK((&TxBuf.data.hdr.hdr),
                          NodeConfig.pan_id,
                          PeerAddress,
                          NodeConfig.short_addr,
                          P2P_WUART_DATA);
    TxBuf.data.hdr.mode = 0x55;

    radio_set_param(RP_IDLESTATE(STATE_RX));
    radio_set_param(RP_CHANNEL(NodeConfig.channel));
    radio_set_param(RP_SHORTADDR(NodeConfig.short_addr));
    radio_set_param(RP_PANID(NodeConfig.pan_id));
    radio_set_state(STATE_RX);
    sei();
}

/**
 * Configure wuart parameters.
 */
void do_configure_dialog(void)
{
bool dirty = false, refresh = true;
uint16_t val, peer;
int inchar;
node_config_t nc;

    memcpy(&nc, &NodeConfig, sizeof(node_config_t) );
    peer = (nc.short_addr == 0xffff) ? 0xffff : nc.short_addr ^ 1;
    do
    {
        //if (refresh)
        //{
            //peer = CALC_PEER_ADDRESS(nc.short_addr);
            //PRINT(EOL"MENU:"EOL);
            //PRINTF("[a] node address: 0x%04x"EOL, nc.short_addr);
            //PRINTF("    peer address: 0x%04x"EOL, peer);
            //PRINTF("[c] channel:      %d"EOL, nc.channel);
            //PRINT("[r] reset changes"EOL
                  //"[e] save and exit"EOL
                  //"[q] discard changes and exit"EOL
                  //"==="EOL);
        //}
        //refresh = true;
//
        //inchar = hif_getc();
//
        switch(inchar)
        {
            case 'a':
//                PRINT("enter short address: ");
                nc.short_addr = get_number(16);
                dirty = true;
                break;
            case 'c':
//                PRINT("enter channel: ");
                val = get_number(10);
                if (((1UL<<val) & TRX_SUPPORTED_CHANNELS) != 0)
                {
                    nc.channel = val;
                    dirty = true;
                }
                else
                {
//                    PRINT("unsupported channel");
                }
                break;
            case 'r':
                memcpy(&nc, &NodeConfig, sizeof(node_config_t) );
                break;
            case 'e':
//                PRINT("save config and exit"EOL);
                memcpy(&NodeConfig, &nc, sizeof(node_config_t) );
                store_node_config_eeprom(&nc, 0);
                break;
            case 'q':
//                PRINT("discard changes and exit"EOL);
                break;

            default:
                refresh = false;
                break;
        }
    }
    while ((inchar != 'e') && (inchar != 'q'));
}

//static inline void send_ping_reply(void)
//{
    //p2p_ping_cnf_t _pr;
    //memcpy_P(&_pr, &pingrep, sizeof(p2p_ping_cnf_t));
    //cli();
    //_pr.hdr.src = NodeConfig.short_addr;
    //_pr.hdr.pan = NodeConfig.pan_id;
    //_pr.hdr.dst = SendPingReply;
    //SendPingReply = 0xffff;
    //TxPending = true;
    //sei();
    //radio_set_state(STATE_TXAUTO);
    //radio_send_frame(sizeof(p2p_ping_cnf_t)+ 2,
                    //(uint8_t*)&_pr, 1);
//
//}


void usr_radio_error(radio_error_t err)
{
//    PRINTF("radio error %d", err);
}


/**
 * Implementation of callback function @ref usr_radio_tx_done.
 */
void usr_radio_tx_done(radio_tx_done_t status)
{
    LED_CLR(1);
    if (status == TX_OK)
    {
        LED_CLR(1);
    }
    TxPending = false;
}

#if 0
/**
 * compute length of a 802.15.4 frame header.
 */
static uint8_t decode_154headerlen(uint8_t *frm)
{
uint8_t ret = 3;
    if (frm[1] & 8)
    {
        ret += 4;
    }
    if (frm[1] & 4)
    {
        ret += 6;
    }
    if (frm[1] & 0x80)
    {
        ret += 4;
    }
    if (frm[1] & 0x40)
    {
        ret += 6;
    }
    if ((frm[1] & 0xc0) != 0 && (frm[0] & 0x40) != 0)
    {
        ret -= 2;
    }
    return ret;
}
#endif

/**
 * Implementation of callback function for wuart.
 * @copydoc    usr_radio_receive_frame
 */
uint8_t * usr_radio_receive_frame(uint8_t len, uint8_t *frm, uint8_t lqi,
                                  int8_t ed, uint8_t crc)
{
uint8_t __sreg = SREG; cli();
p2p_hdr_t *pfrm;

    if ((crc == 0) && (len > sizeof(p2p_hdr_t)))
    {
        pfrm = (p2p_hdr_t*) frm;
        if (pfrm->cmd == P2P_WUART_DATA)
        {
            if( RxBuf[(RxbufIdx^1)].end == 0)
            {
                LED_TOGGLE(0);
                /* yes, we can do a swap */
                RxBuf[RxbufIdx].start = sizeof(p2p_wuart_data_t);
                RxBuf[RxbufIdx].end = len - CRC_SIZE - 1;
                RxbufIdx ^= 1;
                frm = RxBuf[RxbufIdx].data.buf;
            }
        }
        else if (pfrm->cmd == P2P_JUMP_BOOTL)
        {
//            jump_to_bootloader();
        }
        else if ((pfrm->cmd == P2P_PING_REQ) &&
                 (pfrm->dst == NodeConfig.short_addr))
        {
            SendPingReply = pfrm->src;
        }
    }
    SREG = __sreg;
    return frm;
}

/**
 * Timer ISR
 */
ISR(TIMER_IRQ_vect)
{
    if (EscapeTmoCounter > 0)
    {
        EscapeTmoCounter--;
    }

    if (LastTransmitCounter > 0)
    {
        LastTransmitCounter--;
    }
}
/**
 * Enter a integer number.
 *
 * @param base
 *        base value for the number conversion,
 *        e.g. 10 for entering a decimal number.
 *
 * @return 16 bit unsigned integer value.
 */
static  uint16_t get_number(int8_t base)
{
char buf[8];
int idx = 0;
unsigned int tmp;

    buf[0] = 0;
    do
    {
        tmp = hif_getc();
        if (tmp < 0x100)
        {
            buf[idx++] = (char) tmp & 0xff;
            buf[idx] = 0;
            hif_putc(tmp);
        }
    }
    while( (tmp != '\n') && (idx < 7) );
    return (uint16_t)strtol(buf, NULL, base);
}
