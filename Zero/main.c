/*
 * Zero.c
 *
 * Created: 23.11.2020 18:50:02
 * Author : kozak
 */ 

#include "sam.h"
#include "pins.h"
#include <stdlib.h>

#define rx_buf_len 32
#define OSC32_VARIANT 1  // 0 = OSC32KULP; 1 = OSC32K, 2 = XOSC32K

Sercom* SERCOM_DEBUG = SERCOM5;
uint8_t rx_count = 0;        // number of valid characters in serial receive buffer
uint8_t rx_complete = 0;     // serial communication complete (ended by \n)
char rx_buf[rx_buf_len+1];   // serial port receive buffer

const uint16_t servo_cc_middle = 180;
const int8_t servo_pos_max = 100;
// one timer step is 0.008 ms
// 187 cycles -> approx 1.5 ms (theoretical middle position)
// 1 ms = 125, 2 ms = 250; tested safe working range of values 70 - 290


void set_servo_pos(int8_t pos) {
	if (pos > servo_pos_max) pos = servo_pos_max;
	if (pos < -servo_pos_max) pos = -servo_pos_max;
	TCC0->CC[2].reg = (uint16_t) servo_cc_middle + pos;  // set compare channel 2 to desired pulse length value
	while (TCC0->SYNCBUSY.bit.CC2); // wait for sync
}


// sends a null-terminated string via serial port
void USART_send(Sercom* s, char* data) {
	for (char* c=data; *c!='\0'; c++) {  // loop through char array until zero char
		while (! (s->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));  // wait until data ready
		s->USART.DATA.reg = (uint8_t) *c;  // send character
	}
}


void USART_receive_byte(Sercom* s) {
	if (rx_complete) rx_count = 0;
	rx_buf[rx_count] = (char) s->USART.DATA.reg; // get received byte from DATA register
	rx_complete = (rx_buf[rx_count] == '\n');  // end of message
	if ((!rx_complete) & (rx_count < rx_buf_len)) rx_count++;   // advance buffer caret if not complete and if not at end of buffer
}


void RTC_Handler( void ) {
	PORT->Group[PIN_LED.group].OUT.reg ^= 1ul << PIN_LED.pin;  // toggle builtin LED
	if (rx_complete) {  // a complete command is in the serial receive buffer
		rx_buf[rx_count] = '\0'; // insert null to terminate the string
		char *eptr;
		int8_t value = strtol(rx_buf, &eptr, 10);
		
		//USART_send(SERCOM_DEBUG, rx_buf);
		set_servo_pos(value);
		
		//while (! (SERCOM_DEBUG->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));  // wait until data ready
		//SERCOM_DEBUG->USART.DATA.reg = value;  // send character
		
		rx_complete = 0; rx_count = 0; // mark the command as processed
	}
	RTC->MODE0.INTFLAG.reg |= 1; // clear interrupt flag
}


void SERCOM5_Handler (void) {
	if (SERCOM5->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) {
		USART_receive_byte(SERCOM_DEBUG);
		SERCOM5->USART.INTFLAG.reg |= SERCOM_USART_INTFLAG_RXC; // clear interrupt flag
	}
}


void Clocks_init() {
	
	/******  32.768 kHz oscillator  -->  GEN 01 ******/
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(0x1) | GCLK_GENDIV_DIV(1ul);   // configure generator division
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )  // wait for synchronization
	// oscillator start
	switch (OSC32_VARIANT) {
		case 1: {		// use internal oscillator
			uint32_t calib = (*((uint32_t *) FUSES_OSC32K_CAL_ADDR) & FUSES_OSC32K_CAL_Msk) >> FUSES_OSC32K_CAL_Pos;  // get calibration values
			SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(calib) | SYSCTRL_OSC32K_STARTUP( 0x6u ) | // set startup time to appprox 2ms
												SYSCTRL_OSC32K_EN32K | SYSCTRL_OSC32K_ENABLE;
			while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 ); // wait until oscillator is ready
			GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0x1) | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_GENEN;   // configure clock generator source
			while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  // wait for synchronization
		}
		case 2: {     // use external oscillator
			SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | // set startup time to approx 2s
			SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K ;
			SYSCTRL->XOSC32K.bit.ENABLE = 1 ;    // separate call, as described in chapter 15.6.3 
			while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 );  // wait until oscillator is ready
			GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0x1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;   // configure clock generator source
			while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY ); // wait for synchronization
		}
	}
	
	
}


void RTC_init(void) {
	// provide clock from GEN 01 to RTC
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_RTC | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	PM->APBAMASK.bit.RTC_ = 1;  // enable APB bus for RTC module
	RTC->MODE0.CTRL.bit.SWRST=1;   // RTC reset
	while (( RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY ) | (RTC->MODE0.CTRL.reg & RTC_MODE0_CTRL_SWRST)); // sync
	
	RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_MODE_COUNT32 | RTC_MODE0_CTRL_PRESCALER_DIV32 | RTC_MODE0_CTRL_MATCHCLR;
	RTC->MODE0.INTENSET.bit.CMP0 = 1;
	RTC->MODE0.COMP->reg = 1024ul;
	while ( RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY ); // sync
	
	RTC->MODE0.CTRL.bit.ENABLE=1;   // RTC enable
	while ( RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY ); // sync
	NVIC_EnableIRQ(RTC_IRQn);
}


void USART_init(Sercom* s, struct gpio_pin pin_tx, struct gpio_pin pin_rx, uint8_t pad_tx_flag, uint8_t pad_rx_flag, uint32_t baudrate) {
	const uint32_t sample_rate = 8;
	uint32_t baud = (uint32_t) (65536*(1.0 - sample_rate*baudrate/1e6)); 
	PORT->Group[pin_rx.group].DIRCLR.reg = 1ul << pin_rx.pin;
	PORT->Group[pin_tx.group].PINCFG[pin_tx.pin].reg = PORT_PINCFG_PMUXEN;  // enable pin to be used by SERCOM peripheral
	PORT->Group[pin_rx.group].PINCFG[pin_rx.pin].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;  // enable pin to be used by SERCOM peripheral and enable input
	PORT->Group[pin_tx.group].PMUX[pin_tx.pin/2].reg = PORT_PMUX_PMUXE_D | PORT_PMUX_PMUXO_D;  // set peripheral function (D) for both pins
	
	PM->APBCMASK.bit.SERCOM5_ = 1;  // enable APB bus for SERCOM5 module
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN; // set generic clock 0 for SERCOM module (1 MHz)
	s->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |    // internal clock
							SERCOM_USART_CTRLA_TXPO(pad_tx_flag) | SERCOM_USART_CTRLA_RXPO(pad_rx_flag) |  // Tx pad[2], Rx pad[3]
							SERCOM_USART_CTRLA_DORD |  // LSb first
							SERCOM_USART_CTRLA_FORM(0x0) |   // USART frame with no parity check
							SERCOM_USART_CTRLA_SAMPR(0x2);  // 8x oversampling, arithmetic baud generation
	s->USART.CTRLB.reg = 0x0;   // all zeros for CTRLB
	s->USART.BAUD.reg = baud;   // set baud
	s->USART.INTENSET.reg= SERCOM_USART_INTENSET_RXC;  // enable RX start interrupt
	s->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN; //enable Tx and Rx
	while (s->USART.SYNCBUSY.bit.CTRLB); // wait for sync
	s->USART.CTRLA.bit.ENABLE = 1;  // enable SERCOM
	while (s->USART.SYNCBUSY.bit.ENABLE); // wait for sync
	NVIC_EnableIRQ(SERCOM5_IRQn);
	rx_buf[rx_buf_len+1] = '\0';
}


void TCC_init( void ) {
	// init PWM output on pin 6 using TCC0 instance
	PORT->Group[PIN_6.group].DIRSET.reg = 1ul << PIN_6.pin;   // set as output
	PORT->Group[PIN_6.group].PINCFG[PIN_6.pin].reg = PORT_PINCFG_PMUXEN;   // enable use of pin by peripheral
	PORT->Group[PIN_6.group].PMUX[PIN_6.pin/2].reg = PORT_PMUX_PMUXE_F;  // function F for the even pin (PA20)
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC0_TCC1 | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;   // provide clock from GEN 00 (1 MHz)
	PM->APBCMASK.bit.TCC0_ = 1;  // enable APB bus	
	TCC0->CTRLA.bit.ENABLE = 0; // reset TCC
	while (TCC0->SYNCBUSY.bit.ENABLE); // wait for sync
	TCC0->CTRLA.bit.SWRST = 1; // reset TCC
	while (TCC0->SYNCBUSY.bit.SWRST); // wait for sync
	
	TCC0->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC | TCC_CTRLA_PRESCALER_DIV8 | TCC_CTRLA_RESOLUTION_NONE;  // set prescaler to 8, prescaler sync
	// clock pulse length 8/1e6 s = 0.008 ms
	TCC0->WEXCTRL.reg = TCC_WEXCTRL_OTMX(0x0);  // set output matrix to case 0: CC0 -> WO0, CC1 -> WO1, ..., CC0 -> WO4, CC1 -> WO5, ...  
	TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // set normal PWM mode
	while (TCC0->SYNCBUSY.bit.WAVE); // wait for sync
	TCC0->PER.reg = TCC_PER_PER(2499);   // set TOP to 2499, so period will be 2500 cycles -> 20 ms
	while (TCC0->SYNCBUSY.bit.PER); // wait for sync
	TCC0->CC[2].reg = TCC_CC_CC(servo_cc_middle);  // set compare channel 2 to middle position (defined constant)
	while (TCC0->SYNCBUSY.bit.CC2); // wait for sync
	
	TCC0->CTRLA.bit.ENABLE = 1;  // enable TCC
	while (TCC0->SYNCBUSY.bit.ENABLE); // wait for sync
}


int main(void)
{
	Clocks_init();
	RTC_init();
	USART_init(SERCOM_DEBUG, PIN_SERIAL_DEBUG_TX, PIN_SERIAL_DEBUG_RX, 0x1, 0x3, 9600);  // tx pad[2], rx pad[3]
	TCC_init();
	
	PORT->Group[PIN_LED.group].DIRSET.reg = 1ul << PIN_LED.pin;  // set LED as output
	PORT->Group[PIN_LED.group].OUTSET.reg = 1ul << PIN_LED.pin;
	
    /* Replace with your application code */
    while (1) 
    {
		
    }
}
