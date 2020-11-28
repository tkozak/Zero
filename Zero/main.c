/*
 * Zero.c
 *
 * Created: 23.11.2020 18:50:02
 * Author : kozak
 */ 


#include "sam.h"
#include "pins.h"

#define rx_buf_len 32

void USART_send(Sercom* s, char* data);


Sercom* SERCOM_DEBUG = SERCOM5;
uint8_t rx_count = 0;        // number of valid characters in serial receive buffer
uint8_t rx_complete = 0;     // serial communication complete (ended by \n)
char rx_buf[rx_buf_len+1];   // serial port receive buffer


// sends a null-terminated string via serial port
void USART_send(Sercom* s, char* data) {
	for (char* c=data; *c!='\0'; c++) {  // loop through char array until zero char
		while (! (s->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)) {}  // wait until data ready
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
		USART_send(SERCOM_DEBUG, rx_buf);
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


void Clocks_init(void) {
}


void RTC_init(void) {
	// setup RTC clock
	SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP( 0x6u ) | SYSCTRL_OSC32K_EN32K | SYSCTRL_OSC32K_ENABLE;
	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 ); // Wait for oscillator stabilization
	
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(1ul) | GCLK_GENDIV_DIV(1ul);   // configure generator division
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY ) {}  // wait for synchronization

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1ul) | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_GENEN;   // configure clock generator
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY ) {}  // wait for synchronization
	
	// put clock to RTC
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_RTC | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	
	PM->APBAMASK.bit.RTC_ = 1;  // enable APB bus for RTC module
	
	RTC->MODE0.CTRL.bit.SWRST=1;   // RTC reset
	while (( RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY )| (RTC->MODE0.CTRL.reg & RTC_MODE0_CTRL_SWRST)) {} // sync
	
	RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_MODE_COUNT32 | RTC_MODE0_CTRL_PRESCALER_DIV32 | RTC_MODE0_CTRL_MATCHCLR;
	RTC->MODE0.INTENSET.bit.CMP0 = 1;
	RTC->MODE0.COMP->reg = 1024ul;
	while ( RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY ) {} // sync
	
	RTC->MODE0.CTRL.bit.ENABLE=1;   // RTC enable
	while ( RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY ) {} // sync
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
	while (s->USART.SYNCBUSY.bit.CTRLB) {} // wait for sync
	s->USART.CTRLA.bit.ENABLE = 1;  // enable SERCOM
	while (s->USART.SYNCBUSY.bit.ENABLE) {} // wait for sync
	NVIC_EnableIRQ(SERCOM5_IRQn);
	rx_buf[rx_buf_len+1] = '\0';
}


int main(void)
{
	Clocks_init();
	RTC_init();
	USART_init(SERCOM_DEBUG, PIN_SERIAL_DEBUG_TX, PIN_SERIAL_DEBUG_RX, 0x1, 0x3, 9600);  // tx pad[2], rx pad[3]
	
	PORT->Group[PIN_LED.group].DIRSET.reg = 1ul << PIN_LED.pin;  // set LED as output
	PORT->Group[PIN_LED.group].OUTSET.reg = 1ul << PIN_LED.pin;
	
    /* Replace with your application code */
    while (1) 
    {
		
    }
}
