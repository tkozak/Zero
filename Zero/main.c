/*
 * Zero.c
 *
 * Created: 23.11.2020 18:50:02
 * Author : kozak
 */ 


#include "sam.h"
#include "pins.h"


void USART_send(Sercom* s, char* data);

uint32_t value = 0;

// sends a null-terminated string via serial port
void USART_send(Sercom* s, char* data) {
	for (char* c=data; *c != '\0'; c++) {  // loop through char array until zero char
		while (! s->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) {}  // wait until data ready
		s->USART.DATA.reg = (uint8_t) *c;  // send character
	}
}


void RTC_Handler( void ) {
	
	//PORT->Group[PIN_LED.group].OUT.reg
	if (value == 0) {
		value = 1;
		PORT->Group[PIN_LED.group].OUTSET.reg = 1ul << PIN_LED.pin;
	} else {
		value = 0;
		PORT->Group[PIN_LED.group].OUTCLR.reg = 1ul << PIN_LED.pin;
	};
	USART_send(SERCOM_DEBUG, "Hello\n");
	
	RTC->MODE0.INTFLAG.reg |= 1; // clear interrupt flag
}


void init_clocks(void) {
	
}


void RTC_init(void) {
	// setup RTC clock
	SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP( 0x6u ) |
	SYSCTRL_OSC32K_EN32K |
	SYSCTRL_OSC32K_ENABLE;
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
	
	PORT->Group[pin_tx.group].PINCFG[pin_tx.pin].reg = PORT_PINCFG_PMUXEN;  // enable pin to be used by SERCOM peripheral
	PORT->Group[pin_rx.group].PINCFG[pin_rx.pin].reg = PORT_PINCFG_PMUXEN;  // enable pin to be used by SERCOM peripheral
	PORT->Group[pin_tx.group].PMUX[pin_tx.pin/2].reg = PORT_PMUX_PMUXE_D | PORT_PMUX_PMUXO_D;  // set peripheral function (D) for both pins
	
	PM->APBCMASK.bit.SERCOM5_ = 1;  // enable APB bus for SERCOM5 module
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN; // set generic cloc	k 0 for SERCOM module (1 MHz)
	s->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |    // internal clock
							SERCOM_USART_CTRLA_TXPO(pad_tx_flag) | SERCOM_USART_CTRLA_RXPO(pad_rx_flag) |  // Tx pad[2], Rx pad[3]
							SERCOM_USART_CTRLA_DORD |  // LSb first
							SERCOM_USART_CTRLA_FORM(0x0) |   // USART frame with no parity check
							SERCOM_USART_CTRLA_SAMPR(0x2);  // 8x oversampling, arithmetic baud generation
	s->USART.CTRLB.reg = 0x0;   // all zeros for CTRLB
	s->USART.BAUD.reg = baud;   // set baud
	s->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN; //enable Tx and Rx
	while (s->USART.SYNCBUSY.bit.CTRLB) {} // wait for sync
	s->USART.CTRLA.bit.ENABLE = 1;  // enable SERCOM
	while (s->USART.SYNCBUSY.bit.ENABLE) {} // wait for sync		
}



int main(void)
{
	RTC_init();
	USART_init(SERCOM_DEBUG, PIN_SERIAL_DEBUG_TX, PIN_SERIAL_DEBUG_RX, 0x1, 0x3, 9600);  // tx pad[2], rx pad[3]
	
	PORT->Group[PIN_LED.group].DIRSET.reg = 1ul << PIN_LED.pin;  // set LED as output
	PORT->Group[PIN_LED.group].OUTSET.reg = 1ul << PIN_LED.pin;
	value = 1;
	
    /* Replace with your application code */
    while (1) 
    {
		
    }
}
