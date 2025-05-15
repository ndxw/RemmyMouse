/*
 * GccApplication1.c
 *
 * Created: 2/27/2025 11:46:12 PM
 * Author : XxPan
 */ 

#define F_CPU 16000000UL // 16 MHz

#define PORT_SPI PORTB
#define DDR_SPI	DDRB

#define DD_SS	0 // aka NCS
#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

#define SS_LOW	(PORT_SPI &= ~(1<<DD_SS))
#define SS_HIGH	(PORT_SPI |= (1<<DD_SS))

#define MAX_CHECKS 8

//#define VERSION_1_2
#define VERSION_2_0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usb_mouse.h"
#include "circle.h"
#include "srom_3360_0x03.h"
#include "pmw3360_registers.h"

void init_pins();
void toggle_led();
void init_pwm();
static void init_spi();
static inline void spi_send(const uint8_t);
static inline uint8_t spi_recv();
static void init_pmw3360(const uint8_t);
void set_rgb(uint8_t, uint8_t, uint8_t);

// use this instead of bitshifts or LSB/MSB macros.
union MotionData {
	int16_t all;
	struct { uint8_t lo, hi; };
};

volatile int8_t dmwheel = 0;

void init_pins(void)
{
    
	#if defined(VERSION_1_2)
    // RED LED
	DDRD |= (1 << PORTD6);
	
	// RGB LED
	DDRB |= (1 << PORTB6) | (1 << PORTB5); // RGB green, RGB red
	PORTB |= (1 << PORTB6) | (1 << PORTB5);
	DDRD |= (1 << PORTD7); // RGB blue
	PORTD |= (1 << PORTD7);

    // mouse buttons: L, R, Side 1, Side 2, MWheel button
	DDRD &= ~((1 << PORTD0) | (1 << PORTD1) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5));
	PORTD |= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5); // enable pullups

    // MWheel
	DDRE &= ~(1 << PORTE6);
	DDRD &= ~(1 << PORTD2);
	PORTE |= (1 << PORTE6); // enable pullups
	PORTD |= (1 << PORTD2);

	#elif defined(VERSION_2_0)
	// mouse buttons: L, R, Side 1, Side 2, MWheel button
	DDRD &= ~((1 << PORTD3) | (1 << PORTD2) | (1 << PORTD4) | (1 << PORTD6) | (1 << PORTD5));
	PORTD |= (1 << PORTD3) | (1 << PORTD2) | (1 << PORTD4) | (1 << PORTD6) | (1 << PORTD5); // enable pullups

    // MWheel
	DDRD &= ~((1 << PORTD0) | (1 << PORTD1));
	PORTD |= (1 << PORTD0) | (1 << PORTD1); // enable pullups

	#endif
}

void init_interrupts()
{
    #if defined(VERSION_1_2)
	EICRB |= (1 << ISC60); // int triggers on either falling or rising edge
	EIMSK |= (1 << INT6);
    #elif defined(VERSION_2_0)
    EICRA |= (1 << ISC00); // int triggers on either falling or rising edge
    EIMSK |= (1 << INT0);
    #endif
	sei(); // enable global interrupts
}

// spi functions
static void init_spi(void)
{
	DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS); // outputs
	DDRB |= (1 << 0); 
	PORTB |= (1 << 0); // set the hardware SS pin to low to enable SPI
	// MISO pullup input is already done in hardware
	// enable spi, master mode, mode 3, clock rate = fck/4 = 2MHz
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA);
}

static inline void spi_send(const uint8_t b)
{
	SPDR = b;
	while (!(SPSR & (1<<SPIF)));
}

static inline uint8_t spi_recv(void)
{
	spi_send(0x00);
	return SPDR;
}

static inline void spiWrite(const uint8_t addr, const uint8_t data)
{
	spi_send(addr | 0x80);
	spi_send(data);
	_delay_us(180); // maximum of t_SWW, t_SWR
}

static inline uint8_t spiRead(const uint8_t addr)
{
	spi_send(addr);
	_delay_us(160); // t_SRAD
	uint8_t data = spi_recv();
	_delay_us(20);
	return data;
}

// dpi argument is what's written to register 0x0f
// actual dpi value = (dpi + 1) * 100
static void init_pmw3360(const uint8_t dpi)
{
	const uint8_t *psrom = srom;

	SS_HIGH;
	_delay_ms(3);

	// shutdown first
	SS_LOW;
	spiWrite(0x3b, 0xb6);
	SS_HIGH;
	_delay_ms(300);

	// drop and raise ncs to reset spi port
	SS_LOW;
	_delay_us(40);
	SS_HIGH;
	_delay_us(40);

	// power up reset
	SS_LOW;
	spiWrite(0x3a, 0x5a);
	SS_HIGH;
	_delay_ms(50);

	// read from 0x02 to 0x06
	SS_LOW;
	spiRead(0x02);
	spiRead(0x03);
	spiRead(0x04);
	spiRead(0x05);
	spiRead(0x06);

	// srom download
	spiWrite(0x10, 0x00);
	spiWrite(0x13, 0x1d);
	SS_HIGH;
	_delay_ms(10);
	SS_LOW;
	spiWrite(0x13, 0x18);

	spi_send(0x62 | 0x80);
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		_delay_us(16);
		spi_send(pgm_read_byte(psrom++));
	}
	_delay_us(18);
	SS_HIGH;
	_delay_us(200);

	// configuration/settings
	SS_LOW;
	spiWrite(0x10, 0x00); // Rest mode & independent X/Y DPI disabled
	spiWrite(0x0d, 0x00); // Camera angle
	spiWrite(0x11, 0x00); // Camera angle fine tuning
	spiWrite(0x0f, dpi); // DPI
	// LOD Stuff
	spiWrite(0x63, 0x02); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
	spiWrite(0x2b, 0x10); // Minimum SQUAL for zero motion data (default: 0x10)
	spiWrite(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
	SS_HIGH;
	_delay_us(200);
}

#if defined(VERSION_1_2)
void toggle_led(void)
{
	PORTD ^= (1 << PORTD6); // red LED on PD6
}

void init_pwm()
{
	unsigned char sreg;
	sreg = SREG; // Save global interrupt flag
	cli(); // Disable interrupts

	OCR1A = 90*255/100; // RGB red duty cycle
	OCR1B = 90*255/100; // green duty cycle
	OCR4C = 255; // p. 171 in fast PWM mode, timer 4 uses OCR4C reg for TOP
	OCR4D = 90*255/100; // blue duty cycle
	
	// Timer 1 for RGB red (PB5/OC1A)
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // WGM mode 5
	TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // clk prescaler = 1024
	
	// Timer 4 for RGB green (PB6/OC4B) and blue (PD7/OC4D)
	TCCR4B = (1 << CS43) | (1 << CS41) | (1 << CS40); // clk prescaler = 1024
	TCCR4C = (1 << COM4D1) | (1 << PWM4D);
	TCCR4D &= 0b11111100; // waveform generator mode 4[1:0] = 00, fast PWM
	
	SREG = sreg; // Restore global interrupt flag
}

void set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	unsigned char sreg;
	uint8_t red, green, blue;
	
	red = 255 - r;
	green = 255 - g;
	blue = 255 - b;
	//blue = blue / 255.0 * 100;
	
	sreg = SREG;
	cli();
	
	OCR1A = red;
	OCR1B = green;
	OCR4D = blue;
	
	SREG = sreg;
}
#endif

#if defined(VERSION_1_2)
// mwheel
ISR(INT6_vect)
{
	if ((PINE >> PORTE6) & 1){ // A is HIGH

		dmwheel = !((PIND >> PORTD2) & 1)? 1: -1;
	}
	else{ // A is LOW
		dmwheel = ((PIND >> PORTD2) & 1)? 1: -1;
	}
	EIFR |= (1 << INTF6) | (1 << INTF2);
}
#elif defined(VERSION_2_0)
// mwheel
ISR(INT0_vect)
{
	if ((PINE >> PORTE6) & 1){ // A is HIGH

		dmwheel = !((PIND >> PORTD2) & 1)? 1: -1;
	}
	else{ // A is LOW
		dmwheel = ((PIND >> PORTD2) & 1)? 1: -1;
	}
	EIFR |= (1 << INTF6) | (1 << INTF2);
}
#endif

int main(void)
{
	// dpi settings
	uint8_t dpi_index = 2;
	uint8_t dpis[] = {3, 7, 15};
	
	union MotionData dx, dy;
	
	init_pins();
	init_interrupts();
	#if defined(VERSION_1_2)
	init_pwm();
	#endif
	init_spi();
	init_pmw3360(dpis[dpi_index]);
	usb_init();
	while(!usb_configured());
	_delay_ms(500);
	
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blue = 0;
	
	uint8_t button_state[MAX_CHECKS];
	uint8_t index = 0;
	
	while (1)
	{
		_delay_ms(1);
		
		// wheel handled by interrupts

		// sensor
		SS_LOW;
		spi_send(MOTION_BURST);
		_delay_us(35);
		spi_send(0x00);
		spi_send(0x00);
		dx.lo = spi_recv();
		dx.hi = spi_recv();
		dy.lo = spi_recv();
		dy.hi = spi_recv();
		SS_HIGH;
		usb_mouse_move(dx.all, dy.all, dmwheel);
		dmwheel = 0;
		
		// buttons
		button_state[index] = PIND & 0b01111100;
		index++;
		uint8_t debounced_state = 0xff;
		uint8_t i;
		for (i = 0; i < MAX_CHECKS; i++) debounced_state &= button_state[i];
		if (index >= MAX_CHECKS) index = 0;
		debounced_state = ~debounced_state; // buttons are active low
		
		uint8_t mouse1 = (debounced_state >> PORTD3) & 1;
		uint8_t mouse2 = (debounced_state >> PORTD2) & 1;
		uint8_t mouse3 = (debounced_state >> PORTD5) & 1;
		uint8_t mouse4 = (debounced_state >> PORTD4) & 1;
		uint8_t mouse5 = (debounced_state >> PORTD6) & 1;
		usb_mouse_buttons(mouse1, mouse3, mouse2, mouse4, mouse5);
	}
}