/*
 * GccApplication1.c
 *
 * Created: 2/27/2025 11:46:12 PM
 * Author : Andrew Ye
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

#define DPI_INDEX_ADDR 0x0

//#define VERSION_1_2
#define VERSION_2_0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "usb_mouse.h"
#include "srom_3360_0x03.h"
#include "pmw3360_registers.h"

// use this instead of bitshifts or LSB/MSB macros.
union MotionData {
	int16_t all;
	struct { uint8_t lo, hi; };
};

union SK6805_RGB {
	struct {
		uint8_t g;
		uint8_t r;
		uint8_t b;
	} colour;
	
	uint32_t data;
};

void init_pins();
void init_timer();
void init_interrupts();
static void init_spi();
static inline void spi_send(const uint8_t);
static inline uint8_t spi_recv();
static inline void spiWrite(const uint8_t, const uint8_t);
static inline uint8_t spiRead(const uint8_t);
static void init_pmw3360(const uint8_t);
void set_neo_rgb(union SK6805_RGB);

volatile int8_t dmwheel = 0;
volatile uint8_t dpi_index = 1;
const uint8_t dpi_profiles[] = {3, 7, 19, 39, 119}; // 400, 800, 2000, 4000, 12000
#define DPI_PROFILE_COUNT sizeof(dpi_profiles) / sizeof(dpi_profiles[0])
union SK6805_RGB dpi_colours[DPI_PROFILE_COUNT]; 

void init_pins(void)
{
	// mouse buttons: L, R, Side 1, Side 2, MWheel button
	DDRD &= ~((1 << PORTD5) | (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD4) | (1 << PORTD6));
	PORTD |= (1 << PORTD5) | (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD4) | (1 << PORTD6); // enable pullups

    // MWheel
	DDRD &= ~((1 << PORTD3) | (1 << PORTD2));
	PORTD |= (1 << PORTD3) | (1 << PORTD2); // enable pullups
	
	// dpi button
	DDRB &= ~(1 << PORTB4);
	PORTB |= (1 << PORTB4); // enable pullups
	
	// neopixel data
	DDRB |= (1 << PORTB5);
}

void init_timer()
{
	unsigned char sreg;
	sreg = SREG;
	cli();
	
	// no output on Output Compare pins, timer1 in normal mode (TOP = 65536)
	TCCR1B = (1 << CS12) | (1 << CS10); // prescaler = 1024
	TIMSK1 = (1 << TOIE1); // enable timer overflow interrupt
	
	SREG = sreg;
}

void init_interrupts()
{
	// mwheel
    EICRA |= (1 << ISC30); // int triggers on either falling or rising edge
    EIMSK |= (1 << INT3);
	
	// dpi button
	PCICR |= (1 << PCIE0); // enable pin change interrupt
	PCMSK0 |= (1 << PCINT4); // PB4

	//sei(); // enable global interrupts
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
	spiWrite(SHUTDOWN, 0xb6);
	SS_HIGH;
	_delay_ms(300);

	// drop and raise ncs to reset spi port
	SS_LOW;
	_delay_us(40);
	SS_HIGH;
	_delay_us(40);

	// power up reset
	SS_LOW;
	spiWrite(POWER_UP_RESET, 0x5a);
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
	spiWrite(CONFIG2, 0x00);
	spiWrite(SROM_ENABLE, 0x1d);
	SS_HIGH;
	_delay_ms(10);
	SS_LOW;
	spiWrite(SROM_ENABLE, 0x18);

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
	spiWrite(CONFIG2, 0x00); // Rest mode & independent X/Y DPI disabled
	spiWrite(CONTROL, 0x00); // Camera angle
	spiWrite(ANGLE_TUNE, 0x00); // Camera angle fine tuning
	spiWrite(CONFIG1, dpi); // DPI
	// LOD Stuff
	spiWrite(LIFT_CONFIG, 0x03); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
	spiWrite(MIN_SQ_RUN, 0x10); // Minimum SQUAL for zero motion data (default: 0x10)
	spiWrite(RAWDATA_THRESHOLD, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
	SS_HIGH;
	_delay_us(200);
	
	SS_LOW;
	volatile uint8_t id = spiRead(0x00);
	SS_HIGH;
}


void set_neo_rgb(union SK6805_RGB rgb)
{		
	asm("mov xl,%[g_byte]"
	"\n\tmov xh,%[r_byte]"
	"\n\tmov yl,%[b_byte]"
	
	"\n\tldi zl,8" 
	"\ng_loop_%=:"
	"\n\tsbi %[port],%[pin]"
	"\n\tnop"
	"\n\tlsl xl" 
	"\n\tbrcs g_keephi_%=" 
	"\n\tcbi %[port],%[pin]"
	"\ng_keephi_%=:"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tcbi %[port],%[pin]"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tdec zl"
	"\n\tbrne g_loop_%="
	
	"\n\tldi zl,8"
	"\nr_loop_%=:"
	"\n\tsbi %[port],%[pin]"
	"\n\tnop"
	"\n\tlsl xh"
	"\n\tbrcs r_keephi_%="
	"\n\tcbi %[port],%[pin]"
	"\nr_keephi_%=:"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tcbi %[port],%[pin]"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tdec zl"
	"\n\tbrne r_loop_%="
	
	"\n\tldi zl,8"
	"\nb_loop_%=:"
	"\n\tsbi %[port],%[pin]"
	"\n\tnop"
	"\n\tlsl yl"
	"\n\tbrcs b_keephi_%="
	"\n\tcbi %[port],%[pin]"
	"\nb_keephi_%=:"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tcbi %[port],%[pin]"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tnop"
	"\n\tdec zl"
	"\n\tbrne b_loop_%="
	: 
	: [port] "I"(0x05), [pin] "I"(5), [g_byte] "r" (rgb.colour.g), [r_byte] "r" (rgb.colour.r), [b_byte] "r" (rgb.colour.b));
	
	_delay_us(80);
}

// mwheel
ISR(INT3_vect)
{
	_delay_us(500); // crude debouncing
	if ((PIND >> PORTD3) & 1){ // A is HIGH

		dmwheel = !((PIND >> PORTD2) & 1)? -1: 1;
	}
	else{ // A is LOW
		dmwheel = ((PIND >> PORTD2) & 1)? -1: 1;
	}
	EIFR |= (1 << INTF3);
}

// dpi button
ISR(PCINT0_vect)
{
	_delay_us(500); // crude debouncing
	if(!(PINB & (1 << PINB4)))
	{
		dpi_index++;
		if (dpi_index >= DPI_PROFILE_COUNT) dpi_index = 0;
		SS_LOW;
		spiWrite(CONFIG1, dpi_profiles[dpi_index]); // set dpi in sensor
		SS_HIGH;
		set_neo_rgb(dpi_colours[dpi_index]); // set dpi indicator led
		
		eeprom_write_byte(DPI_INDEX_ADDR, dpi_index);
		
		unsigned char sreg;
		sreg = SREG;
		cli();
		
		TCNT1 = 0x0000; // reset timer
		TCCR1B = (1 << CS12) | (1 << CS10); // start timer, prescaler = 1024
		
		SREG = sreg;
	}
}

ISR(TIMER1_OVF_vect)
{
	union SK6805_RGB black;
	black.data = 0;
	set_neo_rgb(black); // turn off dpi indicator led
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // turn off timer
}

union MotionData dx, dy;
volatile uint8_t id;

int main(void)
{

	
	//
	//dpi_colours[0].colour.r = 255; // orange
	//dpi_colours[0].colour.g = 127;
	//dpi_colours[0].colour.b = 0;
	//dpi_colours[1].colour.r = 0; // teal
	//dpi_colours[1].colour.g = 255;
	//dpi_colours[1].colour.b = 200;
	//dpi_colours[2].colour.r = 0; // blue
	//dpi_colours[2].colour.g = 127;
	//dpi_colours[2].colour.b = 255;
	//dpi_colours[3].colour.r = 127; // purple
	//dpi_colours[3].colour.g = 0;
	//dpi_colours[3].colour.b = 255;
	//dpi_colours[4].colour.r = 255; // pink
	//dpi_colours[4].colour.g = 0;
	//dpi_colours[4].colour.b = 255;
	//
	//dpi_index = eeprom_read_byte(DPI_INDEX_ADDR);
	//
	//init_pins();
	//init_timer();
	//init_interrupts();
	init_spi();
	init_pmw3360(dpi_profiles[dpi_index]);
	
	//usb_init();
	//while(!usb_configured());
	//_delay_ms(500);
	
	uint8_t button_state[MAX_CHECKS];
	uint8_t index = 0;
	
	//set_neo_rgb(dpi_colours[dpi_index]);
	
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
		//usb_mouse_move(dx.all, dy.all, dmwheel);
		dmwheel = 0;
		
		// buttons
		button_state[index] = PIND & 0b01110011;
		index++;
		uint8_t debounced_state = 0xff;
		uint8_t i;
		for (i = 0; i < MAX_CHECKS; i++) debounced_state &= button_state[i];
		if (index >= MAX_CHECKS) index = 0;
		debounced_state = ~debounced_state; // buttons are active low
		
		uint8_t mouse1 = (debounced_state >> PORTD5) & 1;
		uint8_t mouse2 = (debounced_state >> PORTD0) & 1;
		uint8_t mouse3 = (debounced_state >> PORTD1) & 1;
		uint8_t mouse4 = (debounced_state >> PORTD4) & 1;
		uint8_t mouse5 = (debounced_state >> PORTD6) & 1;
		//usb_mouse_buttons(mouse1, mouse3, mouse2, mouse4, mouse5);
	}
}