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
void init_pwm();
static void init_spi();
static inline void spi_send(const uint8_t);
static inline uint8_t spi_recv();
static void init_pmw3360(const uint8_t);
void set_rgb(uint8_t, uint8_t, uint8_t);
uint8_t reverse_bits(uint8_t);

// use this instead of bitshifts or LSB/MSB macros.
union MotionData {
	int16_t all;
	struct { uint8_t lo, hi; };
};

volatile int8_t dmwheel = 0;

volatile uint8_t dpi_index = 1;
const uint8_t dpi_profiles[] = {3, 7, 19, 39, 119}; // 400, 800, 2000, 4000, 12000
const uint8_t dpi_colours[][3] = {{255, 90, 90}, {225, 255, 90}, {90, 255, 180}, {90, 115, 255}, {150, 90, 255}}; // red, yellow, teal, blue, lavender
#define DPI_PROFILE_COUNT sizeof(dpi_profiles) / sizeof(dpi_profiles[0])

#define NEO_PORT PORTB
#define NEO_PIN PORTB5

volatile uint8_t overflow_count = 0;

void init_pins(void)
{
	// mouse buttons: L, R, Side 1, Side 2, MWheel button
	DDRD &= ~((1 << PORTD3) | (1 << PORTD2) | (1 << PORTD4) | (1 << PORTD6) | (1 << PORTD5));
	PORTD |= (1 << PORTD3) | (1 << PORTD2) | (1 << PORTD4) | (1 << PORTD6) | (1 << PORTD5); // enable pullups

    // MWheel
	DDRD &= ~((1 << PORTD0) | (1 << PORTD1));
	PORTD |= (1 << PORTD0) | (1 << PORTD1); // enable pullups
	
	// dpi button
	DDRB &= ~(1 << PORTB4);
	PORTB |= (1 << PORTB4); // enable pullups
	
	// neopixel data
	DDRB |= (1 << PORTB5);
}

void init_interrupts()
{
	// mwheel
    EICRA |= (1 << ISC00); // int triggers on either falling or rising edge
    EIMSK |= (1 << INT0);
	
	// dpi button
	PCICR |= (1 << PCIE0); // enable pin change interrupt
	PCMSK0 |= (1 << PCINT4); // PB4

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
}

void init_pwm()
{
	unsigned char sreg;
	sreg = SREG; // Save global interrupt flag
	cli(); // Disable interrupts

	// Timer 4
	OCR4B = 10; // 50% duty
	OCR4C = 20;
	TCCR4A = (1 << COM4B0) | (1 << PWM4B); // clear on compare match 
	TCCR4B = (1 << CS40); // enable timer, no prescaler
	TCCR4D &= 0b11111100; // waveform generator mode 4[1:0] = 00, fast PWM
	
	TIMSK4 = (1 << TOIE4);
	
	SREG = sreg; // Restore global interrupt flag
}

ISR(TIMER4_OVF_vect)
{
	overflow_count++;
	if (overflow_count >= 10) TCCR4B = 0;
}

uint8_t reverse_bits(uint8_t b)
{
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

void set_neo_rgb(uint8_t r, uint8_t g, uint8_t b)
{	
	uint8_t r_rev = reverse_bits(r);
	uint8_t g_rev = reverse_bits(g);
	uint8_t b_rev = reverse_bits(b);
	// bit send order: (first) G7 ... G0, R7 ... R0, B7 ... B0 (last)
	uint32_t brg = ((uint32_t)b_rev << 16) | ((uint32_t)r_rev << 8) | g_rev;
	brg &= 0x00ffffff;
	uint8_t* pb_addr = &PORTB;
	uint8_t cnt = 10;
	
	//asm("head:"
		//"\n\t"
		//"out %a[pb_addr], 0x20"
		//"\n\t"
		//"out %a[pb_addr], 0x00"
		//"\n\t"
		//"dec %[cnt]"
		//"\n\t"
		//"cpi %[cnt], 0x00"
		//"\n\t"
		//"brne head"
		//"\n"
		//: [pb_addr] "+e"(pb_addr), [cnt] "+r"(cnt));
	
	//unsigned char sreg;
	//sreg = SREG;
	//cli(); // disable interrupts
//
	//volatile uint16_t i = numBytes; // Loop counter
	//volatile uint8_t *ptr = pixels, // Pointer to next byte
		//b = *ptr++,                 // Current byte value
		//hi,                         // PORT w/output bit set high
		//lo;                         // PORT w/output bit set low
	//volatile uint8_t n1, n2 = 0; // First, next bits out
    //// Same as above, just switched to PORTB and stripped of comments.
    //hi = PORTB | pinMask;
    //lo = PORTB & ~pinMask;
    //n1 = lo;
    //if (b & 0x80)
    //n1 = hi;

    //asm volatile(
        //"headB:"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n2]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n1]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 6"
        //"\n\t"
        //"mov %[n2]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n1]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n2]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 5"
        //"\n\t"
        //"mov %[n1]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n2]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n1]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 4"
        //"\n\t"
        //"mov %[n2]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n1]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n2]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 3"
        //"\n\t"
        //"mov %[n1]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n2]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n1]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 2"
        //"\n\t"
        //"mov %[n2]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n1]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n2]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 1"
        //"\n\t"
        //"mov %[n1]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n2]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n1]"
        //"\n\t"
        //"rjmp .+0"
        //"\n\t"
        //"sbrc %[byte] , 0"
        //"\n\t"
        //"mov %[n2]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"sbiw %[count], 1"
        //"\n\t"
        //"out  %[port] , %[hi]"
        //"\n\t"
        //"mov  %[n1]   , %[lo]"
        //"\n\t"
        //"out  %[port] , %[n2]"
        //"\n\t"
        //"ld   %[byte] , %a[ptr]+"
        //"\n\t"
        //"sbrc %[byte] , 7"
        //"\n\t"
        //"mov %[n1]   , %[hi]"
        //"\n\t"
        //"out  %[port] , %[lo]"
        //"\n\t"
        //"brne headB"
        //"\n"
        //: [byte] "+r"(b), [n1] "+r"(n1), [n2] "+r"(n2), [count] "+w"(i)
        //: [port] "I"(_SFR_IO_ADDR(PORTB)), [ptr] "e"(ptr), [hi] "r"(hi),
        //[lo] "r"(lo)
	//);
	
	
	//SREG = sreg;

}

// mwheel
ISR(INT0_vect)
{
	_delay_us(500); // crude debouncing
	if ((PIND >> PORTD0) & 1){ // A is HIGH

		dmwheel = !((PIND >> PORTD1) & 1)? -1: 1;
	}
	else{ // A is LOW
		dmwheel = ((PIND >> PORTD1) & 1)? -1: 1;
	}
	EIFR |= (1 << INTF0);
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
		spiWrite(CONFIG1, dpi_profiles[dpi_index]);
		SS_HIGH;
		//set_neo_rgb(dpi_colours[dpi_index]);
	}
}

int main(void)
{

	union MotionData dx, dy;
	
	init_pins();
	init_interrupts();

	//init_pwm();

	init_spi();
	init_pmw3360(dpi_profiles[dpi_index]);
	usb_init();
	while(!usb_configured());
	_delay_ms(500);
	
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blue = 0;
	
	uint8_t button_state[MAX_CHECKS];
	uint8_t index = 0;
	
	//set_neo_rgb(200,200,0);
	
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
		uint8_t mouse4 = (debounced_state >> PORTD6) & 1;
		uint8_t mouse5 = (debounced_state >> PORTD4) & 1;
		usb_mouse_buttons(mouse1, mouse3, mouse2, mouse4, mouse5);
	}
}