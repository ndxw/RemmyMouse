/* firmware for some 3360 mouse
 *
 * Copyright (c) 2016 qsxcv
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// this code assumes

// 3360 NCS			<-> B0
// 3360 SCLK		<-> B1
// 3360 MOSI		<-> B2
// 3360 MISO		<-> B3
// 3360 NRESET set to high by pcb.
// wheel B			<-> C6 (wheel up)
// wheel A			<-> C7 (wheel down)
// left button		<-> D0
// right button		<-> D1
// wheel button		<-> D2
// L back side		<-> D3
// L forwards side	<-> D4
// dpi button		<-> D5
// R LED			<-> B5
// G LED			<-> B6
// B LED			<-> D7


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "srom_3360_0x03.h"
#include "usb_mouse.h"

#define delay_us(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000000))
#define delay_ms(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000))


#define PORT_SPI PORTB
#define DDR_SPI	DDRB

#define DD_SS	0 // aka NCS
#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

#define SS_LOW	(PORT_SPI &= ~(1<<DD_SS))
#define SS_HIGH	(PORT_SPI |= (1<<DD_SS))

#define DEBOUNCE_TIME 32 // debounce time in units of 125us. only affects release latency.

// modify these and corresponding stuff in pins_init() as needed
#define WHL_A_IS_HIGH	(!!(PINC & (1<<6)))
#define WHL_B_IS_HIGH	(!!(PINC & (1<<7)))

// use this instead of bitshifts or LSB/MSB macros.
union motion_data {
	int16_t all;
	struct { uint8_t lo, hi; };
};


static void pins_init(void)
{
	// buttons
	PORTD |= 0b00111111; // L, R, M, FSB, RSB, DPI
	
	// wheel (mechanical encoder, quadrature outputs A/B)
	DDRC &= ~((1<<6) | (1<<7)); // pullup inputs on C6, C7
	PORTC |= (1<<6) | (1<<7);
	
	// teensy LED
	DDRD |= (1<<6);
	
	// RGB LED
	DDRB |= (1<<5) | (1<<6);
	DDRD |= (1<<7);
	PORTB |= (1<<5); // Red (off)
	PORTB |= (1<<6); // Green (off)
	PORTD |= (1<<7); // Blue (off)
	
	// not necssary if NRESET is pulled high in PCB
	//DDRC |= (1<<7); PORTC |= (1<<7); // C7, NRESET high output

	EICRA = 0b01010101; // generate interrupt request on any edge of D0/D1/D2/D3
	EIMSK = 0; // but don't enable any actual interrupts
	EIFR = 0b00001111; // clear EIFR
}


// spi functions
static void spi_init(void)
{
	DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS); // outputs
	DDRB |= (1<<0); PORTB |= (1<<0); // set the hardware SS pin to low to enable SPI
	// MISO pullup input is already done in hardware
	// enable spi, master mode, mode 3, clock rate = fck/4 = 2MHz
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
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

static inline void spi_write(const uint8_t addr, const uint8_t data)
{
	spi_send(addr | 0x80);
	spi_send(data);
	delay_us(180); // maximum of t_SWW, t_SWR
}

static inline uint8_t spi_read(const uint8_t addr)
{
	spi_send(addr);
	delay_us(160); // t_SRAD
	uint8_t data = spi_recv();
	delay_us(20);
	return data;
}

// dpi argument is what's written to register 0x0f
// actual dpi value = (dpi + 1) * 100
static void pmw3360_init(const uint8_t dpi)
{
	const uint8_t *psrom = srom;

	SS_HIGH;
	delay_ms(3);

	// shutdown first
	SS_LOW;
	spi_write(0x3b, 0xb6);
	SS_HIGH;
	delay_ms(300);

	// drop and raise ncs to reset spi port
	SS_LOW;
	delay_us(40);
	SS_HIGH;
	delay_us(40);

	// power up reset
	SS_LOW;
	spi_write(0x3a, 0x5a);
	SS_HIGH;
	delay_ms(50);

	// read from 0x02 to 0x06
	SS_LOW;
	spi_read(0x02);
	spi_read(0x03);
	spi_read(0x04);
	spi_read(0x05);
	spi_read(0x06);

	// srom download
	spi_write(0x10, 0x00);
	spi_write(0x13, 0x1d);
	SS_HIGH;
	delay_ms(10);
	SS_LOW;
	spi_write(0x13, 0x18);

	spi_send(0x62 | 0x80);
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		delay_us(16);
		spi_send(pgm_read_byte(psrom++));
	}
	delay_us(18);
	SS_HIGH;
	delay_us(200);

	// configuration/settings
	SS_LOW;
	spi_write(0x10, 0x00); // Rest mode & independant X/Y DPI disabled
	spi_write(0x0d, 0x00); // Camera angle
	spi_write(0x11, 0x00); // Camera angle fine tuning
	spi_write(0x0f, dpi); // DPI
	// LOD Stuff
	spi_write(0x63, 0x02); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
	spi_write(0x2b, 0x10); // Minimum SQUAL for zero motion data (default: 0x10)
	spi_write(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
	SS_HIGH;
	delay_us(200);
}

// angle snapping
static void angle_init(const uint8_t angle) {
	SS_LOW;
	spi_write(0x42, angle); // Angle snapping: 0x00 = off, 0x80 = on
	SS_HIGH;
}


int main(void)
{
	union motion_data x, y;

	// set clock prescaler for 16MHz
	CLKPR = 0x80;
	CLKPR = 0x00;

	pins_init();

	// previous state to compare against for debouncing
	uint8_t btn_prev = (~PIND) & 0b00111111; // read L, R, M, FSB, RSB, DPI
	// time (in 125us) button has been unpressed.
	// consider button to be released if this time exceeds DEBOUNCE_TIME.
	uint8_t btn_time[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	// RGB Timers
	uint8_t led_max_timer = 16; // Timer maximum
	uint8_t led_r_timer = 0; // Red LED Timer initial value
	uint8_t led_g_timer = 0; // Green LED Timer initial value
	uint8_t led_b_timer = 0; // Blue LED Timer initial value
	// RGB Brightness
	uint8_t led_bright_index = 4;
	float led_bright[] = {0, 0.25, 0.5, 0.75, 1};
	float led_rgb_brightness = led_bright[led_bright_index];
	// RGB Initial Values
	uint8_t led_r_value = 0; // Red LED brightness: 0-255
	uint8_t led_g_value = 0; // Green LED brightness: 0-255
	uint8_t led_b_value = 0; // Blue LED brightness: 0-255
	// RGB Colour Selection
	uint8_t led_colours_index = 1;
	uint8_t led_colours[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
	uint8_t led_colour = led_colours[led_colours_index]; 
	// Profiles Intitial Values
	uint8_t p1_led_colour = 1; // Red
	uint8_t p2_led_colour = 2; // Pink
	uint8_t p3_led_colour = 4; // Purple
	uint8_t p1_led_brightness = 4; // 0 = 0, 1 = 0.25, 2 = 0.5, 3 = 0.75, 4 = 1
	uint8_t p2_led_brightness = 4;
	uint8_t p3_led_brightness = 4;
	// binary OR of all button states since previous usb transmission
	uint8_t btn_usb = 0x00;
	// previously transmitted button state
	uint8_t btn_usb_prev = 0x00;
	// if dpi button is pressed when plugging in, jump to bootloader
	// see https://www.pjrc.com/teensy/jump_to_bootloader.html
	delay_ms(50);
	if (!(PIND & (1<<2)))
		__asm__ volatile ("jmp 0x7e00");
	
	// wheel stuff
	uint8_t whl_prev_same = 0; // what A was the last time A == B
	uint8_t whl_prev_diff = 0; // what A was the last time A != B
	int8_t whl = 0; // scrolls since previous usb transmission

	spi_init();
	
	// Angle snapping settings
	uint8_t angle_index = 0;
	uint8_t angles[] = {0x00, 0x80}; // Off, On
	
	// Initial profile
	uint8_t profile = 1;
	
	// dpi settings
	uint8_t dpi_index = 1;
	uint8_t dpis[] = {3, 7, 15};
	
	// Profile switching when mouse is plugged in
	delay_ms(50);
	if (!(PIND & (1<<3))) {
		// Initial profile
		profile = 2;
		// dpi settings
		dpi_index = 0;
	}
		
	else if (!(PIND & (1<<4))) {
		// Initial profile
		profile = 3;
		// dpi settings
		dpi_index = 2;
	}	
	
	// Init 3360
	pmw3360_init(dpis[dpi_index]);

	// Init angle snapping
	angle_init(angles[angle_index]);
	
	usb_init();
	while (!usb_configured());
	delay_ms(456); // arbitrary

	// set up timer0 to set OCF0A in TIFR0 every 125us
	TCCR0A = 0x02; // CTC
	TCCR0B = 0x02; // prescaler 1/8 = 1us period
	OCR0A = 124; // = 125 - 1

	cli();
	while (1) {
		for (uint8_t i = 0; i < 8; i++) {
		// synchronization to usb frames and 125us intervals
			// polling interrupt flags gives 5 clock cycles or so of
			// jitter. possible to eliminate by going into sleep
			// mode and waking up using interrupts, but whatever.
			if (i == 0) {
				// sync to usb frames (1ms)
				UDINT &= ~(1<<SOFI);
				while(!(UDINT & (1<<SOFI)));
				// reset prescaler phase, not really necessary
				GTCCR |= (1<<PSRSYNC);
				TCNT0 = 0;
			} else {
				// sync to 125us intervals using timer0
				while (!(TIFR0 & (1<<OCF0A)));
			}
			TIFR0 |= (1<<OCF0A); // 0CF0A is cleared by writing 1
		
			// RGB ------------------------------
			
			// Colours
			// Brightness values 0-255. When adding new colours here, add their number to led_colours[] and change value of (pX_led_colour < 3) to suit.
			// White
			if (led_colour == 1) {
				led_r_value = 180;
				led_g_value = 255;
				led_b_value = 255;
			}
			// Red
			if (led_colour == 2) {
				led_r_value = 255;
				led_g_value = 0;
				led_b_value = 0;
			}
			// Pink
			if (led_colour == 3) {
				led_r_value = 255;
				led_g_value = 0;
				led_b_value = 60;
			}
			// Magenta
			if (led_colour == 4) {
				led_r_value = 255;
				led_g_value = 0;
				led_b_value = 255;
			}
			// Violet
			if (led_colour == 5) {
				led_r_value = 100;
				led_g_value = 0;
				led_b_value = 255;
			}
			// Blue
			if (led_colour == 6) {
				led_r_value = 0;
				led_g_value = 0;
				led_b_value = 255;
			}
			// Sky Blue
			if (led_colour == 7) {
				led_r_value = 0;
				led_g_value = 90;
				led_b_value = 255;
			}
			// Cyan
			if (led_colour == 8) {
				led_r_value = 0;
				led_g_value = 255;
				led_b_value = 255;
			}
			// Green
			if (led_colour == 9) {
				led_r_value = 0;
				led_g_value = 255;
				led_b_value = 0;
			}
			// Toxic Green
			if (led_colour == 10) {
				led_r_value = 0;
				led_g_value = 255;
				led_b_value = 64;
			}
			// Yellow
			if (led_colour == 11) {
				led_r_value = 200;
				led_g_value = 255;
				led_b_value = 0;
			}
			// Orange
			if (led_colour == 12) {
				led_r_value = 255;
				led_g_value = 100;
				led_b_value = 0;
			}
			
			
			
			// Convert RGB values from x/255 to same percentage as max timer value
			uint8_t led_r_intensity = ((led_r_value / led_max_timer) + 0.5); 
			uint8_t led_g_intensity = ((led_g_value / led_max_timer) + 0.5);
			uint8_t led_b_intensity = ((led_b_value / led_max_timer) + 0.5);
			
			// // RGB Brightness * individual LED brightness
			uint8_t led_r_on = ((led_rgb_brightness * led_r_intensity) + 0.5); 
			uint8_t led_g_on = ((led_rgb_brightness * led_g_intensity) + 0.5);
			uint8_t led_b_on = ((led_rgb_brightness * led_b_intensity) + 0.5);
			
			// Calculates number to turn off LED
			uint8_t led_r_off = (led_r_on + 1); 
			uint8_t led_g_off = (led_g_on + 1);
			uint8_t led_b_off = (led_b_on + 1);
			
			// Red LED PWM
			// Turn off LED if led_x_on is 0
			if (led_r_on == 0) {
				PORTB |= (1<<5);
			}
			// If led_x_on is greater than zero, enable the timer
			if (led_r_on > 0) {
				// LED Timer
				led_r_timer++;
				// Reset Timer
				if (led_r_timer == led_max_timer) {
					led_r_timer = 0;
				}
				// Turn on LED at led_x_on value
				if (led_r_timer <= led_r_on) {
					PORTB &= ~(1<<5);
				}
				// Turn off LED at led_x_off value
				if (led_r_timer >= led_r_off) {
					PORTB |= (1<<5);
				}
			}
			
			// Green LED PWM
			// Turn off LED if led_x_on is 0
			if (led_g_on == 0) {
				PORTB |= (1<<6);
			}
			// If led_x_on is greater than zero, enable the timer
			if (led_g_on > 0) {
				// LED Timer
				led_g_timer++;
				// Reset Timer
				if (led_g_timer == led_max_timer) {
					led_g_timer = 0;
				}
				// Turn on LED at led_x_on value
				if (led_g_timer <= led_g_on) {
					PORTB &= ~(1<<6);
				}
				// Turn off LED at led_x_off value
				if (led_g_timer >= led_g_off) {
					PORTB |= (1<<6);
				}
			}
			
			// Blue LED PWM
			// Turn off LED if led_x_on is 0
			if (led_b_on == 0) {
				PORTD |= (1<<7);
			}
			// If led_x_on is greater than zero, enable the timer
			if (led_b_on > 0) {
				// LED Timer
				led_b_timer++;
				// Reset Timer
				if (led_b_timer == led_max_timer) {
					led_b_timer = 0;
				}
				// Turn on LED at led_x_on value
				if (led_b_timer <= led_b_on) {
					PORTD &= ~(1<<7);
				}
				// Turn off LED at led_x_off value
				if (led_b_timer >= led_b_off) {
					PORTD |= (1<<7);
				}
			}
			
		// sensor stuff
			union motion_data _x, _y;
			SS_LOW;
			spi_send(0x50);
			_delay_us(35);
			spi_send(0x00); // motion, not used
			spi_send(0x00); // observation, not used
			_x.lo = spi_recv();
			_x.hi = spi_recv();
			_y.lo = spi_recv();
			_y.hi = spi_recv();
			SS_HIGH;

		// wheel stuff
			int8_t _whl = 0; // number of scrolls this 125us
			const uint8_t whl_a = WHL_A_IS_HIGH;
			const uint8_t whl_b = WHL_B_IS_HIGH;
			// calculate number of scrolls
			if (whl_a != whl_b)
				whl_prev_diff = whl_a;
			else if (whl_a != whl_prev_same) {
				_whl = 2 * (whl_a ^ whl_prev_diff) - 1;
				whl_prev_same = whl_a;
			}

		// button stuff
			//high = not pressed, low = pressed
			//PIND 0 EIFR 0: low, no edges -> is low
			//PIND 0 EIFR 1: low, edge -> is low
			//PIND 1 EIFR 0: high, no edges -> always high during last 125us
			//PIND 1 EIFR 1: high, edge -> low at some point in the last 125us
			const uint8_t btn_unpressed = PIND & (~(EIFR) | 0b00110000);
			EIFR = 0b00001111; // clear EIFR
			// manual loop debouncing for every button

			// button debouncing logic
			//          >input<           |        >output<
			//------------------------------------------------------
			// previous    | current      | unclicked  | current
			// dbncd state | actual state | time       | dbncd state
			//-------------+--------------+------------+------------
			//    btn_prev |   ~unpressed | btn_time   |   btn_dbncd
			//-------------+--------------+------------+------------
			//           0 |            0 |         =0 |          =0
			//           0 |            1 |         =0 |          =1
			//           1 |            0 |         ++ | (time < DEBOUNCE_TIME)
			//           1 |            1 |         =0 |          =1
			uint8_t btn_dbncd = 0x00;

			#define DEBOUNCE(index) \
			if ((btn_prev & (1<<index)) && (btn_unpressed & (1<<index))) { \
				btn_time[index]++; \
				if (btn_time[index] < DEBOUNCE_TIME) \
					btn_dbncd |= (1<<index); \
			} else { \
				btn_time[index] = 0; \
				btn_dbncd |= (~btn_unpressed) & (1<<index); \
			}

			DEBOUNCE(0); // L
			DEBOUNCE(1); // R
			DEBOUNCE(2); // M
			DEBOUNCE(3); // RSB
			DEBOUNCE(4); // FSB
			DEBOUNCE(5); // DPI
			
			#undef DEBOUNCE
			
			// Profiles ------------------------
			if (profile == 1) {	
				led_colours_index = p1_led_colour;
				led_colour = led_colours[led_colours_index];
				led_bright_index = p1_led_brightness;
				led_rgb_brightness = led_bright[led_bright_index];
			}

			if (profile == 2) {
				led_colours_index = p2_led_colour;
				led_colour = led_colours[led_colours_index];
				led_bright_index = p2_led_brightness;
				led_rgb_brightness = led_bright[led_bright_index];
			}

			if (profile == 3) {
				led_colours_index = p3_led_colour;
				led_colour = led_colours[led_colours_index];
				led_bright_index = p3_led_brightness;
				led_rgb_brightness = led_bright[led_bright_index];
			}
			
			// usb
			// first make sure it's configured
			sei();
			while (!usb_configured());
			cli();

			// this stuff is very intricate and confusing
			// i'm fairly certain all of it is correct.

			// mask dpi button state for usb
			const uint8_t btn_dbncd_mask = btn_dbncd & 0b00011111;

			// there's nothing to do if nothing's changed in this 125us cycle
			if ((btn_dbncd_mask != (btn_prev & 0b00011111)) || _x.all || _y.all || _whl) {
				UENUM = MOUSE_ENDPOINT;
				if (UESTA0X & (1<<NBUSYBK0)) { // untransmitted data still in bank
					UEINTX |= (1<<RXOUTI); // kill bank; RXOUTI == KILLBK
					while (UEINTX & (1<<RXOUTI));
				} else {
					// transmission's finished, or the data that should be in the
					// bank is exactly the same as what was previously transmitted
					// so that there was nothing worth transmitting before.
					btn_usb_prev = btn_usb;
					btn_usb = 0x00;
					x.all = 0;
					y.all = 0;
					whl = 0;
				}
				
				btn_usb |= btn_dbncd_mask & 0b00011111; // L, R, M, FSB, RSB
				x.all += _x.all;
				y.all += _y.all;
				whl += _whl;
				// only load bank with data if there's something worth transmitting
				if ((btn_usb != btn_usb_prev) || x.all || y.all || whl) {
					UEDATX = btn_usb;
					UEDATX = x.lo;
					UEDATX = x.hi;
					UEDATX = y.lo;
					UEDATX = y.hi;
					UEDATX = whl; // wheel scrolls
					UEINTX = 0x3a;
				}
			}
			
			// update btn_prev
			btn_prev = btn_dbncd;
		}
	}
}