/*
  Firmware for Divergence Meter

  Copyright (c) 2017, Kouichi Kuroi (q61.org)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the q61.org nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL Kouichi Kuroi BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//#define F_CPU		8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#define _nop()  __asm__("nop\n\t")


// ======== PORT MAPPING ========
// PB0 -> 595 LATCH CLK
// PB1 -> (PWM) Optocoupler Cathodes
// PB2 -> RTC Chip Select
// PB3 -> (SPI) MOSI
// PB4 <- (SPI) MISO
// PB5 -> (SPI) SCK
// PC0 -> Charge LED Inhibit
// PC1 <- Charge Status (PU)
// PC2 <- Switch 1 (PU)
// PC3 <- Switch 2 (PU)
// PC4 <> (I2C) SDA
// PC5 <> (I2C) SCL
// PD0 <- (UART) RxD
// PD1 -> (UART) TxD
// PD2 <- RTC SQW
// PD3 <- Switch 3 (PU)
// PD4 <- Switch 4 (PU)
// PD5 <- Power Good
// PD6    NC
// PD7    NC
// ADC6 <- Opto Transistor
// ADC7 <- VBAT

#define DDRB_INIT()		0x2f
#define PORTB_INIT()	0x10
#define DDRC_INIT()		0x01
#define PORTC_INIT()	0x0e
#define DDRD_INIT()		0x00
#define PORTD_INIT()	0x18

#define PORT_RTC_CS_HIGH()			PORTB |= 0x04
#define PORT_RTC_CS_LOW()			PORTB &= 0xfb
#define PORT_595_LATCH_HIGH()		PORTB |= 0x01
#define PORT_595_LATCH_LOW()		PORTB &= 0xfe
#define PORT_OPTO_CATHODE_HIGH()	PORTB |= 0x02
#define PORT_OPTO_CATHODE_LOW()		PORTB &= 0xfd
#define PORT_CHARGE_LED_INHIBIT()	PORTC |= 1
#define PORT_CHARGE_LED_ALLOW()		PORTC &= 0xfe
#define PIN_SWITCH_1()				(PINC & 0x04)
#define PIN_SWITCH_2()				(PINC & 0x08)
#define PIN_SWITCH_3()				(PIND & 0x08)
#define PIN_SWITCH_4()				(PIND & 0x10)

#define SERIAL_BAUD_RATE			9600
#define SERIAL_UBRR					(F_CPU / (16UL * SERIAL_BAUD_RATE)) - 1

volatile static uint8_t g_subsec;		// 256th of a second
volatile static uint8_t g_subsubsec;	// 8192th of a second
volatile static uint8_t g_tmr_decr;		// counts down 8 times a second
volatile static uint8_t g_tmr_relay_intvl;

struct nixiedigit_t {
	uint8_t current;
	uint8_t target;
	uint8_t phase;
	uint8_t phase_acc;
};
static struct nixiedigit_t g_nixiedata[8];


struct settings_t {
	uint8_t transition;
	uint8_t colon_mode;
	uint8_t colon_speed;
	uint8_t flags0;
	uint8_t checksum;
};
#define SETTINGS_T_NUMBYTES 5
enum {
	TRANSITION_NONE = 0,
	TRANSITION_CROSSFADE,
	TRANSITION_RELAY,
	TRANSITION_MAX
};
static struct settings_t g_settings;

#define EEPROM_ADDR_SETTINGS 16


static uint8_t g_btntimer[4];
static uint8_t g_brightness_target;
static uint8_t g_brightness_current;
static uint8_t g_transition;


typedef uint8_t bool;
#define true 1
#define false 0

static bool g_demo_ison;
static uint8_t g_demo_phase;
static uint8_t g_demo_decr;


uint8_t spi_write_sync(uint8_t dt)
{
	SPDR = dt;
	while ((SPSR & (1 << SPIF)) == 0) {
		;
	}
	return (SPDR);
}


uint8_t swap_bit_order(uint8_t n)
{
	uint8_t rt = 0;
	for (uint8_t i = 0; i < 8; i++) {
		rt <<= 1;
		rt |= (n & 1) ? 1 : 0;
		n >>= 1;
	}
	return (rt);
}



void nixie_output_raw(uint8_t a_mask, uint8_t kl_hi, uint8_t kl_lo, uint8_t kr_hi, uint8_t kr_lo)
{
	PORT_595_LATCH_LOW();
	spi_write_sync(a_mask);
	spi_write_sync(kr_hi);
	spi_write_sync(kr_lo);
	spi_write_sync(kl_hi);
	spi_write_sync(kl_lo);
	PORT_595_LATCH_HIGH();
}

void nixie_output(uint8_t digit, uint8_t num_l, uint8_t num_r)
{
	if (digit == 0) {
		nixie_output_raw(0, 0xff, 0xff, 0xff, 0xff);
	} else {
		uint8_t al, ar;
		uint8_t kl_hi, kl_lo;
		uint8_t kr_hi, kr_lo;

		if (num_l > 10) {
			al = 0;
			kl_hi = kl_lo = 255;
		} else {
			al = 0x80 >> (digit - 1);
			if (num_l < 7) {
				kl_hi = 0;
				kl_lo = 2 << num_l;
			} else {
				kl_hi = 1 << (num_l - 6);
				kl_lo = 0;
			}
		}
		if (num_r > 10) {
			ar = 0;
			kr_hi = kr_lo = 255;
		} else {
			ar = 0x08 >> (digit - 1);
			if (num_r < 7) {
				kr_hi = 0;
				kr_lo = 2 << num_r;
			} else {
				kr_hi = 1 << (num_r - 6);
				kr_lo = 0;
			}
		}

		nixie_output_raw(al | ar, kl_hi, kl_lo, kr_hi, kr_lo);
	}
}


void nixie_blank_leading_zeros(uint8_t count)
{
}


void nixie_set_next_digit(uint8_t digit, uint8_t val)
{
	struct nixiedigit_t *dp = &g_nixiedata[digit & 7];

	if (dp->target == val) {
		return;
	}

	switch (g_transition) {
	case TRANSITION_CROSSFADE:
	case TRANSITION_RELAY:
		if (dp->current == dp->target) {
			dp->target = val;
		} else {
			dp->current = dp->target;
			dp->target = val;
			dp->phase = 0;
			dp->phase_acc = 0;
		}
		break;
	default:
		dp->current = dp->target = val;
		dp->phase = 0;
		break;
	}
}


void btns_update()
{
	for (uint8_t i = 0; i < 4; i++) {
		uint8_t dt = 0;
		switch (i) {
			case 0: dt = PIN_SWITCH_1(); break;
			case 1: dt = PIN_SWITCH_2(); break;
			case 2: dt = PIN_SWITCH_3(); break;
			case 3: dt = PIN_SWITCH_4(); break;
		}
		if (dt) {
			g_btntimer[i] = 0;
		} else {
			if (g_btntimer[i] < 255) {
				g_btntimer[i]++;
			}
		}
	}
}

uint8_t btn_state(uint8_t index)
{
	return (g_btntimer[index & 3]);
}

uint8_t btn_state_is_clicked(uint8_t index)
{
	return (btn_state(index) == 1);
}

uint8_t btn_state_is_down(uint8_t index)
{
	return (btn_state(index) >= 1);
}

uint8_t btn_state_is_held_down(uint8_t index)
{
	return (btn_state(index) >= 128);
}

bool btn_state_some_btn_is_clicked()
{
	return (btn_state_is_clicked(0) || btn_state_is_clicked(1) || btn_state_is_clicked(2) || btn_state_is_clicked(3));
}


void rtc_write(uint8_t addr, const uint8_t *src, uint8_t length)
{
	if (length < 1) {
		return;
	}

	PORT_RTC_CS_LOW();
	_nop();
	spi_write_sync(addr | 0x80);
	while (length-- > 0) {
		spi_write_sync(*src++);
	}
	_nop();
	PORT_RTC_CS_HIGH();
}


void rtc_read(uint8_t addr, uint8_t *dst, uint8_t length)
{
	if (length < 1) {
		return;
	}

	PORT_RTC_CS_LOW();
	_nop();
	spi_write_sync(addr);
	while (length-- > 0) {
		*dst++ = spi_write_sync(0);
	}
	_nop();
	PORT_RTC_CS_HIGH();
}



enum {
	ADC_SEL_OPTO = 0,
	ADC_SEL_VBAT
};
static volatile uint8_t g_adc_current_sel;

void adc_read_start(uint8_t sel)
{
	uint8_t adcsrc = (sel == ADC_SEL_OPTO) ? 6 : 7;
	g_adc_current_sel = sel;
	ADMUX = (1 << REFS0) | (1 << ADLAR) | adcsrc; // Vcc ref, left justify
	ADCSRA |= (1 << ADSC);
}

volatile uint8_t adc_is_reading_what()
{
	return (g_adc_current_sel);
}
volatile uint8_t adc_is_reading()
{
	return (ADCSRA & (1 << ADSC));
}

volatile uint8_t adc_read_result()
{
	return (ADCH);
}

volatile uint16_t adc_read_result_16()
{
	uint16_t result = ADCL;
	result |= (uint16_t)ADCH << 8;
	return (result);
}



void brightness_set(uint8_t val, bool immediate)
{
	if (immediate) {
		g_brightness_current = val;
	}
	g_brightness_target = val;
}



uint8_t settings_serialize_get_byte(const struct settings_t *src, uint8_t index)
{
	uint8_t rt = 0;
	switch (index) {
		case 0: rt = src->transition; break;
		case 1: rt = src->colon_mode; break;
		case 2: rt = src->colon_speed; break;
		case 3: rt = src->flags0; break;
		case 4: rt = src->checksum; break;
	}
	return (rt);
}

void settings_deserialize_put_byte(struct settings_t *dst, uint8_t index, uint8_t data)
{
	switch (index) {
		case 0: dst->transition = data; break;
		case 1: dst->colon_mode = data; break;
		case 2: dst->colon_speed = data; break;
		case 3: dst->flags0 = data; break;
		case 4: dst->checksum = data; break;
	}
}


uint8_t settings_calc_checksum(const struct settings_t *src)
{
	uint8_t rt = 0x34;
	for (uint8_t i = 0; i < SETTINGS_T_NUMBYTES - 1; i++) {
		uint8_t dt = settings_serialize_get_byte(src, i);
		uint8_t msb = (rt & 0x80) ? 1 : 0;
		rt <<= 1;
		rt |= msb;
		rt ^= dt;
	}
	return (rt);
}


bool settings_read(struct settings_t *dst)
{
	for (uint8_t i = 0; i < SETTINGS_T_NUMBYTES; i++) {
		uint8_t dt = eeprom_read_byte((uint8_t*)(EEPROM_ADDR_SETTINGS + i));
		settings_deserialize_put_byte(dst, i, dt);
	}
	uint8_t chksum = settings_calc_checksum(dst);
	return (chksum == dst->checksum);
}


void settings_write(const struct settings_t *src)
{
	for (uint8_t i = 0; i < SETTINGS_T_NUMBYTES; i++) {
		uint8_t dt;
		if (i < SETTINGS_T_NUMBYTES - 1) {
			dt = settings_serialize_get_byte(src, i);
			} else {
			dt = settings_calc_checksum(src);
		}
		
		uint8_t *addr = (uint8_t*)(EEPROM_ADDR_SETTINGS + i);
		uint8_t cd = eeprom_read_byte(addr);
		if (cd != dt) {
			eeprom_write_byte(addr, dt);
		}
	}
}


void settings_init(struct settings_t *dst)
{
	dst->transition = TRANSITION_NONE;
	dst->colon_speed = 1;
	dst->flags0 = 0;
}



ISR(INT0_vect)
{ // called 8k times a second
	static uint8_t s_nixie_digit;

	++g_subsubsec;
	g_subsubsec &= 31;
	if (g_subsubsec == 0) {
		++g_subsec;
		if ((g_subsec & 31) == 0) {
			if (g_tmr_decr > 0) {
				--g_tmr_decr;
			}
		}
	}
	
	switch (g_subsubsec & 7) {
	case 0:
		if (++s_nixie_digit > 3) {
			s_nixie_digit = 0;
		}
		{
			struct nixiedigit_t *dp = &g_nixiedata[s_nixie_digit];
			uint8_t val_l = g_nixiedata[s_nixie_digit].current;
			uint8_t val_r = g_nixiedata[s_nixie_digit + 4].current;
			nixie_output(s_nixie_digit + 1, val_l, val_r);
		}
		break;
	case 7:
		nixie_output(0, 255, 255);
		break;
	}

	if (((g_subsec & 3) == 0) && (g_subsubsec == 1)) {
		bool doUpdate = false;
		if (g_brightness_target > g_brightness_current) {
			++g_brightness_current;
			doUpdate = true;
		} else if (g_brightness_target < g_brightness_current) {
			--g_brightness_current;
			doUpdate = true;
		}
		if (doUpdate) {
			OCR1A = ((uint16_t)g_brightness_current * g_brightness_current) >> 8;
		}
	}
}


void setup()
{
	DDRB = DDRB_INIT();
	PORTB = PORTB_INIT();
	DDRC = DDRC_INIT();
	PORTC = PORTC_INIT();
	DDRD = DDRD_INIT();
	PORTD = PORTD_INIT();

	PORT_RTC_CS_HIGH();

	// init global vars
	for (uint8_t i = 0; i < 4; i++) {
		struct nixiedigit_t *dp = &g_nixiedata[i];
		dp->current = dp->target = dp->phase = 0;
	}
	g_transition = TRANSITION_NONE;
	g_tmr_relay_intvl = 0;
	g_brightness_current = g_brightness_target = 255;

	// init spi
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (1 << SPR0);  // master, Mode 3, F_CPU / 16

	// init PWM
	// Fast PWM mode, Clear on compare match, Set at bottom, CLK / 1024
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 255;

	// init Serial
	//UBRR0 = SERIAL_UBRR;
	//UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	// init RTC
	uint8_t dt;
	dt = 0x18;
	rtc_write(0x0e, &dt, 1); // enable square wave output, 8192Hz
	dt = 0x00;
	rtc_write(0x0f, &dt, 1); // disable battery-backed 32kHz output

	// init pin change interrupt
	EICRA = (1 << ISC01) | (1 << ISC00); // interrupt on rising edge
	EIMSK = (1 << INT0);

	// init ADC
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC
	// enable interrupts
	sei();
}




#define AVG_VAL_HISTORY_SIZE	16
#define AVG_VAL_HISTORY_SIZE_SHIFT	4
struct avg_val_16 {
	uint16_t val_history[AVG_VAL_HISTORY_SIZE];
	uint8_t history_pos;
};

void avg_val_16_accum(struct avg_val_16 *val, uint16_t accum)
{
	if (++val->history_pos >= AVG_VAL_HISTORY_SIZE) {
		val->history_pos = 0;
	}
	val->val_history[val->history_pos] = accum;
}
uint16_t avg_val_val(struct avg_val_16 *val)
{
	uint32_t rt = 0;
	for (uint8_t i = 0; i < AVG_VAL_HISTORY_SIZE; i++) {
		rt += val->val_history[i];
	}
	rt >>= AVG_VAL_HISTORY_SIZE_SHIFT;
	return ((uint16_t)rt);
}
uint16_t avg_val_sum(struct avg_val_16 *val)
{
	uint32_t rt = 0;
	for (uint8_t i = 0; i < AVG_VAL_HISTORY_SIZE; i++) {
		rt += val->val_history[i];
	}
	return ((uint16_t)rt);
}




enum {
	DISP_MODE_RTC = 0,
	DISP_MODE_DIV,
	DISP_MODE_RANDOM,
	DISP_MODE_SET_HOUR,
	DISP_MODE_SET_MIN,
	DISP_MODE_SET_TRANSITION,
	DISP_MODE_VOLTAGE,
	DISP_MODE_FIXED
};

#define PRESET_DIV_COUNT 22
PROGMEM const uint32_t g_preset_div[PRESET_DIV_COUNT] = {
	 328403,  334581,  337187,  337199,  337337,  409031, 409420, 409431,
     456903,  456914,  456923,  523299,  523307,  571015, 571024, 571046, 
	1048596, 1048728, 1130205, 1130238, 1130246,  0
};

int main(void)
{
	// hardware setup
	setup();

	// seed random
	{
		unsigned char dt[4];
		rtc_read(0, dt, 4);
		unsigned long seedval = 0;
		for (uint8_t i = 0; i < 4; i++) {
			seedval <<= 8;
			seedval |= dt[i];
		}
		srandom(seedval);
	}


	// lamp check
	for (uint8_t i = 0; i < 12; i++) {
		for (uint8_t dig = 0; dig < 8; dig++) {
			nixie_set_next_digit(dig, (i < 11) ? i : 0);
		}
		if (i == 11) break;
		_delay_ms(100);
	}
	PORT_CHARGE_LED_INHIBIT();


	uint8_t s_last_subsec = 0;
	uint8_t s_last_sec = 0;
	uint8_t s_subsec_zero = 0;
	uint8_t s_disp_mode = 0;
	uint8_t rtcdata[3];
	uint8_t rtc_subsec = 0;
	uint8_t rtc_doset = 0;
	uint8_t settings_doset = 0;
	uint8_t subsec_input_flasher = 0;

	uint8_t s_demo_subsec = 0;
	uint8_t s_demo_sec = 0;
	uint8_t s_demo_phase = 0;
	uint8_t s_random_duration = 0;
	uint32_t s_random_value = 0;
	uint16_t s_adc_value = 0;
	uint8_t s_adc_count = 0;
	
	uint8_t s_div_pos = 0;
	uint32_t s_div_value = pgm_read_dword(&g_preset_div[s_div_pos]);
	
	
	struct avg_val_16 s_adc_val;
	s_disp_mode = DISP_MODE_RANDOM;
	s_random_duration = 30;

    /* Replace with your application code */
    while (1) 
    {
		while (g_subsec == s_last_subsec) {}
		s_last_subsec = g_subsec;

		// -- brightness adjust
		brightness_set(255, false);
/*		if ((g_subsec & 63) == 0) {
			if (!opto_is_reading()) {
				s_opto_val = opto_read_result();
				opto_read_start();
			}

			brightness_set((s_opto_val < 240) ? s_opto_val : 240, false);
		}*/
		uint8_t adc_val_updated = 0;
		// -- read adc
		if ((s_disp_mode == DISP_MODE_VOLTAGE) && ((g_subsec & 3) == 0)) {
			if (!adc_is_reading()) {
				if (adc_is_reading_what() == ADC_SEL_VBAT) {
					s_adc_value = adc_read_result_16() >> 6;
					avg_val_16_accum(&s_adc_val, s_adc_value);
					adc_val_updated = 1;
					++s_adc_count;
				}
				adc_read_start(ADC_SEL_VBAT);
			}
		}

		// -- read RTC
		if (s_disp_mode == DISP_MODE_RTC)  {
			rtc_read(0, rtcdata, 3);

			// synchronize with seconds change
			if (s_last_sec != rtcdata[0]) {
				s_last_sec = rtcdata[0];
				s_subsec_zero = g_subsec;
			}
		}
		rtc_subsec = (uint8_t)(g_subsec - s_subsec_zero);

		// -- random
		if ((s_disp_mode == DISP_MODE_RANDOM) && ((g_subsec & 0x07) == 1)) {
			s_random_value = (((uint32_t)random() << 16) ^ (random())) & 0x1ffffful;
			if (--s_random_duration < 1) {
				s_random_duration = 0;
				s_disp_mode = DISP_MODE_DIV;
			}
		}

		// -- demo
		if (g_demo_ison) {
			if (g_subsec == 13) {
				if (--g_demo_decr == 0) {
					g_demo_phase++;
					if (g_demo_phase == 1) {
						g_demo_decr = 15;
					} else if (g_demo_phase == 2) {
						s_div_pos = random() % (PRESET_DIV_COUNT - 1);
						s_div_value = pgm_read_dword(&g_preset_div[s_div_pos]);
						s_disp_mode = DISP_MODE_RANDOM;
						s_random_duration = 30;						
						g_demo_decr = 2;
						g_demo_phase = 0;
					}
				}
			}
		}
		// -- read buttons
		btns_update();
		if (btn_state_some_btn_is_clicked() || ((g_subsec & 15) == 2)) {
			switch (s_disp_mode) {
				case DISP_MODE_DIV:
				{
					bool do_change_div = false;
					if (btn_state_is_clicked(1)) {
						if (s_div_pos < 1) {
							s_div_pos = PRESET_DIV_COUNT - 1;
						} else {
							s_div_pos--;
						}
						do_change_div = true;
					} else if (btn_state_is_clicked(0)) {
						if (++s_div_pos >= PRESET_DIV_COUNT) {
							s_div_pos = 0;
						}
						do_change_div = true;
					} else if (btn_state_is_clicked(3)) {
						do_change_div = true;
					} else if (btn_state_is_clicked(2)) {
						s_disp_mode = DISP_MODE_VOLTAGE;
					}
					if (do_change_div) {
						s_div_value = pgm_read_dword(&g_preset_div[s_div_pos]);
						s_disp_mode = DISP_MODE_RANDOM;
						s_random_duration = 30;
					}
				}
				break;
				case DISP_MODE_RANDOM:
				{
					if (btn_state_is_down(3)) {
						s_random_duration = 255;
					}
				}
				break;
				case DISP_MODE_VOLTAGE:
				{
					if (btn_state_is_clicked(2)) {
						g_demo_ison = true;
						g_demo_phase = 0;
						g_demo_decr = 1;
						s_disp_mode = DISP_MODE_RANDOM;
						s_random_duration = 30;
					}
				}
				break;
			}
		}

		// -- set display
		++subsec_input_flasher;
		switch (s_disp_mode) {
			case DISP_MODE_RTC:
			case DISP_MODE_SET_HOUR:
			case DISP_MODE_SET_MIN:
				nixie_set_next_digit(0, rtcdata[0] >> 4);
				nixie_set_next_digit(1, rtcdata[0] & 15);
				/*
				nixie_set_next_digit(0, (rtcdata[2] < 0x10) ? 10 : (rtcdata[2] >> 4));
				nixie_set_next_digit(1, rtcdata[2] & 15);
				nixie_set_next_digit(2, rtcdata[1] >> 4);
				nixie_set_next_digit(3, rtcdata[1] & 15);
				*/
				if (subsec_input_flasher & 0x20) {
					if (s_disp_mode == DISP_MODE_SET_HOUR) {
						nixie_set_next_digit(0, 10);
						nixie_set_next_digit(1, 10);
					} else if (s_disp_mode == DISP_MODE_SET_MIN) {
						nixie_set_next_digit(2, 10);
						nixie_set_next_digit(3, 10);
					}
				}

				break;

			case DISP_MODE_FIXED:
				nixie_set_next_digit(0, 2);
				nixie_set_next_digit(1, 3);
				nixie_set_next_digit(2, 4);
				nixie_set_next_digit(3, 5);
				break;

			case DISP_MODE_VOLTAGE:
				if (!adc_val_updated) break;

				uint32_t adc_val = avg_val_sum(&s_adc_val);
				adc_val *= 302; // 3.3V * 1.5 / 16384 * 1000000

				nixie_set_next_digit(0, adc_val / 1000000 % 10);
				nixie_set_next_digit(1, 10);
				nixie_set_next_digit(2, adc_val / 100000 % 10);
				nixie_set_next_digit(3, adc_val / 10000 % 10);
				nixie_set_next_digit(4, adc_val / 1000 % 10);
				nixie_set_next_digit(5, adc_val / 100 % 10);
				nixie_set_next_digit(6, adc_val / 10 % 10);
				nixie_set_next_digit(7, adc_val / 1 % 10);
				break;

			case DISP_MODE_DIV:
			case DISP_MODE_RANDOM:
				{
					uint32_t disp_val = 0;
					switch (s_disp_mode) {
						case DISP_MODE_DIV: disp_val = s_div_value; break;
						case DISP_MODE_RANDOM: disp_val = s_random_value; break;
					}
					nixie_set_next_digit(0, disp_val / 1000000 % 10);
					nixie_set_next_digit(1, 10);
					nixie_set_next_digit(2, disp_val / 100000 % 10);
					nixie_set_next_digit(3, disp_val / 10000 % 10);
					nixie_set_next_digit(4, disp_val / 1000 % 10);
					nixie_set_next_digit(5, disp_val / 100 % 10);
					nixie_set_next_digit(6, disp_val / 10 % 10);
					nixie_set_next_digit(7, disp_val / 1 % 10);
				}
				break;
		}

    }
}

