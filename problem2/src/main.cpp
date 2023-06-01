// ATmega328PB microcontroller controlling a half-bridge for a DC motor controller.
// Your task is to write the C++ firmware for the microcontroller so that it can
// control the motor via PWM and measure/report the motor current to a host computer.
//
// Microcontroller Setup:
// - The uC fuses have been programmed so that it’s operating from its internal RC oscillator
// at 8MHz with no clock divider.
//
// - The uC is running on a 5V supply.
//
// - The half-bridge is connected to GPIO pin PD6.
//
// - The current sensor is connected to ADC0, and provides an analog voltage proportional
// to its measured current: 0 volts for 0 amps, and 5 volts for 100 amps.
//
// - The uC does not have a hardware floating point unit.
//
// - UART0 is connected to the host computer and is magically set up.
// uC -> Host Communication Protocol:
//
// - The uC should send an exponential moving average of the motor current draw to the
// host at 10Hz. The current should be encoded in amps as a single byte. E.g., if the
// average current draw is 50 amps, the uC should write the single byte 0x32 to the UART.
// Requirements:
//
// - The uC should output 50% duty-cycle Phase-Correct PWM using Timer0 on pin PD6 at ~1kHz.
//
// - The uC should perform an ADC conversion at the center of each PWM frame (see
// Timer/Counter Overflow interrupt (TOV0)).
//
// - The running average of the ADC conversion (converted to amps) should be sent over
// UART0 to the host processor at a rate of ~10Hz, using the UARTWrite function provided
// in the starter code.

#define F_CPU 8000000UL

#include <stdint.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void uart_init(uint16_t ubrr) {
    /*Set baud rate */
    UBRR0H = (uint8_t) (ubrr>>8);
    UBRR0L = (uint8_t) ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = _BV(RXEN0)|_BV(TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = _BV(USBS0)|(3<<UCSZ00);
}

void uart_write(uint8_t data) {
    // Straight from AVR datasheet
    while (!(UCSR0A & _BV(UDRE0)));

    UDR0 = data;
}

ISR(ADC_vect) {
    static float adc_reading = 0;

    // read ADC >> 2 (8 bits)
    uint8_t adc = ((ADC) >> 2);

    // calculate exponential running average. No floating point unit, but
    // at 1kHz cycle speed, an 8MHz CPU as ~8000 instructions, and some of these can go into software
    // floating point.
    adc_reading = ((float) adc) * (1 - 0.8) + adc_reading * 0.8;

    // transmit
    uart_write(adc);
}

// TOV0 interrupt, ADC_vect is triggered automatically by this interrupt
ISR(TIMER0_OVF_vect) {
    // nothing
}

void configure_io(void) {
    // PD6 is PWM output (OC0A)
    DDRD |= _BV(6);

    // configure timer
    TCCR0A = 0;
    TCCR0B = 0;

    // Phase correct PWM, count from 0 up to 0xFF and back down
    TCCR0A |= _BV(WGM00);

    // Set OC0A on compare when up-counting, clear on compare when down-counting
    TCCR0A |= _BV(COM0A0) | _BV(COM0A1);

    // divide clock by 16 TODO -> 500kHz counter.
    // Full count loop is up to 256 and back down, for 510 counts (not 512),
    // with a frequency of 980.392157
    TCCR0B |= _BV(CS01);

    // 50% duty cycle, trigger at 127
    OCR0A = 127;

    // Enable Timer0 overflow interrupt (TOV0)
    TIMSK0 |= _BV(TOIE0);

    // PC0 (ADC0) is input
    DDRC &= ~_BV(0);

    // Set reference voltage to AVcc (5V). Mux bits are zero, selects ADC0
    ADMUX = _BV(REFS0);

    // Eanble ADC, enable interrupts
    ADCSRA = _BV(ADEN) | _BV(ADIE);

    // Trigger ADC conversion on TOV0 interrupt
    ADCSRB = _BV(ADTS2);
}

int main(void) {
    configure_io();
    uart_init(51); // prescaler sets 115200 baud at 8MHz clock

    while (1) {
        // All work happens in interrupts
    }

    // unreachable
    return 0;
}
