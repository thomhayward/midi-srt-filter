
// Define an architecture to appease vscode
#ifndef __AVR_ARCH__
#define __AVR_ATtiny84__
#endif

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#define BAUDRATE        31250
#define CYCLES_PER_BIT  (F_CPU / BAUDRATE)
#if (CYCLES_PER_BIT > 255)
#define     DIVISOR     8
#define     PRESCALE    2
#else
#define     DIVISOR     1
#define     PRESCALE    1
#endif
#define FULL_BIT_TICKS  (CYCLES_PER_BIT / DIVISOR )
#define HALF_BIT_TICKS  (FULL_BIT_TICKS / 2)
#define START_DELAY         (42)
#define TIMER_START_DELAY   (START_DELAY  / DIVISOR)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "ring_buffer.hh"

#define BUFFER_LENGTH 64

static ring_buffer<uint8_t, BUFFER_LENGTH> buffer;

static uint8_t tx_buffer;
static volatile uint8_t tx_status = 8;

uint8_t Bit_Reverse(uint8_t x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

void initTx() {
    TCCR1B |= (1 << CS11);
    OCR1A = 80;
    TCCR1B |= (1 << WGM12);
    TIMSK1 |= (1 << OCIE1A);
}

int main() {

    cli();

    // setup status LED
    DDRA |= (1 << PA3);

    // setup rx
    DDRA &= ~((1 << PA6) | (1 << PA5) | (1 << PA4));    // set DI, DO, and SCK as input
    PORTA |= (1 << PA6) | (1 << PA5) | (1 << PA4);      // ... with internal pull-up
    USICR = 0;                                          // Disable USI.
    GIMSK |=  (1 << PCIE0);                             // Enable pin change interrupt
    PCMSK0 |= (1 << PCINT6);

    // setup tx
    DDRA |= (1 << PA7);
    PORTA |= (1 << PA7);
    TCCR1B |= (1 << CS11);
    OCR1A = FULL_BIT_TICKS;
    TCCR1B |= (1 << WGM12);
    TIMSK1 |= (1 << OCIE1A);
    
    sei();

    while (true) { 
        ;
    }

}

ISR (TIM1_COMPA_vect) {
    if (tx_status == 8 && !buffer.empty()) {    // we're not transmitting any data, but have data to send
        PORTA ^= (1 << PA3);                    // flip status led
        tx_buffer = Bit_Reverse(buffer.read());
        if (tx_buffer == 0xfa || tx_buffer == 0xfb || tx_buffer == 0xfc) { // the filter!
            return;
        }
        // we now have data to transmit, so send a start bit.
        PORTA &= ~(1 << PA7);           // set tx line LOW
        tx_status = 0;                  // set the first bit of tx_buffer to transmit on the next timer interrupt
        return;                         // save some cycles return early
    };
    if (tx_status == 8) {               // we're either not transmitting anything or it's time to send a stop bit
        PORTA |= (1 << PA7);            // set tx line HIGH
    }
    else {                              // send a bit of data
        if ((tx_buffer & (1 << tx_status)) == 0) {
            PORTA &= ~(1 << PA7);       // bit is 0, set tx line LOW
        }
        else { 
            PORTA |= (1 << PA7);        // bit is 1, set tx line HIGH
        }
        ++tx_status;                    // set the next bit to transmit on the next timer interrupt
    }
}

ISR (SIG_PIN_CHANGE0) {                                         
    if ((PINA & (1 << PA6)) == 0) {         // If the USI DI pin is low, then it is likely that it was this pin that
                                            // generated the pin change interrupt.
        GIMSK &= ~(1 << PCIE0);             // Disable pin change interrupts
        TCCR0A = (2 << WGM00);              // CTC mode
        TCCR0B = (0 << CS02) | (1 << CS01) | (0 << CS00);
                                            // Set prescaler to cpu clk or clk/8
        GTCCR |= (1 << PSR10);              // Reset prescaler
        TCNT0 = 0;                          // Count up from 0]
        OCR0A = HALF_BIT_TICKS - TIMER_START_DELAY;
        TIFR0 = (1 << OCF0A);               // Clear output compare interrupt flag
        TIMSK0 |= (1 << OCIE0A);            // Enable output compare interrupt
    }
}

ISR (TIM0_COMPA_vect) {
    TIMSK0 &= ~(1<<OCIE0A);                 // Disable COMPA interrupt
    TCNT0 = 0;                              // Count up from 0
    OCR0A = FULL_BIT_TICKS;                 // Shift every bit width
    USICR = (1 << USIOIE) | (0 << USIWM0) | (1 << USICS0);
    USISR = (1 << USIOIF) | 8;
}

ISR (SIG_USI_OVERFLOW) {
    uint8_t data = USIDR;
    buffer.append(data);
    // if (buffer.full) {
    //     PORTA |= (1 << PA3);
    // }
    USICR = 0;
    GIFR = (1 << PCIF0);                    // Clear pin change interrupt flag
    GIMSK |= (1 << PCIE0);                  // Enable pin change interrupts again
}
