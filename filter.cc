
// Define an architecture to appease vscode
#ifndef __AVR_ARCH__
#define __AVR_ATtiny84__
#endif

#ifndef F_CPU
#define F_CPU               20000000UL
#endif

#define BAUDRATE            31250
#define CYCLES_PER_BIT      (F_CPU / BAUDRATE)
#if (CYCLES_PER_BIT > 255)
#define     DIVISOR         8
#define     PRESCALE        2
#else
#define     DIVISOR         1
#define     PRESCALE        1
#endif
#define FULL_BIT_TICKS      (CYCLES_PER_BIT / DIVISOR )
#define HALF_BIT_TICKS      (FULL_BIT_TICKS / 2)
#define START_DELAY         42
#define TIMER_START_DELAY   (START_DELAY / DIVISOR)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "ring_buffer.hh"

#define BUFFER_LENGTH 64

static ring_buffer<uint8_t, BUFFER_LENGTH> buffer;

#define TX_IDLE     -2
#define TX_START    -1
#define TX_STOP     8
#define TX_DATA_START 0
static uint8_t tx_buffer;
static volatile int8_t tx_state = -2;

#define LED_START   32
static uint8_t send_led = 0;
static uint8_t block_led = 0;

uint8_t reverse_byte(uint8_t x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

void update_leds() {
    if (send_led > 0) {
        PORTA |= (1 << PA3);
        send_led -= 1;
    } else {
        PORTA &= ~(1 << PA3);
    }
    if (block_led > 0) {
        PORTA |= (1 << PA2);
        block_led  -= 1;
    }
    else {
        PORTA &= ~(1 << PA2);
    }
}

int main() {

    cli();                                                      // disable interrupts during setup

    // setup status LEDs
    DDRA |= (1 << PA2) | (1 << PA3);

    // setup rx
    DDRA &= ~((1 << PA6) | (1 << PA5) | (1 << PA4));            // set PA6/DI, PA5/DO, and PA5/SCK as input
    PORTA |= (1 << PA6) | (1 << PA5) | (1 << PA4);              // ... with internal pull-up
    USICR = 0;                                                  // disable USI.
    GIMSK |=  (1 << PCIE0);                                     // enable pin change interrupts
    PCMSK0 |= (1 << PCINT6);                                    // .. on PA6/DI

    // setup tx
    DDRA |= (1 << PA7);                                         // tx on PA7
    PORTA |= (1 << PA7);                                        // idle HIGH
    TCCR1B |= (1 << CS11) | (1 << WGM12);                       // setup timer1. CS11 = prescaler clk/8, WGM12 = CTC mode
    TCNT1 = 0;                                                  // start counting from 0
    OCR1A = FULL_BIT_TICKS;                                     // set the baud rate
    TIFR1 = (1 << OCF1A);                                       // clear output compare interrupt flag
    TIMSK1 |= (1 << OCIE1A);                                    // enable output compare interrupt
    
    sei();                                                      // reenable interrupts

    while (true) { 
        ;
    }

}

ISR (SIG_PIN_CHANGE0) {                                         
    if ((PINA & (1 << PA6)) == 0) {                             // PA6/DI has gone low; we've probably detected a start bit
        GIMSK &= ~(1 << PCIE0);                                 // disable pin change interrupts
        TCCR0A = (1 << WGM01);                                  // setup timer0 in CTC mode
        TCCR0B = (0 << CS02) | (1 << CS01) | (0 << CS00);       // set prescaler to cpu/8
        GTCCR |= (1 << PSR10);                                  // reset prescaler
        TCNT0 = 0;                                              // count up from 0
        OCR0A = HALF_BIT_TICKS - TIMER_START_DELAY;             // try to trigger the timer in the middle of a signal
        TIFR0 = (1 << OCF0A);                                   // clear output compare interrupt flag
        TIMSK0 |= (1 << OCIE0A);                                // enable output compare interrupt
    }
}

ISR (TIM0_COMPA_vect) {
    TIMSK0 &= ~(1 << OCIE0A);                                   // disable COMPA interrupt
    TCNT0 = 0;                                                  // count up from 0
    OCR0A = FULL_BIT_TICKS;                                     // shift every bit width
    USICR = (1 << USIOIE) | (0 << USIWM0) | (1 << USICS0);
    USISR = (1 << USIOIF) | 8;
}

ISR (SIG_USI_OVERFLOW) {                                        // USI has read eight bits
    uint8_t data = USIDR;                                       // save USI buffer
    buffer.append(data);                                        // add to our ring buffer
    USICR = 0;                                                  // disable USI; we've finished reading data
    GIFR = (1 << PCIF0);                                        // clear pin change interrupt flag
    GIMSK |= (1 << PCIE0);                                      // enable pin change interrupts again
}

ISR (TIM1_COMPA_vect) {
    update_leds();
    if (tx_state == TX_IDLE && !buffer.empty()) {
        tx_buffer = reverse_byte(buffer.read());
        if (tx_buffer == 0xfa || tx_buffer == 0xfb || tx_buffer == 0xfc) {
            block_led = LED_START;
            return;
        }
        send_led = LED_START;
        tx_state = TX_START;
    }
    switch (tx_state) {
        case TX_IDLE:                                           // not tx'ing
            // PORTA |= (1 << PA7);                             // fallthrough to TX_STOP
        case TX_STOP:                                           // stop bit
            PORTA |= (1 << PA7);                                // set output HIGH to indicate stop bit
            tx_state = TX_IDLE;                                 // idle after stop bit
            break;
        case TX_START:                                          // new tx
            PORTA &= ~(1 << PA7);                               // set output LOW to indicate start bit
            tx_state = TX_DATA_START;                           // start tx'ing bits on the next interrupt
            break;
        default:
            if ((tx_buffer & (1 << tx_state)) == 0) {           // determine value of current bit
                PORTA &= ~(1 << PA7);                           // set LOW iff 0
            }
            else {
                PORTA |= (1 << PA7);                            // set HIGH iff 1
            }
            tx_state += 1;
    }

}
