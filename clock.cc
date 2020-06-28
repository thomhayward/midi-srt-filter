
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

#define BUFFER_LENGTH 128

class CircularBuffer {
public:
    uint8_t buffer[BUFFER_LENGTH];
    uint8_t head = 0;
    uint8_t tail = 0;
    bool full = false;
    //
    bool empty() const {
        //if head and tail are equal, we are empty
        return (!full && (head == tail));
    }
    //
    void append(uint8_t value) {
        buffer[head] = value;
        if (full) {
		    tail = (tail + 1) % BUFFER_LENGTH;
	    }
    	head = (head + 1) % BUFFER_LENGTH;
    	full = head == tail;
    }
    //
    uint8_t read() {
        if (empty()) {
		    return 0x00;
	    }
        //Read data and advance the tail (we now have a free space)
        uint8_t value = buffer[tail];
        full = false;
        tail = (tail + 1) % BUFFER_LENGTH;
        return value;
    }
};

static CircularBuffer buffer;
static volatile uint8_t txData;
static volatile uint8_t txSending = 0;
static volatile int8_t txBit = -1;

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

    if (!txSending && !buffer.empty()) {
        PORTA ^= (1 << PA3);
        txData = Bit_Reverse(buffer.read());
        if (txData == 0xfa || txData == 0xfb || txData == 0xfc) {
            return;
        }
        txSending = 1;
    };

    if (txSending == 0) {
        PORTA |= (1 << PA7);
    } else {
        if (txBit < 0) {
            // Start bit is LOW
            PORTA &= ~(1 << PA7);
            txBit = 0;
        } else if (txBit > 7) {
            // Stop bit is HIGH
            PORTA |= (1 << PA7);
            txBit += 1;
            if (txBit == 9) {
                txSending = 0;
                txBit = -1;
            }
        } else {
            uint8_t bit = txData & (1 << txBit);
            if (bit != 0) {
                PORTA |= (1 << PA7);
            } else {
                PORTA &= ~(1 << PA7);
            }
            txBit += 1;
        }
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
