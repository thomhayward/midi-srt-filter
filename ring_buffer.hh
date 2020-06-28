#include <avr/io.h>

#ifndef __RING_BUFFER_HH__
#define __RING_BUFFER_HH__

template<class T, int SIZE> class ring_buffer {
private:
 
    T buffer[SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
    bool full = false;

public:

    bool empty() const {
        return !full && (head == tail);
    }

    void append(T value) {
        buffer[head] = value;
        if (full) {
		    tail = (tail + 1) % SIZE;
	    }
    	head = (head + 1) % SIZE;
    	full = head == tail;
    }

    T read() {
        if (empty()) {
		    return T();
	    }
        T value = buffer[tail];
        full = false;
        tail = (tail + 1) % SIZE;
        return value;
    }
};

#endif
