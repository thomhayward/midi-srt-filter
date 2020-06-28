
PROJECT = clock

MCU ?= attiny84
PORT ?= $(firstword $(wildcard /dev/cu.usbmodem*))

CXX = avr-g++
CXXSTANDARD = -std=gnu++11
CXXEXTRA = -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections
CXXFLAGS = $(CXXSTANDARD) $(CXXEXTRA) -mmcu=$(MCU)

OBJCOPY = avr-objcopy

%.elf: $(wildcard *.cc)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

.PHONY: build
build: $(PROJECT).hex
	#

.PHONY: upload
upload: $(PROJECT).hex
	# avrdude -p $(MCU) -c arduino -P $(PORT) -D -U $<
	avrdude -v -p $(MCU) -c stk500v1 -P $(PORT) -b 19200 -U $<

.PHONY: clean
clean:
	rm $(PROJECT).hex
