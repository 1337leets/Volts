CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -pedantic -O2
LDLIBS  = -lncurses -lm

volts: volts.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

run: volts
	./volts

clean:
	rm -f volts

.PHONY: run clean
