# Laptop build of the hardware-free modules and their tests.
#   make test    build tests/run_tests and run it (needs gcc)
#   make clean
# The STM32 build (arm-none-eabi-gcc, board.c, vendor drivers) waits for the part number.

CC = gcc
CFLAGS = -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror
SOURCES = protocol.c control.c odom.c safety.c app.c
TESTS = tests/test_main.c tests/test_protocol.c tests/test_control.c tests/test_odom.c \
        tests/test_safety.c tests/test_app.c tests/board_stub.c
HEADERS = $(wildcard *.h tests/*.h)

test: tests/run_tests
	./tests/run_tests
	$(CC) $(CFLAGS) -fsyntax-only main.c

tests/run_tests: $(SOURCES) $(TESTS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $(SOURCES) $(TESTS) -lm

clean:
	rm -f tests/run_tests tests/run_tests.exe

.PHONY: test clean
