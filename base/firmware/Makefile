# Two builds from the same sources.
#   make test      laptop build of the hardware-free modules, run the tests (needs gcc)
#   make firmware  STM32F103C8 build: build/base.elf and build/base.bin (needs arm-none-eabi-gcc)
#   make clean

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

# gnu99, not c99: the vector table in startup.c uses GCC's range initialisers.
ARM = arm-none-eabi-
ARM_FLAGS = -std=gnu99 -mcpu=cortex-m3 -mthumb -Os -Wall -Wextra -Wshadow -Wdouble-promotion \
            -Werror -ffunction-sections -fdata-sections
ARM_LINK = -T stm32f103c8.ld -nostartfiles --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=build/base.map
FIRMWARE = $(SOURCES) main.c board.c startup.c

firmware: build/base.bin

build/base.elf: $(FIRMWARE) $(HEADERS) stm32f103c8.ld
	mkdir -p build
	$(ARM)gcc $(ARM_FLAGS) $(ARM_LINK) -o $@ $(FIRMWARE) -lm
	$(ARM)size $@

build/base.bin: build/base.elf
	$(ARM)objcopy -O binary $< $@
	sha256sum $@

clean:
	rm -rf tests/run_tests tests/run_tests.exe build

.PHONY: test firmware clean
