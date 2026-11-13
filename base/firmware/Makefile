# Two builds from the same sources.
#   make test      laptop build of the hardware-free modules, run the tests (needs gcc)
#   make firmware  STM32F103C8 build (needs arm-none-eabi-gcc and python):
#                    build/full.bin  stage1 + application, for the one flash with the BOOT0 jumper
#                    build/app.bin   application only, for every later update (tools/update.py)
#   make clean

CC = gcc
CFLAGS = -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror
APP_SOURCES = protocol.c control.c odom.c safety.c params.c app.c
SOURCES = $(APP_SOURCES) boot.c
TESTS = tests/test_main.c tests/test_protocol.c tests/test_control.c tests/test_odom.c \
        tests/test_safety.c tests/test_params.c tests/test_boot.c tests/test_app.c tests/board_stub.c
HEADERS = $(wildcard *.h tests/*.h)

test: tests/run_tests
	./tests/run_tests
	$(CC) $(CFLAGS) -fsyntax-only main.c

tests/run_tests: $(SOURCES) $(TESTS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $(SOURCES) $(TESTS) -lm

# gnu99, not c99: the vector tables use GCC's range initialisers.
ARM = arm-none-eabi-
ARM_FLAGS = -std=gnu99 -mcpu=cortex-m3 -mthumb -Os -Wall -Wextra -Wshadow -Wdouble-promotion \
            -Werror -ffunction-sections -fdata-sections
PYTHON = python

# The git hash of the sources, with -dirty if this folder has uncommitted changes. V#
# reports it, and tools/update.py checks it after an update.
BUILD_ID := $(shell git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet HEAD -- . 2>/dev/null || echo -dirty)

firmware: build/full.bin
	$(PYTHON) tools/image.py check build/app.bin
	sha256sum build/app.bin build/full.bin

build/stage1.elf: stage1.c boot.c boot.h stm32f103.h stage1.ld
	mkdir -p build
	$(ARM)gcc $(ARM_FLAGS) -T stage1.ld -nostartfiles -nostdlib -Wl,--gc-sections \
	    -Wl,-Map=build/stage1.map -o $@ stage1.c boot.c -lgcc
	$(ARM)size $@

# Always relinked, so the build id in V# matches the sources.
build/app.elf: FORCE
	mkdir -p build
	$(ARM)gcc $(ARM_FLAGS) -DBUILD_ID='"$(BUILD_ID)"' -T stm32f103c8.ld -nostartfiles \
	    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=build/app.map \
	    -o $@ $(APP_SOURCES) main.c board.c startup.c -lm
	$(ARM)size $@

build/%.raw: build/%.elf
	$(ARM)objcopy -O binary $< $@

build/app.bin: build/app.raw tools/image.py
	$(PYTHON) tools/image.py seal $< $@

build/full.bin: build/stage1.raw build/app.bin tools/image.py
	$(PYTHON) tools/image.py combine build/stage1.raw build/app.bin $@

clean:
	rm -rf tests/run_tests tests/run_tests.exe build

FORCE:

.PHONY: test firmware clean FORCE
