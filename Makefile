CC ?= cc
CFLAGS ?= -std=gnu11 -O2 -Wall -Wextra -Wpedantic -Werror
CPPFLAGS ?= -Ic/autonomy
LDLIBS ?= -lm
AUTONOMY_SRC := $(filter-out c/autonomy/autonomy_main.c,$(wildcard c/autonomy/*.c))
BUILD_DIR := build
AUTONOMY_BIN := $(BUILD_DIR)/autonomy_controller
TEST_BIN := $(BUILD_DIR)/test_autonomy

.PHONY: all test sanitize riscv clean
all: $(AUTONOMY_BIN)

$(BUILD_DIR):
	mkdir -p $@

$(AUTONOMY_BIN): c/autonomy/autonomy_main.c $(AUTONOMY_SRC) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(TEST_BIN): tests/test_autonomy.c $(AUTONOMY_SRC) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@ $(LDLIBS)

test: $(TEST_BIN) $(AUTONOMY_BIN)
	$(TEST_BIN)
	python3 tests/test_integration.py

sanitize: CFLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer
sanitize: clean test

riscv: | $(BUILD_DIR)
	riscv64-linux-gnu-gcc $(CPPFLAGS) -std=gnu11 -Os -Wall -Wextra -Werror -static -ffunction-sections -fdata-sections -Wl,--gc-sections c/autonomy/autonomy_main.c $(AUTONOMY_SRC) -o $(BUILD_DIR)/autonomy_controller-riscv64-static $(LDLIBS)

clean:
	rm -f $(AUTONOMY_BIN) $(TEST_BIN) $(BUILD_DIR)/autonomy_controller-riscv64-static
