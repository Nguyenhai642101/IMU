#==============================================================================
# Makefile — SM-5800 6-DOF IMU (FPGA AXI4-Lite IP + ARM PS Bare-Metal Driver)
#==============================================================================

BUILD_DIR   := build
RTL_SRCS    := rtl/sm5800_param_error.v \
               rtl/sm5800_crc16_ccitt.v \
               rtl/sm5800_clk_ctrl.v \
               rtl/sm5800_sdlc_rx.v \
               rtl/sm5800_msg_parser.v \
               rtl/sm5800_axi4_lite_if.v \
               rtl/sm5800_imu_axi_top.v
TB_SRCS     := tb/sm5800_sensor_model.v \
               tb/tb_sm5800_imu_axi.v

HOST_CC     ?= gcc
ARM_CC      ?= arm-none-eabi-gcc
ARM_OBJDUMP ?= arm-none-eabi-objdump
# AArch64 (Zynq UltraScale+ Cortex-A53): Vitis ships aarch64-none-elf-gcc;
# a distro cross compiler (aarch64-linux-gnu-gcc) is used if that is absent.
A64_CC      ?= $(shell command -v aarch64-none-elf-gcc >/dev/null 2>&1 && echo aarch64-none-elf-gcc || echo aarch64-linux-gnu-gcc)
A64_OBJDUMP ?= $(patsubst %gcc,%objdump,$(A64_CC))
IVERILOG    ?= iverilog
VVP         ?= vvp
VERILATOR   ?= verilator
# Windows PowerShell 5.x ('powershell') or PowerShell 7 ('pwsh', Linux/macOS)
PWSH        ?= $(shell command -v pwsh >/dev/null 2>&1 && echo pwsh || echo powershell)

# Strict DO-178C / MISRA C:2012 / BARR-C:2018 compiler profile (C Rules Sec. 12)
STRICT_CFLAGS := -std=c99 -pedantic-errors -O1 -fno-inline \
                 -Wall -Wextra -Werror -Wconversion -Wsign-conversion \
                 -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef \
                 -Wcast-qual -Wcast-align=strict -Wvla -Wstack-usage=256 \
                 -fstack-usage -fno-common -Idriver

TEST_CFLAGS   := -std=c99 -pedantic-errors -O1 -fno-inline \
                 -Wall -Wextra -Werror -Wconversion -Wsign-conversion \
                 -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef \
                 -Wcast-qual -Wvla -fno-common -Idriver -Itests

.PHONY: all sim xsim lint-rtl check-guards test-c build-arm build-arm64 verify-static clean

all: verify-static sim check-guards test-c

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Gate 1: Offline static verification (RTL lint + Bit-exact SDLC + C Rules + Traceability)
verify-static:
	$(PWSH) -NoProfile -ExecutionPolicy Bypass -File scripts/verify_project.ps1

# Gate 2a: RTL simulation via Icarus Verilog ([T1]-[T18])
sim: $(BUILD_DIR)
	$(IVERILOG) -g2001 -Wall -s tb_sm5800_imu_axi -o $(BUILD_DIR)/sim_sm5800.vvp $(RTL_SRCS) $(TB_SRCS)
	$(VVP) $(BUILD_DIR)/sim_sm5800.vvp

# Gate 2b: RTL simulation via Xilinx Vivado xsim ([T1]-[T18])
xsim:
	xvlog $(RTL_SRCS) $(TB_SRCS)
	xelab -debug typical tb_sm5800_imu_axi -s sim_sm5800
	xsim sim_sm5800 -runall

# Gate 2c: RTL static lint via Verilator
lint-rtl:
	$(VERILATOR) --lint-only -Wall --top-module sm5800_imu_axi_top $(RTL_SRCS)

# Gate 2d: Elaboration-time parameter guards. Each illegal configuration must
# FAIL with the named SM5800_PARAM_ERROR_* (a deliberately missing port of
# rtl/sm5800_param_error.v); each legal one must elaborate.
GUARD_TOP := -s sm5800_imu_axi_top
GUARD_P   := -Psm5800_imu_axi_top.
define guard_must_fail
	@if $(IVERILOG) -g2001 -o /dev/null $(GUARD_TOP) $(1) $(RTL_SRCS) 2>&1 | grep -q "$(2)"; \
	then echo "  [PASS] rejected: $(1)"; \
	else echo "  [FAIL] NOT rejected: $(1) (expected $(2))"; exit 1; fi
endef
define guard_must_pass
	@if $(IVERILOG) -g2001 -o /dev/null $(GUARD_TOP) $(1) $(RTL_SRCS) >/dev/null 2>&1; \
	then echo "  [PASS] accepted: $(if $(1),$(1),defaults)"; \
	else echo "  [FAIL] legal config rejected: $(1)"; exit 1; fi
endef

check-guards:
	$(call guard_must_pass,)
	$(call guard_must_pass,$(GUARD_P)C_CLK_FREQ_HZ=125000000)
	$(call guard_must_pass,$(GUARD_P)C_CLK_FREQ_HZ=33333333)
	$(call guard_must_pass,$(GUARD_P)C_DEFAULT_MSG2_ONLY=1)
	$(call guard_must_fail,$(GUARD_P)C_DEFAULT_MSG2_ONLY=1 $(GUARD_P)C_DEFAULT_TIMEOUT_US=5000,TIMEOUT_US_BELOW_12000)
	$(call guard_must_fail,$(GUARD_P)C_CLK_FREQ_HZ=10500000,RCV_CLK_OUTSIDE_2PCT)
	$(call guard_must_pass,$(GUARD_P)C_CLK_FREQ_HZ=25000000)
	$(call guard_must_fail,$(GUARD_P)C_CLK_FREQ_HZ=15000000,RCV_CLK_OUTSIDE_2PCT)
	$(call guard_must_fail,$(GUARD_P)C_CLK_FREQ_HZ=1500000,CLK_DIVIDER_OUT_OF_16BIT)
	$(call guard_must_fail,$(GUARD_P)C_RESET_PULSE_US=1000,RESET_PULSE_US_NOT_IN)
	$(call guard_must_fail,$(GUARD_P)C_S_AXI_ADDR_WIDTH=6,AXI_ADDR_LT_8)
	@echo "Parameter guards: ALL PASSED"

# Gate 3: Host C Unit Tests ([TC-DRV-01]-[TC-DRV-17], [TC-APP-01]-[TC-APP-02]) with Mock HAL
test-c: $(BUILD_DIR)
	$(HOST_CC) $(STRICT_CFLAGS) -c driver/sm5800_drv.c -o $(BUILD_DIR)/sm5800_drv.o
	$(HOST_CC) $(TEST_CFLAGS) -c tests/mock_sm5800_hal.c -o $(BUILD_DIR)/mock_sm5800_hal.o
	$(HOST_CC) $(TEST_CFLAGS) -c tests/test_sm5800_drv.c -o $(BUILD_DIR)/test_sm5800_drv.o
	$(HOST_CC) $(BUILD_DIR)/sm5800_drv.o $(BUILD_DIR)/mock_sm5800_hal.o $(BUILD_DIR)/test_sm5800_drv.o -o $(BUILD_DIR)/test_sm5800_drv
	./$(BUILD_DIR)/test_sm5800_drv
	$(HOST_CC) $(TEST_CFLAGS) -Dmain=app_example_main -c tests/test_ps_example.c -o $(BUILD_DIR)/test_ps_example.o
	$(HOST_CC) $(BUILD_DIR)/sm5800_drv.o $(BUILD_DIR)/mock_sm5800_hal.o $(BUILD_DIR)/test_ps_example.o -o $(BUILD_DIR)/test_ps_example
	./$(BUILD_DIR)/test_ps_example

# Gate 4: Cross-compile ARM Cortex-A9 PS Driver + emit .su stack usage + check ISR asm
# (also compiles the HAL for Cortex-R5 / ZynqMP RPU, same ARMv7 code path)
build-arm: $(BUILD_DIR)
	$(ARM_CC) -mcpu=cortex-r5 $(STRICT_CFLAGS) -c driver/sm5800_hal.c -o $(BUILD_DIR)/sm5800_hal_r5.o
	$(ARM_CC) -mcpu=cortex-a9 $(STRICT_CFLAGS) -c driver/sm5800_hal.c -o $(BUILD_DIR)/sm5800_hal.o
	$(ARM_CC) -mcpu=cortex-a9 $(STRICT_CFLAGS) -c driver/sm5800_drv.c -o $(BUILD_DIR)/sm5800_drv.o
	$(ARM_CC) -mcpu=cortex-a9 $(STRICT_CFLAGS) -c driver/sm5800_ps_example.c -o $(BUILD_DIR)/sm5800_ps_example.o
	$(ARM_OBJDUMP) -d $(BUILD_DIR)/sm5800_drv.o > $(BUILD_DIR)/sm5800_drv.dis
	@echo "Stack usage files (.su) generated in $(BUILD_DIR)/"

# Gate 4b: Cross-compile for Zynq UltraScale+ APU (Cortex-A53, AArch64)
build-arm64: $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/a53
	$(A64_CC) -mcpu=cortex-a53 $(STRICT_CFLAGS) -c driver/sm5800_hal.c -o $(BUILD_DIR)/a53/sm5800_hal.o
	$(A64_CC) -mcpu=cortex-a53 $(STRICT_CFLAGS) -c driver/sm5800_drv.c -o $(BUILD_DIR)/a53/sm5800_drv.o
	$(A64_CC) -mcpu=cortex-a53 $(STRICT_CFLAGS) -c driver/sm5800_ps_example.c -o $(BUILD_DIR)/a53/sm5800_ps_example.o
	$(A64_OBJDUMP) -d $(BUILD_DIR)/a53/sm5800_drv.o > $(BUILD_DIR)/a53/sm5800_drv.dis
	@echo "AArch64 objects, .su and disassembly generated in $(BUILD_DIR)/a53/"

clean:
	rm -rf $(BUILD_DIR) xsim.dir obj_dir *.jou *.log *.pb *.wdb *.vvp *.vcd
