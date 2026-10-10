# Programming through SPI

HEEPidermis can be programmed directly into SRAM through its SPI slave interface using an external microcontroller. The current implementation uses a Raspberry Pi Pico, which also provides an optional configurable system clock.

The programming tools are located in [`sw/arduino/`](./sw/arduino/):

- `heep_programmer/heep_programmer.ino`: microcontroller firmware for SPI transactions, CPU reset, memory readback, and clock generation.
- `heep_sdk.py`: Python SDK implementing the communication protocol and ELF programming.
- `heep_cli.py`: command-line interface built on top of the SDK.
- `spi.mk`: Makefile targets for programming and controlling the ASIC.

## Setup

Flash `heep_programmer.ino` onto the Raspberry Pi Pico using the Arduino IDE. The current pin assignments are:

| Pico GPIO | Function |
|---|---|
| GP16 | SPI MISO |
| GP17 | SPI CS |
| GP18 | SPI SCK |
| GP19 | SPI MOSI |
| GP21 | ASIC system clock output |
| GP14 / GP15 / GP13 | Serial / SPI-write / SPI-read activity LEDs |

Connect the signals to the appropriate ASIC board interfaces, ensuring compatible logic levels and a common ground.

The host requires Python with `pyserial` and `pyelftools`. Activate the HEEPidermis environment before proceeding.

From the repository root, list available serial ports and configure the external programmer:

```bash
make spi-ports
make spi-port PORT=/dev/ttyACM0
```

The selected port is saved locally in `sw/arduino/.spi-port`. It can also be overridden per command using `SPI_PORT=/dev/ttyACM1`.

## System clock control

The Pico can generate the ASIC system clock through GPIO21. With `CLOCK_ON_AT_STARTUP=0`, the clock is disabled at startup, and GPIO21 is driven LOW.

```bash
make spi-clock-status
make spi-clock-set CLK_FREQ=1000000
make spi-clock-on
make spi-clock-off
```

The frequency is expressed in Hz. By default, frequency changes are prohibited while the clock is running. This behavior is controlled by `ALLOW_LIVE_CLOCK_FREQUENCY_CHANGE` in the Arduino firmware.

**Note:** This controls the ASIC system clock, not the SPI communication frequency. The SDK does not automatically enable the system clock before programming.

## Compiling and programming an application

First, compile the application for on-chip SRAM execution:

```bash
make app PROJECT=<application_name> BOOT_MODE=force
```

The resulting ELF file is normally located at `sw/build/main.elf`.

When the Pico supplies the ASIC system clock, enable it before accessing the chip:

```bash
make spi-clock-on
```

Program the application:

```bash
make spi-program
```

Optional arguments enable memory verification and application execution:

```bash
make spi-program verify
make spi-program run
make spi-program verify run
```

By default, the programmer resets the CPU to Boot ROM before loading the ELF into SRAM. Verification reads the programmed memory back through SPI and compares it byte-for-byte against the expected image. If `run` is specified, the CPU starts executing from the ELF entry point after programming and optional verification.

A different ELF can be selected using:

```bash
make spi-program SPI_ELF=path/to/application.elf verify run
```

CPU reset can also be triggered independently:

```bash
make spi-reset
```

This resets the CPU domain through the power-manager registers; it does not power-cycle or reset the entire board.

The `no-reset` modifier is available for advanced use:

```bash
make spi-program no-reset
```

It should only be used when the CPU state is already controlled, since a running application could modify SRAM during programming or verification.

# Running HEEPidermis from the internal VCO clock

HEEPidermis can operate using **VCOp as its system clock**, removing the need for a continuous external clock. VCOn remains available for skin-conductance sensing, while VCOp can also be monitored as a reference for supply-voltage variations and pseudo-differential measurements.

**1. Initialize VCOp using the external clock**

Start with the Raspberry Pi Pico providing the system clock (1 MHz):

```bash
make spi-clock-on
make VCO_clock
make spi-program run
```

`make VCO_clock` compiles the dedicated `VCO_clock_init` application with `BOOT_MODE=force`. This application enables VCOp and leaves it running continuously.

Do not compile the initialization application with `VCO_IS_SYSCLK`, since it must enable VCOp for the first time.

**2. Switch to the VCO clock**

Once the VCO output is stable:

```bash
make spi-reset
make spi-clock-off
```

The SPI CPU-domain reset returns the processor to Boot ROM without resetting the VCO configuration.

**Physically switch** the ASIC clock input from the Pico-generated clock to the VCOp output. The ASIC can now run from its internal oscillator.

**Important:** `spi-clock-off` drives Pico GP21 LOW rather than setting it to high impedance. The Pico clock output must therefore be physically isolated from the VCO clock output to avoid electrical contention.

**3. Compile and program subsequent applications**

Any application running with VCOp as the clock source must be compiled with:

```bash
make app PROJECT=test_VCO_counter BOOT_MODE=force CDEFS=VCO_IS_SYSCLK
```

The `VCO_IS_SYSCLK` compile-time definition activates protection in `sw/external/lib/drivers/VCO_decoder/VCO_decoder.h`, making calls to `VCOp_enable(true)` and `VCOp_enable(false)` no-ops.

VCOn remains independently configurable.

Program and execute the application using:

```bash
make spi-program verify run
```

The default SPI programming reset only resets the CPU domain, preserving the running VCO. This allows successive applications to be uploaded without switching back to the external clock.

<!-- **4. Limitations and precautions**

- Never directly modify the VCOp enable bit while it supplies the system clock. The compile-time flag protects calls through the driver, not arbitrary register writes.
- When both VCOs are enabled, the hardware decoder's combined output represents the difference between their measurements. Independent P/N coarse counters remain readable through `VCOp_get_coarse()` and `VCOn_get_coarse()`.
- Avoid modifying VCOp biasing or supply conditions in ways that could stop oscillation or change the clock frequency outside the CPU's operating limits.
- When changing `CDEFS`, ensure CMake is reconfigured so that the new definition is actually applied.
- The VCO clock configuration is retained across the tested CPU-domain reset, **not** across power cycling or a full hardware reset.
- All SPI logic levels must remain compatible; the tested setup uses discrete level shifting for the ASIC-to-Pico MISO signal. -->

**Validation:** The clock handover and subsequent SPI programming with VCOp supplying the system clock have been demonstrated on the HEEPidermis hardware.