# Drone_fc_v1 (F411_FC-code)

A from-scratch flight controller firmware for the STM32F411 ("Black Pill"),
written in bare-metal C on top of STM32Cube HAL. It runs a 1 kHz sensor →
attitude → PID → mixer → DSHOT control loop, decodes SBUS RC input, and
streams live JSON telemetry over a USB CDC virtual COM port that's
Betaflight-Configurator-compatible.

This is a personal/hobby project, not a maintained library — expect sharp
edges and in-progress code.

## Status

- Sensors, RC input, PID cascade, and DSHOT motor output are all written and
  build clean (`make -j4`, zero warnings, ~57 KB flash).
- **Not yet flight-tested.** Motor output (DSHOT300 timing, DMA stream
  mapping, Quad-X sign convention) has not been verified on real hardware —
  only bench-checked on paper. See [Safety verification](#safety-verification)
  before ever spinning motors with props on.
- Blackbox logging (flash chip is wired but unused), I-term relax, and notch
  filtering are not started yet — see [Roadmap](#roadmap).

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | STM32F411CEU6 (Black Pill, UFQFPN48), 96 MHz | — |
| IMU | ICM-20602 (gyro + accel) | SPI1, CS = PA4 |
| Barometer | BMP388 | I2C1, addr 0x76 |
| Flash | W25Q128 (16 MB, for future blackbox logging) | SPI2, CS = PB12 |
| RC receiver | SBUS (100000 baud, 8E2) | USART1 |
| Host link | USB CDC virtual COM | USB OTG FS |
| Motors | 4x via DSHOT300 | TIM2 CH1-4, PA0-PA3 |

The F411 can't invert a UART signal in hardware, so **SBUS requires an
external inverter circuit** (single NPN transistor) between the receiver and
PA10. Full pinout, wiring diagrams, and power-supply notes are in
[`CLAUDE.md`](CLAUDE.md).

## Firmware architecture

No RTOS — a hardware timer (`scheduler.c`, TIM4) drives a 1 kHz tick consumed
by a super-loop in `main.c`.

| Module | Role |
|---|---|
| `drivers/icm20602` | IMU driver: init, WHO_AM_I check, gyro calibration, burst read |
| `drivers/bmp388` | Barometer driver: NVM calibration, Bosch compensation, altitude |
| `drivers/w25q128` | SPI NOR flash driver (read/write/erase) — not yet used in the main loop |
| `drivers/sbus` | Interrupt-driven SBUS frame decoder (16 channels + failsafe flags) |
| `attitude` | Complementary-filter roll/pitch estimator |
| `filter` | Reusable first-order low-pass filter |
| `pid` | Generic PID: anti-windup, derivative-on-measurement, D-term LPF, feedforward |
| `flight_control` | Cascaded angle→rate PID (roll/pitch) + rate PID (yaw), with TPA |
| `motor_mixer` | Quad-X mix of PID output + throttle; arm/disarm gated on SBUS ch5 |
| `dshot` | DSHOT300 encode + TIM2/DMA output to 4 motors |

Each control tick: read IMU → read SBUS → `FlightControl_Update` →
`MotorMixer_Update` → `DShot_Write`. Every 20 Hz, a JSON telemetry line
(`{"ax":...,"gz":...,"alt":...,"m1":...,"m4":...}`) is pushed over USB CDC.

## Project layout

```
Core/Inc, Core/Src/         Application code (main.c + modules above)
Core/Inc/drivers, .../Src/drivers/   Sensor/peripheral drivers
Drivers/CMSIS/               ARM CMSIS core + device headers
Drivers/STM32F4xx_HAL_Driver/  ST HAL (RCC, GPIO, SPI, I2C, UART, TIM, DMA, USB, ...)
Middlewares/ST/STM32_USB_Device_Library/  USB CDC middleware
USB_DEVICE/                  USB CDC application glue
Drone_fc_v1.ioc              STM32CubeMX peripheral configuration
Makefile                     GNU Makefile build (arm-none-eabi-gcc)
```

## Build

Primary build path is the root Makefile (GCC ARM Embedded toolchain):

```bash
make -j4
# Output: build/Drone_fc_v1.elf, build/Drone_fc_v1.bin, build/Drone_fc_v1.hex
```

An STM32CubeIDE managed-build project (`.project`/`.cproject`, output in
`Debug/`) is kept in parallel for dual-verification; it's optional if you're
just building from the CLI.

## Flash

**DFU (no ST-Link needed)** — hold BOOT0, press and release RESET, release
BOOT0, then:

```bash
dfu-util -a 0 -s 0x08000000:leave -D build/Drone_fc_v1.bin
```

**ST-Link / OpenOCD:**

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c 'program build/Drone_fc_v1.bin 0x08000000 verify reset exit'
```

## Debug

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
# in another terminal:
arm-none-eabi-gdb build/Drone_fc_v1.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) load
```

A Cortex-Debug launch config for VS Code is provided in `.vscode/launch.json`.

## Telemetry / Betaflight Configurator

The board enumerates as a normal USB CDC COM port and streams a 20 Hz JSON
telemetry line, so it shows up in Betaflight Configurator like any other FC
serial port. For flashing actual Betaflight firmware, the closest stock
target is **MAMBAF411**; resource mapping needs to be adjusted in the
Betaflight CLI to match this board's pinout.

## Safety verification

Do this with **props off** before ever mounting propellers:

1. Clean build, flash.
2. Arm switch (SBUS ch5) low → `m1..m4` in telemetry must stay 0 regardless of
   stick input.
3. Arm switch high, throttle at minimum → `m1..m4` must stay 0
   (throttle-stop threshold).
4. Raise throttle slowly → all four motors should spin up smoothly and in
   sync. Erratic revving indicates a DMA/timing bug — stop and recheck the
   DMA stream mapping (RM0383 Table 27) and DSHOT bit timing before continuing.
5. Hold throttle steady, move roll/pitch/yaw one axis at a time → confirm the
   expected differential response per motor. This is the point to fix a
   backwards sign in the Quad-X mix, before props are ever attached.

## Roadmap

- Hardware bench verification of DSHOT/mixer (above) — blocks first flight.
- Blackbox flight logger backed by the W25Q128 flash chip.
- I-term relax + anti-gravity.
- RPM-based notch filtering (needs bidirectional DSHOT telemetry; current
  DSHOT implementation is FC→ESC only).
- Dynamic/FFT notch filtering (blocked on pulling CMSIS-DSP into the build).

## Further reading

- [`CLAUDE.md`](CLAUDE.md) — full hardware reference: pinout, wiring
  diagrams, power supply notes, coding conventions.
- [`HANDOFF.md`](HANDOFF.md) — development session notes for the current
  (DSHOT/mixer) phase.

## License

No license file has been chosen yet for this project's own code. Vendored ST
and ARM code under `Drivers/CMSIS/` and `Drivers/STM32F4xx_HAL_Driver/`
carries its own respective license (see `LICENSE.txt` in those directories).
