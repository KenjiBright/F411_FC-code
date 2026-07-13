# Drone FC v1 — Claude Code Project Brief

## Tổng quan dự án

Tự xây dựng Flight Controller (FC) hoàn chỉnh từ phần cứng đến firmware, tương thích song song với **Betaflight** và **firmware FC tự viết**. Nạp firmware qua USB OTG FS (DFU bootloader) hoặc ST-Link.

---

## Phần cứng

### MCU
- **STM32F411CEUx** (Black Pill, UFQFPN48)
- Clock: HSE 25MHz → PLL → HCLK 96MHz, USB 48MHz
- Nạp firmware: DFU qua BOOT0+RESET, hoặc ST-Link SWD

### Cảm biến & ngoại vi
| Module | Giao tiếp | Địa chỉ/CS |
|---|---|---|
| ICM-20602 (IMU) | SPI1 | CS → PA4 |
| BMP388 (Barometer) | I2C1 | 0x76 (SDO→GND, CS→3.3V) |
| W25Q128 (Flash 16MB) | SPI2 | CS → PB12 |
| USB CDC | USB OTG FS | PA11/PA12 |
| RC Receiver | USART1 | PA9/PA10 |

### Pinout STM32F411
```
SPI1 (ICM-20602):
  PA4  → GPIO_Output   (CS)
  PA5  → SPI1_SCK
  PA6  → SPI1_MISO
  PA7  → SPI1_MOSI

I2C1 (BMP388):
  PB8  → I2C1_SCL
  PB9  → I2C1_SDA

SPI2 (W25Q128):
  PB12 → GPIO_Output   (CS)
  PB13 → SPI2_SCK
  PB14 → SPI2_MISO
  PB15 → SPI2_MOSI

USB OTG FS:
  PA11 → USB_OTG_FS_DM
  PA12 → USB_OTG_FS_DP

USART1 (RC Receiver - SBUS):
  PA9  → USART1_TX
  PA10 → USART1_RX (qua mạch đảo tín hiệu, xem bên dưới)
  Baud: 100000, 8E2, non-inverted ở phía MCU (SBUS được đảo bằng phần cứng ngoài)
```

> **Lưu ý quan trọng:** STM32F411's USART không có khả năng đảo tín hiệu (RX invert) trong phần cứng
> (không có `AdvancedInit`/RXINV trong HAL và CMSIS của dòng F411 — khác với F413/F7 trở lên).
> Vì SBUS là tín hiệu UART bị đảo, **bắt buộc phải có mạch đảo tín hiệu ngoài** giữa chân SBUS
> của receiver và PA10. Firmware cấu hình USART1 như UART bình thường (không invert).

### Nối cảm biến vật lý
```
ICM-20602 module:
  VCC  → 3.3V
  GND  → GND
  SCL/SPC → PA5
  SDA/SDI → PA7
  SA0/SDO → PA6  (MISO)
  CS      → PA4

BMP388 module:
  VIN  → 3.3V
  GND  → GND
  SCK  → PB8
  SDI  → PB9
  CS   → 3.3V   ← bắt buộc để vào I2C mode
  SDO  → GND    ← địa chỉ 0x76
  INT  → (không nối)

W25Q128 module:
  VCC  → 3.3V
  GND  → GND
  CS   → PB12
  CLK  → PB13
  DO   → PB14
  DI   → PB15
  WP   → 3.3V   ← bắt buộc
  HOLD → 3.3V   ← bắt buộc

SBUS inverter (bắt buộc, vì STM32F411 không invert được bằng phần cứng):
  Mạch đảo NPN đơn giản (1 transistor NPN + 2 điện trở):
    SBUS (từ receiver) → R1 10kΩ → Base
    Emitter → GND
    Collector → PA10 (USART1_RX)
    Collector → R2 10kΩ (pull-up) → 3.3V   ← dùng 3.3V, KHÔNG dùng 5V
  Transistor: BC547 / 2N3904 / 2N2222 / S8050 (loại nào cũng được, SBUS chỉ 100kbps)
  GND chung giữa receiver, mạch đảo, và Black Pill
```

### Nguồn điện
```
BEC 5V → chân 5V Black Pill → AMS1117-3.3 onboard → 3.3V rail
Tụ lọc: 100µF hóa trên 5V rail + 100µF hóa trên 3.3V rail
USB VBUS và BEC 5V: cần Schottky diode OR-ing (SS14) cho custom PCB
```

---

## Cấu trúc project

```
Drone_fc_v1/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── drivers/         ← thêm vào
│   │       ├── icm20602.h
│   │       ├── bmp388.h
│   │       └── w25q128.h
│   └── Src/
│       ├── main.c           ← entry point
│       └── drivers/         ← thêm vào
│           ├── icm20602.c
│           ├── bmp388.c
│           └── w25q128.c
├── Drivers/                 ← HAL do CubeMX generate
├── Middlewares/
├── USB_DEVICE/              ← USB CDC
└── Makefile
```

---

## Firmware — những gì cần làm

### 1. Sensor drivers (viết trong `Core/Src/drivers/`)

**ICM-20602 (SPI1):**
- Khởi tạo SPI1, CS = PA4
- Đọc WHO_AM_I (expect 0x12)
- Cấu hình: Gyro 2000dps, Accel 16g, DLPF, ODR 1kHz
- Burst read 14 bytes: AccelXYZ + Temp + GyroXYZ
- Gyro calibration 500 samples khi khởi động (tính offset)

**BMP388 (I2C1):**
- Địa chỉ 0x76
- Đọc CHIP_ID (expect 0x50)
- Đọc calibration data từ NVM
- Cấu hình: OSR pressure x8, OSR temp x1, ODR 50Hz
- Tính altitude từ pressure (công thức Bosch compensation)

**W25Q128 (SPI2):**
- CS = PB12
- Verify JEDEC ID (0xEF, 0x40, 0x18)
- Hàm: read, page write, sector erase, chip erase
- Blackbox storage: lưu flight log dạng binary

### 2. USB CDC output
```c
// Trong USER CODE vùng main loop
// Output JSON 20Hz qua USB CDC
// Format: {"ax":0.0,"ay":0.0,"az":0.0,"gx":0.0,"gy":0.0,"gz":0.0,"alt":0.0}
```

### 3. RC Receiver (USART1 - SBUS)
```c
// Sau HAL_UART_Init:
huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
huart1.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
HAL_UART_Init(&huart1);
// SBUS: 25 bytes/frame, 100000 baud, 8E2
```

### 4. PID cascade control (sau khi sensor đọc được)
- Outer loop: Angle PID → rate setpoint
- Inner loop: Rate PID → motor output
- Timer: TIM1/TIM2 PWM output cho 4 motor (DSHOT hoặc PWM 50Hz)

---

## Build & Flash

### Build
```bash
make -j4
# Output: build/Drone_fc_v1.elf, build/Drone_fc_v1.bin
```

### Flash qua DFU (không cần ST-Link)
```bash
# Giữ BOOT0 + nhấn RESET + thả RESET + thả BOOT0
dfu-util -a 0 -s 0x08000000:leave -D build/Drone_fc_v1.bin
```

### Flash qua ST-Link
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c 'program build/Drone_fc_v1.bin 0x08000000 verify reset exit'
```

### Debug qua ST-Link
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
# Kết nối GDB trong terminal khác:
arm-none-eabi-gdb build/Drone_fc_v1.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) load
```

---

## Betaflight compatibility

- Board dùng USB OTG FS làm Virtual COM Port → Betaflight Configurator nhận ra như COM port thường
- Flash Betaflight target qua DFU mode (cùng cơ chế với firmware tự viết)
- Target phù hợp nhất: **MAMBAF411** hoặc tự build custom target
- Resource mapping cần chỉnh lại trong Betaflight CLI cho khớp pinout này

---

## Quy tắc code

- Chỉ viết code trong vùng `/* USER CODE BEGIN */` và `/* USER CODE END */`
- Code ngoài vùng này sẽ bị CubeMX xóa khi regenerate
- Sensor driver đặt trong `Core/Src/drivers/` và `Core/Inc/drivers/`
- Dùng HAL functions của STM32 (HAL_SPI_Transmit, HAL_I2C_Master_Transmit, v.v.)
- Language: C (C99)

---

## Trạng thái hiện tại

- [x] CubeMX cấu hình xong (SPI1, SPI2, I2C1, USB CDC, USART1, Clock 96MHz)
- [x] Code generate với Makefile
- [x] Viết driver ICM-20602
- [x] Viết driver BMP388
- [x] Viết driver W25Q128
- [x] USB CDC JSON output (20Hz, `-u _printf_float` cho snprintf %f)
- [x] SBUS RC receiver parsing (firmware xong — cần lắp mạch đảo tín hiệu phần cứng, xem phần Nối cảm biến vật lý)
- [x] PID control loop (cascade angle→rate, `flight_control.c`)
- [x] Motor output — DSHOT300 on TIM2 CH1-4 (PA0-PA3), DMA per channel (`dshot.c`),
      Quad-X mixer + arm-switch gating (`motor_mixer.c`). **Bench-verify props OFF
      before flying — sign convention + DMA stream mapping unconfirmed on hardware.**
