# firmware_COM4_verify reverse notes

Source files:

- `C:/Users/34494/Desktop/firmware_COM4_verify.bin`
- `C:/Users/34494/Documents/Codex/2026-07-08/jiang/outputs/firmware_COM4_verify.hex`

Important addresses are from the verified firmware image base `0x08000000`.

## Interrupt structure

- Vector table entry 39 points to `0x080067f2`. On STM32F103 this is `EXTI9_5_IRQHandler`.
- `0x080067f2` first reads bit-band alias `0x42218124`.
- `0x42218124` maps to GPIOB IDR bit 9, so the handler only runs when PB9 is low.
- The handler writes `0x200` to `0x40010414`, clearing EXTI pending bit 9.

Conclusion: MPU6050 INT is PB9 and is handled as active-low/falling-edge.

## Control ISR flow

At `0x080067f2`:

- `0x080037e4(4)` reads and clears TIM4 counter, stored at RAM `0x20000048`.
- `0x080037e4(8)` reads and clears TIM8 counter, negated and stored at RAM `0x2000004c`.
- `0x08006580(mode)` updates MPU attitude/control state. `mode` is loaded from RAM `0x2000001c`.
- `0x08002e8a()` runs an auxiliary periodic task.
- `0x0800641e(left_count, right_count)` converts encoder counts to wheel speed.
- A byte counter at RAM `0x20000015` divides by 10 before calling `0x08002a0a()` and `0x08002baa()`.
- `0x0800641e` uses a `200.0` double constant before wheel-speed scaling.
- The main control dispatch around `0x080069c8` calls:
  - `0x08005ed8(angle, gyro)` for the angle PD output.
  - `0x08005a5e(left_count, right_count)` for the velocity PI output.
  - a turn output path, then combines motor outputs as left = balance + velocity + turn and right = balance + velocity - turn.
  - `0x0800596a` clamps both final motor outputs to approximately `+/-6900`.

Conclusion: the successful firmware runs the fast balance path from PB9 EXTI, not from a main-loop tick poll. The `200.0` speed-scale constant shows the fast loop is 200Hz / 5ms.

## PID and PWM scale

- The scatter-load table at `0x0800f9f8` expands compressed startup data from `0x0800fa18` to RAM `0x20000000`.
- Decoded startup RAM defaults:
  - `0x20000080` = `32000.0f` (`Balance_Kp`)
  - `0x20000084` = `120.0f` (`Balance_Kd`)
  - `0x20000088` = `410.0f` (`Velocity_Kp`)
  - `0x2000008c` = `2.0f` (`Velocity_Ki`)
- `0x08005ed8` divides the angle-loop parameters by `100.0f` before calculating output.
- `0x08005a5e` uses velocity filtering constants `0.84` and `0.16`, accumulates a velocity integral, clamps that integral around `+/-380000`, and applies the velocity feedback with negative proportional/integral sign.
- `0x0800043e-0x08000444` initializes the TIM3 PWM path with period `0x1c1f`, while `0x0800572c` writes CCR values around `0x1c20` (`7200`).
- `0x0800572c` writes TIM3 CCR1-CCR4 directly:
  - left positive: CCR1 = `7200`, CCR2 = `7200 - output`
  - left negative: CCR1 = `7200 + output`, CCR2 = `7200`
  - right positive: CCR3 = `7200 - output`, CCR4 = `7200`
  - right negative: CCR3 = `7200`, CCR4 = `7200 + output`

Conclusion: the verified firmware uses raw PWM-scale motor output, not a 0-100 percent abstraction. The PID defaults are menu-style values scaled by 100.

## MPU6050 raw path

Inside `0x08006580`:

- If `mode == 1`, the firmware calls DMP/eMPL routines.
- Otherwise it reads raw MPU6050 registers through function `0x08003bc2(device, reg)`.
- Device address passed is `0xd0`, the 8-bit write address for MPU6050 address `0x68`.
- Raw register pairs read:
  - `0x3b/0x3c`
  - `0x3d/0x3e`
  - `0x3f/0x40`
  - `0x43/0x44`
  - `0x45/0x46`
  - `0x47/0x48`
- The firmware uses float math and stores computed attitude values in RAM around `0x20000060`, `0x20000064`, `0x20000068`, and `0x20000078`.

Conclusion: a direct raw-register MPU6050 path is valid, but it must be synchronized by PB9 data-ready interrupt.

## Filter constants found in firmware

The Kalman/filter section around `0x0800779c` includes:

- `0x3a83126f` = `0.001f`
- `0x3b449ba6` = `0.003f`
- `0x3ca3d70a` = `0.02f`

Conclusion: the verified firmware uses small Kalman/process-noise constants and a 0.02-style blend/step constant in its attitude filtering code.

## UART/menu strings

The firmware contains these strings near `0x08006f2c`:

- `DMP`
- `Kalman`
- `C F`
- `Angle`
- `Gyrox`

Conclusion: the original firmware exposes DMP, Kalman, complementary-filter, angle, and gyro-rate diagnostics. The current project should keep the angle-control entry simple but use a filter structure that matches this behavior.
