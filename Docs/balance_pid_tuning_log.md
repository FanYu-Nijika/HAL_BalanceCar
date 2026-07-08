# Balance PID Tuning Log

## Hardware Notes Read From Documents

- Main controller: STM32F103RCT6 on C10B/B585 board.
- Motor PWM: TIM3, PA6/PA7/PB0/PB1.
- Encoders: TIM8 PC6/PC7 and TIM4 PB6/PB7.
- MPU6050 reserved interface: PB9/PB14/PB15. Current project uses PB14/PB15 as software I2C SCL/SDA.
- OLED: PB3/PB4/PB5/PC14, software SPI.
- USART1: PA9/PA10.

## Current Tuning Scope

Only the angle loop PD is enabled.

- Speed loop: disabled.
- Turn loop: disabled.
- Encoder feedback: initialized, but not used by the current angle-loop-only tuning.
- Control period: 10 ms, matching current eMPL DMP output rate of 100 Hz.
- OLED display period: 100 ms, separated from the control loop so screen refresh does not slow motor response.
- Balance angle source: complementary filter, using gyro integration for short-term response and accelerometer pitch for long-term correction.

## Tuning Entry

Edit these variables in `Users/balance_control.c`:

```c
volatile float Balance_Kp = 0.0f;
volatile float Balance_Kd = 0.0f;
volatile float Balance_Target_Angle = 0.0f;
volatile int Balance_Output_Dir = 1;
volatile int Balance_Left_Motor_Dir = 1;
volatile int Balance_Right_Motor_Dir = -1;
```

Display meanings on OLED:

- Left top: fused angle, from gyro integration plus accelerometer pitch correction.
- Left middle: gyro rate, from `mpu6050_get_pitch_gyro_rate()`.
- Left bottom: output PWM.
- Right top: `Balance_Kp`.
- Right middle: `Balance_Kd`.
- Right bottom: fall flag, 0 means active, 1 means stopped.

`Balance_Target_Angle` is automatically calibrated during `balance_control_init()`.
Hold the car at the real mechanical balance point during boot. The firmware averages
50 accelerometer-angle samples and uses that average as the stable upright angle.

## Safety Procedure

1. Lift the car so both wheels are off the ground.
2. Set `Balance_Kp = 0.0f`, `Balance_Kd = 0.0f`, build and download.
3. Hold the car at its real mechanical balance point for about 1 second after reset so zero-angle calibration can finish.
4. Tilt the car forward and backward by hand. Confirm `angle` changes sign and PWM remains 0.
5. Set `Balance_Kp = 2.0f`, `Balance_Kd = 0.0f`, keep wheels off the ground, and tilt forward.
6. The wheels must chase the falling direction. If they push the car further down, change only `Balance_Output_Dir` from `1` to `-1`.
7. Put the car on the ground only after motor direction is correct.

## Angle PD Tuning Process

Start with `Balance_Kd = 0.0f`, increase `Balance_Kp` first:

- Soft and falls slowly: increase `Balance_Kp`.
- Starts to stand but swings widely: keep `Balance_Kp`, increase `Balance_Kd`.
- High-frequency shaking: reduce `Balance_Kp` or increase `Balance_Kd` slightly.
- Very stiff and wheel output often hits limit: reduce `Balance_Kp`.
- Can stand for 30 seconds: record the parameters as a candidate.

Suggested first sweep for current PWM range `-100..100`:

| Trial | Balance_Kp | Balance_Kd | Expected observation | Result |
| --- | ---: | ---: | --- | --- |
| 1 | 2.0 | 0.0 | weak correction, check direction only | |
| 2 | 4.0 | 0.0 | stronger, may still fall | |
| 3 | 6.0 | 0.0 | may start oscillating | |
| 4 | 6.0 | 0.2 | adds damping | |
| 5 | 8.0 | 0.3 | candidate range | |
| 6 | 10.0 | 0.4 | may become stiff | |

Record actual tuning:

| Time | Balance_Kp | Balance_Kd | Target angle | Output dir | Observation | Stand time |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| | | | | | | |

## Stop Conditions

Stop testing immediately if:

- Angle display is stuck near 0 while tilting by hand.
- `dmp_read_fifo fail` repeats on serial output.
- Wheels drive in the wrong correction direction.
- Car tilts more than about 35 degrees.
- PWM stays near +/-80 while the car is still falling.
