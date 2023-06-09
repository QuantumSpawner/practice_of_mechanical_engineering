# Practice of Mechanical Engineering

## Introduction

- Pins are denoted as `PIN`, e.g. `5V`, `PA0`, etc.

## STM32 Peripherals Used

- TIM1 channel * 4, TIM3 channel * 4 -> Servo
- ADC0 channel * 4, TIM8 -> Grayscale sensor
- GPIO * 3 (pull-down) -> Button
- GPIO * 3 -> LED
- GPIO * 4 (pull-down) -> Microswitch
- GPIO * 4 -> Motor controller

## Power

- Battery power `VBAT`
- 5V `5V`
- 3.3V `3V3`

Common Ground for all

### Battery - 18650 * 2

Battery `VCC` -> `VBAT`

### Linear Voltage Regulator

#### 5V

- `IN` -> `VBAT`
- `OUT` -> `5V`

#### 3.3V

- `IN` -> `VBAT`
- `OUT` -> `3V3`

## Wiring

### Nucleo

- Nucleo `5V` -> rocker switch -> `5V`

### Button

- Button1 `NO` -> Nucleo GPIO `PB1`
- Button2 `NO` -> Nucleo GPIO `PB2`
- Button3 `NO` -> Nucleo GPIO `PB6`

All `C` -> `3V3`

### Microswitch - 10T85

- MicroFB `NO` -> Nucleo GPIO `PC11`
- MicroFT `NO` -> Nucleo GPIO `PC10`
- MicroRB `NO` -> Nucleo GPIO `PD2`
- MicroRT `NO` -> Nucleo GPIO `PC12`

All `C` -> `3V3`

### Analog Sensor

- AngFL1 `OUT` -> Nucleo ADC1 IN13 `PC3`
- AngFL2 `OUT` -> Nucleo ADC1 IN12 `PC2`
- AngFR1 `OUT` -> Nucleo ADC1 IN10 `PC0`
- AngFR2 `OUT` -> Nucleo ADC1 IN11 `PC1`
- AngRL `OUT` -> Nucleo ADC1 IN7 `PA7`
- AngRR `OUT` -> Nucleo ADC1 IN15 `PC5`
- All `VCC` -> `3V3`

### LED

- LED_GREEN `VCC` -> Nucleo GPIO `PB10`
- LED_YELLOW `VCC` -> Nucleo GPIO `PB5`
- LED_RED `VCC` -> Nucleo GPIO `PB4`

### Servo - MG996R

- ServoFL `PWM` -> Nucleo TIM1 PWM1 `PA8`
- ServoFR `PWM` -> Nucleo TIM1 PWM3 `PA10`
- ServoRL `PWM` -> Nucleo TIM3 PWM4 `PC9`
- ServoRR `PWM` -> Nucleo TIM3 PWM3 `PC8`
- ServoRev1 `PWM` -> Nucleo TIM1 PWM2 `PA9`
- ServoRev2 `PWM` -> Nucleo TIM3 PWM2 `PC7`
- ServoRev3 `PWM` -> Nucleo TIM3 PWM1 `PA6`
- ServoRev4 `PWM` -> Nucleo TIM1 PWM4 `PA11`
- All `VCC` -> `VBAT`

### Motor Controller - DRV8833

- MotConRight `AIN1` -> Nucleo GPIO `PB13`
- MotConRight `AIN2` -> Nucleo GPIO `PC4`
- MotConRight `BIN1` -> Nucleo GPIO `PB14`
- MotConRight `BIN2` -> Nucleo GPIO `PB15`
- MotConLeft `AIN1` -> Nucleo GPIO `PA1`
- MotConLeft `AIN2` -> Nucleo GPIO `PA0`
- MotConLeft `BIN1` -> Nucleo GPIO `PA4`
- MotConLeft `BIN2` -> Nucleo GPIO `PB0`
- All `VM` -> `VBAT`
- All `STBY` -> `3V3`

### Motor

- MotFL `Red` -> MotConLeft `BO1`
- MotFL `Black` -> MotConLeft `BO2`
- MotFR `Red` -> MotConRight `AO1`
- MotFR `Black` -> MotConRight `AO2`
- MotRL `Red` -> MotConLeft `AO1`
- MotRL `Black` -> MotConLeft `AO2`
- MotRR `Red` -> MotConRight `BO1`
- MotRR `Black` -> MotConRight `BO2`

<div style="page-break-after: always;"></div>

## Appendix

### MCU - NUCLEO-F446RE

#### Arduino Pinout

<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_arduino_left_2021_10_26.png"  width="80%" height="40%">
<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_arduino_right_2021_10_26.png"  width="80%" height="40%">

<div style="page-break-after: always;"></div>

#### ST Morpho Pinout

<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_morpho_left_2021_10_26.png"  width="90%" height="45%">
<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_morpho_right_2021_10_26.png"  width="90%" height="45%">

<div style="page-break-after: always;"></div>

### Servo - MG996R

#### Timing

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/6c/Servomotor_Timing_Diagram.svg/1280px-Servomotor_Timing_Diagram.svg.png"  width="70%" height="35%">

#### Pinout

<img src="https://www.botnroll.com/11296-medium_default/mg996r-servo-metal-gear-high-torque.jpg"  width="70%" height="35%">

<div style="page-break-after: always;"></div>

### Motor Controller - DRV8833

#### Pinout

<img src="https://electropeak.com/learn/wp-content/uploads/2021/01/DRV8833-Dual-Driver-Pinout.jpg"  width="90%" height="45%">
