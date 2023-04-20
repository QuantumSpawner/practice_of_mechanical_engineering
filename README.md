# Practice of Mechanical Engineering

## Introduction

## Peripherals Used

- TIM3 -> PWM for servo
- ADC0 channel * 2 -> Grayscale sensor
- GPIO * 4 (pull down) -> Microswitch

## Hardware Pin

### Servo - MG996R

1. Front left -> TIM3 PWM1 (PA6)
2. Front right -> TIM3 PWM2 (PA7)
3. Rear left -> TIM3 PWM3 (PB0)
4. Rear right -> TIM3 PWM4 (PB1)
5. Other servo1 -> TIM1 PWM1 (PA8)
6. Other servo2 -> TIM1 PWM2 (PA9)
7. Other servo3 -> TIM1 PWM3 (PA10)
8. Other servo4 -> TIM1 PWM4 (PA11)

#### Bonus

Servo timing

![](https://upload.wikimedia.org/wikipedia/commons/thumb/6/6c/Servomotor_Timing_Diagram.svg/1280px-Servomotor_Timing_Diagram.svg.png)

Servo pinout

![](https://www.botnroll.com/11296-medium_default/mg996r-servo-metal-gear-high-torque.jpg)

### Motor Controller - TB6612FNG

1. Front1 -> GPIO (B12)
2. Front2 -> GPIO (B13)
3. Rear1 -> GPIO (B14)
4. Rear2 -> GPIO (B15)

#### Bonus

TB6612FNG pinout

![](https://content.instructables.com/FCN/O9VG/JHATTMWR/FCNO9VGJHATTMWR.png)

[datasheet](https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf)

### Grayscale Sensor

1. Left -> ADC1 IN14 (PC4)
2. Right -> ADC IN15 (PC5)
3. Other sensor1 -> ADC IN0 (PA0)
4. Other sensor2 -> ADC IN1 (PA1)

### Microswitch - 10T85

1. Front bottom -> GPIO (PC0)
2. Front top -> GPIO (PC1)
3. Rear bottom -> GPIO (PC2)
4. Rear top -> GPIO (PC3)

### Buck Converter - MP1584EN

#### Bonus

MP1584EN pinout

![](https://components101.com/sites/default/files/component_pin/MP1584-Pinout.jpg)

## NUCLEO-F446RE Pinout

### Arduino Pinout

![](https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_arduino_left_2021_10_26.png)
![](https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_arduino_right_2021_10_26.png)

### ST Morpho Pinout

![](https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_morpho_left_2021_10_26.png)
![](https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f446re_morpho_right_2021_10_26.png)
