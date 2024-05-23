# Laser Tag System

## Overview

This project implements a recreational laser tag system that allows players to engage in a simulated shooting game. The system includes features such as shooting detection, score tracking, and real-time feedback to enhance the gaming experience. Check out the [demo video](https://www.youtube.com/watch?v=3bv4x8FTByI).

## Features

- **Button, Laser Emitting Diode, and Light Sensor Module**: 
  - K1 and K2 buttons on the STM board for initial team choosing.
  - The button attached to the gun acts as the trigger, emitting the laser beam for 0.5 seconds when pressed.
  
- **Infrared Emitter and Infrared Sensor**: 
  - Custom IR protocol and decoding functions.
  - Players on different teams shoot different IR waveforms to solve "friendly firing" issues (teammates shooting each other).
  
- **Variable Resistor**: 
  - Determines the number of bullets shot in auto-shooting mode.
  
- **Accelerometer**: 
  - Used in the gun reloading mechanism. When the gun is pointed upwards over a certain angle, it reloads one bullet every 0.75 seconds.
  
- **Programmable LED Strip**: 
  - One LED strip on the top of the gun indicating bullet count, auto-shooting mode, and team status.
  - Another LED strip wrapped around the target showing the player's team and hit status.
  
- **LCD**: 
  - Displays player information such as remaining lives, team, and scores.
  
- **Bluetooth**: 
  - Keeps track of players' teams and scores.

## Components

### Target

- 1 * Programmable RGB LED Strip WS2812B
- 1 * Buzzer
- 1 * Infrared Receiver
- 1 * LCD
- 1 * Bluetooth Module HC-05

### IR Gun

- 1 * Laser Diode Module ky-008
- 1 * Infrared Emitting Diode
- 1 * Variable Resistor
- 1 * Programmable RGB LED Strip WS2812B
- 1 * Button
- 1 * Gyroscope (Accelerometer)

## Pin Choices and Protocols

- **LED Strip**: PWM, DMA
- **Buzzer**: GPIO
- **Infrared Receiver**: GPIO
- **LCD**: FSMC
- **Bluetooth Module**: UART
- **Laser Diode Module**: GPIO
- **Infrared Emitting Diode**: GPIO
- **Variable Resistor**: ADC
- **Button**: GPIO
- **Gyroscope (Accelerometer)**: I2C

<img width="1118" alt="Pin Choices" src="https://github.com/seanpanpan321/Laser_Tag/assets/89929031/9a1a6d4d-d95d-4184-8d6c-2fa316c52739">


## Infrared Encoding/Decoding

We implemented our own basic IR protocol and decoding functions, so players on different teams shoot out different IR waveforms, solving "friendly firing" issues.
<img width="867" alt="Infrared Protocol" src="https://github.com/seanpanpan321/Laser_Tag/assets/89929031/401b59c7-64f0-4e4e-acca-bc59a32849a7">

