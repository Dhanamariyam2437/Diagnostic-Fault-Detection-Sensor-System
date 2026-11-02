Diagnostic Fault-Detection Sensor System

A microcontroller project that monitors a small DC motor and raises an alert when temperature, vibration or current becomes abnormal. Built on an STM32 Blue Pill.

Features
Continuous monitoring of motor temperature, vibration and current
Fault confirmation by repeated readings, to avoid false alarms
Fault classification: thermal, electrical, mechanical
16x2 I2C LCD shows live readings and fault messages
Buzzer alert, and the motor is stopped on any fault
Push button resets the system after a fault
Hardware
Component	Purpose
STM32 Blue Pill	Main controller
DS18B20	Temperature sensor (1-Wire)
SW-420	Vibration sensor
ACS712	Current sensor (ADC input)
16x2 I2C LCD	Display
Buzzer	Audible alert
L298N + DC motor	Motor driver and motor under test
Push button	Fault reset
Pin Connections
Blue Pill pin	Connected to
PA0	DS18B20 data (4.7k pull-up to 3.3V)
PA1	SW-420 digital output
PA2	ACS712 output (ADC)
PA8	Buzzer (+)
PA9	Push button (other side to GND)
PB0	L298N IN1
PB1	L298N IN2
PB10	L298N ENA
I2C1	LCD (SCL, SDA)
PC13	Onboard LED (slow blink normal, fast blink on fault)

All grounds (Blue Pill, sensors, L298N, battery) must be common.

Fault Detection
Fault	Sensor	Trigger	LCD message
Thermal	DS18B20	Temperature above 50 C	FAULT: OVER TEMP
Electrical	ACS712	Current above 1.0 A in 3 consecutive readings (50 ms apart)	FAULT: OVERCURR
Mechanical	SW-420	2 vibration pulses within 300 ms	FAULT: VIBRATION

The limits are set in the settings block at the top of main.c and can be changed.

How It Works

The system uses four states:

NORMAL: readings are safe, the motor runs.
MONITORING: an abnormal reading is seen and checked for persistence.
FAULT: the abnormal condition persists and the fault type is identified.
ALERT: the motor stops, the buzzer sounds and the LCD shows the fault.

Pressing the button clears the fault and restarts the motor.

Project Files
main.c, main.h: application logic and settings
i2c_lcd.c, i2c_lcd.h: LCD driver (PCF8574 I2C backpack)
ds18b20.c, ds18b20.h: DS18B20 temperature driver
Build and Flash
Open the project in STM32CubeIDE.
Build the project.
Connect an ST-Link to the Blue Pill (SWDIO, SWCLK, GND, 3.3V) and flash.
Keep the BOOT0 jumper on the 0 side so the program runs on power-up.

Note: if your Blue Pill is a low-density clone (32 KB flash, 10 KB RAM), set FLASH LENGTH = 32K and RAM LENGTH = 10K in the linker (.ld) file.

Future Scope
Wireless monitoring
Data logging and trend analysis
Mobile or web dashboard
Accelerometer for better vibration sensing
Testing on more motor conditions
