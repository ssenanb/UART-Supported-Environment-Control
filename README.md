# UART-Supported-Environment-Control
-> Project Description

This project monitors the environment by collecting data from temperature and motion sensors. Based on the collected data, it controls a fan (DC motor with a propeller) and the brightness of an LED. The system supports both manual and automatic operation modes. It can be manually controlled via UART and displays its current status on an LCD screen.


-> Compenents Used

STM32FODISC

IR Distance Sensor

DHT11 Temperature and Humidity Sensor

LED

Resistance(330)

3-6V DC Motor

L298N Motor Driver

4 * 1.5V Batteries

UART Module

16x2 I2C LCD 

Jumper Cables


-> Features

* Automatic Mode : If the temperature is higher than the threshold, the fan turns on and if the distance is smaller than the threshold, the LED turns on. All of the status display in the LCD screen.
  
* Manuel Mode : If you send the 'MANUAL' command via UART, the system switches to manual mode. All of the command in the below :

  AUTO -> The system switches to automatic mode again. (Auto Mode' is printed to the Termite terminal via UART.)
  
  MANUEL -> The system switches to manuel mode. (Manuel Mode' is printed to the Termite terminal via UART)

  FAN ON -> The fan turns on.

  FAN OFF -> The fan turns off.

  LED ON -> The LED turns on.

  LED OFF -> The LED turns off.

All of the stages display in the LCD screen.

Figure 1 : System photo

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/system_installation.jpeg" alt="System Configuration" width="500"/>

Figure 2 : Automatic Mode

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/auto_mode.jpeg" alt="Auto Mode" width="500"/>

Figure 3 : Manuel Mode

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/manuel_mode.jpeg" alt="Manuel Mode" width="500"/>

Fİgure 4 : In the Termite

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/termite_display.png" alt="Termite" width="500"/>




