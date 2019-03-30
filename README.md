# ABB Modbus Simulation
![IMG_8265](https://user-images.githubusercontent.com/12455851/55266059-c02ed980-5283-11e9-92b1-9f2d4de2fefe.jpg)

The project is a simulation of a ventilation sysem consisting of an LPCXpresso1549 board and an emulation of an ABB drive.
The LPC board controls the drive by Modbus protocol through which it can command the motor to run at specific speeds as well as
read pressure sensor value. All of that information is governed by PID algorithm .ie the LPC board takes the pressure setpoint and drive
the motor to meet it.

## Technical topics covered
**PID algorithm:** a controlling policy that takes action based on the error between the current state and the goal state
**Modbus:** a serial communication protocol commonly used in industial electronic devices.


## Demo

The demos below show two modes: setting fan speed and displaying corresponding pressure, modulating fan speed to meet desired pressure
as fast as posible. The modes are switched back and forth by the upper left push button (the furthest one in the videos) while the two buttons
in the bottom row are to adjust the current value on each mode.<br/>

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/w_OgqJI0uC8/0.jpg)](https://www.youtube.com/embed/w_OgqJI0uC8?start=0&end=90)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/s1ySed8BaQ0/0.jpg)](https://www.youtube.com/watch?v=s1ySed8BaQ0)

## Useful links
**1. Mobus:** https://en.wikipedia.org/wiki/Modbus<br/>
**2. PID control:** https://en.wikipedia.org/wiki/PID_controller
