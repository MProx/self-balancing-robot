# self-balancing-robot

This repository will contain all code associated with Debra, the self-balancing robot. 

<img src="https://github.com/MProx/self-balancing-robot/blob/master/Robot.jpg" width="612" height="1088" />

![Photo](  =612x1088)

The robot consists of a the following components (photos to be added soon):

* Raspberry Pi Zero W: Single-board computer, with wifi built in for uploading code and adjusting tuning parameters. Located inside a black enclosure.

* Frame: Made from masonite board (CNC routed), and 4x threaded rods. Motor mounting brackets and fasteners were supplied with the motors.

* Motor driver: Break-out board for an L298 dual motor driver, capable of up to 2A per motor.

* Motors: Dual motors (350RPM, 12v) which include encoders (11ppr on the motor output shaft). Note that the motor output has a 34:1 gearing ratio, so rotating the wheel once results in 374 pulses.

* Wheels: 65mm diameter, supplied with the motors

* IMU: MPU6050, 6-degree-of-freedom accelerometer/gyroscope (breakout board)

* Step-down converter: To regulate the battery voltage (9V - 12.5V) to 5V for the raspberry pi and peripheral circuitry.

* Battery: 2200mAh lipo battery, canabalized from another project. It is quite heavy, and aides in balancing when mounted high up on the frame. Includes a voltage monitor with low-voltage alarm.

The code operates by counting the encoder pulses to monitor the position of the wheels relative to some setpoint. The position is the average encoder position of the two wheels. The position is then fed into a cascading PID control system - the first level takes the position and outputs a desired ideal lean angle to achieve it. The second PID loop takes this desired lean angle, and outputs motor drive PWM to achieve it.

A third PID loop is also included, to control turning on the spot. This uses the difference between the left and right position encoders as an input, and outputs a turning compensation value that is added to the final drive PWM signal. Thus, if the robot starts to rotate left, the left motor is driven forward at a slightly faster rate and turns the robot right until the deviation is corrected.

Control, steering and parameter tuning are handled in a separate thread. A TCP socket is created, and waits incoming instructions. A phone app (such as RoboRemo) can be used to connect to this and send text strings. These strings are used to adjust the values od the P, I and D control coefficients to adjust behaviour, as well as the position and turning setpoints to adjust 
