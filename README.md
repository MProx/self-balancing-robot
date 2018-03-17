# self-balancing-robot

A work in progress - this repository will contain all code and CAD models (to be added) associated with Debra, the self-balancing robot. 

The robot consists of a the following components (photos to be added soon):

* Raspberry Pi Zero W: Single-board computer, with wifi built in for uploading code and adjusting tuning paramters.

* Frame: Made from masonite board (CNC routed), and 4x threaded rods. Motor mounting brackets were supplied with the motors.

* Drive: Dual motors (350RPM, 12v) which include encoders (11ppr on the motor output shaft). Note that the motor output has a 34:1 gearing ratio, so rotating the wheel once results in 374 pulses.

* Wheels: 65mm diameter, supplied with the motors

* IMU: MPU6050, 6-degree-of-freedom accelerometer/gyroscope (breakout board)

* Motor driver: Break-out board for an L298 dual motor driver, capable of up to 2A per motor.

* Battery: 2200mAh lipo battery, canabalized from another project. It is quite heavy, and aides in balancing when mounted high up on the frame (this makes sense: imagine balancing a broomstick on your finger, vs balancing a pencil).

The code operates by monitoring the position of the wheels relative to some setpoint, and uses a cascading PID control system - the first level takes in a position setpoint (for now, immutable as the zero position - where the robot was when booted up), and outputs a desired ideal lean angle to achieve it. The second PID loop takes this desired lean angle, and outputs motor drive PWM. 

A third PID loop is also included, to control turning on the sopt. This uses the difference between the left and right encoders as an input, and outputs a turning compensation value that is added to the final drive PWM signal. Thus, if the robot starts to rotate left, the left motor is driven forward at a slightly faster rate and turns the robot right until the deviation is corrected. As with the position setpoint, there is no user input available yet, so the tuning setpoint is just zero - the robot will continue to face the direction it was faced in when first righted. 
