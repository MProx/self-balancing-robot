# self-balancing-robot

A work in progress - this repository will contain all code and CAD models (to be added) associated with Debra, the self-balancing robot. 

The robot operates with a Raspberry Pi Zero W, and consists of a Pi, an MPU6050 IMU, a dual-motor driver board and a pair of geared DC electric motors with encoders on the outputs (photos to be added soon).

The code operates by monitoring the position of the wheels relative to some setpoint, and uses a cascading PID control system - the first level takes in a position setpoint (for now, immutable as the zero position - where the robot was when booted up), and outputs a desired ideal lean angle to achieve it. The second PID loop takes this desired lean angle, and outputs motor drive PWM. 

A third PID loop is also included, to control turning on the sopt. This uses the difference between the left and right encoders as an input, and outputs a turning compensation value that is added to the final drive PWM signal. Thus, if the robot starts to rotate left, the left motor is driven forward at a slightly faster rate and turns the robot right until the deviation is corrected. As with the position setpoint, there is no user input available yet, so the tuning setpoint is just zero - the robot will continue to face the direction it was faced in when first righted. 
