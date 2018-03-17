# self-balancing-robot

A work in progress - this repository will contain all code and CAD models (to be added) associated with Debra the self-balancing robot. 

The robot operates with a Raspberry Pi Zero W, and consists of a Pi, an MPU6050 IMU, a dual-motor driver board and a pair of geared DC electric motors with encoders on the outputs (photos to be added soon).

The code operates by monitoring the position of the wheels relative to some setpoint, and uses a cascading PID control system - the first level takes in a position setpoint (for now, immutable as the zero position - where the robot was when booted up), and outputs a desired ideal lean angle to achieve it. The second PID loop takes this desired lean angle, and outputs motor drive PWM. 

For now, slight manufacturing differences in the motors result in the robot slowly pivoting on the spot. This is because the robot postion is calculated as the average of the two encoder positions. The code doesn't react if one goes up and the other goes down. In the future, I may add a third PID level, to control motor speeds, and thus allow more precise turning control. This will be useful when I eventually add wifi control so that the user can steer and drive around.
