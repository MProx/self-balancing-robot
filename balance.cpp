/*
Before compiling: 
- make sure i2c is enabled ("sudo raspi-config")
- make sure wiringPi is installed and configured (test with "gpio i2cdetect")

To compile:
- "g++ balance.cpp -o balance -lwiringPi -lpthread"

To run: 
- "sudo ./balance" - NB: always run with root privileges

Raspberrt Pi pin numbering reference:
=======================
MPU6050_SDA = GPIO 2
MPU6050_CLK = GPIO 3
EncoderLPinA = GPIO 26
EncoderLPinB = GPIO 19
EncoderRPinA = GPIO 20
EncoderRPinB = GPIO 16
DIRLA = GPIO 17
DIRLB = GPIO 27
DIRRA = GPIO 22
DIRRB = GPIO 10
OutputPinL = GPIO 18
OutputPinR = GPIO 13
*/

#include "balance.h"

int main()
{
	printf("Initializing:\n");
	init();	  

	printf("Waiting to be picked up..\n");
	
	while(1)
	{	
		acclY = read_word_i2c(0x3D);
		acclZ = read_word_i2c(0x3F);
		acclY_scaled = acclY/16384.0;
		acclZ_scaled = acclZ/16384.0;
		 
		theta_acc = atan(acclY_scaled/acclZ_scaled)*180/M_PI;
		
		if (abs(theta_acc) < 3 || flag == 1)
			break;
	}
		
	//program loop:
	while(1)
	{
		start_time = nanos();	//Get ns value
		   
		acclY = read_word_i2c(0x3D);
		acclZ = read_word_i2c(0x3F);
		acclY_scaled = acclY/16384.0;
		acclZ_scaled = acclZ/16384.0;
		 
		gyroX = read_word_i2c(0x43);
		gyroX_scaled = gyroX/262.0;
	    
		theta_acc = atan(acclY_scaled/acclZ_scaled)*180/M_PI;
		dTheta = gyroX_scaled - gyro_calib;

		theta = 0.99*(theta + dTheta*period) + 0.01*theta_acc; //dTheta term negative due to axes directions

		dTheta_s = dTheta_s*0.7 + dTheta*0.3; //smooth dTheta for PID integral term

		//velocity control mode:		
		//Read wifi for steering/velocity input (not implemented yet):

		//Add user input acquisition code here

			//read encoders:
			cycleCounter++;		
			if (cycleCounter == 5) //calculate on every nth cycle, so that recordable velocity resolution is finer
			{	
				cycleCounter = 0;
				velocityL = (posL - posLold)/5/period;
				posLold = posL; //posL updated in EncoderL_ISR
				velocityR = (posR - posRold)/5/period;
				posRold = posR; //posR updated in EncoderB_ISR
				
				velocityOld = velocity; 
				velocity = 0.6*velocity + 0.4*(velocityL + velocityR)/2;
				pos = (posL + posR)/2;
	
				//PID (outer) - takes user input and outputs reguired angle:
				setpoint_angle = PID(setpoint_pos, pos, velocity, &Ei_pos, 5*period, 0.0020, 0.003, 0.00315, 2.5);
				
				if (setpoint_angle > 15.0)
					setpoint_angle = 15.0;
				else if (setpoint_angle < -15.0)
					setpoint_angle = -15.0;
			}

		//PID (inner) - takes desired angle and outputs drive PWM signal:
		if (abs(setpoint_angle - theta) > 2.0)
			drive = PID(setpoint_angle, theta, dTheta_s, &Ei_angle, period, 180.0, 0.0, 2.5, 700.0);
		else
			drive = PID(setpoint_angle, theta, dTheta_s, &Ei_angle, period, 115.0, 0.0, 0.6, 700.0);

		pos_turn = (posL - posR)*0.10446; //in degrees, calculated using wheel diam and distance between wheels
 		turnSig = PID(setpoint_turn, pos_turn, 0, &Ei_turn, period, 05.0, 0, 0.0, 500.0);
		
		if (turnSig > 250)
		  turnSig = 250;
		else if (turnSig < -250)
		  turnSig = -250;

		pwmFinalL = int(drive + turnSig);
		pwmFinalR = int(drive - turnSig);
		
		//constrain final PWM to be between 0 and 1023
		if (pwmFinalL > 1023)
		  pwmFinalL = 1023;
		else if (pwmFinalL < -1023)
		  pwmFinalL = -1023;
		if (pwmFinalR > 1023)
		  pwmFinalR = 1023;
		else if (pwmFinalR < -1023)
		  pwmFinalR = -1023;

	
		//Set direction pins on motor driver:	
		if (pwmFinalL >= 0)
		{
		  digitalWrite(DIRLB, LOW);
		  digitalWrite(DIRLA, HIGH);
		}
		else
		{
		  digitalWrite(DIRLA, LOW);
		  digitalWrite(DIRLB, HIGH);
		  pwmFinalL *= -1; //take absolute value of E if negative
		}
	

		if (pwmFinalR >= 0)
		{
		  digitalWrite(DIRRB, LOW);
		  digitalWrite(DIRRA, HIGH);
		}
		else
		{
		  digitalWrite(DIRRA, LOW);
		  digitalWrite(DIRRB, HIGH);
		  pwmFinalR *= -1; //take absolute value of E if negative
		}

		pwmWrite(OutputPinL, pwmFinalL + deadspace); //signal between 0 and 1023
		pwmWrite(OutputPinR, pwmFinalR + deadspace); //signal between 0 and 1023

		if (flag) //Exit loop if flag triggered by keyboard interrupt
		  break; 

		//maintain timing:
		while(1)
		{
		  time_difference = nanos() - start_time;
		  if (time_difference < 0)
		    time_difference += 1000000000;	      //nanos() rolls over back to zero every 1E9 nanoseconds (1 second)
	    	
		  if (time_difference > period*1000000000)  //loop time for nS
		  {
		    count = 0;
		    break;
		  }
		  else
		  //if none of the above conditions met, increment counter and continue to loop:
		  count++;
		}
	}

	//on loop exit, kill PWM before ending program
	digitalWrite(DIRLA, LOW);
	digitalWrite(DIRLB, LOW);
	digitalWrite(DIRRA, LOW);
	digitalWrite(DIRRB, LOW);
	pwmWrite(OutputPinL, 0);
	pwmWrite(OutputPinR, 0);
return 0;
}
