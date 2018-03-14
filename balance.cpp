/*
Before compiling: 
- make sure i2c is enabled ("sudo raspi-config")
- make sure wiringPi is installed and configured (test with "gpio i2cdetect")

To compile:
- "g++ balance.cpp -o balance -lwiringPi -lpthread"

To run: 
- "sudo ./balance" - NB: always run with root privileges


TO DO: 
- Assign pin numbers for encoders
- Test interrupt ISR and monitoring
- develop steering control (possibly just adjust turnsig if posA != posB)
- possibly use encoders with external arduino nano to offload interrupts and timing

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

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <signal.h>  //  For keyboard interrupt handling (kill PWM before exit)
#include <stdio.h>
#include <math.h>
#include <time.h>

//variable definitions:
//===============================
//for PID:
double error_angle, error_velocity;
double setpoint_angle = 1.05, setpoint_pos = 0;
double Ei_angle = 0, Ei_pos = 0;
double drive_raw, drive;

//for encoders:
int EncoderLPinA = 26, EncoderLPinB = 19, EncoderRPinA = 20, EncoderRPinB = 16;
int cycleCounter = 0, velCountL = 0, velCountR = 0;
int posL, posR, pos, posLold, posRold;
double velocityL, velocityR, velocityOld, velocity;

//for i2c:
int fd; //for i2c setup and ID
int acclX, acclY, acclZ, gyroX, gyroY, gyroZ;
double acclY_scaled, acclZ_scaled, gyroX_scaled;
double theta, theta_acc, dTheta, dTheta_s;
double gyro_calib; //to be done

//for PWM:
int PWML = 0, PWMR = 0, flag = 0, turnSig = 0;
int DIRLA = 17, DIRLB = 27, DIRRA = 22, DIRRB = 10;
int OutputPinL = 18, OutputPinR = 13;

//for timing:
unsigned long int start_time, time_difference;;
double period = 0.01; //in seconds, 0.01 = 100Hz
unsigned int count = 0;
struct timespec gettime_now;

int deadspace = 200;

//functions:
//===============================

void EncoderL_ISR(void)
{
  if (digitalRead(EncoderLPinA) == digitalRead(EncoderLPinB))
	posL--;
  else
	posL++;
}

void EncoderR_ISR(void)
{
  if (digitalRead(EncoderRPinA) == digitalRead(EncoderRPinB))
	posR++;
  else
	posR--;
}

int read_word_i2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

unsigned int nanos(void)
{
  clock_gettime(CLOCK_REALTIME, &gettime_now);
  return gettime_now.tv_nsec;		//Get ns value
}


void KillFunction(int sig)
{
    flag = 1;  //flag to break loop & kill motors if keyboard interrupt signal is issued
}

double gyroCalibration()
{	
	for (count = 0; count < 200; count++)
	{
		gyroX = read_word_i2c(0x43);
		gyroX_scaled += gyroX / 131.0;
	}
	
	return gyroX_scaled/200;	

	count = 0;
}

double PID(double setpoint, double PV, double dPVdT, double *Ei, double loopPer, double kP, double kI, double kD, double windup_lim)
{
	double error = PV - setpoint;	
	double Ep = error*kP;
	*Ei += error*loopPer*kI; //increment Ei and save to address
	double Ed = dPVdT*kD;
		//prevent integral windup:
		if (*Ei > windup_lim)
		  *Ei = windup_lim;
		else if (*Ei < -windup_lim)
		  *Ei = -windup_lim;
	
	return Ep + *Ei + Ed;
}

void init()
{
	
	//detect keyboard interrupt (program end) - kill PWM
	printf("\tInitializing keyboard interrupt\n");
	signal(SIGINT, KillFunction); 

	//pin mode setup
	printf("\tInitializing pin modes\n");
	wiringPiSetupGpio(); //use GPIO pin numbering
	pinMode(OutputPinL,PWM_OUTPUT);
	pinMode(OutputPinR,PWM_OUTPUT);
	pinMode(EncoderLPinA,INPUT);
	pinMode(EncoderLPinB,INPUT);
	pinMode(EncoderRPinA,INPUT);
	pinMode(EncoderRPinB,INPUT);
	pinMode(DIRLA,OUTPUT);
	pinMode(DIRRA,OUTPUT);
	pinMode(DIRLB,OUTPUT);
	pinMode(DIRRB,OUTPUT);
	
	//enable pull-ups on encoders:
	printf("\tInitializing encoder pull-up resistors\n");
	pullUpDnControl(EncoderLPinA, PUD_UP);
	pullUpDnControl(EncoderLPinB, PUD_UP);
	pullUpDnControl(EncoderRPinA, PUD_UP);
	pullUpDnControl(EncoderRPinB, PUD_UP);

	//set up interrupts for encoders:
	printf("\tInitializing encoder interrupt\n");
 	wiringPiISR(EncoderLPinA, INT_EDGE_BOTH,  EncoderL_ISR);
 	wiringPiISR(EncoderRPinA, INT_EDGE_BOTH,  EncoderR_ISR);
	
	//initiate i2c and configure MPU6050			
	printf("\tInitializing accelerometer/gyro\n");
	fd = wiringPiI2CSetup (0x68);
	wiringPiI2CWriteReg8 (fd,0x6B,0x00);//disable sleep mode 

	printf("\tCalibrating gyro:\n");
	gyro_calib = gyroCalibration();

	printf("\nInitializing complete\n");
}

//main function:
//===============================
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
		dTheta = -1*gyroX_scaled + gyro_calib;

		theta = 0.99*(theta - dTheta*period) + 0.01*theta_acc; //dTheta term negative due to axes directions

//		printf("%.8f %.8f %.8f\n", theta, theta_acc, dTheta);

		dTheta_s = dTheta_s*0.9 + dTheta*0.1;

		//velocity control mode:		
		//Read wifi for steering/velocity input (not implemented yet):
//		setpoint_pos = 0;

//		turnSig = 0;

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
				velocity = 0.8*velocity + 0.2*(velocityL + velocityR)/2;
				pos = (posL + posR)/2;
	
				//PID (outer) - takes user input and outputs reguired angle:
				if (abs(pos - setpoint_pos) > 300)
					setpoint_angle = PID(setpoint_pos, pos, velocity, &Ei_pos, 5*period, 0.0030, 0.0040, 0.0026, 15.0);
				else
					setpoint_angle = PID(setpoint_pos, pos, velocity, &Ei_pos, 5*period, 0.0025, 0.0040, 0.0022, 15.0);
			}

		//PID (inner) - takes desired angle and outputs drive PWM signal:
		if (abs(theta - setpoint_angle) > 3)
			drive = PID(setpoint_angle, theta, dTheta_s, &Ei_angle, period, 60.0, 3500.0, 5.0, 900.0);
		else
			drive = PID(setpoint_angle, theta, dTheta_s, &Ei_angle, period, 130.0, 2500.0, 3.0, 900.0);

		PWML = int(drive + turnSig);
		PWMR = int(drive - turnSig);

		//constrain drive PWM to be between 0 and 1023
		if (PWML > 1023)
		  PWML = 1023;
		else if (PWML < -1023)
		  PWML = -1023;
		if (PWMR > 1023)
		  PWMR = 1023;
		else if (PWMR < -1023)
		  PWMR = -1023;

		//Set direction pins on motor driver:	
		if (PWML >= 0)
		{
		  digitalWrite(DIRLB, LOW);
		  digitalWrite(DIRLA, HIGH);
		}
		else
		{
		  digitalWrite(DIRLA, LOW);
		  digitalWrite(DIRLB, HIGH);
		  PWML *= -1; //take absolute value of E if negative
		}
	

		if (PWMR >= 0)
		{
		  digitalWrite(DIRRB, LOW);
		  digitalWrite(DIRRA, HIGH);
		}
		else
		{
		  digitalWrite(DIRRA, LOW);
		  digitalWrite(DIRRB, HIGH);
		  PWMR *= -1; //take absolute value of E if negative
		}

		pwmWrite(OutputPinL, PWML + deadspace); //signal between 0 and 1023
		pwmWrite(OutputPinR, PWMR + deadspace); //signal between 0 and 1023

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
//		    if (count == 0)
//		    printf("CAUTION: looptime too fast\n\n");    
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
