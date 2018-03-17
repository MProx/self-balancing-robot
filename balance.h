/*
Raspberry Pi pin numbering reference:
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
double setpoint_angle = 1.05, setpoint_pos = 0, setpoint_turn = 0;
double Ei_angle = 0, Ei_pos = 0, Ei_turn;
double drive_raw, drive, turnSig;

//for encoders:
int EncoderLPinA = 26, EncoderLPinB = 19, EncoderRPinA = 20, EncoderRPinB = 16;
int cycleCounter = 0, velCountL = 0, velCountR = 0;
int posL, posR, pos, posLold, posRold;
double velocityL, velocityR, pos_turn, velocityOld, velocity;

//for i2c:
int fd; //for i2c setup and ID
int acclX, acclY, acclZ, gyroX, gyroY, gyroZ;
double acclY_scaled, acclZ_scaled, gyroX_scaled;
double theta, theta_acc, dTheta, dTheta_s;
double gyro_calib; //to be done

//for PWM:
int flag = 0, pwmFinalL, pwmFinalR;
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
