
/*
  motorwheel library version 1.1,compatible with maple.
*/


/*for maple*/
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	#include "wirish.h" 
	#include "./../PID_Beta6/PID_Beta6.h"
	#include "ext_interrupts.h"

/*for arduino*/	
#else
	#include<WProgram.h>
	#include<PID_Beta6.h>
	#include<PinChangeInt.h>
	
#endif

/*

	class PID
	class Motor
	class GearedMotor
	class Wheel

 */

#ifndef MotorWheel_H
#define MotorWheel_H

#define DIR_ADVANCE HIGH
#define DIR_BACKOFF LOW

#define  PIN_UNDEFINED 255
#define  REF_VOLT 12
/*for maple*/
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	#define  MAX_PWM  3599

/*for arduino*/	
#else
	#define  MAX_PWM  200	// 255
	
#endif

#ifndef PI
#define PI 3.1416
#endif

#define  MAX_SPEEDRPM 7000
#define  REDUCTION_RATIO 64
#define MAX_SPEEDMMPS ( (MAX_SPEEDRPM)/(REDUCTION_RATIO) * 5*PI/3 )
#define  DESIRED_SPEEDRPM 3000
#define  SPEED_CONSTANT_RPM 713
#define  CPR 12
#define  PWM2SPEED_RESOLUTION 143 //713 * 0.2

#define  SEC_PER_MIN 60
#define  MICROS_PER_SEC 1000000
//#define  SPEEDPPS2SPEEDRPM(freq) ((freq)*((SEC_PER_MIN)/(CPR))) //(freq*SEC_PER_MIN/CPR)
#define  SPEEDPPS2SPEEDRPM(freq) ((unsigned long)(freq)*(SEC_PER_MIN)/(CPR)) //(freq*SEC_PER_MIN/CPR)
#define  SPEEDPPS2SPEED2VOLT(spd) ((spd)/SPEED_CONSTANT_RPM)
#define  MAX_SPEEDPPS ((MAX_SPEEDRPM*CPR)/SEC_PER_MIN)
#define  MIN_SPEEDPPS 0

#define  KC           0.26
#define  TAUI         0.01
#define  TAUD         0.00

#define Baudrate	19200
/*
#ifndef DEBUG
#define DEBUG
#endif
 */
#ifdef DEBUG
#define debug() { \
	if(!Serial.available()) Serial.begin(Baudrate); \
	Serial.println(__func__);}
#else
#define debug() {}
#endif


#define irqISR(y,x) \
    void x(); \
    struct ISRVars y={x}; \
    void x() { \
        static bool first_pulse=true; \
		/* ++y.pulses; */ \
        y.pulseEndMicros=micros(); \
        if(first_pulse==false && y.pulseEndMicros>y.pulseStartMicros) { \
             y.speedPPS=MICROS_PER_SEC/(y.pulseEndMicros-y.pulseStartMicros); \
        } else \
            first_pulse=false; \
        y.pulseStartMicros=y.pulseEndMicros; \
		if(y.pinIRQB!=PIN_UNDEFINED) y.currDirection=digitalRead(y.pinIRQB); \
		y.currDirection==DIR_ADVANCE?++y.pulses:--y.pulses; \
    } 

struct ISRVars {
	void (*ISRfunc)();
	//volatile unsigned long pulses;
	volatile long pulses;	// 201104, direction sensitive
	volatile unsigned long pulseStartMicros;
	volatile unsigned long pulseEndMicros;
	volatile unsigned int  speedPPS;
	volatile bool currDirection;
	unsigned char pinIRQB;
};



class Motor: public PID {
public:
	Motor(unsigned char _pinPWM,unsigned char _pinDir,
			unsigned char _pinIRQ,unsigned char _pinIRQB,
			struct ISRVars* _isr);
	
	void setupInterrupt();

	unsigned char getPinPWM() const;
	unsigned char getPinDir() const;
	unsigned char getPinIRQ() const;
	unsigned char getPinIRQB() const;
	
	unsigned int runPWM(unsigned int PWM,bool dir,bool saveDir=true);
	unsigned int getPWM() const;
	unsigned int advancePWM(unsigned int PWM);
	unsigned int backoffPWM(unsigned int PWM);

	bool setDesiredDir(bool dir);
	bool getDesiredDir() const;
	bool reverseDesiredDir();

	bool setCurrDir();
	bool getCurrDir() const;

	unsigned int getSpeedRPM() const;
	unsigned int setSpeedRPM(int speedRPM,bool dir);
	
	void simpleRegulate();

	bool PIDSetup(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int sampleTime=1000);
	bool PIDGetStatus() const;
	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int sampleTime=1000);
	bool PIDDisable();
	bool PIDReset();
	bool PIDRegulate(bool doRegulate=true);
	unsigned int PIDSetSpeedRPMDesired(unsigned int speedRPM);
	unsigned int PIDGetSpeedRPMDesired() const;

	void debugger() const;

	int getSpeedPPS() const;
	long getCurrPulse() const;
	long setCurrPulse(long _pulse);
	long resetCurrPulse();
	
	struct ISRVars* isr;

private:
	unsigned char pinPWM;
	unsigned char pinDir;
	unsigned char pinIRQ;
	unsigned char pinIRQB;

	//bool currDirection;		// current direction
	bool desiredDirection;	// desired direction
	unsigned int speedPWM;	// current PWM

	int speedRPMInput;		// RPM: Round Per Second
	int speedRPMOutput;		// RPM
	int speedRPMDesired;	// RPM
	//float PWMEC;
	float speed2DutyCycle;
/*
	// the followings are defined in struct ISRvars, 
	// because ISR must be a global function, without parameters and no return value

	volatile unsigned int speedPPS;	// PPS: Pulses Per Second
	volatile unsigned long pulseStartMicros;
	volatile unsigned long pulseEndMicros;
 */
	bool pidCtrl;

	Motor();

};



class GearedMotor: public Motor {	// RPM
public:
	GearedMotor(unsigned char _pinPWM,unsigned char _pinDir,
					unsigned char _pinIRQ,unsigned char _pinIRQB,
					struct ISRVars* _isr,
					unsigned int _ratio=REDUCTION_RATIO);
	float getGearedSpeedRPM() const;
	float setGearedSpeedRPM(float gearedSpeedRPM,bool dir);
	unsigned int getRatio() const;
	unsigned int setRatio(unsigned int ratio=REDUCTION_RATIO);
private:
	unsigned int _ratio;
};


#ifndef PI
#define PI 3.1416
#endif
//#define CIR 31.4	// cm
#define CIRMM 314	// mm
class MotorWheel: public GearedMotor {	// 
public:
	MotorWheel(unsigned char _pinPWM,unsigned char _pinDir,
				unsigned char _pinIRQ,unsigned char _pinIRQB,
				struct ISRVars* _isr,
				unsigned int ratio=REDUCTION_RATIO,unsigned int cirMM=CIRMM);

	unsigned int getCirMM() const;
	unsigned int setCirMM(unsigned int cirMM=CIRMM);

	unsigned int getSpeedCMPM() const;	// cm/min
	unsigned int setSpeedCMPM(unsigned int cm,bool dir);
	unsigned int getSpeedMMPS() const; // mm/s
	unsigned int setSpeedMMPS(unsigned int mm,bool dir);
	int getSignedSpeedMMPS() const; // mm/s with the direction

	//unsigned int runTime(unsigned int speedMMPS,bool dir,unsigned int TimeMS);
	//unsigned int runDistance(unsigned int speedMMPS,bool dir,unsigned int distanceMM);

private:
	unsigned int _cirMM;
};

/*
class samePID: public PID {
public:
	samePID(int* input,int* output,int* setPoint,float kc,float taui,float taud)
	:PID(input,output,setPoint,kc,taui,taud) { }
};

class servoMotor: public Motor,public samePID {
public:
	servoMotor(unsigned char _pinPWM,unsigned char _pinDir,
			unsigned char _pinIRQ,unsigned char _pinIRQB,
			struct ISRVars* _isr);

	int SPIDSetPulseBoundary(int min,int max);
	bool SPIDSetup(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int sampleTime=10);
	bool PIDRegulate(bool doRegulate=true);

	enum {
		SPIDMODE_UNKNOWN,
		SPIDMODE_SPEED,
		SPIDMODE_STOP,
		SPIDMODE_PULSE,
	};
	unsigned char getSPIDMode() const;
	unsigned char setSPIDMode(unsigned char _mode);

	int getPulseDesired() const;
	int setPulseDesired(int _pulse);
	int incPulseDesired();
	int decPulseDesired();

	bool backToZero();
	bool scanZero();

	void debugger() const;

private:
	int pulseInput;		//
	int pulseOutput;	//
	int pulseDesired;	//

	int pulse2SpeedRPM;

	unsigned char SPIDMode;

	bool SPIDRegulateSpeed(bool doRegulate=true);	// 201104
	bool SPIDRegulateStop(bool doRegulate=true);
	bool SPIDRegulatePulse(bool doRegulate=true);

};
 */



#endif

