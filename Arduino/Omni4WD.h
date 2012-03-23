#include<MotorWheel.h>

#ifndef Omni4WD_H
#define Omni4WD_H

#define ROBOT_RADIUS 200 //Radius of the robot in [mm]
/*
	Mecanum4WD
			  Front MOTORS_FB
	wheelUL	\\		// wheelUR


	wheelLL	//		\\ wheelLR
			  Back MOTORS_BF
 */


/*
	Omni4WD
			  Front MOTORS_FB
	wheelUL	//		\\ wheelUR


	wheelLL	\\		// wheelLR
			  Back MOTORS_BF
 */
class Omni4WD {
public:
	Omni4WD(MotorWheel* wheelUL,MotorWheel* wheelLL,
			MotorWheel* wheelLR,MotorWheel* wheelUR);
	unsigned char switchMotors();
	unsigned char switchMotorsReset();
	
	unsigned int setMotorAll(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int setMotorAllStop();
	unsigned int setMotorAllAdvance(unsigned int speedMMPS=0);
	unsigned int setMotorAllBackoff(unsigned int speedMMPS=0);
	unsigned int setCarStop();
	unsigned int setCarAdvance(unsigned int speedMMPS=0);
	unsigned int setCarBackoff(unsigned int speedMMPS=0);
	unsigned int setCarLeft(unsigned int speedMMPS=0);
	unsigned int setCarRight(unsigned int speedMMPS=0);
	unsigned int setCarRotateLeft(unsigned int speedMMPS=0);
	unsigned int setCarRotateRight(unsigned int speedMMPS=0);
	unsigned int setCarUpperLeft(unsigned int speedMMPS=0);
	unsigned int setCarLowerLeft(unsigned int speedMMPS=0);
	unsigned int setCarUpperRight(unsigned int speedMMPS=0);
	unsigned int setCarLowerRight(unsigned int speedMMPS=0);
	unsigned int setCarAnyDirection(float speed=0, float rotSpeed=0, float direction=0);
	
	unsigned int getCarSpeedMMPS() const;
	unsigned int setCarSpeedMMPS(unsigned int speedMMPS=0,unsigned int ms=1000);
	unsigned int setCarSlow2Stop(unsigned int ms=1000);

	unsigned int wheelULGetSpeedMMPS() const;
	int wheelULGetSignedSpeedMMPS() const;
	unsigned int wheelULSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelLLGetSpeedMMPS() const;
	int wheelLLGetSignedSpeedMMPS() const;
	unsigned int wheelLLSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelURGetSpeedMMPS() const;
	int wheelURGetSignedSpeedMMPS() const;
	unsigned int wheelURSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelLRGetSpeedMMPS() const;
	int wheelLRGetSignedSpeedMMPS() const;
	unsigned int wheelLRSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);

	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int interval=1000);
	bool PIDRegulate();
	void delayMS(unsigned int ms=100, bool debug=false);
	void demoActions(unsigned int speedMMPS=100,unsigned int duration=5000,unsigned int uptime=500,bool debug=false);
	void debugger(bool wheelULDebug=true,bool wheelLLDebug=true,
					bool wheelLRDebug=true,bool wheelURDebug=true) const;

	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_LEFT,
			STAT_RIGHT,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
			STAT_UPPERLEFT,
			STAT_LOWERLEFT,
			STAT_LOWERRIGHT,
			STAT_UPPERRIGHT,
	};
	unsigned char getCarStat() const;

	enum {
		MOTORS_FB,	// FrontBack
		MOTORS_BF,	// BackFront
	};
	unsigned char getSwitchMotorsStat() const;
	
private:
	MotorWheel* _wheelUL;	// UpperLeft
	MotorWheel* _wheelLL;	// LowerLeft
	MotorWheel* _wheelLR;	// LowerRight
	MotorWheel* _wheelUR;	// UpperRight


	unsigned char _carStat;
	unsigned char setCarStat(unsigned char carStat);

	unsigned char _switchMotorsStat;
	unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);

	Omni4WD();	

};

#endif





