#include<MotorWheel.h>

Motor::Motor(unsigned char _pinPWM,unsigned char _pinDir,
				unsigned char _pinIRQ,unsigned char _pinIRQB,
				struct ISRVars* _isr)
		:PID(&speedRPMInput,&speedRPMOutput,&speedRPMDesired,KC,TAUI,TAUD),
		 pinPWM(_pinPWM),pinDir(_pinDir),pinIRQ(_pinIRQ),pinIRQB(_pinIRQB),isr(_isr) {
	debug();

/*for maple*/	
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	pinMode(pinPWM,PWM);
	pinMode(pinDir,OUTPUT);
	pinMode(pinIRQ,INPUT_FLOATING);

/*for arduino*/	
#else
	pinMode(pinPWM,OUTPUT);
	pinMode(pinDir,OUTPUT);
	pinMode(pinIRQ,INPUT);
	
#endif

	if(pinIRQB!=PIN_UNDEFINED) {
		pinMode(pinIRQB,INPUT);
		isr->pinIRQB=pinIRQB;
	} else isr->pinIRQB=PIN_UNDEFINED;
//	if(_pinIRQ==2 || _pinIRQ==3) attachInterrupt(pinIRQ-2,isr->ISRfunc,RISING);
//	else {
//		PCattachInterrupt(pinIRQ,isr->ISRfunc,RISING);	// it should be in setup()?
//	}
	PIDDisable();
}

void Motor::setupInterrupt() {
/*for maple*/
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	attachInterrupt(pinIRQ,isr->ISRfunc,RISING);	// it should be in setup()?

/*for arduino*/
#else
	if(pinIRQ==2 || pinIRQ==3) attachInterrupt(pinIRQ-2,isr->ISRfunc,RISING);
	else {
		PCattachInterrupt(pinIRQ,isr->ISRfunc,RISING);	// it should be in setup()?
	}
	
#endif
}

unsigned char Motor::getPinPWM() const {
	debug();
	return pinPWM;
}
unsigned char Motor::getPinDir() const {
	debug();
	return pinDir;
}
unsigned char Motor::getPinIRQ() const {
	debug();
	return pinIRQ;
}
unsigned char Motor::getPinIRQB() const {
	debug();
	return pinIRQB;
}

unsigned int Motor::runPWM(unsigned int PWM,bool dir,bool saveDir) {
	debug();
	speedPWM=PWM;
	if(saveDir) desiredDirection=dir;
	analogWrite(pinPWM,PWM);
	digitalWrite(pinDir,dir);
	return PWM;
}
unsigned int Motor::advancePWM(unsigned int PWM) {
	debug();
	return runPWM(PWM,DIR_ADVANCE);
}
unsigned int Motor::backoffPWM(unsigned int PWM) {
	debug();
	return runPWM(PWM,DIR_BACKOFF);
}
unsigned int Motor::getPWM() const {
	debug();
	return speedPWM;
}
bool Motor::setDesiredDir(bool dir) {
	debug();
	//runPWM(getPWM(),dir);	// error
	desiredDirection=dir;
	return getDesiredDir();
}
bool Motor::getDesiredDir() const {
	debug();
	return desiredDirection;
}
bool Motor::reverseDesiredDir() {
	debug();
	runPWM(getPWM(),!getDesiredDir());
	return getDesiredDir();
}

bool Motor::getCurrDir() const {
	return isr->currDirection;
}
bool Motor::setCurrDir() {
	if(getPinIRQB()!=PIN_UNDEFINED)
		return isr->currDirection=digitalRead(pinIRQB);
	return false;
}


unsigned int Motor::getSpeedRPM() const {
	debug();
	return SPEEDPPS2SPEEDRPM(isr->speedPPS);
}
unsigned int Motor::setSpeedRPM(int speedRPM,bool dir) {
	debug();
	PIDSetSpeedRPMDesired(speedRPM);
	setDesiredDir(dir);
	return getSpeedRPM();
}	



bool Motor::PIDGetStatus() const {
	debug();
	return pidCtrl;
}
bool Motor::PIDEnable(float kc,float taui,float taud,unsigned int sampleTime) {
	debug();
	setupInterrupt();
	PIDSetup(kc,taui,taud,sampleTime);
	return pidCtrl=true;
}
bool Motor::PIDDisable() {
	debug();

	return pidCtrl=false;
}
bool Motor::PIDReset() {
	debug();
	if(PIDGetStatus()==false) return false;
	PID::Reset();
	return true;
}

bool Motor::PIDSetup(float kc,float taui,float taud,unsigned int sampleTime) {
	debug();
	PID::SetTunings(kc,taui,taud);
	PID::SetInputLimits(0,MAX_SPEEDRPM);
	PID::SetOutputLimits(0,MAX_SPEEDRPM);
	PID::SetSampleTime(sampleTime);
	PID::SetMode(AUTO);
	return true;
}
unsigned int Motor::PIDSetSpeedRPMDesired(unsigned int speedRPM) {
	debug();
	if(speedRPM>MAX_SPEEDRPM) speedRPMDesired=MAX_SPEEDRPM;
	else speedRPMDesired=speedRPM;
	return PIDGetSpeedRPMDesired();
}
unsigned int Motor::PIDGetSpeedRPMDesired() const {
	debug();
	return speedRPMDesired;
}

bool Motor::PIDRegulate(bool doRegulate) {
	debug();
	if(PIDGetStatus()==false) return false;
	if(getPinIRQB()!=PIN_UNDEFINED && getDesiredDir()!=getCurrDir()) {
		speedRPMInput=-SPEEDPPS2SPEEDRPM(isr->speedPPS);
	} else {
		speedRPMInput=SPEEDPPS2SPEEDRPM(isr->speedPPS);
	}

	PID::Compute();
	if(doRegulate && PID::JustCalculated()) {
		speed2DutyCycle+=speedRPMOutput;

		if(speed2DutyCycle>=MAX_SPEEDRPM) speed2DutyCycle=MAX_SPEEDRPM;
		else if(speed2DutyCycle<=-MAX_SPEEDRPM)  speed2DutyCycle=-MAX_SPEEDRPM;
		if(speed2DutyCycle>=0) {
			runPWM(map(speed2DutyCycle,0,MAX_SPEEDRPM,0,MAX_PWM),getDesiredDir(),false);
		} else {
			runPWM(map(abs(speed2DutyCycle),0,MAX_SPEEDRPM,0,MAX_PWM),!getDesiredDir(),false);
		}
		return true;
	}
	return false;
}
/*
bool Motor::PIDRegulate(bool doRegulate) {
	debug();
	if(PIDGetStatus()==false) return false;

	speedPRMInput=SPEEDPPS2SPEEDRPM(isr->speedPPS);

	PID::Compute();
	if(doRegulate && PID::JustCalculated()) {
		if(speed2DutyCycle<-(MAX_SPEEDRPM>>3) && speedRPMOutput<-(MAX_SPEEDRPM>>3))
			speed2DutyCycle-=speedPRMOutput;
		else speed2DutyCycle+=speedRPMOutput;

		if(speed2DutyCycle>=MAX_SPEEDRPM) speed2DutyCycle=MAX_SPEEDRPM;
		else if(speed2DutyCycle<=-MAX_SPEEDRPM)  speed2DutyCycle=-MAX_SPEEDRPM;
		if(speed2DutyCycle>=0) {
			runPWM(map(speed2DutyCycle,0,MAX_SPEEDRPM,0,MAX_PWM),getDesiredDir());
		} else {
			runPWM(map(abs(speed2DutyCycle),0,MAX_SPEEDRPM,0,MAX_PWM),!getDesiredDir(),false);
		}
		return true;
	}
	return false;
}
 */
void Motor::simpleRegulate() {	// to be improved
	if(getDesiredDir()==DIR_ADVANCE) speedRPMInput=SPEEDPPS2SPEEDRPM(isr->speedPPS);
	else speedRPMInput=-SPEEDPPS2SPEEDRPM(isr->speedPPS);
	long speedDelta=speedRPMInput-speedRPMDesired;
	runPWM(50+map(speedDelta,-5000,5000,50,-50),getDesiredDir());
}
/*
void Motor::irqISR() {	// ISR must be void func(void);
	debug();
	static bool first_pulse=true;
	pulseEndMicros=micros();
	if(first_pulse==false && pulseEndMicros>pulseStartMicros) {
			speedPPS=MICROS_PER_SEC/(pulseEndMicros-pulseStartMicros);
	} else {
		first_pulse=false;
	}
	pulseStartMicros=pulseEndMicros;
}
 */
int Motor::getSpeedPPS() const {
	return isr->speedPPS;
}
long Motor::getCurrPulse() const {
	return isr->pulses;
}
long Motor::setCurrPulse(long _pulse) {
	isr->pulses=_pulse;
	return getCurrPulse();
}
long Motor::resetCurrPulse() {
	return setCurrPulse(0);
}

// to reduce the binary for Atmega168 (16KB), Serial output is disabled
void Motor::debugger() const {	
	debug()

#ifdef DEBUG
	if(!Serial.available()) Serial.begin(Baudrate);
/*
	Serial.print("pinPWM -> ");
	Serial.println(pinPWM,DEC);
	Serial.print("pinDir -> ");
	Serial.println(pinDir,DEC);
	Serial.print("pinIRQ -> ");
	Serial.println(pinIRQ,DEC);
	Serial.print("pinIRQB-> ");
	Serial.println(pinIRQB,DEC);
 */

	Serial.print("DesiredDir -> ");
	Serial.println(desiredDirection);
	Serial.print("currDir ->");
	Serial.println(isr->currDirection);

	Serial.print("PWM    -> ");
	Serial.println(speedPWM,DEC);
	Serial.print("Input  -> ");
	Serial.println(speedRPMInput,DEC);
	Serial.print("Output -> ");
	Serial.println(speedRPMOutput,DEC);
	Serial.print("Desired-> ");
	Serial.println(speedRPMDesired,DEC);
 
/*
	Serial.print("speed2DutyCycle-> ");
	Serial.println(speed2DutyCycle);
	Serial.print("speedPPS> ");
	Serial.println(isr->speedPPS,DEC);
	Serial.print("pulses -> ");
	Serial.println(isr->pulses,DEC);
 */

#endif

}


GearedMotor::GearedMotor(unsigned char _pinPWM,unsigned char _pinDir,
			unsigned char _pinIRQ,unsigned char _pinIRQB,
			struct ISRVars* _isr,unsigned int ratio):
				Motor(_pinPWM,_pinDir,_pinIRQ,_pinIRQB,_isr),_ratio(ratio) {
	debug();
}
unsigned int GearedMotor::getRatio() const {
	return _ratio;
}
unsigned int GearedMotor::setRatio(unsigned int ratio) {
	_ratio=ratio;
	return getRatio();
}
float GearedMotor::getGearedSpeedRPM() const {
	debug();
	//return (float)Motor::getSpeedRPM()/REDUCTION_RATIO;
	return (float)Motor::getSpeedRPM()/_ratio;
}
float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM,bool dir) {
	debug();
	//Motor::setSpeedRPM(abs(gearedSpeedRPM*REDUCTION_RATIO),dir);
	Motor::setSpeedRPM(abs(gearedSpeedRPM*_ratio),dir);
	return gearedSpeedRPM;
}


MotorWheel::MotorWheel(unsigned char _pinPWM,unsigned char _pinDir,
						unsigned char _pinIRQ,unsigned char _pinIRQB,
						struct ISRVars* _isr,
						unsigned int ratio,unsigned int cirMM):
						GearedMotor(_pinPWM,_pinDir,_pinIRQ,_pinIRQB,_isr,ratio),_cirMM(cirMM) {
	debug();

}
unsigned int MotorWheel::getCirMM() const {
	return _cirMM;
}
unsigned int MotorWheel::setCirMM(unsigned int cirMM) {
	if(cirMM>0) _cirMM=cirMM;
	return getCirMM();
}
unsigned int MotorWheel::getSpeedCMPM() const {
	debug();
	//return int(GearedMotor::getGearedSpeedRPM()*CIR);
	return int(GearedMotor::getGearedSpeedRPM()*getCirMM()/10);
}
unsigned int MotorWheel::setSpeedCMPM(unsigned int cm,bool dir) {
	debug();
	//GearedMotor::setGearedSpeedRPM(cm/CIR,dir);
	GearedMotor::setGearedSpeedRPM(cm*10.0/getCirMM(),dir);
	return getSpeedCMPM();
}
unsigned int MotorWheel::getSpeedMMPS() const {
	debug();
	return int(getSpeedCMPM()/6);
}
unsigned int MotorWheel::setSpeedMMPS(unsigned int mm,bool dir) {
	debug();
	setSpeedCMPM(mm*6,dir);
	return getSpeedMMPS();
}
int MotorWheel::getSignedSpeedMMPS() const {
	int speed=0;
	debug();
	if(Motor::getCurrDir()==DIR_BACKOFF)
	{
	  speed=-getSpeedMMPS();
	}
	else // DIR_ADVANCE
	{
	  speed=getSpeedMMPS();
	}
	return speed;
}

/*
unsigned int MotorWheel::runTime(unsigned int speedMMPS,bool dir,unsigned int TimeMS) {
	setSpeedMMPS(speedMMPS,dir);
	for(int i=0;i<TimeMS;i+=10) {
		PIDRegulate();
	}
	return TimeMS;
}
unsigned int MotorWheel::runDistance(unsigned int speedMMPS,bool dir,unsigned int distanceMM) {
	unsigned int timeMS=(distanceMM*1000)/speedMMPS;
	return runTime(speedMMPS,dir,timeMS);
}
 */





/*
servoMotor::servoMotor(unsigned char _pinPWM,unsigned char _pinDir,
					unsigned char _pinIRQ,unsigned char _pinIRQB,
					struct ISRVars* _isr) 
		:Motor(_pinPWM,_pinDir,_pinIRQ,_pinIRQB,_isr),
		samePID(&pulseInput,&pulseOutput,&pulseDesired,KC,TAUI,TAUD) {
	debug();
	SPIDSetup(KC,TAUI,TAUD,10);
}
int servoMotor::SPIDSetPulseBoundary(int min,int max) {
	debug();
	samePID::SetInputLimits(min,max);
	samePID::SetOutputLimits(min,max);
	return samePID::GetINMax();
}
bool servoMotor::SPIDSetup(float kc,float taui,float taud,unsigned int sampleTime) {
	debug();
	samePID::SetTunings(kc,taui,taud);
	samePID::SetSampleTime(10);
	samePID::SetMode(AUTO);
	return true;
}

unsigned char servoMotor::getSPIDMode() const {
	debug();
	return SPIDMode;
}
unsigned char servoMotor::setSPIDMode(unsigned char _mode) {
	debug();
	if(SPIDMODE_UNKNOWN<=_mode && _mode<=SPIDMODE_PULSE) SPIDMode=_mode;
	else SPIDMode=SPIDMODE_UNKNOWN;
	if(getSPIDMode()==SPIDMODE_STOP) setPulseDesired(getCurrPulse());
	return getSPIDMode();
}

int servoMotor::getPulseDesired() const {
	debug();
	return pulseDesired;
}
int servoMotor::setPulseDesired(int _pulse) {
	debug();
	if(samePID::GetINMin()<=_pulse && _pulse<=samePID::GetINMax()) pulseDesired=_pulse;
	else if(samePID::GetINMin()>_pulse) pulseDesired=samePID::GetINMin();
	else pulseDesired=samePID::GetINMax();
	return getPulseDesired();
}
int servoMotor::incPulseDesired() {
	return setPulseDesired(getPulseDesired()+1);
}
int servoMotor::decPulseDesired() {
	return setPulseDesired(getPulseDesired()-1);
}

bool servoMotor::PIDRegulate(bool doRegulate) {	// 201104
	debug();
	if(PIDGetStatus()==false) return false;

	switch(getSPIDMode()) {
		case SPIDMODE_SPEED:
			SPIDRegulateSpeed(doRegulate);break;
		case SPIDMODE_STOP:
			SPIDRegulateStop(doRegulate);break;
		case SPIDMODE_PULSE: 
			SPIDRegulatePulse(doRegulate);break;
		default:
			SPIDRegulateSpeed(doRegulate);break;
	}
	return Motor::PIDRegulate(doRegulate);
}
bool servoMotor::SPIDRegulateSpeed(bool doRegulate) {
	debug();
	return doRegulate;
}
bool servoMotor::SPIDRegulateStop(bool doRegulate) {
	debug();
	return SPIDRegulatePulse(doRegulate);
}
bool servoMotor::SPIDRegulatePulse(bool doRegulate) {
	debug();
	pulseInput=getCurrPulse();

	samePID::Compute();
	if(doRegulate && samePID::JustCalculated()) {
		pulse2SpeedRPM+=pulseOutput;
		if(pulse2SpeedRPM<=samePID::GetINMin()) pulse2SpeedRPM=samePID::GetINMin();
		else if(pulse2SpeedRPM>=samePID::GetINMax()) pulse2SpeedRPM=samePID::GetINMax();

		setSpeedRPM(
			map(abs(pulse2SpeedRPM),0,samePID::GetINMax(),0,1000),
			pulse2SpeedRPM>=0?DIR_ADVANCE:DIR_BACKOFF
		);
 
	}
	return doRegulate;
}

void servoMotor::debugger() const {
	Serial.print("Input -> ");
	Serial.println(pulseInput);
	Serial.print("Output-> ");
	Serial.println(pulseOutput);
	Serial.print("Desired> ");
	Serial.println(pulseDesired);
	Serial.print("pulse2S> ");
	Serial.println(pulse2SpeedRPM);
}
 */




