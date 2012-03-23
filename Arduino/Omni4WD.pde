#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include "RobotKit.h"

/*

              /                \
   wheel1    /                  \   wheel4
   Left     /                    \   Right


            \                    /
   wheel2    \                  /   wheel3
   Left       \                /   Right
    
 */

/*
irqISR(irq1,isr1);
MotorWheel wheel1(5,4,12,13,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(6,7,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,11,18,19,&irq4);
 */
 
#define MESSAGE_END 0x45 //(= 69 in decimal)

const int ledPin =  13;      // the number of the LED pin
const unsigned int interval_PID=10; //10ms

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);// Motor PWM:Pin3, DIR:Pin2, Encoder A:Pin4, B:Pin5

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);

Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);

void setup() {
	//TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
	TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
	TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
        pinMode(ledPin, OUTPUT);    
        Serial.begin(115200); // initialize serial communication:
	Omni.PIDEnable(0.3,0.01,0,interval_PID);//P was 0.31
        Serial.print("Start of the program\n");
}
/*
This loop do:
- Blink a LED
- Run a PID controller
- Read and send odometry with UART 
- Receive speed command with UART
*/

void loop() {
 
  unsigned long currentMillis = millis();
  static unsigned long previous_millis_odometry = 0, previous_millis_led=0,previous_millis_PID=0;
  unsigned int interval_odometry=20,interval_led=400;
  static data_t data;

  unsigned long time_tic=0,time_toc=0;
//  time_tic=micros();
//  time_toc=micros()-time_tic;
//  Serial.print(" Time measured:");
//  Serial.println(time_toc);
//  Serial.print("   ");  

 //send_uart_to_computer();

  if(Serial.available()>0)//this scope is executed in about 4000us
  {
    receive_message(&data,13);
    unsigned int current_speed=Omni.setCarAnyDirection(data.lin_speed, data.rot_speed, data.orientation);
  }
  
  if(currentMillis - previous_millis_odometry > interval_odometry)
  {
    previous_millis_odometry = currentMillis;  
    read_and_send_odometry();
  }
  
  if(currentMillis-previous_millis_PID > interval_PID)//this scope is executed in about 60us or 930us
  {
    previous_millis_PID = currentMillis;
    Omni.PIDRegulate();
  }
  
   if(currentMillis - previous_millis_led > interval_led)
  {
    previous_millis_led = currentMillis;  
    blinkLED();
  } 
  
 // polygone(1500,2000,200,1);// square with spinning
 //Omni.delayMS(50,false);//wait
//  Omni.demoActions(100,1000,500,false);
    
}

//This function is executed in about 1000us without any print
void read_and_send_odometry(void)
{
  int odom_wheel[4];
  static long int previous_time=0;
  long int delta_t=0;
  if(previous_time==0)
  {
    delta_t=20;//first call, so 20ms after the start of the program
    previous_time=millis();
  }
  else
  {
   delta_t=millis()-previous_time;
   previous_time=millis(); 
  }
  odom_wheel[0]=Omni.wheelULGetSignedSpeedMMPS()*delta_t;
  odom_wheel[1]=Omni.wheelLLGetSignedSpeedMMPS()*delta_t;
  odom_wheel[2]=Omni.wheelLRGetSignedSpeedMMPS()*delta_t;
  odom_wheel[3]=Omni.wheelURGetSignedSpeedMMPS()*delta_t;
  
  send_message(odom_wheel);
//  Serial.print("New");
//  Serial.println(odom_wheel[0]);
//  Serial.println(odom_wheel[1]);
//  Serial.println(odom_wheel[2]);
//  Serial.println(odom_wheel[3]);
}

int send_message(int message[4])
{
  unsigned int length=9;
  byte buf[length];
 
  for(int i=0;i<4;i++)//fill the buffer with the message
  {
    buf[2*i]=byte(message[i]>>8);
    buf[2*i+1]=byte(message[i]);
  }
  buf[length-1]=MESSAGE_END;
 Serial.write(buf,length);//send the message
 return(1); //TODO_ return 1 if message is delivery, if not 0
}

//This function is executed in about 1050s without any print
int receive_message(data_t* data, int length)
{
  int i=0,j=0,k=0;
  byte message[length]; //raw datas
  
  union u_tag //to transform byte to float
  {
    byte byte_array[4];
    float float_number;
  } lin_speed,rot_speed,orientation;
  
  unsigned long l=0,lmax=100;//avoid infinite loop during waiting for new data

  //store all incoming bytes in message
  message[i]=Serial.read();
  message[length-1]=0;
  //Serial.write(message[i]);
  i++;
  while(i<length && message[length-1] != MESSAGE_END && l<lmax)
  { 
    if(Serial.available()>0)
    {
      message[i]=Serial.read();
      //Serial.write(message[i]);
      i++;
      l=0;
    }
    l++; 
  }

  if(i==length && message[length-1]==MESSAGE_END)  //good message
  {// put each 4bytes in a float    
    for(k=0;k<4;k++)
    {
      lin_speed.byte_array[k] = message[3-k];
      rot_speed.byte_array[k] = message[7-k];
      orientation.byte_array[k] = message[11-k];
    }
    data->lin_speed=lin_speed.float_number;
    data->rot_speed=rot_speed.float_number;
    data->orientation=orientation.float_number;
    
  
//    Serial.print("Message");
//    Serial.println(i);
//    for(k=0;k<i;k++)
//    {
//      Serial.println(message[k],HEX);
//    }
  
  //  Serial.println(data->lin_speed);
  //  Serial.println(data->rot_speed);
  //  Serial.println(data->orientation);
    
    return(1);
  }
  else //wrong message
  {
    Serial.print("Message");
    Serial.println(i);
    if(l==lmax)
      Serial.print("Reception: TimeOut");
    else if(i<length)
      Serial.print("Reception: Not enough bytes");
    else if(message[length-1]!= MESSAGE_END)
      Serial.print("Reception: Too many bytes");
    else
      Serial.print("Reception: Error");    
    return(0);
  }
}

//This function is executed in about 10us
void blinkLED(void)
{
  static int ledState = LOW;             // ledState used to set the LED

  // if the LED is off turn it on and vice-versa:
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;

  // set the LED with the ledState of the variable:
  digitalWrite(ledPin, ledState);  
}

void polygone(unsigned int length, unsigned int driving_speed, int rot_speed, unsigned int nb_sides)
{
  unsigned long int time;
  unsigned int time_step=100;//100ms
  float orientation=0;
  float d_orientation;
  
  time=length; // finally time=(length*10/driving_speed)*100;
  time*=10;
  time/=driving_speed;
  time*=100;
 
  d_orientation = 1.24*(rot_speed*1.0*time_step)/(1000.0*190.0);
  
  for(unsigned j=0;j<nb_sides;j++)
  {
    for(unsigned int i=0;i<time;i=i+time_step)
    {
      Omni.setCarAnyDirection(driving_speed, rot_speed, orientation);
      Omni.delayMS(time_step,false);//wait
      orientation=orientation-d_orientation;
    }
    orientation+=2*PI/nb_sides;
  }
  Omni.setMotorAllStop();
}

void send_uart_to_computer(void)
{
   while(Serial.available()>0)
 {
   Serial.write(Serial.read());
 }
}
