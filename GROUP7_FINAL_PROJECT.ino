/*

FINAL PROJECT: Automatic Food Conveyor System
CODED BY: Czarlean Grace Najito and Marjorie Dublois

The goal of this program is to deliver the food to the 
pickup point without having contact to the costumer.

INSTRUCTIONS:
- Upon clicking the Start simulation button the bulb will turn
turn on indicating that the system is on.

- Pressing the power button in IR remote control will activate 
the conveyor(dc) and other components. The previous temp reading
will also be seen in the lcd upon clicking the power button. The
green LED will also turn on to indicate that the circuit is 
running.

-Adjusting the ultrasonic sensor is to indicate how far is the
order from the pick up point. If it is >100 the blink rate of the
bulb is much faster and it indicates that the order is near the 
pickup point.

-By clicking the power button again, the updated value 
from ultrasonic sensor and temperature sensor are being printed 
in the serial monitor.

-Clicking the arrow up button the servo will move 90 degrees to 
put the order  on the drive thru pick up point.

-Clicking the arrow down button will move the servo in
180 degrees to put the order on the pick up point inside.

-By clicking the func/stop button in the IR remote the system 
will turn off and the red LED will turn on to indicate that
the system is off.

*/


#include <LiquidCrystal_I2C.h>
#include<Wire.h>
#include<Servo.h>
#include<IRremote.h>
LiquidCrystal_I2C lcd(32,16,2);


//setting up pins
#define sensorPin A0
#define oneBulb 4
#define dcMotor 2
#define echoPin 5 
#define trigPin 3 
int gLED=7;
int rLED=12;

//Variables for duration and the distance
long duration;
int distance;

//delays
int t1=500;
int t3=2000;

//Servo
#define servo1Pin 6
#define servo2Pin 9
Servo NServo;
Servo MServo;

//Servo Variables

int angleL=90;
int angleR=20;

//bulb variables
unsigned long fromStartms;
unsigned long prevBulbms=0;
unsigned long Bulb_interval=1000;

int Bulbstate;  

//IR
int IRpin=10;
IRrecv IR(IRpin);


void setup()
{
// ultra sonic  
pinMode(trigPin,OUTPUT);
pinMode(echoPin,INPUT); 
//Servo
NServo.attach(servo1Pin);
MServo.attach(servo2Pin);  
//LCD
lcd.begin(16,2);
lcd.init();
lcd.backlight();
Serial.begin(9600);

//bulb variables
pinMode(oneBulb,OUTPUT);

//dcmotor
pinMode(dcMotor,OUTPUT);  

//IR
pinMode(IRpin, INPUT);
IR.enableIRIn();  

//LED
pinMode(gLED,OUTPUT);  
pinMode(rLED,OUTPUT);  
}

void loop()
{//loop
conditions();//Conditions    
while(IR.decode())
{//while IR
if(IR.decodedIRData.decodedRawData==0xFF00BF00){//if decoded power
digitalWrite(gLED,HIGH);
digitalWrite(dcMotor,HIGH); 
ultrasonic();//VOID ULTRASONIC
temp();  //VOID TEMP  

}//IF decoded power 
IR.resume(); 
  
 if(IR.decodedIRData.decodedRawData==0xF708BF00)
 {
//Serial.println("ARROW DOWN");
NServo.write(180),MServo.write(0);
delay(t1);
NServo.write(90),MServo.write(90); 
 }
  
if(IR.decodedIRData.decodedRawData==0xF50ABF00)
 {
//Serial.println("ARROW UP");
NServo.write(90),MServo.write(0);
delay(t1);              
NServo.write(0),MServo.write(90); // move the servo to desired angle        
 }  

if(IR.decodedIRData.decodedRawData==0xFD02BF00)
{
lcd.clear();
digitalWrite(dcMotor,LOW);
digitalWrite(rLED,HIGH);
digitalWrite(gLED,LOW);
digitalWrite(oneBulb,LOW);  

//Serial.println("FUNC/STOP");
} 
  
  
}//while IR   
delay(t1);
}//loop

void ultrasonic()
{
Serial.print("Distance = ");  
Serial.println(distance);
Serial.println();    
digitalWrite(trigPin, LOW); 
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance= (duration*0.034)/2; 
} //end VOID ULTRASONIC

void temp()
{  
int reading = analogRead(sensorPin);
float voltage = reading * 4.68;
  voltage /= 1024.0;
    

  float temperatureC = (voltage - 0.5) * 100;
Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" C");
  Serial.println();
  lcd.setCursor(0,0);
  lcd.print("MACHINE Operatnl");
  lcd.setCursor(0,1);
  lcd.print("Rm Temp=");
  lcd.print(temperatureC);
  delay(t1);
 
}//END OF VOID TEMP

void conditions()
{
fromStartms = millis();  

if (distance < 100){//if ultrasonic  
if(fromStartms - prevBulbms > Bulb_interval)
{
prevBulbms = fromStartms;  
if(Bulbstate==LOW)
{
Bulbstate=HIGH;
digitalWrite(oneBulb,Bulbstate); 
}
else
{
Bulbstate=LOW;
digitalWrite(oneBulb,Bulbstate);  
} 
}
  
}//if ultrasonic out
 else {//else in
  if(fromStartms - prevBulbms > Bulb_interval)
{
prevBulbms = fromStartms;  
if(Bulbstate==LOW)
{
Bulbstate=HIGH;
digitalWrite(oneBulb,Bulbstate);
 
}
else
{
Bulbstate=LOW;
digitalWrite(oneBulb,Bulbstate);  
}  
} 
  }// else out  
 
}  
