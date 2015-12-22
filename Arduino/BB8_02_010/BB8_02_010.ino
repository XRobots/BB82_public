//Droid

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <Servo.h>

#define TCAADDR 0x70

#define PIN            31
#define NUMPIXELS      2

Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

Servo headServo;
Servo panelServo1;
Servo panelServo2;

int ch0;  // throttle
int ch1;  // flywheel
int ch2;  // head BF
int ch3;  // head spin
int ch4;  // Head LR
int but1; // sounds
int but2;  // parking brake
int panelToggle = 0;  // toggle open/close panels
int tilt1 = 1; // tilt switches
int tilt2 = 1;

unsigned long lastDataTime;           // clock for safeties

double SetpointY, InputY, OutputY;    // PID variables body IMU
double SetpointY2, InputY2, OutputY2;    // PID variables head stick IMU - Back to front
double SetpointX, InputX, OutputX;    // PID variables head stick IMU - side to side

int driveHead;
int drive;

double Yort;
double diffY;
double HeadY;                         // head Y axis IMU

double Pky = 7;                      // All PID tuning variables
double Iky = 0;
double Dky = 1.4;

double Pky2 = 18;
double Iky2 = 0;
double Dky2 = 0.7;

double Pkx = 5;
double Ikx = 0.5;
double Dkx = 0.25;


unsigned long previousDriveMillis = 0;   // clock to disable drive when spinning
long driveTime = 1500;                   // time to disable drive when spinning
int driveFlag1 = 0;                      // flag to disable drive when spinning
long debounceTime = 500;                       // sound/panel debounce
unsigned long previousdebounceTime = 0;
unsigned long currentMillis = 0;


PID myPIDY(&InputY, &OutputY, &SetpointY, Pky, Iky , Dky, DIRECT);    // Y axis PID
PID myPIDY2(&InputY2, &OutputY2, &SetpointY2, Pky2, Iky2, Dky2, DIRECT);    // Y axis PID
PID myPIDX(&InputX, &OutputX, &SetpointX, Pkx, Ikx , Dkx, DIRECT);    // X axis PID


void tcaselect(uint8_t i) {             // I2c multiplexer selection function
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup()
{

   pixels.begin(); // This initializes the NeoPixel library.
   
   Serial1.begin(38400);                // bluetooth date rx
   Serial.begin(38400);                 // debug
 
  // Initialise the 1st sensor 
  tcaselect(0);
  bno1.begin();
  // Initialise the 2nd sensor 
  tcaselect(1);
  bno2.begin();   
  // #1 and again with error handling! 
  tcaselect(0); 
    if(!bno1.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }  
  else{ 
    Serial.println("#1 is all ok!"); 
  }
  delay(1000);
  bno1.setExtCrystalUse(true);
  // #2 and again with error handling! 
  tcaselect(1); 
    if(!bno2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    else{ 
    Serial.println("#2 is all ok!"); 
  }
  delay(1000);
  bno2.setExtCrystalUse(true); 

  //increase PWM freq beyond audible range
  TCCR3B = TCCR3B & 0b11111000 | 2; //set PWM ports to higher frequency
  TCCR1B = TCCR1B & 0b11111000 | 2; //set PWM ports to higher frequency
  TCCR4B = TCCR4B & 0b11111000 | 2; //set PWM ports to higher frequency 
  
   
   myPIDY.SetMode(AUTOMATIC);
   myPIDY.SetOutputLimits(-255,255);    // limits for PID loops
   myPIDY.SetSampleTime(10);
   
   myPIDY2.SetMode(AUTOMATIC);
   myPIDY2.SetOutputLimits(-255,255);    // limits for PID loops
   myPIDY2.SetSampleTime(10);
   
   myPIDX.SetMode(AUTOMATIC);
   myPIDX.SetOutputLimits(-100,100);    // limits for PID loops
   myPIDX.SetSampleTime(10);
   
   pinMode(5, OUTPUT);
   pinMode(6, OUTPUT);
   pinMode(8, OUTPUT);
   pinMode(4, OUTPUT);
   pinMode(7, OUTPUT);
   pinMode(12, OUTPUT);
   pinMode(22, OUTPUT);  // sound trigger - unused
   pinMode(24, OUTPUT);  // sound trigger
   headServo.attach(11);
   panelServo1.attach(33);
   panelServo2.attach(35);
   
   digitalWrite(24, HIGH);  // set second sound pin high

   pixels.setPixelColor(0, pixels.Color(85,0,0)); 
   pixels.setPixelColor(1, pixels.Color(0,0,85)); 
   pixels.show(); // This sends the updated pixel color to the hardware.

   pinMode(37, INPUT_PULLUP);
   pinMode(39, INPUT_PULLUP);
  
   headServo.write(90); // head Servo
   panelServo1.write(160);  // panel servos
   panelServo2.write(160); // lower value opens
}

void loop() {
  
  // if there's any serial available, read it:
  while (Serial1.available() > 0) {

    // look for the next valid integer in the incoming serial stream:

    but1 = Serial1.parseInt();
    but2 = Serial1.parseInt();
    ch0 = Serial1.parseInt();
    ch1 = Serial1.parseInt();
    ch2 = Serial1.parseInt();
    ch3 = Serial1.parseInt();
    ch4 = Serial1.parseInt();
    
    // look for the newline. That's the end of your sentence:
    if (Serial1.read() == '\n') { 
       lastDataTime = millis(); //bookmark the time for safties
       currentMillis = millis();
         } 


//drive motors

ch1 = map(ch1, 0,1024,-255,255);   // flywheel start
  if (ch1<=-80) {
    driveFlag1 = 1;  // main drive disabled
    ch1 = abs(ch1);
    ch1 = constrain(ch1,0,255);
    analogWrite(5, 0);
    analogWrite(6, ch1);
    previousDriveMillis = currentMillis;  // reset the clock
  }
  else if (ch1>=80) {
    driveFlag1 = 1;  // main drive disabled
    ch1 = abs(ch1);
    ch1 = constrain(ch1,0,255);
    analogWrite(6, 0);
    analogWrite(5, ch1);
    previousDriveMillis = currentMillis;  // reset the clock
  }
  else {
    analogWrite(5, 0);
    analogWrite(6, 0);
  }   // flywheel end
 
  
  ch3 = map(ch3, 0,1024,-255,255);   // Head Spin start
  
  if (ch3<=-20) {
    ch3 = abs(ch3);
    ch3 = constrain(ch3,0,255);
    analogWrite(4, 0);
    analogWrite(8, ch3);
  }
  else if (ch3>=20) {
    ch3 = constrain(ch3,0,255);
    analogWrite(8, 0);
    analogWrite(4, ch3);
  }
  else {
    analogWrite(4, 0);
    analogWrite(8, 0);
  }   // Head Spin end

  
  if (driveFlag1 == 1 && (currentMillis - previousDriveMillis >= driveTime)) {  // if timer has expired - wait for magnetometer to settle after spinning metal has stopped
    driveFlag1 = 0;  // main drive enabled
  }
  
  if (driveFlag1 == 1) {
    analogWrite(2, 0);
    analogWrite(3, 0);  // stop main drive until timer has expired
  }
  
  else if (driveFlag1 == 0) {   // main drive go!

    sensors_event_t event; 
    tcaselect(1);               // select head stick IMU
    bno2.getEvent(&event);
    HeadY = ((event.orientation.y)*-1);     // head Y axis IMU value inverted
    
    //Head Drive

    ch2 = (map(ch2,0,1024,-20,20)+2);
    SetpointY2 = ch2;                      // head back/forwards input and PID set point
    InputY2 = HeadY*-1;                       // IMU is also input to PID loop 
    myPIDY2.Compute();
    
    diffY = Yort-HeadY;                     // difference between body and head IMUs in Y axis - Yort is set the last time aroudn the loop lower down
    driveHead = OutputY2;                   // output to drive head stick   

    
    if (driveHead < -0 && diffY > -15 ) {   // limit for rear movement
    driveHead = abs(driveHead);
    driveHead = constrain(driveHead,0,255);
    analogWrite(12, 0);
    analogWrite(7, driveHead);
  }
  
  else if (driveHead >= 0 && diffY < 10) {  // limit for forward movement
    driveHead = abs(driveHead);
    driveHead = constrain(driveHead,0,255);
    analogWrite(7, 0);
    analogWrite(12, driveHead);
  }
  else {
    analogWrite(7, 0);
    analogWrite(12, 0);
  }

    tcaselect(0);                             // select body IMU 
    bno1.getEvent(&event);

    // Main Drive

    ch0 = (map(ch0,0,1024,-20,20));           // inout and setpoint for throttle
    SetpointY = ch0;   
    
    Yort = ((event.orientation.y)*-1);        // extra orientation value inverted for main drive stability    
    
    if (but2 == 0) {
        InputY = Yort - (HeadY/1.6);            // if handbrake is on then subtract head stick angle to maintain centre of gravity
    }
    else {
    InputY = Yort;                            // if handbrake is off then drive normally.
    }
    
    myPIDY.Compute();    
    drive = OutputY;                          //output for driving
    
  if (drive <-1) {
    drive = abs(drive);
    drive = constrain(drive,0,255);
    analogWrite(3, 0);
    analogWrite(2, drive);
  }
  else if (drive>1) {
    drive = constrain(drive,0,255);
    analogWrite(2, 0);
    analogWrite(3, drive);
  }
  else {
    analogWrite(2, 0);
    analogWrite(3, 0);
  }   
    
    // Head Servo
    
    ch4 = map(ch4,0,1024,30,150);  // turn head
    headServo.write(ch4);


   // ******* Sounds and panels *********
    
   if (but1 == 0 && panelToggle == 0) {
      digitalWrite(22, LOW);   // play sound
      panelToggle = 1;
   }

   else if (but1 == 0 && panelToggle == 1) {
      digitalWrite(22, LOW);   // play sound
      panelToggle = 0;    
    }
    else {
      digitalWrite(22, HIGH);   
    }

   tilt1 = digitalRead(37);
   tilt2 = digitalRead(39);

   if (panelToggle == 1 && tilt1 == LOW && currentMillis - previousdebounceTime >= debounceTime) {
    panelServo1.write(60);  // panel servos
    previousdebounceTime = currentMillis;  
   }
   else if (panelToggle == 0 && tilt1 == LOW && currentMillis - previousdebounceTime >= debounceTime) {
    panelServo1.write(160);  // panel servos
    previousdebounceTime = currentMillis;
   }    

   if (panelToggle == 1 && tilt2 == LOW && currentMillis - previousdebounceTime >= debounceTime) {
    panelServo2.write(40);  // panel servos
    previousdebounceTime = currentMillis;  
   }
   else if (panelToggle == 0 && tilt2 == LOW && currentMillis - previousdebounceTime >= debounceTime) {
    panelServo2.write(160);  // panel servos
    previousdebounceTime = currentMillis;
   }  
    

    }  

    }

         if (millis() - lastDataTime > 200) {  //safties
            digitalWrite(6, LOW);  // flywheel
            digitalWrite(5, LOW);  // flywheel
            analogWrite(4, 0);
            analogWrite(8, 0);
            analogWrite(7, 0);
            analogWrite(12, 0);
            analogWrite(2, 0);
            analogWrite(3, 0);
            headServo.write(90); // head Servo
       }  
     
 


}

       







