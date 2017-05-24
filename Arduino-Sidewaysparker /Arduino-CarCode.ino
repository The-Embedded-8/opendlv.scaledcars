#include <Servo.h>
#include <Wire.h>
#include <Smartcar.h>
#include <Adafruit_NeoPixel.h>

#define steeringPin 3 // steering
#define drivePin 5 // gas
#define escPin 6
#define irSensorFront A3
#define irSensorSide A2
#define irSensorRear A1
#define encoderPin 2
#define PIN      9
#define N_LEDS 8

//global var
Servo myservo;
Servo esc;
Odometer encoder;

unsigned char serialRead;
unsigned char rcOn;
unsigned int velocity;
unsigned int steer;
unsigned int rcSteerRead;
unsigned int rcDriveRead;
unsigned char ultraFront;
unsigned char ultraRight;
unsigned char irSideFront;
unsigned char irSideBack;
unsigned char irRear;
int ODO;
int counter; // odometer counter
unsigned char pulse;
unsigned char angle;
unsigned char speedy;

//For LEDS
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);
unsigned long interval = 1000;
unsigned long previousMillis=0;
uint32_t blue = strip.Color(0, 0, 255);
uint32_t white = strip.Color(255,255,255);
uint32_t orange = strip.Color(255,165,0);
uint32_t red = strip.Color(255, 0, 0);
uint16_t leftPixel = 0;
uint16_t rightPixel = 6;

void setup() {
  Wire.begin(); 
  strip.begin();
  strip.setBrightness(127); // 50% duty cycle
  strip.show();
  pinMode(steeringPin, INPUT);
  pinMode(drivePin, INPUT);
  pinMode(escPin, OUTPUT);
  Serial.begin(115200);
  myservo.attach(11, 1440, 1800);
  esc.attach(escPin);
  pinMode(irSensorFront, INPUT);
  pinMode(irSensorSide, INPUT);
  pinMode(irSensorRear, INPUT);
  encoder.attach(encoderPin);

  //init variables
  serialRead = 0;
  rcOn = 0;
  velocity = 1360;
  steer = 0;
  rcSteerRead = 0;
  rcDriveRead = 0;
  ODO=0;
  counter = 0; // Odometer counter
  pulse=0;
}

void loop() {
    //If the RC controll is on the loop will get values from the RC.
    rcOn = ((((rcSteerRead = pulseIn(steeringPin, HIGH, 25000)) > 0) && ((rcDriveRead = pulseIn(drivePin, HIGH, 25000)) > 0))); //boolean
    if (rcOn)
    {
      rcLights(); 
      rc_control();
    } else
    {

    //ultrasonicFront();
    //ultrasonicRight();
    irFrontSide();
    irBackSide();
    irBack();
    manual_control();
    odometer();
  }

} 

//The manual controll receives values from the proxy and make it to 
//steering and speed values
void manual_control() {
//If it is something in the serila port read it
  if ( Serial.available() > 0 ) {
    char c = Serial.read();
    //Unpacks to angle and speed
    angle = c & 3;
    speedy = c & 12;
    // angle turn 1 = left, 2 = straight, 3 = right
      switch (angle)
  {
    case 1:
      leftLights();
      steer = 30;
      break;
    case 2:    
      steer = 83;
      break;
    case 3:
       rightLights();
       steer = 120;
      break;
    default:
      // delay(70);
      break;
  }
  myservo.write(steer);
  // speed move idle = 4, forward = 8, backward = 12
    switch (speedy)
  {
    case 4:
      velocity = 1360;
      break;
    case 8:
      forwardLights();
      velocity = 1450;
      break;
    case 12:
      stopLights();
      velocity = 1077;
      break;
      default:
      // delay(70);
      break;
  }
  esc.write(velocity);
//    myservo.write((int) c); //set angle of wheel
    
  }
}

//The getting values from the irSensor code is from the smart car libra
void irFrontSide(){
    unsigned int volts = analogRead(irSensorFront);  
    irSideFront =((2914 / (volts + 5)) - 1)/5; 
   //If the sensor values is over 7 make it seven
    if(irSideFront>7) irSideFront =7;
    //Set flag for proxy to find
    irSideFront=irSideFront|0x20;
    //Serial.print("Infrared front: ");
    //Print one char to serial port. 
    Serial.print((char)irSideFront);                                     
}
//The getting values from the irSensor code is from the smart car libray
void irBackSide(){
  unsigned int volts = analogRead(irSensorSide); 
  //Calculate sensor values to cm.  
    irSideBack =((2914 / (volts + 5)) - 1)/10;          
    //If the sensor values is over 7 make it seven 
    if(irSideBack>7) irSideBack =7;
    //Set flag for proxy to find
    irSideBack=irSideBack|0x28;
    //Serial.print("Infrared back side: ");
    //Print one char to serial port.
    Serial.print((char)irSideBack);  
}

//The getting values from the irSensor code is from the smart car libray
void irBack(){
  unsigned int volts = analogRead(irSensorRear);  
    irRear =((2914 / (volts + 5)) - 1)/10;          
    //If the sensor values is over 7 make it seven 
    if(irRear>7) irRear =7;
    //Set flag for proxy to find
    irRear=irRear|0x30;
    //Serial.print("Infrared back side: ");
    Serial.print((char)irRear);

}
// I2C SRF10 or SRF08 Devantech Ultrasonic Ranger Finder
// by Nicholas Zambetti <http://www.zambetti.com>
// and James Tichenor <http://www.jamestichenor.net>
// Demonstrates use of the Wire library reading data from the
// Devantech Utrasonic Rangers SFR08 and SFR10
// Created 29 April 2006
// This ultrasonics code is in the public domain.
void ultrasonicFront(){
     // step 1: instruct sensor to read echoes
    Wire.beginTransmission(112); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
   // use 0x51 for centimeters
   // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting
    // step 2: wait for readings to happen
    delay(70);                   // datasheet suggests at least 65 milliseconds
    // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(112); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting
    // step 4: request reading from sensor
    Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112
  // step 5: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    ultraFront = Wire.read();  // receive high byte (overwrites previous reading)
    ultraFront = ultraFront << 8;    // shift high byte to be high 8 bits
    ultraFront |= Wire.read(); // receive low byte as lower 8 bits
    //Devide sensor values with 10 to get values under 7
    ultraFront=ultraFront/10;
    //Because of the sensors values starts getting unstable values
    //around 80 we only sens values up to 7
    if(ultraFront>7) ultraFront =7;
    //Set a flag so the proxy will distinguish 
    //what sensor it reads. 
    ultraFront=ultraFront|0x10;
   // Serial.print("Ultrasonic front: ");
    Serial.print((char)ultraFront);
  }
}

void ultrasonicRight(){
      // step 1: instruct sensor to read echoes
    Wire.beginTransmission(115); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting
  
    // step 2: wait for readings to happen
    delay(70);                   // datasheet suggests at least 65 milliseconds
  
    // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(115); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting
  
    // step 4: request reading from sensor
    Wire.requestFrom(115, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    ultraRight = Wire.read();  // receive high byte (overwrites previous reading)
    ultraRight = ultraRight << 8;    // shift high byte to be high 8 bits
    ultraRight |= Wire.read(); // receive low byte as lower 8 bits
    //Devide sensor values with 10 to get values under 7
    ultraRight=ultraRight/10;
     //Because of the sensors values starts getting unstable values
    //around 80 we only sens values up to 7
    if(ultraFront>7) ultraFront =7;
   //Set a flag so the proxy will distinguish 
    //what sensor it reads. 
    ultraRight=ultraRight|0x18;
    //Serial.print("Ultrasonic right: ");
    Serial.print((char) ultraRight);   // print the reading
  }
}

void rc_control()
{
  if (rcSteerRead > 1370 && rcSteerRead < 1580) {
    myservo.write(80);
  }
  else if (rcSteerRead > 1580 ) {
    myservo.write(170);
  }
  else if (rcSteerRead < 1370) {
    myservo.write(10);
  }

  if (rcDriveRead > 1480) {
    esc.write(1480);

  }
  else if (rcDriveRead < 1000) 
  {
    esc.write(1000);

  }
  else {
    esc.write(rcDriveRead);
  }

  //Serial.println(rcDriveRead);// logging

}
//Function to receive values from the wheel encoder.
void odometer(){
  //If the encoder has incremented the counter to 3
  //resets counter and if the puls is 1 sets it to 0 else to 1.
  if(counter>=3){
    counter=0;
    if(pulse!=0){
      pulse=0;
    }
    else pulse=1;
  } 
  //Uses the smartcar library to get the distanse from the encoder.
  if(ODO < encoder.getDistance()){
    ODO=encoder.getDistance();
    counter++;
  }
  //Sends the puls to the serial port with a set flag.
  Serial.print((char)(pulse|0x40));
}

void rcLights(){
  if ((unsigned long)(millis() - previousMillis) >= interval) {
  previousMillis = millis();
  for(uint16_t i=0; i<strip.numPixels(); i++){
  strip.setPixelColor(i, blue);
   strip.show();
  }
  for(uint16_t i=0; i<strip.numPixels(); i++){
  strip.setPixelColor(i, 0);
   strip.show();
  }        
}
}

void forwardLights(){
  for(int i=2; i<6; i++){
  strip.setPixelColor(i, white);
  }
    strip.show(); 
}
void stopLights(){
  strip.setPixelColor(2, 255,0,0);
    strip.setPixelColor(3, 255,0,0);
    strip.setPixelColor(4, 255,0,0);
    strip.setPixelColor(5, 255,0,0);
    strip.show();
}

    
void leftLights() {
 if ((unsigned long)(millis() - previousMillis) >= interval) {
  previousMillis = millis();
  for(uint16_t i=0; i<strip.numPixels(); i++){
  strip.setPixelColor(i, blue);
   strip.show();
  }
  for(uint16_t i=0; i<strip.numPixels(); i++){
  strip.setPixelColor(i, 0);
   strip.show();
  } 
  }
}

void rightLights() {
if ((unsigned long)(millis() - previousMillis) >= interval) {
  previousMillis = millis();
  strip.setPixelColor(rightPixel, orange);
  strip.show();
  rightPixel++;
  strip.setPixelColor(rightPixel-1,0);
  if(rightPixel == 8){
    rightPixel = 6;
    }

  }
}

