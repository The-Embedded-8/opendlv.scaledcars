#include <Servo.h>
#include <Wire.h>
#include <Smartcar.h>

#define steeringPin 3 // steering
#define drivePin 5 // gas
#define escPin 6
#define irSensorFront A3
#define irSensorSide A2
#define irSensorRear A1
#define encoderPin 2


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
unsigned char angle = 0;
unsigned char speedy = 0;


void setup() {
  Wire.begin(); 
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
  ODO = 0;
  counter = 0; // Odometer counter
  pulse = 0;
}

void loop() {
  // Boolean && update the reads for the Gaz
  rcOn = ((rcDriveRead = pulseIn(drivePin, HIGH, 25000)) > 0);
  if (rcOn)
    esc.write(rcDriveRead);
  else
    esc.write(1421);

  // Get the steering commands from the overtaker/laneFollower
  manual_control(); 
  // Send sensor data
  sensors_read(); 
}
/*
* Read from the Serial and write the value (angle) to the servo
*/
void sensors_read(){

  ultrasonic();
  infraread();
}

void manual_control() {

  if ( Serial.available()) {
    // Read the angle from the proxy
    char c = Serial.read();
    // Steer in range of 0, 160
    myservo.write((int) c);
  }

}

void infraread(){
    // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
    unsigned int volts = analogRead(irSensorFront);
    // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
    irSideFront = ((2914 / (volts + 5)) - 1) / 5;
    irSideFront = (irSideFront > 7 ? 7 : irSideFront);
    unsigned int volt = analogRead(irSensorSide);   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
    irSideBack =((2914 / (volt + 5)) - 1) / 10;          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk

    // Limit the value to 7
    irSideBack = (irSideBack > 7 ? 7 : irSideBack);
    // put both reads in one byte
    irSideBack = irSideBack | irSideFront << 3;
    // Set the flag
    irSideBack = irSideBack | 64;
    // Write it to the serial
    Serial.write(irSideBack);

    unsigned int v = analogRead(irSensorRear);   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
    irRear =((2914 / (v + 5)) - 1)/10;          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk

    // Limit the value to 7
    irRear = (irRear > 7 ? 7 : irRear);
    // Set the flag 
    irRear=irRear | 128;
    // update the odometer reads
    odometer();
    // Pack the odometer with the ir reads
    irRear = irRear | pulse << 3;
    Serial.write(irRear);                        
}

void ultrasonic(){
    unsigned char old = ultraFront;
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
    ultraFront = ultraFront/10;
    // Limit the value to 7
    ultraFront = (ultraFront > 7 ? 7 : ultraFront);
    // filter the noise
    ultraFront = (ultraFront == 0 ? (old == 7 ? old : ultraFront) : ultraFront );
    // mock the read as a right center ultrasonic and pack it in one byte
    ultraFront = ultraFront | ultraFront << 3;
    Serial.write(ultraFront);
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
}

void odometer(){

    if(counter>=3){
      counter=0;
    if(pulse!=0){
      pulse=0;
    }
    else 
      pulse=1;
    } 
    if(ODO < encoder.getDistance()){
    ODO = encoder.getDistance();
    counter++;
    }
}

