#include "variant.h"
#include <stdio.h>
#include <adk.h>
#include <stdlib.h>
#include <Servo.h>
#include <Scheduler.h>
#include <Wire.h>

char applicationName[] = "AndroidAdkADC"; // the app on your phone
char accessoryName[] = "Arduino Due"; // your Arduino board
char companyName[] = "GhOST";
char versionNumber[] = "0.1";
char serialNumber[] = "1";
char url[] = "http://dox.bg/files/dw?a=dc9cf6d003";
USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName,versionNumber,url,serialNumber);
uint8_t adkVarA0[6];

int tmp102Address = 0x48;
int bmp085Address = 0x77;
const byte OSS = 2;

short ac1 ;                 // AC1
short ac2  ;                // AC2
short ac3 ;                 // AC3
unsigned short ac4  ;       // AC4
unsigned short ac5 ;        // AC5
unsigned short ac6 ;        // AC6
short b1 ;                  // B1
short b2 ;                  // B2
short mb ;                  // MB
short mc  ;                 // MC
short md ;                  // MD
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 


int R = 10;
int G = 9;
int B = 8;
int W = 11;
int RGB[] = {
  0,0,0};
unsigned long lastmillis=0;
int distval = 0;
unsigned int servoval;
Servo myservo;
int tempp;
float cel;
float temp1;
float pres;
float rh;
int celsius;
int relativeHumidity;
float pressure;






void setup() {
  cpu_irq_enable();
  digitalWrite(13, LOW);
  delay(2000);
  digitalWrite(13, HIGH);
  for(int i = 0; i <= 2; i++){
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }

  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(W, OUTPUT);

  //Serial.begin(9600);
  Wire.begin();
  myservo.attach(12);
  analogReadResolution(10);
  bmp085Calibration();
  
  Scheduler.startLoop(loop2);
  Scheduler.startLoop(servos);

#define RCVSIZE 128
}

void loop() {
  cpu_irq_enable();
  float cel = getTemperature();
  float temp1 = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pres = bmp085GetPressure(bmp085ReadUP());
  float rh  = getHumidity(cel);


  if((millis() - lastmillis) > 500)
  { 
//        Serial.print("Temperature:  ");
//        Serial.print(cel); 
//        Serial.println(" Degrees Celsius");
//        Serial.print("Pressure:  ");
//        Serial.print((pres/100), 2); //whole number only.
//        Serial.println(" hPa");
//        Serial.print("Humidity:  ");
//        Serial.print(rh);
//        Serial.println(" RH%");
//        Serial.println();//line break


    //lastmillis = millis();
    uint8_t buf[RCVSIZE];
    uint32_t nbread = 0;

    Usb.Task();
    if (adk.isReady()){

      adk.read(&nbread, RCVSIZE, buf);
      if (nbread > 0){
        uint8_t dacValue = buf[0];

      }
      int celsius = cel;
      int relativeHumidity = rh;
      pres /=100;
      int pressure = pres;
      if (relativeHumidity >= 100) { relativeHumidity = 100; }
      adkVarA0[5] = pressure & 0xFF;
      adkVarA0[4] = (pressure >> 8) & 0x0F;
      adkVarA0[3] = relativeHumidity & 0xFF;
      adkVarA0[2] = (relativeHumidity >> 8) & 0x0F;
      adkVarA0[1] = celsius & 0xFF;
      adkVarA0[0] = (celsius >> 8) & 0x0F;
      adk.write(6, adkVarA0); 
      lastmillis = millis();
    }

    
  }
}


float getTemperature(){
  Wire.requestFrom(tmp102Address,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
}

void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  // ac1 = 408;                  // AC1
  // ac2 = -72;                  // AC2
  // ac3 = -14383;               // AC3
  // ac4 = 32741;                // AC4
  // ac5 = 32757;                // AC5
  // ac6 = 23153;                // AC6
  // b1 = 6190;                  // B1
  // b2 = 4;                     // B2
  // mb = -32767;                // MB
  // mc = -8711;                 // MC
  // md = 2868;                  // MD

}

float getHumidity(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 3.3;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(A0);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}


float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(bmp085Address);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(bmp085Address, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(bmp085Address);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(bmp085Address, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(bmp085Address);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(bmp085Address);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

void loop2()
{
  for (float x=0;x<PI;x=x+0.00001)
  {
    RGB[0]=255-(255*abs(sin(x*(180/PI))));           // calculate the brightness for the red led
    RGB[1]=255-(255*abs(sin((x+PI/3)*(180/PI))));    // calculate the brightness for the green led
    RGB[2]=255-(255*abs(sin((x+(2*PI)/3)*(180/PI))));
    analogWrite(R, RGB[0]);
    analogWrite(G, RGB[1]);
    analogWrite(B, RGB[2]);
    delay(1);
    int wval = 255-(255*abs(sin((x+(6*PI)/11)*(180/PI))));
    analogWrite(W, wval);
  }

}


void servos()
{ 
  int pot = analogRead(A6);
  if(pot > 0 && pot < 100)
  {
    servoval = 180/10;
  }
  if(pot > 0 && pot < 100)
  {
    servoval = 180/10;
  }
  if(pot > 100 && pot < 200)
  {
    servoval = 2*18;
  }
  if(pot > 200 && pot < 300)
  {
    servoval = 3*18;
  }
  if(pot > 300 && pot < 400)
  {
    servoval = 4*18;
  }
  if(pot > 400 && pot < 500)
  {
    servoval = 5*18;
  }
  if(pot > 500 && pot < 600)
  {
    servoval = 6*18;
  }
  if(pot > 600 && pot < 700)
  {
    servoval = 7*18;
  }
  if(pot > 700 && pot < 800)
  {
    servoval = 8*18;
  }
  if(pot > 800 && pot < 900)
  {
    servoval = 9*18;
  }
  if(pot > 900)
  {
    servoval = 179;
  }

  myservo.write(servoval);
  delay(50);

}
















