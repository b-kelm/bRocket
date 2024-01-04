
#include <MPU9250_asukiaaa.h>
#include "Adafruit_BMP085.h"

Adafruit_BMP085 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
int last_micros;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  last_micros = micros();
  
  Wire1.begin();
  mySensor.setWire(&Wire1);

  // BMP085_STANDARD
  bme.begin(BMP085_ULTRALOWPOWER,&Wire1); // Replace 0x76 with the actual I2C address of your sensor
  
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {
  // Print Time
  Serial.println(micros()-last_micros);
  last_micros = micros();

  if (mySensor.accelUpdate() == 0) {
    
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.print("accelX: " + String(aX) + ",");
    Serial.print("\taccelY: " + String(aY)+ ",");
    Serial.print("\taccelZ: " + String(aZ)+ ",");
    //Serial.print("\taccelSqrt: " + String(aSqrt)+ ",");
    
  }

  if (mySensor.gyroUpdate() == 0) {
    
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.print("\tgyroX: " + String(gX)+ ",");
    Serial.print("\tgyroY: " + String(gY)+ ",");
    Serial.print("\tgyroZ: " + String(gZ)+ ",");
    
  }
  
 /* 
  if (mySensor.magUpdate() == 0) {
  
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.print("\tmagX: " + String(mX));
    Serial.print("\tmaxY: " + String(mY));
    Serial.print("\tmagZ: " + String(mZ));
    Serial.print("\thorizontalDirection: " + String(mDirection));
    
  }
*/
  //if(millis() % 20 == 0){
    Serial.print("\tTemperature(*C): ");
    Serial.print(bme.readTemperature());
  
    //Serial.print("Pressure(Pa): ");
    //Serial.print(bme.readPressure());

    
    Serial.print("\tApproxAltitude(m): ");
    Serial.print(bme.readAltitude(101325)); // this should be adjusted to your local forcase

 // }
    
    Serial.println(""); // Add an empty line

    
  }
