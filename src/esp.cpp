#include <Wire.h>
#include <Seeed_Arduino_SSCMA.h>  // Assuming you are using Seeed's AI library
#include <LSM303D.h>

SSCMA AI;  // Initialize the AI object

#define ARDUINO_ADDRESS 8  // I2C slave address for Arduino Mega
#define GYRO_ADRESS 0x1E  

int16_t accel[3];  // we'll store the raw acceleration values here
int16_t mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;


void setup() {
  AI.begin();  // Initialize the AI module
  Wire.begin(5, 6);  // ESP32 I2C: SDA = 5, SCL = 6 (adjust as needed)
  Serial.begin(9600);

  char rtn = 0;
  Serial.begin(9600);  // Serial is used for debugging
  Serial.println("\r\npower on");
  rtn = Lsm303d.initI2C();
  //rtn = Lsm303d.initSPI(SPI_CS);
  if(rtn != 0)  // Initialize the LSM303, using a SCALE full-scale range
	{
		Serial.println("\r\nLSM303D is not found");
		while(1);
	}
	else
	{
		Serial.println("\r\nLSM303D is found");
	}
}

void loop() {
//här blir de yäni kompass?
  Lsm303d.getAccel(accel);
	while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
	Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
	
	for (int i=0; i<3; i++)
	{
		realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
	}
	heading = Lsm303d.getHeading(mag);
	titleHeading = Lsm303d.getTiltHeading(mag, realAccel);
	
  Serial.println(titleHeading, 3);

  Wire.beginTransmission(GYRO_ADRESS);
  Wire.write(0x03);  // Set the register to read from

  if (false) {  // Run AI model, no filter, no image
  // if (!AI.invoke(1, false, false)) {  // Run AI model, no filter, no image
    //Serial.print("invoke success");


    // Check if at least one object is detected
    if (AI.boxes().size() > 0) {
      // Extract the first detection data
      int target = AI.boxes()[0].target;
      int score = AI.boxes()[0].score;
      int x = AI.boxes()[0].x;
      int y = AI.boxes()[0].y;
      int w = AI.boxes()[0].w;
      int h = AI.boxes()[0].h;

      // Print data to Serial Monitor for debugging
      Serial.print("Target: "); Serial.print(target);
      Serial.print(", Score: "); Serial.print(score);
      Serial.print(", X: "); Serial.print(x);
      Serial.print(", Y: "); Serial.print(y);
      Serial.print(", W: "); Serial.print(w);
      Serial.print(", H: "); Serial.println(h);

      // Send data to Arduino Mega via I2C
      Wire.beginTransmission(ARDUINO_ADDRESS);
      Wire.write(target);  // Send object type (1 byte)
      Wire.write(score);   // Send confidence score (1 byte)
      Wire.write(x >> 8);  // Send high byte of x
      Wire.write(x & 0xFF); // Send low byte of x
      Wire.write(y >> 8);  // Send high byte of y
      Wire.write(y & 0xFF); // Send low byte of y
      Wire.write(w >> 8);  // Send high byte of w
      Wire.write(w & 0xFF); // Send low byte of w
      Wire.write(h >> 8);  // Send high byte of h
      Wire.write(h & 0xFF); // Send low byte of h
      Wire.endTransmission();
      
      Serial.println("Data sent to Arduino Mega.");
    }
  }
}