#include <Servo.h>
Servo xservo;

#include <Wire.h>
#include "Kalman.h"
Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68; // MPU6050 Address

/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

int moveX;
int mapX;
int displayX;
int correctionXgyro;
int correctionXservo;

double accXangle;
double accYangle;
double gyroXangle = 9;      
double gyroYangle = 180;
double compAngleX = 90;    
double compAngleY = 90;
double kalAngleX;
double kalAngleY;

uint32_t timer;
//smoothing vars
int smoothAccYangle;
int smoothAccXangle;
int smoothGyroY;
int smoothGyroX;
//filter vars
int sensVal;           // for raw sensor values 
//float filterVal;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float smoothedVal;     // this holds the last loop value just use a unique variable for every different sensor that needs smoothing

float filterVal = .75;
float filterValGyro = .75;

int gpxPin = 11;
double gpxRatio = 2.8;
double gpxOut;

// ----------  VOID SETUP START -------------- /
void setup() { 
  Serial.begin(115200);
  xservo.attach(18);
 
 
  Wire.begin(); 
  i2cWrite(0x6B,0x00);           // Disable sleep mode 
  if(i2cRead(0x75,1)[0] != 0x68) {   // Read "WHO_AM_I" register
    Serial.print(F("MPU-6050 with address 0x"));
    Serial.print(IMUAddress,HEX);
    Serial.println(F(" is not connected"));
    while(1);
  }
  //set TMC settings to MPU6050
  i2cWrite(0x6B,00110000);
  //set sample rate. Sample rate = Gyro Output Rate / (1 + this value). so bigger values mean smaller sample freq.
  i2cWrite(0x19,0x10); 
  
  i2cWrite(0x1B,0x00); //set gyro to +/- 250 degrees
/*
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 */
  i2cWrite(0x1A,0x06); //set DLPF high to filter noise
 /*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 */
  i2cWrite(0x1C,0x00); //set accel output to +/- 16g instead of 2g default
/*
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
  //set angles
  kalmanX.setAngle(90);  // Set starting angle
  kalmanY.setAngle(90);
  timer = micros();
}

// ----------  VOID SETUP END -------------- /


// ---------------------- VOID LOOP START -------------- /
void loop() {
  /* Update all the values */
  uint8_t* data = i2cRead(0x3B,14); 
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]); 
  tempRaw = ((data[6] << 8) | data[7]); 
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
 
  /* Calculate the angles based on the different sensors and algorithm */
 
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  smoothAccYangle = smooth(accYangle, filterVal,smoothAccYangle);
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  smoothAccXangle = smooth(accXangle,filterVal,smoothAccXangle);
  smoothGyroX = smooth(gyroX,filterValGyro,smoothGyroX);
  smoothGyroY = smooth(gyroY,filterValGyro,smoothGyroY);
  double gyroXrate = (double)smoothGyroX/131.0;
  double gyroYrate = -((double)smoothGyroY/131.0); 
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter 

  gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*smoothAccXangle); // Calculate the angle using a Complimentary filter
 compAngleX = round(compAngleX);
  kalAngleX = kalmanX.getAngle(smoothAccXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleX =round(kalAngleX);
  timer = micros();
  mapX = map(kalAngleX, 0, 200, 0, 180);     //calculate limitation of servo mechanical
 
 
// /////////////////////////////

    correctionXservo = 10;     // EDIT THIS VALUE FOR SERVO CORRECTION ANGLE
    correctionXgyro = -90;    // EDIT THIS VALUE FOR THE POSITION OF THE GYRO ON THE BIKE

// ////////////////////////////


  moveX = 270 - (kalAngleX) + correctionXservo;
  displayX = 270 - (kalAngleX) + correctionXgyro;
  if(displayX == 0){
    displayX = displayX + 1;
  }
  
  if(displayX < 0) {
    displayX = displayX * -1;
  }  
  gpxOut = ((displayX * displayX)/displayX) * gpxRatio;
  
  // ------- SEND TO SERIAL PRINT START ----- /

  Serial.print("acc X: ");
  Serial.print(accXangle);
  Serial.print(" |Lean X: ");
  Serial.print(displayX);
  Serial.print(" |gyro angle: ");
  Serial.print(gyroXangle);
  Serial.print(" |comp angle: ");
  Serial.print(compAngleX);
  Serial.print(" |kal angle: ");
  Serial.print(kalAngleX);
  Serial.print(" |gpx out: ");
  Serial.print(gpxOut);
  Serial.print("\t");
  Serial.print("\n");
 
// ------- SEND TO SERIAL PRINT END ----- / 


// --------- SEND DATA THROUGH PWM TO GPX --------- /


analogWrite(gpxPin, gpxOut);

// --------- END GPX DATA SENDING ----------------- /


// ------- SEND TO SERVO START ----- /

   xservo.write(moveX);   // Send signal to servo
   delay(15);     // delay to allow servos to move (ms) 
  
// ------- SEND TO SERVO END ----- /


 
  delay(1); // The accelerometer's maximum samples rate is 1kHz
}
// ---------------------- VOID LOOP END -------------- /



// -- FUNCTIONS START --
void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();                       // Send stop
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes]; 
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes);   // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data[i] = Wire.read();
  return data;
}

int smooth(int data, float filterVal, float smoothedVal){
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return (int)smoothedVal;
}
// -- FUNCTIONS END --

// GYROCAM BY SAFT7.COM //

// END
