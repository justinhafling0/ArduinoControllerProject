
  
  
  
  int xPin =  A0;
  int yPin = A1;
  int zPin = A2;

  int buttonPin = 13;
  
  int x_Normal = 0;
  int y_Normal = 0;
  int z_Normal = 0;
  
  int x_Threshold = 25;
  int y_Threshold = 43;
  int z_Threshold = 25;
  
  float x,y,z, r1, r2;
  
  String movementString;
  String jumpString;
  String clickString;

  float yVision = 0;
  float xVision = 0;
  float xVisionPrev = 0;
  float yVisionPrev = 0;

  int zeroCounter = 0;

  
  float visionTiming = 0.01;
/*

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"



#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(500000);
  Serial.println("Starting Up...");

  while(!Serial){};


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.println("Here");
  Serial.println(c);
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    

    // Calibrate gyro and accelerometers, load biases in bias registers

        
    //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    

    myIMU.initMPU9250();
    Serial.println("Initialized");
    
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    //Serial.println("AK8963 initialized for active data mode....");

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();


  } // if (c == 0x71)
  else
  {
    Serial.flush();
    Serial.println("Aborted");
    delay(5);
    abort();
  }
 
}

void loop()
{
  //delay(5000);

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  myIMU.updateTime();



  

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);




                     

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!

        // Print gyro values in degree/sec
        //Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        //Serial.println(" degrees/sec ");

        //Serial.print("X: "); Serial.print(myIMU.gx, 3);
        //Serial.print(" Y: "); Serial.println(myIMU.gy, 3);
    /*  
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        */

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        //Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        //Serial.println(" degrees C");
      }


      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)

    x = analogRead(A0);
    y = analogRead(A1);
    z = analogRead(A2);

    //Serial.println("Read data from Accelerometer");
    r1 = analogRead(A4);
    r2 = analogRead(A5);

    //Serial.println("Read data from Gyroscope");
    movementString = "0000";
  
    //Detecting if we should move left
    if(x - x_Normal > x_Threshold){   
      movementString[3] = '1';
    }else{
      movementString[3] = '0';
    }
  
    //Detecting if we should move right
    if(x_Normal - x > x_Threshold){   
      movementString[1] = '1';
    }else{
      movementString[1] = '0';
    }
    //Detecting if we should move forward here
    if(z - z_Normal > z_Threshold){   
      movementString[0] = '1';
    }else{
      movementString[0] = '0';
    }
  
    //Detecting if we should move backward here
    if(z_Normal - z > z_Threshold){
      movementString[2] = '1';
    }else{
      movementString[2] = '0';
    }
  
    if(y - y_Normal > y_Threshold){
      jumpString = '1';
      //Serial.println("Jump:");
    }else{
      jumpString = '0';
    }
  
  
  
    /*
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" z: ");
    Serial.print(z);
    Serial.print(" Total: ");
    Serial.println(x+y+z);
    */


    int buttonState = digitalRead(buttonPin);

    if(buttonState == LOW){
      //Serial.println("Don't Click");
      clickString = '0';
    }else{
      clickString = '1';
      //Serial.println("Click");
    }
 
    
    String Data = "QOVEMENT: " + movementString + " CLICKK: " + clickString + " JUMP: " + jumpString + " VISION: ";
    //1.22 1.33";

    if(zeroCounter > 10){
      Serial.print(Data);
  
  
      float scalingFactor = 0.4;
  
      int gyroThreshold = 7.5;
      int gyroCorrection = 20;
      
      float gyroX = 0;
      float gyroY = 0;
      
      if(abs(myIMU.gx) > gyroThreshold){
        gyroX = myIMU.gx;
        xVision = xVision + visionTiming * myIMU.gx;
  
      }
      //Serial.println(myIMU.gx);
      //Serial.println(myIMU.yaw);
  
      if(abs(myIMU.gy) > gyroThreshold){
        yVision = yVision + visionTiming * myIMU.gy;
      }
  
  
      //Serial.print("X: "); Serial.print(xVision - xVisionPrev); Serial.print(" Y: "); Serial.println(yVision - yVisionPrev);
      
      Serial.print(xVision - xVisionPrev, 3);
      Serial.print(" ");
      Serial.println(yVision - yVisionPrev, 3);
      //Serial.print("Yaw: "); Serial.println(myIMU.yaw);
      //Serial.print("Pitch: "); Serial.println(myIMU.pitch);
      
      xVisionPrev = xVision;
      yVisionPrev = yVision;    
      
    }else{
      if(zeroCounter == 10){
        x_Normal /= 10;
        y_Normal /= 10;
        z_Normal /= 10;


        Serial.print("X Normal: "); Serial.print(x_Normal); Serial.print(" Y: "); Serial.print(y_Normal); Serial.print(" Z: "); Serial.println(z_Normal);
        
      }else{
        x_Normal += x;
        y_Normal += y;
        z_Normal += z;
      }
      zeroCounter += 1;
    }

    
    delay(5);
    



  
}
