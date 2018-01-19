
#include "Arduino.h"
#include "Wire.h"
#include "Constants.h"
#include "Components.h"

void setup()
{
#ifdef TEENSY
    //  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
      // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
      Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
#else
      Wire.begin();
#endif
      delay(4000);
      Serial.begin(38400);

      // Set up the interrupt pin, its set as active high, push-pull
      pinMode(intPin, INPUT);
      pinMode(myLed, OUTPUT);
      digitalWrite(myLed, HIGH);

      display.begin(); // Initialize the display
      display.setContrast(40); // Set the contrast

    // Start device display with ID of sensor
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0,0); display.print("MPU9250");
      display.setTextSize(1);
      display.setCursor(0, 20); display.print("9-DOF 16-bit");
      display.setCursor(0, 30); display.print("motion sensor");
      display.setCursor(20,40); display.print("60 ug LSB");
      display.display();
      delay(1000);

    // Set up for data display
      display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
      display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
      display.clearDisplay();   // clears the screen and buffer

      I2Cscan();// look for I2C devices on the bus

      // Read the WHO_AM_I register, this is a good test of communication
      Serial.println("MPU9250 9-axis motion sensor...");
      byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
      Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
      display.setCursor(20,0); display.print("MPU9250");
      display.setCursor(0,10); display.print("I AM");
      display.setCursor(0,20); display.print(c, HEX);
      display.setCursor(0,30); display.print("I Should Be");
      display.setCursor(0,40); display.print(0x71, HEX);
      display.display();
      delay(1000);

      if (c == 0x71) // WHO_AM_I should always be 0x68
      {
        Serial.println("MPU9250 is online...");

        MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
        delay(1000);

       // get sensor resolutions, only need to do this once
       getAres();
       getGres();
       getMres();

       Serial.println(" Calibrate gyro and accel");
       accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
       Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
       Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

      display.clearDisplay();

      display.setCursor(0, 0); display.print("MPU9250 bias");
      display.setCursor(0, 8); display.print(" x   y   z  ");

      display.setCursor(0,  16); display.print((int)(1000*accelBias[0]));
      display.setCursor(24, 16); display.print((int)(1000*accelBias[1]));
      display.setCursor(48, 16); display.print((int)(1000*accelBias[2]));
      display.setCursor(72, 16); display.print("mg");

      display.setCursor(0,  24); display.print(gyroBias[0], 1);
      display.setCursor(24, 24); display.print(gyroBias[1], 1);
      display.setCursor(48, 24); display.print(gyroBias[2], 1);
      display.setCursor(66, 24); display.print("o/s");

      display.display();
      delay(1000);

      initMPU9250();
      Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

      // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
      byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
      display.clearDisplay();
      display.setCursor(20,0); display.print("AK8963");
      display.setCursor(0,10); display.print("I AM");
      display.setCursor(0,20); display.print(d, HEX);
      display.setCursor(0,30); display.print("I Should Be");
      display.setCursor(0,40); display.print(0x48, HEX);
      display.display();
      delay(1000);

      // Get magnetometer calibration from AK8963 ROM
      initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

      magcalMPU9250(magBias, magScale);
      Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
      Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
      delay(2000); // add delay to see results before serial spew of data

      if(SerialDebug) {
    //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
      }

      display.clearDisplay();
      display.setCursor(20,0); display.print("AK8963");
      display.setCursor(0,10); display.print("ASAX "); display.setCursor(50,10); display.print(magCalibration[0], 2);
      display.setCursor(0,20); display.print("ASAY "); display.setCursor(50,20); display.print(magCalibration[1], 2);
      display.setCursor(0,30); display.print("ASAZ "); display.setCursor(50,30); display.print(magCalibration[2], 2);
      display.display();
      delay(1000);

      // Reset the MS5637 pressure sensor
      MS5637Reset();
      delay(100);
      Serial.println("MS5637 pressure sensor reset...");
      // Read PROM data from MS5637 pressure sensor
      MS5637PromRead(Pcal);
      Serial.println("PROM dta read:");
      Serial.print("C0 = "); Serial.println(Pcal[0]);
      unsigned char refCRC = Pcal[0] >> 12;
      Serial.print("C1 = "); Serial.println(Pcal[1]);
      Serial.print("C2 = "); Serial.println(Pcal[2]);
      Serial.print("C3 = "); Serial.println(Pcal[3]);
      Serial.print("C4 = "); Serial.println(Pcal[4]);
      Serial.print("C5 = "); Serial.println(Pcal[5]);
      Serial.print("C6 = "); Serial.println(Pcal[6]);

      nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
      Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);

      display.clearDisplay();
      display.setCursor(20,0); display.print("MS5637");
      display.setCursor(0,10); display.print("CRC is "); display.setCursor(50,10); display.print(nCRC);
      display.setCursor(0,20); display.print("Should be "); display.setCursor(50,30); display.print(refCRC);
      display.display();
      delay(1000);

      attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

      }
      else
      {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        while(1) ; // Loop forever if communication doesn't happen
      }
}

