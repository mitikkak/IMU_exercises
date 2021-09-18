
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <SparkFun_ADXL345.h>

#define LCD

//#define RELEASE_BOARD
#if defined RELEASE_BOARD
const uint8_t CLK = 14;
const uint8_t DIN = 27;
const uint8_t DC = 26;
const uint8_t CE = 25;
const uint8_t RESET = 33;
#else
const uint8_t CLK = 25;
const uint8_t DIN = 26;
const uint8_t DC = 27;
const uint8_t CE = 14;
const uint8_t RESET = 12;
#endif
Adafruit_PCD8544 display = Adafruit_PCD8544(CLK, DIN, DC, CE, RESET);

ADXL345 acc;
void initAdxl345()
{
	  acc.powerOn();                     // Power on the ADXL345

	  acc.setRangeSetting(4);           // Give the range settings
	                                      // Accepted values are 2g, 4g, 8g or 16g
	                                      // Higher Values = Wider Measurement Range
	                                      // Lower Values = Greater Sensitivity
}
void setup()
{
  Serial.begin(115200);
#ifdef LCD
  display.begin();
  display.setContrast(58);
  display.clearDisplay();

  display.setTextSize(1);
  display.setRotation(2);
  display.setCursor(0,0); display.print("accelerometer_tilt begins"); display.display();
#endif
  delay(1000);
#ifdef LCD
  display.clearDisplay();
  display.setCursor(0,0); display.print("connecting to wifi"); display.display();
#endif
  //serveOtaFlashPeriod();
  //startWiFi();
  //udpConnect();

  initAdxl345();

#ifdef LCD
  display.setCursor(0,20); display.print("imu begin done"); display.display();
#endif
  Serial.println("imu begin done");
  delay(1000);

#if 0
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
#endif

  Serial.println("setup done!");
  delay(1000);
}

struct TiltValues
{
	double roll{0.0f};
	double pitch{0.0f};
};
double fXg, fYg, fZg;
typedef unsigned long TIME;
TiltValues getTiltValues()
{
   TiltValues values;
   double pitch, roll = 0;
   int Xg, Yg, Zg = 0;
   acc.readAccel(&Xg, &Yg, &Zg);
   //Low Pass Filter
   const float alpha = 0.5;
   fXg = Xg * alpha + (fXg * (1.0 - alpha));
   fYg = Yg * alpha + (fYg * (1.0 - alpha));
   fZg = Zg * alpha + (fZg * (1.0 - alpha));
   //Roll & Pitch Equations
   values.roll = (atan2(-fYg, fZg)*180.0)/M_PI;
   values.pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
   return values;
}

TIME const timeLogThreshold = 200;
TIME timePrevLog = 0;
void loop()
{
	TiltValues tilt = getTiltValues();
	TIME const timeNow = millis();
	if (timeNow - timePrevLog > timeLogThreshold)
	{
		//Serial.print("still alive: "); Serial.println(timeNow);
		display.clearDisplay();
		display.display();
        int textSize = 2;
        display.setTextSize(textSize);
        display.setCursor(0, 0);
        display.print(tilt.pitch);
        display.setCursor(0, 1*8*textSize);
        display.print(tilt.roll);
        display.display();
		timePrevLog = timeNow;
	}
}
