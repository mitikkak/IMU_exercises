
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <SparkFun_ADXL345.h>
#include <SparkFunMPU9250-DMP.h>

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

#ifdef ADXL345
ADXL345 acc;
void initAdxl345()
{
	  acc.powerOn();                     // Power on the ADXL345

	  acc.setRangeSetting(4);           // Give the range settings
	                                      // Accepted values are 2g, 4g, 8g or 16g
	                                      // Higher Values = Wider Measurement Range
	                                      // Lower Values = Greater Sensitivity
}
#else
MPU9250_DMP imu;
void initMpu9250()
{
	if (imu.begin() != INV_SUCCESS)
	{
		display.setCursor(0, 20);
		display.print("imu failure");
		display.display();
		delay(5000);
		return;
	}
	imu.setSensors(INV_XYZ_ACCEL);
	imu.setAccelFSR(4);
}
#endif


void setup()
{
  Serial.begin(115200);
  display.begin();
  display.setContrast(58);
  display.clearDisplay();

  display.setTextSize(1);
  display.setRotation(2);
  display.setCursor(0,0); display.print("accelerometer_tilt begins"); display.display();
  delay(3000);

#ifdef ADXL345
  initAdxl345();
#else
  initMpu9250();
#endif

  display.setCursor(0,20); display.print("imu begin done"); display.display();
  delay(1000);
}

struct TiltValues
{
	double roll{0.0f};
	double pitch{0.0f};
	double yaw{0.0f};
};
typedef unsigned long TIME;

double fXg, fYg, fZg;
TiltValues getTiltValues()
{
   TiltValues values;
   double pitch, roll = 0;
   int Xg, Yg, Zg = 0;
#ifdef ADXL345
   acc.readAccel(&Xg, &Yg, &Zg);
#else
	if ( imu.dataReady() )
	{
	    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
		imu.update(UPDATE_ACCEL);
	}
	Xg = imu.ax;
	Yg = imu.ay;
	Zg = imu.az;
#endif
	//Low Pass Filter
	const float alpha = 0.5;
	fXg = Xg * alpha + (fXg * (1.0 - alpha));
	fYg = Yg * alpha + (fYg * (1.0 - alpha));
	fZg = Zg * alpha + (fZg * (1.0 - alpha));
	//Roll & Pitch Equations
	values.roll = (atan2(-fYg, fZg) * 180.0) / M_PI;
	values.pitch = (atan2(fXg, sqrt(fYg * fYg + fZg * fZg)) * 180.0) / M_PI;
	return values;
}

TIME const timeLogThreshold = 200;
TIME timePrevLog = 0;
void loop()
{
	TIME const timeNow = millis();
	if (timeNow - timePrevLog > timeLogThreshold)
	{
		TiltValues tilt = getTiltValues();
		//Serial.print("still alive: "); Serial.println(timeNow);
		display.clearDisplay();
		display.display();
        int textSize = 2;
        display.setTextSize(textSize);
        display.setCursor(0, 0);
        display.print(tilt.pitch);
        display.setCursor(0, 1*8*textSize);
        display.print(tilt.roll);
        display.setCursor(0, 2*8*textSize);
        display.print(tilt.yaw);
        display.display();
		timePrevLog = timeNow;
	}
}
