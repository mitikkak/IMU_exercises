/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>

MPU9250_DMP imu;
Adafruit_PCD8544 display = Adafruit_PCD8544(D4, 15, 14, 13, 12);
ESP8266WiFiMulti wifiMulti;
WebSocketsServer webSocket(80);
const char* my_ssid     = MY_SSID;
const char* my_password = MY_WIFI_PASSWD;

void startWiFi()
{
    wifiMulti.addAP(my_ssid, my_password);
    Serial.println("Connecting");
    Serial.println(my_ssid);
    Serial.println(my_password);
    while (wifiMulti.run() != WL_CONNECTED)
    {  // Wait for the Wi-Fi to connect
      delay(250);
      Serial.print('.');
    }
    Serial.println("\r\n");
    Serial.print("Connected to ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP());
}
void sendOrientationMessage(uint8_t const num)
{
    String payload2 = "Orientation: ";
    payload2 += String(imu.pitch);
    payload2 += String(" ");
    payload2 += String(imu.roll);
    payload2 += String(" ");
    payload2 += String(imu.yaw);
//    Serial.println(payload2);
    webSocket.sendTXT(num, payload2.c_str());
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
    // Figure out the type of WebSocket event
    switch(type) {

      // Client has disconnected
      case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", num);
        break;

      // New client has connected
      case WStype_CONNECTED:
        {
          IPAddress ip = webSocket.remoteIP(num);
          Serial.printf("[%u] Connection from ", num);
          Serial.println(ip.toString());
        }
        break;

      // Echo text message back to client
      case WStype_TEXT:
//        Serial.printf("[%u] Text: %s\n", num, payload);
        sendOrientationMessage(num);
        break;

      // For everything else: do nothing
      case WStype_BIN:
      case WStype_ERROR:
      case WStype_FRAGMENT_TEXT_START:
      case WStype_FRAGMENT_BIN_START:
      case WStype_FRAGMENT:
      case WStype_FRAGMENT_FIN:
      default:
        break;
    }
}
void startWebSocket()
{
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started.");
}

void setup() 
{
  Serial.begin(115200);
  display.begin();
  display.setContrast(58);
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0,0); display.print("sparkfun_dmp begins"); display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(0,0); display.print("connecting to wifi"); display.display();
  startWiFi();
  startWebSocket();

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      display.setCursor(0,20); display.print("imu failure"); display.display();
      delay(5000);
    }
  }
  display.setCursor(0,20); display.print("imu begin done"); display.display();
  Serial.println("imu begin done");
  delay(1000);
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  display.setCursor(0,30); display.print("setup done!");
  Serial.println("setup done!");
  delay(1000);
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  Serial.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  Serial.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
  display.clearDisplay();
  display.setCursor(0,0); display.print("P: "); display.print(imu.pitch);
  display.setCursor(0,20); display.print("R: "); display.print(imu.roll);
  display.setCursor(0,40); display.print("Y: "); display.print(imu.yaw);
  display.display();

}

void loop() 
{
    webSocket.loop();
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      printIMUData();
    }
  }
}

