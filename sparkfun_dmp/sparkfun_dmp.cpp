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
#ifdef ESP32
#include "WiFi.h"
#include "AsyncUDP.h"
#else
#include "ESP8266WiFi.h"
#include "ESPAsyncUDP.h"
#endif



AsyncUDP udp;

MPU9250_DMP imu;

#if !defined ARDUINO_ESP8266_GENERIC && !defined ESP32
#define LCD
#endif

#ifdef LCD
Adafruit_PCD8544 display = Adafruit_PCD8544(D4, 15, 14, 13, 12);
#endif

#if 0
const char* my_ssid     = MY_SSID;
const char* my_password = MY_WIFI_PASSWD;
#else
// async_udp server's
const char* const ssid = "kayak_logger_ap";
const char* const password = "";
#endif

void startWiFi()
{
    Serial.begin(115200);
    Serial.println("client begins");
    delay(1000);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        delay(1000);
    }
    Serial.printf("Connected to: ");
    Serial.println(WiFi.SSID());
    Serial.printf("IP address: ");
    Serial.println(WiFi.localIP());
}
void udpConnect()
{
    if(udp.connect(IPAddress(192,168,4,1), 1234)) {
        Serial.println("UDP connected");
        udp.onPacket([](AsyncUDPPacket packet) {
            Serial.println("TODO: server sent something...");
#if 0
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
#endif
            //reply to the client
            //packet.printf("Got %u bytes of data", packet.length());
        });
        //Send unicast
//        udp.print("Hello Server!");
    }
}
unsigned int numOfMessages{0};
unsigned int sn{0};
int toInteger(const float val)
{
    return static_cast<int>(std::ceil(val));
}
void sendOrientationMessage(uint8_t const num)
{
    (void) num;
    const String separator(";");
    String payload{millis()};
    payload += String(separator);
    payload += String(sn++);
    payload += String(separator);
    payload += String(toInteger(imu.pitch));
    payload += String(separator);
    payload += String(toInteger(imu.roll));
    payload += String(separator);
    payload += String(toInteger(imu.yaw));
    payload += String(separator);

    udp.print(payload);
    numOfMessages++;
}
#if 0
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
        //sendOrientationMessage(num);
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
#endif

void setup() 
{
  Serial.begin(115200);
#ifdef LCD
  display.begin();
  display.setContrast(58);
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0,0); display.print("sparkfun_dmp begins"); display.display();
#endif
  delay(1000);
#ifdef LCD
  display.clearDisplay();
  display.setCursor(0,0); display.print("connecting to wifi"); display.display();
#endif
  startWiFi();
  udpConnect();
//  startWebSocket();

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
#ifdef LCD
      display.setCursor(0,20); display.print("imu failure"); display.display();
#endif
      delay(5000);
    }
  }
#ifdef LCD
  display.setCursor(0,20); display.print("imu begin done"); display.display();
#endif
  Serial.println("imu begin done");
  delay(1000);
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  Serial.println("setup done!");
  delay(1000);
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
//  float q0 = imu.calcQuat(imu.qw);
//  float q1 = imu.calcQuat(imu.qx);
//  float q2 = imu.calcQuat(imu.qy);
//  float q3 = imu.calcQuat(imu.qz);
//
//  Serial.println("Q: " + String(q0, 4) + ", " +
//                    String(q1, 4) + ", " + String(q2, 4) +
//                    ", " + String(q3, 4));
  Serial.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();

#ifdef LCD
  display.clearDisplay();
  display.setCursor(0,0); display.print("P: "); display.print(imu.pitch);
  display.setCursor(0,20); display.print("R: "); display.print(imu.roll);
  display.setCursor(0,40); display.print("Y: "); display.print(imu.yaw);
  display.display();
#endif

}

unsigned long prevTimeSent = 0;
bool timeToSend()
{
    static const unsigned long interval = 200;
    return (millis() - prevTimeSent > interval);
}

unsigned long prevTimeLogged = 0;
const unsigned long loggingThreshold = 5000;
void loop() 
{
//  webSocket.loop();
  // Check for new data in the FIFO
#if 1
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      //printIMUData();
    }
  }
#endif
  if (timeToSend())
  {
      sendOrientationMessage(0);
      prevTimeSent = millis();
  }
  unsigned long timeNow = millis();
  if (timeNow - prevTimeLogged > loggingThreshold)
  {
      prevTimeLogged = timeNow;
      Serial.println(numOfMessages);
      numOfMessages = 0;
  }
}

