/*!
 *@file getPidVid.ino
 *@brief Retrieve the device's PID and VID
 *@details  This code retrieves and displays the Product ID (PID) and Vendor ID (VID) of the DFRobot GestureFaceDetection sensor. It also shows how to get the number of detected faces.
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2025-03-31
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
*/

#include "DFRobot_GestureFaceDetection.h"

// Define the device ID for the sensor
#define DEVICE_ID 0x72
#define DEVICE_BAUD 9600
/* ---------------------------------------------------------------------------------------------------------------------
  *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
  *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
  *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
  *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
  *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
  * ----------------------------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed  */
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial mySerial1(4, 5);
DFRobot_GestureFaceDetection_UART gfd(&mySerial1, DEVICE_ID);
#else
// Create an instance of DFRobot_GestureFaceDetection_UART with the specified device ID and Serial1 for UART communicatio
DFRobot_GestureFaceDetection_UART gfd(&Serial1, DEVICE_ID);
#endif

void setup() {
  // Wait for the sensor to start.
  delay(5000);
  // Initialize Serial1 for UART communication with the sensor
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  mySerial1.begin(DEVICE_BAUD);
#elif defined(ESP32)
  Serial1.begin(DEVICE_BAUD, SERIAL_8N1, D2, D3);
#else
  Serial1.begin(DEVICE_BAUD);
#endif

  // Initialize serial communication for debugging purposes at 115200 baud rate
  Serial.begin(115200);

  while (!gfd.begin()) {
    Serial.println("Communication with device failed, please check connection");
    delay(1000);
  }
  // Retrieve and print the Product ID (PID) of the sensor
  Serial.print("PID: ");
  Serial.println(gfd.getPid());

  // Retrieve and print the Vendor ID (VID) of the sensor
  Serial.print("VID: ");
  Serial.println(gfd.getVid());
}

void loop() {
  delay(1000);
}
