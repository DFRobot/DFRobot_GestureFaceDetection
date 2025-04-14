/*!
 *@file getPidVid.ino
 *@brief Retrieve the device's PID and VID
 *@details  This code demonstrates how to retrieve and display the Product ID (PID) and Vendor ID (VID) of the DFRobot GestureFaceDetection sensor. It also shows how to get the number of detected faces.
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2025-03-31
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
*/

#include "DFRobot_GestureFaceDetection.h"

// Define the device ID for the GestureFaceDetection sensor
#define DEVICE_ID 0x72

// Create an instance of DFRobot_GestureFaceDetection_I2C with the specified device ID
DFRobot_GestureFaceDetection_I2C gfd(DEVICE_ID);


void setup() {
  // Wait for the sensor to start.
  delay(5000);

  // Initialize I2C communication
  gfd.begin(&Wire);

  // Initialize serial communication for debugging purposes
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
