/*!
 *@file configGesture.ino
 *@brief Config gestures
 *@details  This code configure the position, score of faces, and gestures along with their scores. It interacts with the DFRobot GestureFaceDetection sensor to perform face and gesture detection and display the results.
 *@details  The configuration needs to be powered off to take effect.
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

// Buffer for formatted output
char str[100];

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
  // Set the face detection threshold. Face scores range from 0 to 100.
  // Faces with scores above this threshold will be detected.
  if (gfd.setFaceDetectThres(60)) {
    Serial.println("Set the face detection threshold success.");
  } else {
    Serial.println("Set the face detection threshold fail.");
  }

  // Set the gesture detection threshold. Gesture scores range from 0 to 100.
  // Gestures with scores above this threshold will be detected.
  if (gfd.setGestureDetectThres(60)) {
    Serial.println("Set the gesture detection threshold success.");
  } else {
    Serial.println("Set the gesture detection threshold fail.");
  }

  // Set the gesture detection range.
  // The range is from 0 to 100; 0 has the smallest detection range, and 100 has the largest.
  if (gfd.setDetectThres(100)) {
    Serial.println("Set the gesture detection range success.");
  } else {
    Serial.println("Set the gesture detection range fail.");
  }
}

void loop() {
  delay(1000);
}
