/*!
 *@file getPidVid.ino
 *@brief Retrieve the device's PID and VID
 *@details  This code retrieves and displays the Product ID (PID) and Vendor ID (VID) of the DFRobot GestureFaceDetection sensor. It also shows how to get the number of detected faces.
 *@copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [fengli](li.feng@dfrobot.com)
 *@version  V1.0
 *@date  2024-08-09
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
*/

#include "DFRobot_GestureFaceDetection.h"

// Define the device ID for the sensor
#define DEVICE_ID  0x72 

// Create an instance of DFRobot_GestureFaceDetection_UART with the specified device ID and Serial1 for UART communication
DFRobot_GestureFaceDetection_UART gfd(&Serial1, DEVICE_ID);

void setup(){
    // Initialize Serial1 for UART communication with the sensor at 9600 baud rate
    Serial1.begin(9600);

    // Initialize serial communication for debugging purposes at 115200 baud rate
    Serial.begin(115200);

    // Retrieve and print the Product ID (PID) of the sensor
    Serial.println(gfd.getPid());

    // Retrieve and print the Vendor ID (VID) of the sensor
    Serial.println(gfd.getVid());
}

void loop(){
    // Retrieve and print the number of faces detected
    Serial.println(gfd.getFaceNumber());

    // Delay before the next loop iteration
    delay(1500);
}
