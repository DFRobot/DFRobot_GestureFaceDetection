/*!
 *@file getPidVid.ino
 *@brief Retrieve the device's PID and VID
 *@details  This code demonstrates how to retrieve and display the Product ID (PID) and Vendor ID (VID) of the DFRobot GestureFaceDetection sensor. It also shows how to get the number of detected faces.
 *@copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [fengli](li.feng@dfrobot.com)
 *@version  V1.0
 *@date  2024-08-09
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
*/

#include "DFRobot_GestureFaceDetection.h"

// Define the device ID for the GestureFaceDetection sensor
#define DEVICE_ID  0x72 

// Create an instance of DFRobot_GestureFaceDetection_I2C with the specified device ID
DFRobot_GestureFaceDetection_I2C gfd(DEVICE_ID);


void setup(){
    // Initialize I2C communication
    gfd.begin(&Wire);

    // Initialize serial communication for debugging purposes
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
