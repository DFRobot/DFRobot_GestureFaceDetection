/*!
 *@file config.ino
 *@brief Configure sensor parameters
 *@details  This code configures the sensor's address and serial communication parameters.
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

// Create an instance of DFRobot_GestureFaceDetection_UART with the specified device ID and Serial1 for UART communication
DFRobot_GestureFaceDetection_UART gfd(&Serial1, DEVICE_ID);


void setup(){
    // Initialize Serial1 for UART communication with the sensor
    Serial1.begin(9600);

    // Initialize serial communication for debugging purposes 
    Serial.begin(115200);

    // Configure the UART settings of the sensor
    gfd.configUart(eBaud_115200, UART_CFG_PARITY_NONE, UART_CFG_STOP_BITS_2);

    // Set the device address of the sensor
    gfd.setDeviceAddr(0x72);
}


void loop()
{
    // Main loop code would go here
}
