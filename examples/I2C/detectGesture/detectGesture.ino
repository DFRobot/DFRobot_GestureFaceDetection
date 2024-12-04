/*!
 *@file detectGesture.ino
 *@brief Detect gestures
 *@details  This code detects the position, score of faces, and gestures along with their scores. It interacts with the DFRobot GestureFaceDetection sensor to perform face and gesture detection and display the results.
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

// Buffer for formatted output
char str[100];


void setup(){
    // Initialize I2C communication
    gfd.begin(&Wire);

    // Initialize serial communication for debugging purposes
    Serial.begin(115200);

    // Set the face detection threshold. Face scores range from 0 to 100.
    // Faces with scores above this threshold will be detected.
    gfd.setFaceDetectThres(60);

    // Set the gesture detection threshold. Gesture scores range from 0 to 100.
    // Gestures with scores above this threshold will be detected.
    gfd.setGestureDetectThres(60);	

    // Set the gesture detection range. 
    // The range is from 0 to 100; 0 has the smallest detection range, and 100 has the largest.
    gfd.setDetectThres(100);
}


void loop(){
    // Check if any faces are detected
    if(gfd.getFaceNumber() > 0){
        // Retrieve face score and location
        uint16_t faceScore = gfd.getFaceScore();
        uint16_t faceX = gfd.getFaceLocationX();
        uint16_t faceY = gfd.getFaceLocationY();
        
        // Print the face detection results
        sprintf(str, "detect face at (x = %d, y = %d, score = %d)\n", faceX, faceY, faceScore);
        Serial.print(str);
        
        // Print the gesture detection results
        // - 1: LIKE (👍) - blue
        // - 2: OK (👌) - green
        // - 3: STOP (🤚) - red
        // - 4: YES (✌) - yellow
        // - 5: SIX (🤙) - purple
        uint16_t gestureType = gfd.getGestureType();
        uint16_t gestureScore = gfd.getGestureScore();
        
        // Print the gesture detection results
        sprintf(str, "detect gesture %d, score = %d\n", gestureType, gestureScore);
        Serial.print(str);
    }
    
    // Delay before the next loop iteration
    delay(1500);
}
