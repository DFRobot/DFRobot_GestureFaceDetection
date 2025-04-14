'''
  @file detect_gesture.py
  @brief Detect gestures
  @details  This code detects the location, score of faces, and gestures along with their scores.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT license (MIT)
  @author [thdyyl](yuanlong.yu@dfrobot.com)
  @version  V1.0
  @date  2025-03-31
  @https://github.com/DFRobot/DFRobot_GestureFaceDetection
'''

# -*- coding: utf-8 -*-
import time
import serial
import sys
sys.path.append("../")
from DFRobot_GestureFaceDetection import DFRobot_GestureFaceDetection_I2C, DFRobot_GestureFaceDetection_UART

# Macro definition: Set to True to use I2C, False to use UART
USE_I2C = False  # Set to True to use I2C, False to use UART

# Define device address and baud rate
DEVICE_ID = 0x72
UART_BAUD_RATE = 9600

# Choose between I2C or UART based on the macro definition
if USE_I2C:
    # Using I2C interface
    gfd = DFRobot_GestureFaceDetection_I2C(bus=1, addr=DEVICE_ID)  # Assuming I2C bus 1 is used
else:
    # Using UART interface
    gfd = DFRobot_GestureFaceDetection_UART(baud=UART_BAUD_RATE, addr=DEVICE_ID)

def setup():
    """
    @brief Setup function for initializing sensor thresholds.
    
    This function gets the thresholds for face detection and gesture detection.
    """
    # Wait for the sensor to start.
    time.sleep(5)

    while gfd.begin() == False:
        print("Communication with device failed, please check connection")
        time.sleep(1)

    # Set face detection score threshold (0~100)
    print("face detection threshold: {}".format(gfd.get_face_detect_thres()))
    
    # Set gesture detection score threshold (0~100)
    print("gesture detection threshold: {}".format(gfd.get_gesture_detect_thres()))
    
    # Set detection range, 0~100
    print("gesture detection range: {}".format(gfd.get_detect_thres()))

def loop():
    """
    @brief Main loop function for continuous detection.
    
    This function continuously checks for faces and gestures, and prints their details.
    """
    while True:
        # Check if any faces are detected
        if gfd.get_face_number() > 0:
            # Get face score and position coordinates
            face_score = gfd.get_face_score()
            face_x = gfd.get_face_location_x()
            face_y = gfd.get_face_location_y()
            
            print("Detect face at (x = {}, y = {}, score = {})".format(face_x, face_y, face_score))
            
            # Get gesture type and score
            # - 1: LIKE  - blue
            # - 2: OK  - green
            # - 3: STOP  - red
            # - 4: YES - yellow
            # - 5: SIX  - purple
            gesture_type = gfd.get_gesture_type()
            gesture_score = gfd.get_gesture_score()
            
            print("Detect gesture {}, score = {}".format(gesture_type, gesture_score))
        
        # Delay for 500 milliseconds
        time.sleep(0.5)

# Execute setup function
setup()

# Execute loop function
loop()
