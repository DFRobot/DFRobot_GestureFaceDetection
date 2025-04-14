'''
  @file config_gesture.py
  @brief Config gestures
  @details  This code configure the location, score of faces, and gestures along with their scores.
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
    
    This function sets the thresholds for face detection and gesture detection.
    """
    # Wait for the sensor to start.
    time.sleep(5)

    while gfd.begin() == False:
        print("Communication with device failed, please check connection")
        time.sleep(1)

    # Set face detection score threshold (0~100)
    if gfd.set_face_detect_thres(60):
        print("Face detection threshold set to 60.")
    else:
        print("Set the face detection threshold fail.")
    # Set gesture detection score threshold (0~100)
    if gfd.set_gesture_detect_thres(60):
        print("Gesture detection threshold set to 60.")
    else:
        print("Set the gesture detection threshold fail.")
    # Set detection range, 0~100
    if gfd.set_detect_thres(100):
        print("Detection range set to maximum.")
    else:
        print("Set the gesture detection range fail.")


# Execute setup function
setup()
