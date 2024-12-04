'''
  @file get_pid_vid.py
  @brief Retrieve the device's PID and VID
  @details  This code retrieves and displays the Product ID (PID) and Vendor ID (VID) of the DFRobot GestureFaceDetection sensor. It also shows how to get the number of detected faces.
  @copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT license (MIT)
  @author [fengli](li.feng@dfrobot.com)
  @version  V1.0
  @date  2024-08-09
  @https://github.com/DFRobot/DFRobot_GestureFaceDetection
'''

# -*- coding: utf-8 -*-
import smbus
import time
import serial
import sys
sys.path.append("../")
from python.raspberrypi.DFRobot_GestureFaceDetection import DFRobot_GestureFaceDetection_I2C, DFRobot_GestureFaceDetection_UART

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

def main():
    """
    @brief Main function to retrieve and display PID and VID.
    
    This function retrieves and prints the Product ID (PID) of the DFRobot GestureFaceDetection sensor.
    """
    # Retrieve and print PID
    pid = gfd.read_pid()
    print("PID: {}".format(pid))
    vid = gfd.read_vid()
    print("VID: {}".format(vid))
# Execute the main function if the script is run directly
if __name__ == "__main__":
    main()
