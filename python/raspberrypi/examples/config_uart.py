'''
  @file config_uart.py
  @brief Configure sensor parameters
  @details  This code configures the sensor's address and serial communication parameters.
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

# Define macros (simulated using variables)
USE_I2C = False  # Set to True to use I2C; set to False to use UART

# Define device address and baud rate
DEVICE_ID = 0x72
UART_BAUD_RATE = 9600

# Set a new device address
NEW_DEVICE_ID = 0x72
# Choose between I2C or UART based on the macro
if USE_I2C:
    # Using I2C interface
    gfd = DFRobot_GestureFaceDetection_I2C(bus=1, addr=DEVICE_ID)  # Assuming I2C bus 1 is used
else:
    # Using UART interface
    gfd = DFRobot_GestureFaceDetection_UART(baud=UART_BAUD_RATE, addr=DEVICE_ID)

def setup():
    """
    @brief Setup function for initializing the sensor and communication.
    
    This function initializes the sensor and configures the communication settings
    based on the selected interface (I2C or UART).
    """
    # Wait for the sensor to start.
    time.sleep(5)
    while gfd.begin() == False:
        print("Communication with device failed, please check connection")
        time.sleep(1)

    if USE_I2C:
        # I2C does not require additional initialization
        print("I2C setup complete.")
    else:
        # Configure UART communication
        if gfd.config_uart(baud=gfd.EBAUD_115200, parity=gfd.UART_CFG_PARITY_NONE, stop_bit=gfd.UART_CFG_STOP_BITS_1) == gfd.SUCCESS:
            print("UART configured success.")
        else:
            print("UART configuration failed.")
        # Set the device address
        if gfd.set_addr(NEW_DEVICE_ID):
            print("Device address set to: 0x{:x}".format(NEW_DEVICE_ID))
        else:
            print("Failed to set device address.")
# Execute the setup function
setup()

