# -*- coding: utf-8 -*-
'''
  @file DFRobot_GestureFaceDetection.py
  @brief Define the basic structure and methods of the DFRobot_GestureFaceDetection class.
  @copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT license (MIT)
  @author [fengli](li.feng@dfrobot.com)
  @version  V1.0
  @date  2024-8-09
  @https://github.com/DFRobot/DFRobot_GestureFaceDetection
'''

import serial
import time
import smbus
from DFRobot_RTU import *

class DFRobot_GestureFaceDetection(object):
    # Define register address constants
    REG_GFD_ADDR = 0x0000
    REG_GFD_BAUDRATE = 0x0001
    REG_GFD_VERIFY_AND_STOP = 0x0002
    REG_GFD_FACE_THRESHOLD = 0x0003
    REG_GFD_FACE_SCORE_THRESHOLD = 0x0004
    REG_GFD_GESTURE_SCORE_THRESHOLD = 0x0005

    REG_GFD_PID = 0x0000
    REG_GFD_VID = 0x0001
    REG_GFD_HW_VERSION = 0x0002
    REG_GFD_SW_VERSION = 0x0003
    REG_GFD_FACE_NUMBER = 0x0004
    REG_GFD_FACE_LOCATION_X = 0x0005
    REG_GFD_FACE_LOCATION_Y = 0x0006
    REG_GFD_FACE_SCORE = 0x0007
    REG_GFD_GESTURE_TYPE = 0x0008
    REG_GFD_GESTURE_SCORE = 0x0009
    INPUT_REG_OFFSET = 0x06

    def __init__(self):
        # Initialize the class
        pass

    '''
      @brief Get the device PID
      @return Returns the device PID
    '''
    def read_pid(self):
        return self.readInputReg(self.REG_GFD_PID)

    '''
      @brief Get the device VID
      @return Returns the device VID
    '''
    def read_vid(self):
        return self.readInputReg(self.REG_GFD_VID)

    '''
      @brief Get the number of detected faces
      @return Returns the number of detected faces
    '''
    def get_face_number(self):
        return self.readInputReg(self.REG_GFD_FACE_NUMBER)

    '''
      @brief Configure UART
      @param baud Baud rate
      @param parity Parity bit
      @param stop_bit Stop bits
    '''
    def config_uart(self, baud, parity, stop_bit):
        # Combine parity and stop bits into a 16-bit value
        verify_and_stop = (parity << 8) | (stop_bit & 0xff)
        # Set baud rate
        self.writeHoldingReg(self.REG_GFD_BAUDRATE, baud)
        # Set parity and stop bits
        return self.writeHoldingReg(self.REG_GFD_VERIFY_AND_STOP, verify_and_stop)

    '''
      @brief Get the X location of the face
      @return Returns the X location
    '''
    def get_face_location_x(self):
        return self.readInputReg(self.REG_GFD_FACE_LOCATION_X)

    '''
      @brief Get the Y location of the face
      @return Returns the Y location
    '''
    def get_face_location_y(self):
        return self.readInputReg(self.REG_GFD_FACE_LOCATION_Y)

    '''
      @brief Get the face score
      @return Returns the face score
    '''
    def get_face_score(self):
        return self.readInputReg(self.REG_GFD_FACE_SCORE)

    '''
      @brief Get the gesture type
             - 1: LIKE (👍) - blue
             - 2: OK (👌) - green
             - 3: STOP (🤚) - red
             - 4: YES (✌) - yellow
             - 5: SIX (🤙) - purple
      @return Returns the gesture type
    '''
    def get_gesture_type(self):
        return self.readInputReg(self.REG_GFD_GESTURE_TYPE)

    '''
      @brief Get the gesture score
      @return Returns the gesture score
    '''
    def get_gesture_score(self):
        return self.readInputReg(self.REG_GFD_GESTURE_SCORE)

    '''
      @brief Set the face detection threshold
      @n Sets the threshold for face detection (0-100). Default is 60%
      @param score Threshold score
    '''
    def set_face_detect_thres(self, score):
        return self.writeHoldingReg(self.REG_GFD_FACE_THRESHOLD, score)

    '''
      @brief Set the face score threshold
      @n Sets the threshold for detecting the X coordinate (0-100). Default is 60%.
      @param x Threshold value
    '''
    def set_detect_thres(self, x):
        return self.writeHoldingReg(self.REG_GFD_FACE_SCORE_THRESHOLD, x)

    '''
      @brief Set the gesture detection threshold
      @n Sets the threshold for gesture detection (0-100). Default is 60%.
      @param score Threshold score
    '''
    def set_gesture_detect_thres(self, score):
        return self.writeHoldingReg(self.REG_GFD_GESTURE_SCORE_THRESHOLD, score)

    '''
      @brief Set the device address
      @param addr Address to set
    '''
    def set_addr(self, addr):
        return self.writeHoldingReg(self.REG_GFD_ADDR, addr)

class DFRobot_GestureFaceDetection_I2C(DFRobot_GestureFaceDetection): 
    def __init__(self, bus, addr):
        # Initialize I2C address and bus
        self.__addr = addr
        self.i2cbus = smbus.SMBus(bus)
        super(DFRobot_GestureFaceDetection_I2C, self).__init__()

    '''
      @fn write_reg
      @brief Write data to a register
      @param reg 16-bit register address
      @param data 8-bit register value
    '''
    def write_reg(self, reg, data):
        # Split data into high and low 8 bits and write to I2C register
        val_high_byte = (data >> 8) & 0xFF 
        val_low_byte = data & 0xFF         
        self.i2cbus.write_i2c_block_data(self.__addr, reg, [val_high_byte, val_low_byte])
        return 0

    '''
      @fn read_reg
      @brief Read data from a register
      @param reg 16-bit register address
      @param length Length of data to read
      @return Data read from the register
    '''
    def read_reg(self, reg, length):
        val_high_byte = (reg >> 8) & 0xFF 
        val_low_byte = reg & 0xFF         
        self.i2cbus.write_i2c_block_data(self.__addr, val_high_byte, [val_low_byte])
        time.sleep(0.15)
        data = self.i2cbus.read_i2c_block_data(self.__addr, reg, length)
        time.sleep(0.1)
        return (data[0] << 8) | data[1]

    def writeHoldingReg(self, reg, data):
        return self.write_reg(reg, data)

    def readInputReg(self, reg):
        return self.read_reg(self.INPUT_REG_OFFSET + reg, 2)

    def readHoldingReg(self, reg):
        return self.read_reg(reg, 2)

class DFRobot_GestureFaceDetection_UART(DFRobot_GestureFaceDetection, DFRobot_RTU): 
    def __init__(self, baud, addr):
        # Initialize UART baud rate and address
        self.__baud = baud
        self.__addr = addr
        DFRobot_GestureFaceDetection.__init__(self)
        DFRobot_RTU.__init__(self, baud, 8, 'N', 1)

    def writeHoldingReg(self, reg, data):
        return self.write_holding_register(self.__addr, reg, data)

    def readInputReg(self, reg):
        try:
            data = self.read_input_registers(self.__addr, reg, 1)
            
            # Ensure data list has at least three elements
            if len(data) >= 3:
                regData = (data[1] << 8) | data[2]
            else:
                regData = 0
            
            return regData
        
        except Exception as e:
            return 0
    
    def readHoldingReg(self, reg):
        return self.read_holding_registers(self.__addr, reg, 1)
