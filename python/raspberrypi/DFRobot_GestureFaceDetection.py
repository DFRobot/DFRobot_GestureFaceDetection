# -*- coding: utf-8 -*-
'''
  @file DFRobot_GestureFaceDetection.py
  @brief Define the basic structure and methods of the DFRobot_GestureFaceDetection class.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT license (MIT)
  @author [thdyyl](yuanlong.yu@dfrobot.com)
  @version  V1.0
  @date  2025-03-17
  @https://github.com/DFRobot/DFRobot_GestureFaceDetection
'''

import serial
import time
from DFRobot_RTU import *
from smbus2 import SMBus, i2c_msg

class DFRobot_GestureFaceDetection(object):
    # Define register address constants
    REG_GFD_ADDR = 0x0000                       #< Device address register
    REG_GFD_BAUDRATE = 0x0001                   #< Baud rate configuration register
    REG_GFD_VERIFY_AND_STOP = 0x0002            #< Parity and stop bits configuration register
    REG_GFD_FACE_THRESHOLD = 0x0003             #< Face detection threshold, X coordinate
    REG_GFD_FACE_SCORE_THRESHOLD = 0x0004       #< Face score threshold
    REG_GFD_GESTURE_SCORE_THRESHOLD = 0x0005    #< Gesture score threshold

    GFD_PID = 0x0272                            #< Product ID
    # Error codes for UART configuration
    ERR_INVALID_BAUD = 0x0001           #< Invalid baud rate
    ERR_INVALID_PARITY = 0x0002         #< Invalid parity setting
    ERR_INVALID_STOPBIT = 0x0004        #< Invalid stop bit
    ERR_CONFIG_BUAD = 0x0010            #< Baud rate configuration failed.
    ERR_CONFIG_PARITY_STOPBIT = 0x0020  #< Failed to configure checksum and stop bit.
    SUCCESS = 0x0000                    #< Operation succeeded

    REG_GFD_PID = 0x0000                        #< Product ID register
    REG_GFD_VID = 0x0001                        #< Vendor ID register
    REG_GFD_HW_VERSION = 0x0002                 #< Hardware version register
    REG_GFD_SW_VERSION = 0x0003                 #< Software version register
    REG_GFD_FACE_NUMBER = 0x0004                #< Number of detected faces
    REG_GFD_FACE_LOCATION_X = 0x0005            #< Face X coordinate
    REG_GFD_FACE_LOCATION_Y = 0x0006            #< Face Y coordinate
    REG_GFD_FACE_SCORE = 0x0007                 #< Face score
    REG_GFD_GESTURE_TYPE = 0x0008               #< Gesture type
    REG_GFD_GESTURE_SCORE = 0x0009              #< Gesture score

    INPUT_REG_OFFSET = 0x06                     #< Input register offset


    EBAUD_1200 = 1         #< Baud rate 1200
    EBAUD_2400 = 2         #< Baud rate 2400
    EBAUD_4800 = 3         #< Baud rate 4800
    EBAUD_9600 = 4         #< Baud rate 9600
    EBAUD_14400 = 5        #< Baud rate 14400
    EBAUD_19200 = 6        #< Baud rate 19200
    EBAUD_38400 = 7        #< Baud rate 38400
    EBAUD_57600 = 8        #< Baud rate 57600
    EBAUD_115200 = 9       #< Baud rate 115200
    EBAUD_230400 = 10      #< Baud rate 230400
    EBAUD_460800 = 11      #< Baud rate 460800
    EBAUD_921600 = 12      #< Baud rate 921600

    UART_CFG_PARITY_NONE = 0      #< No parity
    UART_CFG_PARITY_ODD = 1       #< Odd parity
    UART_CFG_PARITY_EVEN = 2      #< Even parity
    UART_CFG_PARITY_MARK = 3      #< Mark parity
    UART_CFG_PARITY_SPACE = 4     #< Space parity

    UART_CFG_STOP_BITS_0_5 = 0    #< 0.5 stop bits
    UART_CFG_STOP_BITS_1 = 1      #< 1 stop bit
    UART_CFG_STOP_BITS_1_5 = 2    #< 1.5 stop bits
    UART_CFG_STOP_BITS_2 = 3      #< 2 stop bits

    I2C_RETRY_MAX = 3
    def __init__(self):
        # Initialize the class
        pass


    def begin(self):
        '''
          @brief Init function
          @return True if initialization is successful, otherwise false.
        '''
        if self.readInputReg(self.REG_GFD_PID) == self.GFD_PID:
            return True
        return False


    def read_pid(self):
        '''
          @brief Get the device PID
          @return Return the device PID 
        '''
        return self.readInputReg(self.REG_GFD_PID)


    def read_vid(self):
        '''
          @brief Get the device VID
          @return Return the device VID
        '''
        return self.readInputReg(self.REG_GFD_VID)


    def config_uart(self, baud, parity, stop_bit):
        '''
          @brief Configure UART
          @n !!!However, the current CSK6 chip's serial port only supports changing the baud rate, and the stop and check bits should be set to default.
          @param baud Baud rate  EBAUD_1200 ~ EBAUD_921600
          @param parity Parity bit UART_CFG_PARITY_NONE ~ UART_CFG_PARITY_SPACE 
          @param stop_bit Stop bits UART_CFG_STOP_BITS_0_5 ~ UART_CFG_STOP_BITS_2
          @return Return 0 if configuration is successful, otherwise return error code.
        '''
        if (baud < self.EBAUD_1200) or (baud > self.EBAUD_921600):
            return self.ERR_INVALID_BAUD
        if (parity < self.UART_CFG_PARITY_NONE) or (parity > self.UART_CFG_PARITY_SPACE):
            return self.ERR_INVALID_PARITY
        if (stop_bit < self.UART_CFG_STOP_BITS_0_5) or (stop_bit > self.UART_CFG_STOP_BITS_2):
            return self.ERR_INVALID_STOPBIT
        # Set baud rate
        if not self.writeHoldingReg(self.REG_GFD_BAUDRATE, baud):
            return self.ERR_CONFIG_BUAD
        # Set parity and stop bits
        verify_and_stop = (parity << 8) | (stop_bit & 0xff)
        if not self.writeHoldingReg(self.REG_GFD_VERIFY_AND_STOP, verify_and_stop):
            return self.ERR_CONFIG_PARITY_STOPBIT
        return self.SUCCESS
    

    def get_face_number(self):
        '''
          @brief Get the number of detected faces
          @return Return the number of detected faces
        '''
        return self.readInputReg(self.REG_GFD_FACE_NUMBER)


    def get_face_location_x(self):
        '''
          @brief Get the X location of the face
          @return Return the X location
        '''
        return self.readInputReg(self.REG_GFD_FACE_LOCATION_X)


    def get_face_location_y(self):
        '''
          @brief Get the Y location of the face
          @return Return the Y location
        '''
        return self.readInputReg(self.REG_GFD_FACE_LOCATION_Y)


    def get_face_score(self):
        '''
          @brief Get the face score
          @return Return the face score
        '''
        return self.readInputReg(self.REG_GFD_FACE_SCORE)


    def get_gesture_type(self):
        '''
          @brief Get the gesture type
                - 1: LIKE (👍) - blue
                - 2: OK (👌) - green
                - 3: STOP (🤚) - red
                - 4: YES (✌) - yellow
                - 5: SIX (🤙) - purple
          @return Return the gesture type
        '''
        return self.readInputReg(self.REG_GFD_GESTURE_TYPE)


    def get_gesture_score(self):
        '''
          @brief Get the gesture score
          @return Return the gesture score
        '''
        return self.readInputReg(self.REG_GFD_GESTURE_SCORE)


    def set_face_detect_thres(self, score):
        '''
          @brief Set the face detection threshold
          @n Sets the threshold for face detection (0-100). Default is 60%
          @param score Threshold score
        '''
        if (0 >= score) or (score > 100):
            return False
        return self.writeHoldingReg(self.REG_GFD_FACE_SCORE_THRESHOLD, score)


    def get_face_detect_thres(self):
        '''
          @brief Get the face detection threshold
          @n Get the threshold for face detection (0-100). Default is 60%
          @return Return the face detection threshold
        '''
        return self.readHoldingReg(self.REG_GFD_FACE_SCORE_THRESHOLD)


    def set_detect_thres(self, x):
        '''
          @brief Set the x-range for face detection
          @n Sets the threshold for detecting the X coordinate (0-100). Default is 60%.
          @param x Threshold value
        '''
        if (0 >= x) or (x > 100):
            return False
        return self.writeHoldingReg(self.REG_GFD_FACE_THRESHOLD, x)


    def get_detect_thres(self):
        '''
          @brief Get the x-range for face detection
          @n Get the threshold for detecting the X coordinate (0-100). Default is 60%.
          @return Return the x-range for face detection
        '''
        return self.readHoldingReg(self.REG_GFD_FACE_THRESHOLD)


    def set_gesture_detect_thres(self, score):
        '''
          @brief Set the gesture detection threshold
          @n Sets the threshold for gesture detection (0-100). Default is 60%.
          @param score Threshold score
        '''
        if (0 >= score) or (score > 100):
            return False
        return self.writeHoldingReg(self.REG_GFD_GESTURE_SCORE_THRESHOLD, score)


    def get_gesture_detect_thres(self):
        '''
          @brief Get the gesture detection threshold
          @n Get the threshold for gesture detection (0-100). Default is 60%.
          @return Return the threshold for gesture detection
        '''
        return self.readHoldingReg(self.REG_GFD_GESTURE_SCORE_THRESHOLD)


    def set_addr(self, addr):
        '''
          @brief Set the device address
          @param addr Address to set
        '''
        if (addr < 1) or (addr > 0xF7):
            return False
        return self.writeHoldingReg(self.REG_GFD_ADDR, addr)

class DFRobot_GestureFaceDetection_I2C(DFRobot_GestureFaceDetection): 
    def __init__(self, bus, addr):
        # Initialize I2C address and bus
        self.__addr = addr
        self.__i2cbus = bus
        super(DFRobot_GestureFaceDetection_I2C, self).__init__()

    def calculate_crc(self, data):
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc


    def write_reg(self, reg, data):
        '''
          @fn write_reg
          @brief Write data to a register
          @param reg 16-bit register address
          @param data 8-bit register value
        '''
        # Split data into high and low 8 bits and write to I2C register
        val_high_byte = (data >> 8) & 0xFF 
        val_low_byte = data & 0xFF    
        reg_high_byte = (reg >> 8) & 0xFF
        reg_low_byte = reg & 0xFF
        crc = self.calculate_crc([reg_high_byte, reg_low_byte, val_high_byte, val_low_byte])
        for i in range(self.I2C_RETRY_MAX):
            with SMBus(self.__i2cbus) as bus:
                msg = i2c_msg.write(self.__addr, [reg_high_byte, reg_low_byte, val_high_byte, val_low_byte, crc])
                bus.i2c_rdwr(msg)
                time.sleep(0.05) # Because the slave has a clock extension, it needs to wait.
                msg = i2c_msg.read(self.__addr, 3)
                bus.i2c_rdwr(msg)
                data = list(msg)
                ret_data = (data[0] << 8) | data[1]
                if self.calculate_crc(data[:2]) == data[2] and ret_data == crc:
                    return True
        return False

    def read_reg(self, reg, length):
        '''
          @fn read_reg
          @brief Read data from a register
          @param reg 16-bit register address
          @param length Length of data to read
          @return Data read from the register
        '''
        reg_high_byte = (reg >> 8) & 0xFF 
        reg_low_byte = reg & 0xFF         
        crc = self.calculate_crc([reg_high_byte, reg_low_byte])
        for i in range(self.I2C_RETRY_MAX):
            with SMBus(self.__i2cbus) as bus:
                msg = i2c_msg.write(self.__addr, [reg_high_byte, reg_low_byte, crc])
                bus.i2c_rdwr(msg)
                time.sleep(0.02)
                msg = i2c_msg.read(self.__addr, 3)
                bus.i2c_rdwr(msg)
                data = list(msg)
                ret_data = (data[0] << 8) | data[1]
                if self.calculate_crc(data[:length]) == data[length] and ret_data != 0xFFFF:
                    return ret_data
        return 0

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
        ret = self.write_holding_register(self.__addr, reg, data)
        return ret == 0

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
        try:
            data = self.read_holding_registers(self.__addr, reg, 1)
            
            # Ensure data list has at least three elements
            if len(data) >= 3:
                regData = (data[1] << 8) | data[2]
            else:
                regData = 0
            
            return regData
        except Exception as e:
            return 0
