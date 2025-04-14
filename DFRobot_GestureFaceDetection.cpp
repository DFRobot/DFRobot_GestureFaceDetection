/*!
 *@file DFRobot_GestureFaceDetection.cpp
 *@brief Define the basic structure of class DFRobot_GestureFaceDetection, the implementation of basic methods.
 *@details this module is used to identify the information in the QR code
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@License     The MIT License (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2025-04-02
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
 */

#include "DFRobot_GestureFaceDetection.h"
DFRobot_GestureFaceDetection::DFRobot_GestureFaceDetection()
{
}
bool DFRobot_GestureFaceDetection::begin()
{
    if (reaInputdReg(REG_GFD_PID) == GFD_PID)
    {
        return true;
    }
    return false;
}
uint16_t DFRobot_GestureFaceDetection::getPid()
{
    return reaInputdReg(REG_GFD_PID);
}
uint16_t DFRobot_GestureFaceDetection::getVid()
{
    return reaInputdReg(REG_GFD_VID);
}

uint16_t DFRobot_GestureFaceDetection::getFaceNumber()
{
    return reaInputdReg(REG_GFD_FACE_NUMBER);
}
uint16_t DFRobot_GestureFaceDetection::configUart(eBaudConfig_t baud, eParityConfig_t parity, eStopbits_t stopBit)
{
    uint16_t ret = 0;
    if(baud < eBaud_1200 || baud >= eBaud_MAX){
        ret |= ERR_INVALID_BAUD;
    }
    if(static_cast<uint8_t>(parity) >= UART_CFG_PARITY_MAX){
        ret |= ERR_INVALID_PARITY;
    }
    if(static_cast<uint8_t>(stopBit) >= UART_CFG_STOP_MAX){
        ret |= ERR_INVALID_STOPBIT;
    }
    if(ret != 0){
        return ret;
    }
    uint16_t baudRate = baud;

    uint16_t verifyAndStop = ((uint16_t)parity << 8) | ((uint16_t)stopBit & 0xff);
    ret |= writeIHoldingReg(REG_GFD_BAUDRATE, baudRate)? SUCCESS : ERR_CONFIG_BUAD;
    ret |= writeIHoldingReg(REG_GFD_VERIFY_AND_STOP, verifyAndStop)? SUCCESS: ERR_CONFIG_PARITY_STOPBIT;
    return ret;
}

uint16_t DFRobot_GestureFaceDetection::getFaceLocationX()
{
    return reaInputdReg(REG_GFD_FACE_LOCATION_X);
}

uint16_t DFRobot_GestureFaceDetection::getFaceLocationY()
{
    return reaInputdReg(REG_GFD_FACE_LOCATION_Y);
}
uint16_t DFRobot_GestureFaceDetection::getFaceScore()
{
    return reaInputdReg(REG_GFD_FACE_SCORE);
}
uint16_t DFRobot_GestureFaceDetection::getGestureType()
{
    return reaInputdReg(REG_GFD_GESTURE_TYPE);
}
uint16_t DFRobot_GestureFaceDetection::getGestureScore()
{

    return reaInputdReg(REG_GFD_GESTURE_SCORE);
}

bool DFRobot_GestureFaceDetection::setFaceDetectThres(uint16_t score)
{
    if (score > 100)
    {
        return false;
    }
    return writeIHoldingReg(REG_GFD_FACE_SCORE_THRESHOLD, score);
}
uint16_t DFRobot_GestureFaceDetection::getFaceDetectThres()
{
    return readHoldingReg(REG_GFD_FACE_SCORE_THRESHOLD);
}
bool DFRobot_GestureFaceDetection::setDetectThres(uint16_t x)
{
    if (x > 100)
    {
        return false;
    }
    return writeIHoldingReg(REG_GFD_FACE_THRESHOLD, x);
}
uint16_t DFRobot_GestureFaceDetection::getDetectThres(){
    return readHoldingReg(REG_GFD_FACE_THRESHOLD);
}
bool DFRobot_GestureFaceDetection::setGestureDetectThres(uint16_t score)
{
    if (score > 100)
    {
        return false;
    }
    return writeIHoldingReg(REG_GFD_GESTURE_SCORE_THRESHOLD, score);
}
uint16_t DFRobot_GestureFaceDetection::getGestureDetectThres(){
    return readHoldingReg(REG_GFD_GESTURE_SCORE_THRESHOLD);
}
bool DFRobot_GestureFaceDetection::setDeviceAddr(uint16_t addr)
{
    if ((addr == 0) || (addr > 0xF7))
    {
        return false;
    }
    return writeIHoldingReg(REG_GFD_ADDR, addr);
}

DFRobot_GestureFaceDetection_UART::DFRobot_GestureFaceDetection_UART(Stream *s_, uint8_t addr)
    : DFRobot_RTU(s_)
{
    _addr = addr;
}

bool DFRobot_GestureFaceDetection_UART::begin()
{
    return DFRobot_GestureFaceDetection::begin();
}

uint16_t DFRobot_GestureFaceDetection_UART::reaInputdReg(uint16_t reg)
{
    delay(20);
    return readInputRegister(_addr, reg) ;
}
uint16_t DFRobot_GestureFaceDetection_UART::readHoldingReg(uint16_t reg)
{
    delay(20);
    return readHoldingRegister(_addr, reg);
}
bool DFRobot_GestureFaceDetection_UART::writeIHoldingReg(uint16_t reg, uint16_t data)
{

    delay(20);
    uint16_t ret = writeHoldingRegister(_addr, reg, data);
    LDBG(ret);

    return ret == 0;
}

DFRobot_GestureFaceDetection_I2C::DFRobot_GestureFaceDetection_I2C(uint8_t addr)
{
    _addr = addr;
}

bool DFRobot_GestureFaceDetection_I2C::begin(TwoWire *pWire)
{

    _pWire = pWire;
    pWire->setClock(100000);
    pWire->begin();
    pWire->setClock(100000);
    return DFRobot_GestureFaceDetection::begin();
}
uint8_t DFRobot_GestureFaceDetection_I2C::calculate_crc(const uint8_t *data, size_t length)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
        crc &= 0xFF;
    }
    return crc;
}

bool DFRobot_GestureFaceDetection_I2C::writeReg(uint16_t reg, uint16_t data)
{
    uint8_t retry = 0;
    const uint8_t max_retry = 3;
    bool success = false;
    uint8_t crc_datas[] = {(uint8_t)(reg >> 8),
                           (uint8_t)(reg & 0xFF),
                           (uint8_t)(data >> 8),
                           (uint8_t)(data & 0xFF)};
    uint8_t crc = calculate_crc(crc_datas, 4);
    do
    {
        _pWire->beginTransmission(_addr);
        _pWire->write((uint8_t)(reg >> 8));
        _pWire->write((uint8_t)(reg & 0xFF));
        _pWire->write((uint8_t)(data >> 8));
        _pWire->write((uint8_t)(data & 0xFF));
        _pWire->write(crc);
        uint8_t i2c_error = _pWire->endTransmission();
        if (i2c_error != 0)
        {
            retry++;
            delay(10);
            continue;
        }
        #if defined(ESP32)
        delay(70);
        #else
        delay(5);
        #endif
        uint8_t bytes_read = _pWire->requestFrom(_addr, (uint8_t)3);
        if (bytes_read != 3)
        {
            retry++;
            continue;
        }
        uint8_t redatas[] = {(uint8_t)_pWire->read(), (uint8_t)_pWire->read(), (uint8_t)_pWire->read()};
        uint8_t re_crc = calculate_crc(redatas, 2);
        if (re_crc != redatas[2] || ((redatas[0] << 8) | redatas[1]) != crc)
        {
            retry++;
        }
        else
        {
            success = true;
            break;
        }
    } while (retry < max_retry);

    return success;
}
uint16_t DFRobot_GestureFaceDetection_I2C::readReg(uint16_t reg)
{
    const uint8_t max_retry = 3;
    uint8_t retry = 0;
    uint16_t value = 0xFFFF;
    uint8_t crc_datas[] = {(uint8_t)(reg >> 8),
                           (uint8_t)(reg & 0xFF)};
    uint8_t crc = calculate_crc(crc_datas, 2);
    do
    {
        _pWire->beginTransmission(_addr);
        _pWire->write((uint8_t)(reg >> 8));
        _pWire->write((uint8_t)(reg & 0xff));
        _pWire->write(crc);
        uint8_t i2c_error = _pWire->endTransmission();
        if (i2c_error != 0)
        {
            retry++;
            continue;
        }
        delay(5);

        uint8_t bytes_read = _pWire->requestFrom(_addr, (uint8_t)3);

        if (bytes_read != 3)
        {
            retry++;
            continue;
        }
        uint8_t redatas[] = {(uint8_t)_pWire->read(), (uint8_t)_pWire->read(), (uint8_t)_pWire->read()};
        uint8_t re_crc = calculate_crc(redatas, 2);
        uint16_t data = (redatas[0] << 8) | redatas[1];
        if (data == 0xFFFF || re_crc != redatas[2])
        {
            retry++;
            continue;
        }
        value = data;
        break;
    } while (retry < max_retry);
    return value;
}
bool DFRobot_GestureFaceDetection_I2C::writeIHoldingReg(uint16_t reg, uint16_t data)
{

    return writeReg(reg, data);
}
uint16_t DFRobot_GestureFaceDetection_I2C::reaInputdReg(uint16_t reg)
{
    return readReg(INPUT_REG_OFFSET + reg);
}

uint16_t DFRobot_GestureFaceDetection_I2C::readHoldingReg(uint16_t reg)
{
    return readReg(reg);
}
