# DFRobot_GestureFaceDetection

* [中文版](./README_CN.md)

GestureFaceDetection is a sensor capable of tracking human head and shoulders and detecting five types of gestures from up to 3 meters away.
   
   
![正反面svg效果图](https://github.com/DFRobot/DFRobot_GestureFaceDetection/raw/master/resources/images/SEN00626svg.png)

## Product Link（https://www.dfrobot.com/)
    SEN0626:Gesture Sensor 

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary


Provide an Arduino library to control 

## Installation

1.To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.
2.Use the library also need to download depends on: https://github.com/DFRobot/DFRobot_RTU
## Methods
```C++

    /**
     * @fn getPid
     * @brief Get the PID of the device.
     * @return PID of the device.
     */
    uint16_t getPid();

    /**
     * @fn getVid
     * @brief Get the VID of the device.
     * @return VID of the device.
     */
    uint16_t getVid();

    /**
     * @fn setDeviceAddr
     * @brief Set the device address.
     * @param addr Device address.
     * @return True if the address is set successfully, otherwise false.
     */
    bool setDeviceAddr(uint16_t addr);

    /**
     * @fn configUart
     * @brief Configure UART settings.
     * 
     * This method is used to set the UART communication parameters for the device, including baud rate, parity, and stop bits. 
     * Users can choose the appropriate parameters based on their needs to ensure stable and effective communication with the device.
     *
     * @param baud Baud rate configuration, of type `eBaudConfig_t`, with possible values including:
     *              - `eBaud_1200`  - 1200 baud
     *              - `eBaud_2400`  - 2400 baud
     *              - `eBaud_4800`  - 4800 baud
     *              - `eBaud_9600`  - 9600 baud
     *              - `eBaud_14400` - 14400 baud
     *              - `eBaud_19200` - 19200 baud
     *              - `eBaud_38400` - 38400 baud
     *              - `eBaud_57600` - 57600 baud
     *              - `eBaud_115200`- 115200 baud
     *              - `eBaud_230400`- 230400 baud
     *              - `eBaud_460800`- 460800 baud
     *              - `eBaud_921600`- 921600 baud
     *
     * @param parity Parity configuration, of type `eParityConfig_t`, with possible values including:
     *              - `UART_CFG_PARITY_NONE`  - No parity
     *              - `UART_CFG_PARITY_ODD`   - Odd parity
     *              - `UART_CFG_PARITY_EVEN`  - Even parity
     *              - `UART_CFG_PARITY_MARK`  - Mark parity
     *              - `UART_CFG_PARITY_SPACE` - Space parity
     *
     * @param stopBit Stop bits configuration, of type `eStopbits_t`, with possible values including:
     *                - `UART_CFG_STOP_BITS_0_5` - 0.5 stop bits
     *                - `UART_CFG_STOP_BITS_1`   - 1 stop bit
     *                - `UART_CFG_STOP_BITS_1_5` - 1.5 stop bits
     *                - `UART_CFG_STOP_BITS_2`   - 2 stop bits
     *
     * @return Status of the configuration, returning the status code if the configuration is successful; otherwise, it returns an error code.
     */
    uint16_t configUart(eBaudConfig_t baud, eParityConfig_t parity, eStopbits_t stopBit);

    /**
     * @fn setFaceDetectThres
     * @brief Set face detection threshold.
     * 
     * Sets the threshold for face detection (0-100). Default is 60%.
     *
     * @param score Threshold value.
     * @return True if successful, otherwise false.
     */
    bool setFaceDetectThres(uint16_t score);
    
    /**
     * @fn setDetectThres
     * @brief Set detection threshold for X coordinate.
     * 
     * Sets the threshold for detecting the X coordinate (0-100). Default is 60%.
     *
     * @param x Threshold value.
     * @return True if successful, otherwise false.
     */
    bool setDetectThres(uint16_t x);
    
    /**
     * @fn setGestureDetectThres
     * @brief Set gesture detection threshold.
     * 
     * Sets the threshold for gesture detection (0-100). Default is 60%.
     *
     * @param score Threshold value.
     * @return True if successful, otherwise false.
     */
    bool setGestureDetectThres(uint16_t score);


    /**
     * @fn getFaceNumber
     * @brief Get the number of faces detected by the device.
     * @return Number of faces detected.
     */
    uint16_t getFaceNumber();

    /**
     * @fn getFaceLocationX
     * @brief Get the X coordinate of the detected face.
     * @return X coordinate of the face.
     */
    uint16_t getFaceLocationX();

    /**
     * @fn getFaceLocationY
     * @brief Get the Y coordinate of the detected face.
     * @return Y coordinate of the face.
     */
    uint16_t getFaceLocationY();

    /**
     * @fn getFaceScore
     * @brief Get the score of the detected face.
     * @return Score of the face.
     */
    uint16_t getFaceScore();  

    /**
     * @fn getGestureType
     * @brief Get the type of detected gesture.
     * 
     * This method retrieves the currently detected gesture type. The gesture recognition feature can be used in various applications, such as human-machine interaction or control systems. 
     * The returned gesture type corresponds to the following values:
     * - 1: LIKE (👍) - blue
     * - 2: OK (👌) - green
     * - 3: STOP (🤚) - red
     * - 4: YES (✌️) - yellow
     * - 5: SIX (🤙) - purple
     * 
     * If no gesture is detected, the return value may be a specific invalid value (e.g., 0).
     *
     * @return The detected gesture type, returning the type identifier for the gesture.
     */
    uint16_t getGestureType();

    /**
     * @fn getGestureScore
     * @brief Get the score of the detected gesture.
     * @return Gesture score.
     */
    uint16_t getGestureScore();  

```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
FireBeetle-ESP32        |      √       |              |             | 
Arduino MEGA2560        |      √       |              |             | 
Arduino Leonardo|      √       |              |             | 
Micro:bit        |      √       |              |             | 
FireBeetle-M0        |      √       |              |             | 
Raspberry Pi      |      √       |              |             | 

## History

- Date 2024-8-1
- Version V0.1
## Credits
Written by fengli(li.feng@dfrobot.com), 2024.7.22 (Welcome to our [website](https://www.dfrobot.com/))
## History