# DFRobot_GestureFaceDetection

* [English Version](./README.md)

GestureFaceDetection是一款可以在最远3米距离跟踪人体头肩和检测五种手势的传感器

![正反面svg效果图](https://github.com/DFRobot/DFRobot_GestureFaceDetection/raw/master/resources/images/SEN0626svg.png)

## 产品链接（https://www.dfrobot.com.cn/)

SEN0626: GestureFaceDetection Sensor 手势传感器

## 目录

* [简介](#简介)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介

提供用于控制GestureFaceDetection传感器的Arduino库。

## 安装

1.要使用此库，首先下载库文件，将其粘贴到 `\Arduino\libraries` 目录中，然后打开示例文件夹并运行其中的示例。
2.使用该库还需下载依赖：https://github.com/DFRobot/DFRobot_RTU
## 方法
```c++
    /**
     * @fn getPid
     * @brief 获取设备的PID。
     * @return 设备的PID。
     */
    uint16_t getPid();
    
    /**
     * @fn getVid
     * @brief 获取设备的VID。
     * @return 设备的VID。
     */
    uint16_t getVid();
    
    /**
     * @fn setDeviceAddr
     * @brief 设置设备地址。
     * @param addr 设备地址。
     * @return 如果成功设置地址，则返回真；否则返回假。
     */
    bool setDeviceAddr(uint16_t addr);
    
    /**
     * @fn configUart
     * @brief 配置UART设置。
     * 
     * 此方法用于设置设备的UART通信参数，包括波特率、校验位和停止位。
     * 用户可以根据需求选择适当的参数，以确保与设备的稳定有效通信。
     *
     * @param baud 波特率配置，类型为`eBaudConfig_t`，可能的值包括：
     *              - `eBaud_1200`  - 1200波特
     *              - `eBaud_2400`  - 2400波特
     *              - `eBaud_4800`  - 4800波特
     *              - `eBaud_9600`  - 9600波特
     *              - `eBaud_14400` - 14400波特
     *              - `eBaud_19200` - 19200波特
     *              - `eBaud_38400` - 38400波特
     *              - `eBaud_57600` - 57600波特
     *              - `eBaud_115200`- 115200波特
     *              - `eBaud_230400`- 230400波特
     *              - `eBaud_460800`- 460800波特
     *              - `eBaud_921600`- 921600波特
     *
     * @param parity 校验位配置，类型为`eParityConfig_t`，可能的值包括：
     *              - `UART_CFG_PARITY_NONE`  - 无校验
     *              - `UART_CFG_PARITY_ODD`   - 奇校验
     *              - `UART_CFG_PARITY_EVEN`  - 偶校验
     *              - `UART_CFG_PARITY_MARK`  - 标记校验
     *              - `UART_CFG_PARITY_SPACE` - 空格校验
     *
     * @param stopBit 停止位配置，类型为`eStopbits_t`，可能的值包括：
     *                - `UART_CFG_STOP_BITS_0_5` - 0.5停止位
     *                - `UART_CFG_STOP_BITS_1`   - 1停止位
     *                - `UART_CFG_STOP_BITS_1_5` - 1.5停止位
     *                - `UART_CFG_STOP_BITS_2`   - 2停止位
     *
     * @return 配置状态，如果配置成功返回状态码；否则返回错误码。
     */
    uint16_t configUart(eBaudConfig_t baud, eParityConfig_t parity, eStopbits_t stopBit);
    
    /**
     * @fn setFaceDetectThres
     * @brief 设置人脸检测阈值。
     * 
     * 设置人脸检测的阈值（0-100）。默认值为60%。
     *
     * @param score 阈值。
     * @return 如果成功，返回真；否则返回假。
     */
    bool setFaceDetectThres(uint16_t score);
    
    /**
     * @fn setDetectThres
     * @brief 设置X坐标的检测阈值。
     * 
     * 设置检测X坐标的阈值（0-100）。默认值为60%。
     *
     * @param x 阈值。
     * @return 如果成功，返回真；否则返回假。
     */
    bool setDetectThres(uint16_t x);
    
    /**
     * @fn setGestureDetectThres
     * @brief 设置手势检测阈值。
     * 
     * 设置手势检测的阈值（0-100）。默认值为60%。
     *
     * @param score 阈值。
     * @return 如果成功，返回真；否则返回假。
     */
    bool setGestureDetectThres(uint16_t score);
    
    /**
     * @fn getFaceNumber
     * @brief 获取设备检测到的人脸数量。
     * @return 检测到的人脸数量。
     */
    uint16_t getFaceNumber();
    
    /**
     * @fn getFaceLocationX
     * @brief 获取检测到的人脸的X坐标。
     * @return 人脸的X坐标。
     */
    uint16_t getFaceLocationX();
    
    /**
     * @fn getFaceLocationY
     * @brief 获取检测到的人脸的Y坐标。
     * @return 人脸的Y坐标。
     */
    uint16_t getFaceLocationY();
    
    /**
     * @fn getFaceScore
     * @brief 获取检测到的人脸的得分。
     * @return 人脸的得分。
     */
    uint16_t getFaceScore();
    
    /**
     * @fn getGestureType
     * @brief 获取检测到的手势类型。
     * 
     * 此方法检索当前检测到的手势类型。手势识别功能可以用于各种应用，如人机交互或控制系统。
     * 返回的手势类型对应以下值：
     * - 1: LIKE (👍) - 蓝色
     * - 2: OK (👌) - 绿色
     * - 3: STOP (🤚) - 红色
     * - 4: YES (✌️) - 黄色
     * - 5: SIX (🤙) - 紫色
     * 
     * 如果没有检测到手势，返回值可能是特定的无效值（例如，0）。
     *
     * @return 检测到的手势类型。
     */
    uint16_t getGestureType();
    
    /**
     * @fn getGestureScore
     * @brief 获取检测到的手势得分。
     * @return 手势得分。
     */
    uint16_t getGestureScore();


```


## 兼容性

MCU                | 表现良好	|表现异常	|未测试	|备注 |
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
FireBeetle-ESP32        |      √       |              |             | 
Arduino MEGA2560        |      √       |              |             | 
Arduino Leonardo|      √       |              |             | 
Micro:bit        |      √       |              |             | 
FireBeetle-M0        |      √       |              |             | 
Raspberry Pi      |      √       |              |             | 

## 历史

- Date 2024-8-1
- Version V0.1


## 贡献者

Written by fengli(li.feng@dfrobot.com), 2024.8.01 (Welcome to our [website](https://www.dfrobot.com/))

## History