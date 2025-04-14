# DFRobot_GestureFaceDetection

* [English Version](./README.md)

GestureFaceDetection是一款可以在最远3米距离跟踪人体头肩和检测五种手势的传感器

![Product Image](../../resources/images/SEN0626svg.png)


## 产品链接（https://www.dfrobot.com/product-2914.html)

  SEN0626:手势识别传感器

## 目录

* [简介](#简介)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介

提供用于控制GestureFaceDetection传感器的python库。

## 安装

要使用这个库，首先将库下载到Raspberry Pi 并下载smbus2

```
pip install smbus2
pip3 install smbus2
```

然后打开用例文件夹。要执行一个用例demox.py，请在命令行中输入python demox.py。例如，要执行get_pid_vid.py用例，你需要输入：

```
python get_pid_vid.py
```

## 方法
```python
  
    def read_pid(self):
    '''
      @brief 获取设备 PID
      @return 返回设备 PID
    '''

    def read_vid(self):
    '''
      @brief 获取设备 VID
      @return 返回设备 VID
    '''

    def get_face_number(self):
    '''
      @brief 获取检测到的面部数量
      @return 返回检测到的面部数量
    '''

    def config_uart(self, baud, parity, stop_bit):
    '''
      @brief 配置 UART
      @param baud 波特率
      @param parity 奇偶校验位
      @param stop_bit 停止位
    '''

    def get_face_location_x(self):
    '''
      @brief 获取面部的 X 位置
      @return 返回 X 位置
    '''

    def get_face_location_y(self):
    '''
      @brief 获取面部的 Y 位置
      @return 返回 Y 位置
    '''

    def get_face_score(self):
    '''
      @brief 获取面部分数
      @return 返回面部分数
    '''

    def get_gesture_type(self):
    '''
      @brief 获取手势类型
             - 1: LIKE (👍) - 蓝色
             - 2: OK (👌) - 绿色
             - 3: STOP (🤚) - 红色
             - 4: YES (✌) - 黄色
             - 5: SIX (🤙) - 紫色
      @return 返回手势类型
    '''

    def get_gesture_score(self):
    '''
      @brief 获取手势分数
      @return 返回手势分数
    '''

    def set_face_detect_thres(self, score):
    '''
      @brief 设置面部检测阈值
      @n 设置面部检测的阈值（0-100）。默认值为 60%
      @param score 阈值分数
    '''

    def set_detect_thres(self, x):
    '''
      @brief 设置检测 X 位置的阈值
      @n 设置检测 X 位置的阈值（0-100）。默认值为 60%。
      @param x 阈值
    '''

    def set_gesture_detect_thres(self, score):
    '''
      @brief 设置手势检测阈值
      @n 设置手势检测的阈值（0-100）。默认值为 60%。
      @param score 阈值分数
    '''

    def get_face_detect_thres(self):
    '''
      @brief 获取面部检测阈值
      @n 获取面部检测的阈值（0-100）。默认值为 60%。
      @return 返回面部检测的阈值
    '''

    def get_detect_thres(self):
    '''
      @brief 获取检测 X 位置的阈值
      @n 获取检测 X 位置的阈值（0-100）。默认值为 60%。
      @return 返回检测 X 位置的阈值
    '''

    def get_gesture_detect_thres(self):
    '''
      @brief 获取手势检测阈值
      @n 获取手势检测的阈值（0-100）。默认值为 60%。
      @return 返回手势检测的阈值
    '''

    def set_addr(self, addr):
    '''
      @brief 设置设备地址
      @param addr 要设置的地址
    '''
```

## 兼容性

| MCU         | Work Well | Work Wrong  | Untested | Remarks |
| ------------ | :--: | :----: | :----: | :--: |
| RaspberryPi4 |  √   |        |        |      |

* Python version 

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :--: | :----: | :----: | ---- |
| Python2 |  √   |        |        |      |
| Python3 |  √   |        |        |      |
## 历史 

- 2025/03/17 - Version 1.0.0 released.

## 贡献者

Written by thdyyl(yuanlong.yu@dfrobot.com), 2025.03.17 (Welcome to our [website](https://www.dfrobot.com/))





