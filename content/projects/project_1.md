+++
title = "Lab 1"
description = "Artemis Basics and Bluetooth Communication"
weight = 1

[extra]
local_image = "/projects/bluetooth.jpg"
+++

Part A
======

The first part of this lab focused on becoming familiar with the Arduino IDE by running example programs on the Artemis MCU.

After installing the Arduino IDE and the Artemis Board Manager, I connected the Artemis to my laptop using a USB-C cable. This allowed me to download and execute code from my laptop onto the Artemis.

Blink
------
The "Blink" program configures the onboard LED in its output state. Then, the program turns the LED on and off by writing a digital 1 and 0 respectively to the LED's pin. To make the LED blink, the code waits one second in between each logic transition. In summary, the LED turns on for one second, then turns off for one second, and then the process repeats. Here is a video of my Artemis executing the blink program:
<br>
<a href="https://www.youtube.com/shorts/u5rFXIvPauA" title="Blinky"><img src="/projects/ArtemisBlink.PNG" alt="Blinky" width="400" height="300" /></a>

Serial
------
The next example program demonstrated how to use the serial monitor with the Arduino IDE. This program uses one of the Artemis' UART peripherals to communicate with my laptop. The program takes the input entered into the Arduino IDE's Serial Monitor and returns an echoed string. Here is an example of various strings being typed into the Serial Monitor. Note that initally I had set the baud rate incorrectly, which is why the mysterious shapes and symbols appear at the top of the serial monitor.
<br>
<img src="/projects/Lab1Serial.jpg" alt="Serial Monitor" width="600" height="400">

Analog Read
------
The third example demonstrated how the Artemis board is capable of reading analog data. The Artemis has an on-board temperature sensor which communicates data with an ADC to convert readings into digital values with a resolution of 16 bits. Here is a video of the temperature readings from the Artemis. Note that as I press on the temperature sensor, the recorded temperature slowly begins to rise.
<br>
<a href="https://youtube.com/shorts/pZ0US88pzos?feature=share" title="Temperature"><img src="/projects/Lab1Temp.PNG" alt="Temperature" width="400" height="300" /></a>

Microphone Output
------
The last example program demonstrated the use of the Artemis' Pulse Density Microphone (PDM). This program computes the FFT of input sound waveforms and prints the highest measured frequency present. Here is a video of the programs output. I used an online tone generator to produce a pure tone of 2830 Hz. When this tone is played, you can see the output of the program print a constant value of 2380 Hz. When the tone is not present, the program outputs the highest frequency of the ambient noise in its environment.
<br>
<a href="https://youtube.com/shorts/4cJIEfhqPyY?feature=share" title="Microphone"><img src="/projects/Lab1Mic.PNG" alt="Microphone" width="400" height="300" /></a>

Part B
======

Overview
------
This section of the lab introduced the Bluetooth Low Energy (BLE) communication protocol. Since the Artemis will eventually be mounted onto my robot, it is important to establish and test a reliable wireless communication channel so that the robot can recieve and execute commands without needing a cable connection to my laptop. The laptop (PC) code is written in Python, while the Artemis code is written in the Ardunio variant of C++.

BLE operates differently than the standard Bluetooth protocol. In BLE, communication functions similar to a community bulletin board. In this model, peripheral devices (such as the Artemis board) provide services that contain multiple characteristics. Characteristics are specific pieces of data that can be read and written to -- for example, a string. Central devices (such as my PC) interact with the "bulletin board" by reading and writing characteristic values. In larger systems, a peripheral's characteristics may be read by multiple central devices. However, this robot's system only uses one peripheral and one central device. Also, although the BLE protocol supports data of up to 512 bytes, this robot has a capacity of 150 bytes. BLE is a good choice for wireless communication systems that don't need to transfer large amounts of data at fast rates. Additionally, since my robot will be powered by an 850 mAH battery, it is important to have a communication protocol that doesn't use a large amount of power.

Setup
------
Although I already had Python installed on my laptop, I needed to establish a virtual environment to ensure that my Python code would run properly. To do this, I created a directory called ./ECE4160 to organize my work for this class, and then entered the following commands into my laptop's shell:

```
python3 -m pip install --user virtualenv
python3 -m venv FastRobots_ble
.\FastRobots_ble\Scripts\activate
```

To develop and execute Python code on my PC, I used a Jupyter Notebook. I launched this process by executing this command into my shell:
```
jupyter lab
```

Lastly, I installed the ArduinoBLE library on the Arduino IDE. This is necessary for the provided Arduino source code files to compile and execute properly. 

Bluetooth Configuration
------
When the Artemis loads the BLE control code, it prints a unique mac address. This is used by the PC to ensure that it communicates only with my Artemis board. In lab settings, there are 20-30 other Artemis boards and robots, as well as numerous other bluetooth devices, so it's important to ensure that I am communicating with my board and not another device. In addition to configuring the MAC address, I also generated a Universally Unique Identifier (UUID) to ensure that my PC would communicate with my specific Artemis Board. This UUID is used in both the PC and Artemis code as seen below:
```
artemis_address: 'C0:81:D5:22:09:64'

ble_service: '15eadab3-adb1-4b42-8cde-012462138583'

characteristics:
  TX_CMD_STRING: '9750f60b-9c9c-4158-b620-02ec9521cd99'

  RX_FLOAT: '27616294-3063-4ecc-b60b-3470ddef2938'
  RX_STRING: 'f235a225-6735-4d73-94cb-ee5dfce9ba83'
```

Tasks
------


