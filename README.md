# ESP32-MI48-ToF Example Projects

This repository contains the example projects for the ESP32-MI48-ToF development kit from [Meridian Innovation](https://www.meridianinno.com/).

Currently, all projects are Arduino-based.

# ESP32-MI48-ToF Development Kit

The development kit is composed of the following major hardware components:
* NodeMCU-32S
* MI48 Core Board
* VL53L1X ToF Sensor Module
* ST7735-based 160x128 TFT LCD Screen

# Development Environment Setup

The following details the steps required to setup a development environment for this development kit. This is written for Windows users with minimal experience with Arduino.

## Download and Installing Arduino IDE

1. Download the [Arduino IDE](https://www.arduino.cc/en/Guide/Windows)
1. Run the downloaded file to install Arduino

## Configuring the Arduino IDE for the ESP32-based NodeMCU-32S
By default, the Arduino IDE does not support ESP32-based platforms. Some configuration is needed:
1. Launch the Arduino IDE. Select **File -> Preferences**.
1. In the *Preferences* window, enter "https://dl.espressif.com/dl/package_esp32_index.json" into the *Additional Boards Manager URLs* textbox.
1. Press *OK* to close the *Preferences* window.
1. Select **Tools -> Board: -> Board Manager...**
1. In the *Boards Manager* window, enter *esp32* into the *Filter your search...* textbox.
1. Select the option for **esp32 by Espressif Systems**. Install **version 1.0.2** of this board package.
1. After the installation finishes, press *Close* to close the Boards Manager window.
1. In the Arduino IDE, select:
  1. **Tools -> Board: NodeMCU-32S** to choose the correct board variant.
  1. **Tools -> Upload Speed: 921600**
  1. **Tools -> Flash Frequency: 80MHz**

## Configuring the Arduino IDE for the VL53L1X ToF Sensor Module

The VL53L1X Laser Ranging Time-of-Flight (ToF) Distance Measurement Sensor is developed by [ST Microelectronics](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html).

All code in this repository is based on this driver: https://github.com/pololu/vl53l1x-arduino

To setup the driver:
1. In the Arduino IDE, select **Sketch -> Include Library -> Manage Libraries"**.
1. In the *Library Manager*, enter *VL53L1X* into the *Filter your search...* textbox.
1. Select the option for **VL53L1X by Pololu**. Install **version 1.0.1** of this library.
1. After the installation finishes, press *Close* to close the *Library Manager* window.

## Configuring the Arduino IDE for the 160x128 TFT LCD Screen

All code in this repository is based on this driver: https://github.com/Bodmer/TFT_eSPI

To setup the driver:
1. In the Arduino IDE, select **Sketch -> Include Library -> Manage Libraries".
1. In the *Library Manager*, enter *TFT_eSPI* into the *Filter your search...* textbox.
1. Select the option for **TFT_eSPI by Bodmer**. Install **version 1.4.20** of this library.
1. After the installation finishes, press *Close* to close the *Library Manager* window.

After the above steps, the configuration of the Arduino IDE is finished.

# Compiling and Downloading the Code

To import the Arduino code into the Arduino IDE, select **File -> Open...** then select whichever file with suffix *.ino* in the file dialog.

Note that `User_Setup.h` is not directly `#include`'d by the code, but by the `TFT_eSPI` library. Follow the steps below:
1. Make a backup of the default `User_Setup.h` file from the `TFT_eSPI` library, which (by default for Windows) is located at `C:\Users\%USER%\Documents\Arduino\libraries\TFT_eSPI\User_Setup.h`
1. Copy the `User_Setup.h` into the above location

To compile the code, press *Verify* button in the Arduino IDE. It is located in the top left with a "tick" symbol.

To download the compiled code onto the development kit:
1. Connect a USB cable to the development kit
1. Confirm the settings in the **Tools** menu is as stated above
1. Confirm the correct port is selected in **Tools -> Port**
1. Press the *Upload* button in the Arduino IDE.
1. **Press and release the middle button repeatedly** until the download starts.
