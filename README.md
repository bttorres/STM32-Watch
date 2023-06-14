# This... is My STM32 Watch

My STM32 Watch is a custom designed watch with a TFT display.

[Video Demo](https://youtu.be/s8slsMMHKZk)

## The Code

All of the code was written in C language with the STM32CubeIDE.

## The Main Components and Their Functions:

In this repository is a PDF of the schematic depicting the components and how they are connected.
* ST7789 TFT LCD
  * Displays the watch faces
* 220mAh 402030 LiPo
  * Powers the PCB and TFT display
* STM32F103C8T6
  * Microcontroller for processing
* USB
  * Charges the LiPo
* TPAP2112K-3.3TRG1
  * Lowers the voltage from the lipo battery to 3.3V
* MCP73831T-2ACI/OT
  * Allows the USB port to charge the LiPo
* TS24CA
  * Buttons for user input
* SSSS811101
  * Boot switch for programming
* 2x2 Header pins
  * Connects the ST-Link V2 for programming

## More About the Watch

My STM32 Watch has an analog face and a digital face. The time can be changed and set in either of the faces. The battery, PCB, and display is housed in a 3D printed body that I designed. The watch band is a 20mm pin style band.

