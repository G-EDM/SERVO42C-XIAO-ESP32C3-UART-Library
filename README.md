# SERVO42C-XIAO-ESP32C3-UART-Library
</br></br>
A small library to control the makerbase servo42c via UART from a XIAO ESP32C3. Written in cpp for vstudio with platform.io. For other ESP32 boards the platform.ini file needs to be changed to fit the board.

</br></br>
# Other ESP boards
For other board please start with this version:
https://github.com/G-EDM/SERVO42C-ESP32WROOM32-UART-Library

This variant is made for the XIAO ESP32C2 and uses UART1. Using UART1 is not possible without issues on other ESP boards and I don't have time right now to convert this version. 
I think the other version will work with the XIAO board too if the platform.ini is replaced.

</br></br>
Flash it with vstudio code and platform.io (extension for vstudio code)
Details to flash it onto the XIAO ESP32C3 can be found in the main.cpp file

ESP RX to MKS42C TX
ESP TX to MKS42C RX
MKS42C UART GND to the 12v GND.
MKS42C UART 3.3v is not needed in this configuration.

12v +- to the pwer input of the MKS42C ( there are on the right of the 6 Pin connector.

