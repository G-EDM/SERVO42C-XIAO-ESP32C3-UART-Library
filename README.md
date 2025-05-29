# SERVO42C-XIAO-ESP32C3-UART-Library
</br></br>
A small library to control the makerbase servo42c via UART from a XIAO ESP32C3. Written in cpp for vstudio with platform.io. For other ESP32 boards the platform.ini file needs to be changed to fit the board.

</br></br>
Flash it with vstudio code and platform.io (extension for vstudio code)
Details to flash it onto the XIAO ESP32C3 can be found in the main.cpp file

ESP RX to MKS42C TX
ESP TX to MKS42C RX
MKS42C UART GND to the 12v GND.
MKS42C UART 3.3v is not needed in this configuration.

12v +- to the pwer input of the MKS42C ( there are on the right of the 6 Pin connector.

</br></br>
For different ESP32 boards pelase adjust the platform.ini file
