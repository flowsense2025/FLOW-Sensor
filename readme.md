# Firmware and Electronics for the FLOW device

## Electronics
The device will use the Seeed Studio XIAO SAMD21 chip to read the flow sensor and communicate with the smartphone app via Bluetooth Low Energy

https://wiki.seeedstudio.com/Seeeduino-XIAO/

## Firmware
We will use the Espressif SDK alongside the VSCode extension 

https://github.com/espressif

### Setup Instructions

Please follow the setup instructions laid out in this video to install the toolchain (v5.4.0)

https://www.youtube.com/watch?v=XDDcS7HQNlI


### Build Instructions
Open VSCode int the flow_firmware directory 
* Open the command palette (Ctrl + Shift + P) 
   * Select ESP-IDF: Build Your Project
   * Run ESD-IDF: Flash (UART) Your Project
    
