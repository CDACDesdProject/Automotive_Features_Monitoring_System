# Automotive Features Monitoring System

## Project Overview
This project focuses on monitoring automotive environmental parameters (temperature & pressure) using the CAN communication protocol and visualizing the data online via AWS IoT Core. It also provides real-time data display on an OLED screen.

## Features
- **BMP280 Sensor Integration**: Captures temperature & pressure data using I2C.
- **CAN Communication**: Transfers data between STM32 microcontrollers.
- **ESP32 Cloud Connectivity**: Sends data to AWS IoT Core.
- **OLED Display**: Local real-time visualization.
- **Multiple Implementation Approaches**:
  - CAN communication using Bare Metal programming.
  - CAN communication using STM32 HAL API.

## Repository Structure
```
Automotive_Features_Monitoring_System/
│── CAN_Bare_Metal/                  # Low-level CAN communication implementation
│── ESP32_Cloud_Gateway/             # ESP32 module for cloud data transmission via MQTT
│── STM32_Display_Node/              # STM32 node handling OLED display for real-time data
│── STM32_Sensor_Gateway/            # STM32 node acquiring sensor data and transmitting via CAN
│── STM32F407VGT6_HAL_API_Based_CAN/ # HAL API-based CAN protocol implementation for STM32
│── README.md                        # Project documentation
```

## Hardware Components
- **Microcontrollers**: STM32F407VGT6 (Two units)
- **Sensors**: BMP280 (Temperature & Pressure)
- **Communication Modules**:
  - CAN Transceivers (for STM32 to STM32 communication)
  - ESP32 (for cloud connectivity via MQTT)
- **Display**: OLED

## Communication Protocols & Wiring Details
- **I2C**: BMP280 → First STM32 Board  
  - SDA (BMP280) → PB7 (STM32)
  - SCL (BMP280) → PB6 (STM32)
  - VCC → 3.3V, GND → GND
- **CAN**: First STM32 → Second STM32  
  - CAN_H → CAN_H
  - CAN_L → CAN_L
  - Requires external CAN transceivers (e.g., MCP2551 or TJA1050)
- **UART**: Second STM32 → ESP32  
  - TX (STM32) → RX (ESP32)
  - RX (STM32) → TX (ESP32)
  - GND → GND
- **MQTT (via WiFi)**: ESP32 → AWS IoT Core  
  - Requires WiFi credentials and AWS IoT Core setup
- **OLED (I2C)**: OLED Display → STM32  
  - SDA → PB9, SCL → PB8

## Code Details
### 1. BMP280 Sensor Interfacing (I2C)
- Configures BMP280 over I2C to fetch temperature and pressure data.
- Uses STM32 HAL library for I2C communication.
- Data is transmitted to another STM32 board via CAN.

### 2. CAN Communication (Bare Metal & HAL API)
- **Bare Metal**: Direct register-level programming for CAN communication.
- **HAL API**: Uses STM32Cube HAL library for CAN setup and data transfer.
- Data is forwarded from the first STM32 to the second STM32 over CAN.

### 3. ESP32 Cloud Connectivity (AWS IoT Core via MQTT)
- Receives processed data over UART from STM32.
- Transmits data to AWS IoT Core using MQTT protocol.

### 4. OLED Display Integration
- Displays real-time data received from CAN communication.
- Uses I2C communication to interface with STM32.

## How to Use
1. **Setup Hardware Connections**
   - Follow the wiring details mentioned above.
   - Ensure CAN transceivers are correctly connected.
   - Connect ESP32 to WiFi and AWS IoT Core.

2. **Flash Firmware**
   - Compile and flash respective codes onto STM32 and ESP32.
   - Use STM32CubeIDE for STM32 firmware.
   - Use Arduino IDE for ESP32 firmware.

3. **Monitor Data**
   - Check OLED display for real-time data.
   - Observe data transmission on AWS IoT Core.

## Future Improvements
- Implement advanced error handling in CAN communication.
- Optimize power consumption for embedded systems.
- Enhance real-time visualization with graphical representations.


