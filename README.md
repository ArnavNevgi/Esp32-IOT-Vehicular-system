# Esp32-IOT-Vehicular-system

This repository contains a complete implementation of a Vehicular IoT system designed using ESP32 microcontrollers, nRF24L01+ RF modules, and Microsoft Azure IoT cloud services. The goal of the project is to enable real-time monitoring of critical parameters within a vehicle using onboard sensors, cloud integration, and dashboard-based data visualization.

## Project Objective

The system is designed to collect real-time data from multiple sensors, transmit the data wirelessly using a custom client-server architecture, and visualize it through a cloud-connected dashboard. It also includes mechanisms for local alerts in case of abnormal or emergency conditions.

## Key Features

- ESP32-based two-layer architecture: client handles sensor input, server handles cloud communication.
- Wireless data transfer via nRF24L01+ using the SPI protocol.
- Secure and encrypted communication with Azure IoT Hub using MQTT or HTTPS.
- Real-time dashboard visualization using Azure Power BI.
- On-device alerts using buzzer and LED for critical sensor readings.
- Modular design allows easy addition of new sensors or nodes.

## Hardware Setup

| Component              | Description                                   |
|------------------------|-----------------------------------------------|
| ESP32 (Client)         | Collects and preprocesses data from sensors   |
| ESP32 (Server)         | Receives data from client and sends to cloud  |
| nRF24L01+              | RF module for wireless communication          |
| DHT11 (x2)             | Temperature and humidity sensors              |
| Accelerometer          | Detects motion and orientation                |
| PIR Sensor             | Detects motion in surroundings                |
| Shock Sensor           | Detects vibration or crash impact             |
| Flame Sensor           | Detects fire or flame presence                |
| Buzzer + LED           | Provides local alerts                         |

## System Architecture

Client ESP32 collects data from all sensors and transmits the processed data using nRF24L01+ to a server ESP32. The server acts as a bridge to the cloud, uploading data to Microsoft Azure IoT Hub. Azure services such as Stream Analytics, Data Storage, and Power BI are used for anomaly detection, historical data logging, and real-time dashboard visualization.

## Workflow Diagram

Sensor Layer → Client ESP32  
Client ESP32 → nRF24L01+ → Server ESP32  
Server ESP32 → Azure IoT Hub → Azure Stream Analytics / Azure Power BI 
