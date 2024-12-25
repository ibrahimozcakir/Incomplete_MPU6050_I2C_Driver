### README.md


## STATUS

**This driver is a work in progress.** 
This driver is currently under development and is not yet fully completed. 
Some features may not be implemented or tested. Updates and improvements will be made in the near future. Please check for new releases!

---

# MPU6050 I2C Driver

This repository contains an I2C driver for the MPU6050 sensor. The driver allows users to configure the sensor, read data from its accelerometer, gyroscope, and temperature sensor, and manage its operational modes.

---

## Features

- Initialize and configure the MPU6050 sensor.
- Read raw and processed accelerometer, gyroscope, and temperature data.
- Manage power modes, including sleep and reset functionality.
- Enable or disable interrupts.
- Configure sensor ranges for gyroscope and accelerometer.

---

## Getting Started

### Prerequisites

- A microcontroller or development board with I2C support.
- MPU6050 sensor module.
- Proper connection of the sensor's SDA and SCL pins to the microcontroller's I2C pins.

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/mpu6050-driver.git
   ```
2. Include the driver files in your project.
3. Ensure you have the required header files and dependencies for I2C communication.

---

## Usage

### Step 1: Create a Sensor Handle
Create a `SensorHandle_t` object to manage the MPU6050's state:
```c
SensorHandle_t sensor;
```

### Step 2: Initialize the Sensor
Use the `MPU6050_Init` function to initialize the sensor with your desired configurations:
```c
SensorStatus_t status;
status = MPU6050_Init(&sensor, FS_SEL_500, AFS_SEL_4G);
```
- Replace `FS_SEL_500` and `AFS_SEL_4G` with your preferred gyroscope and accelerometer ranges.

### Step 3: Read Sensor Data
You can read specific data from the sensor or all available data:
```c
status = MPU6050_ReadDataAll(&sensor);
```
This will populate the `sensor` object with:
- Raw and processed accelerometer data.
- Raw and processed gyroscope data.
- Raw and converted temperature data.

### Additional Features
- **Enable Interrupts**:
  ```c
  MPU6050_InterruptEnable(INT_LEVEL_HIGH, INT_OPEN_DRAIN);
  ```
- **Disable Interrupts**:
  ```c
  MPU6050_InterruptDisable();
  ```
- **Set Sleep Mode**:
  ```c
  MPU6050_SleepMode(SleepMode_ON);
  ```
- **Reset Device**:
  ```c
  MPU6050_DeviceReset();
  ```

---

## API Reference

### Data Types
- `SensorHandle_t`: Structure to manage sensor data and configurations.
- `SensorStatus_t`: Enum to represent sensor operation statuses.

### Key Functions
| Function                              | Description                                    |
|---------------------------------------|------------------------------------------------|
| `MPU6050_Init`                        | Initializes the sensor.                       |
| `MPU6050_ControlStatus`               | Checks if the sensor is ready.                |
| `MPU6050_ControlId`                   | Verifies the sensor's ID.                     |
| `MPU6050_SleepMode`                   | Toggles sleep mode.                           |
| `MPU6050_DeviceReset`                 | Resets the sensor.                            |
| `MPU6050_SetAccRange`                 | Configures accelerometer range.               |
| `MPU6050_SetGyroRange`                | Configures gyroscope range.                   |
| `MPU6050_ReadDataAll`                 | Reads all sensor data (accelerometer, gyroscope, temperature). |
| `MPU6050_ReadDataAcc`                 | Reads accelerometer data only.                |
| `MPU6050_ReadDataGyro`                | Reads gyroscope data only.                    |
| `MPU6050_ReadDataTemp`                | Reads temperature data only.                  |
| `MPU6050_InterruptEnable`             | Enables sensor interrupts.                    |
| `MPU6050_InterruptDisable`            | Disables sensor interrupts.                   |

---



Feel free to customize or extend the driver as per your project requirements!
