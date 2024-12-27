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

### Configuration

Before using the driver, ensure that the `mpu6050.h` file is configured according to your setup:

```c
/*
 * Please define the I2C interface, AD0 pin configuration, and I2C clock speed according to your setup:
 *
 * 1. Select the correct I2C interface:
 *    - If you are using I2C1, define: #define _I2C1_
 *    - If you are using I2C2, define: #define _I2C2_
 *    - If you are using I2C3, define: #define _I2C3_
 *
 * 2. Set the AD0 pin state:
 *    - If AD0 is LOW, define: #define MPU6050_AD0_LOW
 *    - If AD0 is HIGH, define: #define MPU6050_AD0_HIGH
 *
 * 3. Select the I2C clock speed:
 *    - For Standard Mode (100 kHz), define: #define _I2C_CLK_SPEED_STANDARD_
 *    - For Fast Mode (400 kHz), define: #define _I2C_CLK_SPEED_FAST_
 *
 * Make sure to modify these settings based on your hardware configuration.
 */

#define _I2C1_
#define MPU6050_AD0_LOW
#define _I2C_CLK_SPEED_STANDARD_
```

### Resetting the I2C Bus

In some cases, the I2C bus might get stuck after resetting the STM32 board. To resolve this issue, you can use the `I2C_ResetI2C` function provided in the `I2C_Reset.h` file. This function releases the I2C bus and ensures proper initialization. Include the following header file:

```c
#include "stm32f4xx_hal.h"
#include "I2C_Reset.h"

void I2C_ResetI2C(I2C_HandleTypeDef* I2C_Handle);
```

Make sure to call this function before initializing the I2C peripheral, typically before the `MX_I2C1_Init()` function in your `main.c` file:

```c
I2C_ResetI2C(&hi2c1);
MX_I2C1_Init();
```

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
status = MPU6050_ReadData(&sensor, MeasureType_ALL);
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
  MPU6050_ResetDevice();
  ```

---

## API Reference

### Data Types

- `SensorHandle_t`: Structure to manage sensor data and configurations.

```c
/**
 * @brief Sensor handle structure.
 * Contains all relevant sensor data including raw and scaled values, temperature, and coefficients.
 */
typedef struct {
    AxisRawVal_t AccRaw;
    AxisRawVal_t GyroRaw;
    AxisVal_t Acc;
    AxisVal_t Gyro;
    int16_t TempRaw;
    double TempC;
    SensorCoef_t SensorCoef;
} SensorHandle_t;
```

- `MeasureType_t`: Enum to define which type of measurement to read:

```c
/* @brief Measure Type of Readings Definitions */
typedef enum {
    MeasureType_ALL,
    MeasureType_ACC,
    MeasureType_GYRO,
    MeasureType_TEMP,
} MeasureType_t;
```

### Key Functions

| Function                              | Description                                    |
|---------------------------------------|------------------------------------------------|
| `MPU6050_Init`                        | Initializes the sensor.                       |
| `MPU6050_ControlStatus`               | Checks if the sensor is ready.                |
| `MPU6050_ControlId`                   | Verifies the sensor's ID.                     |
| `MPU6050_SleepMode`                   | Toggles sleep mode.                           |
| `MPU6050_ResetDevice`                 | Resets the sensor.                            |
| `MPU6050_SetAccRange`                 | Configures accelerometer range.               |
| `MPU6050_SetGyroRange`                | Configures gyroscope range.                   |
| `MPU6050_ReadData`                    | Reads data based on the specified measurement type. |
| `MPU6050_InterruptEnable`             | Enables sensor interrupts.                    |
| `MPU6050_InterruptDisable`            | Disables sensor interrupts.                   |
| `MPU6050_SampleRateConfig`            | Configures the sample rate and divider.       |

---

Feel free to customize or extend the driver as per your project requirements!
