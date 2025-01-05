#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stm32f4xx_hal.h"
#include "I2C_Reset.h"
#include <stdint.h>

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


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#ifdef _I2C1_
extern I2C_HandleTypeDef hi2c1;
#define I2C_Handle hi2c1
#endif

#ifdef _I2C2_
extern I2C_HandleTypeDef hi2c2;
#define I2C_Handle hi2c2
#endif

#ifdef _I2C2_
extern I2C_HandleTypeDef hi2c3;
#define I2C_Handle hi2c3
#endif

#ifdef MPU6050_AD0_LOW
#define MPU6050_DEVICE_ADDR   							(0x68 << 1)
#endif
#ifdef MPU6050_AD0_HIGH
#define MPU6050_DEVICE_ADDR   							(0x69 << 1)
#endif


#ifdef _I2C_CLK_SPEED_STANDARD_
#define SAMPLE_RATE										(0x1) // DLPF_CFG Bits
#define SMPLRT_DIV										(0x9) // SMPLRT_DIV Bits
#endif

#ifdef _I2C_CLK_SPEED_FAST_
#define SAMPLE_RATE										(0x0) // DLPF_CFG Bits
#define SMPLRT_DIV										(0x6) // SMPLRT_DIV Bits
#endif

/**************************************************************************************/



/*
 * @brief MPU6050 Register Definitions
 *
 */
#define MPU6050_REG_ADDR_SELF_TEST_X					(0x0D)
#define MPU6050_REG_ADDR_SELF_TEST_Y					(0x0E)
#define MPU6050_REG_ADDR_SELF_TEST_Z					(0x0F)
#define MPU6050_REG_ADDR_SELF_TEST_A					(0x10)
#define MPU6050_REG_ADDR_SMPLRT_DIV						(0x19)
#define MPU6050_REG_ADDR_CONFIG							(0x1A)
#define MPU6050_REG_ADDR_GYRO_CONFIG					(0x1B)
#define MPU6050_REG_ADDR_ACCEL_CONFIG					(0x1C)
#define MPU6050_REG_ADDR_FIFO_EN						(0x23)
#define MPU6050_REG_ADDR_I2C_MST_CTRL					(0x24)
#define MPU6050_REG_ADDR_I2C_MST_STATUS     			(0x36)
#define MPU6050_REG_ADDR_INT_PIN_CFG        			(0x37)
#define MPU6050_REG_ADDR_INT_ENABLE         			(0x38)
#define MPU6050_REG_ADDR_INT_STATUS         			(0x3A)
#define MPU6050_REG_ADDR_ACCEL_XOUT_H       			(0x3B)
#define MPU6050_REG_ADDR_ACCEL_XOUT_L       			(0x3C)
#define MPU6050_REG_ADDR_ACCEL_YOUT_H       			(0x3D)
#define MPU6050_REG_ADDR_ACCEL_YOUT_L       			(0x3E)
#define MPU6050_REG_ADDR_ACCEL_ZOUT_H       			(0x3F)
#define MPU6050_REG_ADDR_ACCEL_ZOUT_L       			(0x40)
#define MPU6050_REG_ADDR_TEMP_OUT_H         			(0x41)
#define MPU6050_REG_ADDR_TEMP_OUT_L         			(0x42)
#define MPU6050_REG_ADDR_GYRO_XOUT_H        			(0x43)
#define MPU6050_REG_ADDR_GYRO_XOUT_L        			(0x44)
#define MPU6050_REG_ADDR_GYRO_YOUT_H       	 			(0x45)
#define MPU6050_REG_ADDR_GYRO_YOUT_L        			(0x46)
#define MPU6050_REG_ADDR_GYRO_ZOUT_H        			(0x47)
#define MPU6050_REG_ADDR_GYRO_ZOUT_L        			(0x48)
#define MPU6050_REG_ADDR_I2C_MST_DELAY_CTRL 			(0x67)
#define MPU6050_REG_ADDR_SIGNAL_PATH_RESET  			(0x68)
#define MPU6050_REG_ADDR_USER_CTRL		    			(0x6A)
#define MPU6050_REG_ADDR_PWR_MGMT_1	        			(0x6B)
#define MPU6050_REG_ADDR_PWR_MGMT_2 	    			(0x6C)
#define MPU6050_REG_ADDR_FIFO_COUNTH	    			(0x72)
#define MPU6050_REG_ADDR_FIFO_COUNTL	    			(0x73)
#define MPU6050_REG_ADDR_FIFO_R_W		    			(0x74)
#define MPU6050_REG_ADDR_WHO_AM_I						(0x75)




/*
 *  @brief MPU6050 Bit Definitions
 *
 */
#define MPU6050_BIT_PWR_MGMT1_SLEEP_MODE_POS			(0x6)
#define MPU6050_BIT_PWR_MGMT1_SLEEP_MODE				(0x1 << MPU6050_BIT_PWR_MGMT1_SLEEP_MODE_POS)

#define MPU6050_BIT_ACCEL_CONFIG_AFS_SEL_POS			(0x3)
#define MPU6050_BIT_ACCEL_CONFIG_AFS_SEL				(0x3 << MPU6050_BIT_ACCEL_CONFIG_AFS_SEL_POS)

#define MPU6050_BIT_GYRO_CONFIG_FS_SEL_POS				(0x3)
#define MPU6050_BIT_GYRO_CONFIG_FS_SEL					(0x3 << MPU6050_BIT_GYRO_CONFIG_FS_SEL_POS)

#define MPU6050_BIT_INT_STATUS_DATA_RDY_INT_POS			(0x0)
#define MPU6050_BIT_INT_STATUS_DATA_RDY_INT				(0x1 << MPU6050_BIT_INT_STATUS_DATA_RDY_INT_POS)

#define MPU6050_BIT_INT_ENABLE_DATA_RDY_EN_POS			(0x0)
#define MPU6050_BIT_INT_ENABLE_DATA_RDY_EN				(0x1 << MPU6050_BIT_INT_ENABLE_DATA_RDY_EN_POS)

#define MPU6050_BIT_INT_PIN_CFG_INT_LEVEL_POS			(0x7)
#define MPU6050_BIT_INT_PIN_CFG_INT_LEVEL				(0x1 << MPU6050_BIT_INT_PIN_CFG_INT_LEVEL_POS)

#define MPU6050_BIT_INT_PIN_CFG_INT_OPEN_POS			(0x6)
#define MPU6050_BIT_INT_PIN_CFG_INT_OPEN				(0x1 << MPU6050_BIT_INT_PIN_CFG_INT_OPEN_POS)

#define MPU6050_BIT_INT_PIN_CFG_INT_RD_CLEAR_POS 		(0x4)
#define MPU6050_BIT_INT_PIN_CFG_INT_RD_CLEAR	 		(0x1 << MPU6050_BIT_INT_PIN_CFG_INT_RD_CLEAR_POS)

#define MPU6050_BIT_PWR_MGMT_1_DEVICE_RESET_POS	 		(0x7)
#define MPU6050_BIT_PWR_MGMT_1_DEVICE_RESET	 	 		(0x1 << MPU6050_BIT_PWR_MGMT_1_DEVICE_RESET_POS)

#define MPU6050_BIT_CONFIG_DLPF_CFG_POS			 		(0x0)
#define MPU6050_BIT_CONFIG_DLPF_CFG		 				(0x7 << MPU6050_BIT_CONFIG_DLPF_CFG_POS)

#define MPU6050_BIT_SMPRT_DIV_SMPLRT_DIV_POS			(0x0)
#define MPU6050_BIT_SMPRT_DIV_SMPLRT_DIV				(0xFF << MPU6050_BIT_SMPRT_DIV_SMPLRT_DIV_POS)


/*
 * @brief Function Like Macro
 *
 */
#define ASIZE(a)										(sizeof(a) / sizeof(a[0]))


/*
 * @brief Sensor Constants
 *
 */
#define TIMEOUT_VAL										(200)
#define TRIALS_NUM										(0x5)
#define MPU6050_REG_VALUE_WHO_AM_I						(0x68)



/*
 * @brief Gyro Sample Rate Definitions
 *
 */
#define SAMPLERATE_GYRO_8KHZ							(0x0)
#define SAMPLERATE_GYRO_1KHZ_BW_188HZ					(0x1)
#define SAMPLERATE_GYRO_1KHZ_BW_98HZ					(0x2)
#define SAMPLERATE_GYRO_1KHZ_BW_42HZ					(0x3)
#define SAMPLERATE_GYRO_1KHZ_BW_20HZ					(0x4)
#define SAMPLERATE_GYRO_1KHZ_BW_10HZ					(0x5)
#define SAMPLERATE_GYRO_1KHZ_BW_5HZ				     	(0x6)




/*
 * @brief Interrupt pin active level configuration.
 *
 */
typedef enum {
	INT_LEVEL_HIGH,
	INT_LEVEL_LOW
}IntLevel_t;


/*
 * @brief Interrupt pin output type configuration
 *
 */
typedef enum {
	INT_OPEN_PUSHPULL,
	INT_OPEN_OPENDRAIN
}IntOpen_t;


/*
 * @brief Gyroscope full-scale range selection.
 *
 */
typedef enum {
	FS_250,
	FS_500,
	FS_1000,
	FS_2000
}FsSel_t;


/*
 * @brief Accelerometer full-scale range selection.
 *
 */
typedef enum {
	AFS_2G,
	AFS_4G,
	AFS_8G,
	AFS_16G
}AfsSel_t;


/*
 * @brief Sensor operation status.
 *
 */
typedef enum {
	SENSOR_ERROR,
	SENSOR_OK,
}SensorStatus_t;


/*
 * @brief Sleep mode configuration for the sensor.
 *
 */
typedef enum {
	SleepMode_OFF,
	SleepMode_ON,
}SleepMode_t;


/*
 * @brief Raw axis values structure.
 * Contains 16-bit raw sensor readings for X, Y, and Z axes.
 */
typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
}AxisRawVal_t;


/*
 * @brief Scaled axis values structure.
 * Contains double-precision scaled sensor readings for X, Y, and Z axes.
 */
typedef struct {
	double X;
	double Y;
	double Z;
}AxisVal_t;


/*
 * @brief Sensor coefficient structure.
 * Holds scaling coefficients for accelerometer and gyroscope.
 */
typedef struct {
    double AccCo;
    double GyroCo;
}SensorCoef_t;



/*
 * @brief Measure Type of Readings Definitions
 *
 */
typedef enum {
	MeasureType_ALL,
	MeasureType_ACC,
	MeasureType_GYRO,
	MeasureType_TEMP,
}MeasureType_t;


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
}SensorHandle_t;



SensorStatus_t MPU6050_ControlStatus(void);
SensorStatus_t MPU6050_ControlId(void);
SensorStatus_t MPU6050_Init(SensorHandle_t *pSensor_Handle, FsSel_t FsSel, AfsSel_t AfsSel);
SensorStatus_t MPU6050_SleepMode(SleepMode_t SleepMode);
SensorStatus_t MPU6050_ResetDevice(void);
SensorStatus_t MPU6050_SetAccRange(SensorHandle_t *pSensor_Handle, AfsSel_t AfsSel);
SensorStatus_t MPU6050_SetGyroRange(SensorHandle_t *pSensor_Handle, FsSel_t FsSel);
SensorStatus_t MPU6050_InterruptEnable(IntLevel_t IntLevel, IntOpen_t IntOpen);
SensorStatus_t MPU6050_InterruptDisable(void);
SensorStatus_t MPU6050_ReadDataIT(SensorHandle_t *pSensor_Handle);
SensorStatus_t MPU6050_ReadData(SensorHandle_t *pSensor_Handle, MeasureType_t MeasureType);
SensorStatus_t MPU6050_SampleRateConfig(uint8_t SampleRate, uint8_t SampleDiv);

#endif /* INC_MPU6050_H_ */
