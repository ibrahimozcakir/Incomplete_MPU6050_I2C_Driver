#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "main.h"
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
#define SMPLRT_DIV										(0x0) // SMPLRT_DIV Bits
#endif

#ifdef _I2C_CLK_SPEED_FAST_
#define SAMPLE_RATE										(0x0) // DLPF_CFG Bits
#define SMPLRT_DIV										(0x2) // SMPLRT_DIV Bits
#endif

/**************************************************************************************/

/**
 * Register Definition
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
#define MPU6050_REG_ADDR_I2C_SLV0_ADDR					(0x25)
#define MPU6050_REG_ADDR_I2C_SLV0_REG					(0x26)
#define MPU6050_REG_ADDR_I2C_SLV0_CTRL					(0x27)
#define MPU6050_REG_ADDR_I2C_SLV1_ADDR      			(0x28)
#define MPU6050_REG_ADDR_I2C_SLV1_REG       			(0x29)
#define MPU6050_REG_ADDR_I2C_SLV1_CTRL      			(0x2A)
#define MPU6050_REG_ADDR_I2C_SLV2_ADDR      			(0x2B)
#define MPU6050_REG_ADDR_I2C_SLV2_REG       			(0x2C)
#define MPU6050_REG_ADDR_I2C_SLV2_CTRL      			(0x2D)
#define MPU6050_REG_ADDR_I2C_SLV3_ADDR      			(0x2E)
#define MPU6050_REG_ADDR_I2C_SLV3_REG       			(0x2F)
#define MPU6050_REG_ADDR_I2C_SLV3_CTRL      			(0x30)
#define MPU6050_REG_ADDR_I2C_SLV4_ADDR      			(0x31)
#define MPU6050_REG_ADDR_I2C_SLV4_REG       			(0x32)
#define MPU6050_REG_ADDR_I2C_SLV4_DO        			(0x33)
#define MPU6050_REG_ADDR_I2C_SLV4_CTRL      			(0x34)
#define MPU6050_REG_ADDR_I2C_SLV4_DI        			(0x35)
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
#define MPU6050_REG_ADDR_EXT_SENS_DATA_00   			(0x49)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_01   			(0x4A)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_02   			(0x4B)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_03   			(0x4C)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_04   			(0x4D)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_05   			(0x4E)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_06   			(0x4F)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_07   			(0x50)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_08   			(0x51)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_09   			(0x52)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_10   			(0x53)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_11   			(0x54)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_12   			(0x55)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_13   			(0x56)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_14   			(0x57)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_15   			(0x58)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_16   			(0x59)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_17   			(0x5A)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_18   			(0x5B)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_19   			(0x5C)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_20   			(0x5D)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_21   			(0x5E)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_22   			(0x5F)
#define MPU6050_REG_ADDR_EXT_SENS_DATA_23   			(0x60)
#define MPU6050_REG_ADDR_I2C_SLV0_DO	    			(0x63)
#define MPU6050_REG_ADDR_I2C_SLV1_DO	    			(0x64)
#define MPU6050_REG_ADDR_I2C_SLV2_DO	    			(0x65)
#define MPU6050_REG_ADDR_I2C_SLV3_DO	    			(0x66)
#define MPU6050_REG_ADDR_I2C_MST_DELAY_CTRL 			(0x67)
#define MPU6050_REG_ADDR_SIGNAL_PATH_RESET  			(0x68)
#define MPU6050_REG_ADDR_USER_CTRL		    			(0x6A)
#define MPU6050_REG_ADDR_PWR_MGMT_1	        			(0x6B)
#define MPU6050_REG_ADDR_PWR_MGMT_2 	    			(0x6C)
#define MPU6050_REG_ADDR_FIFO_COUNTH	    			(0x72)
#define MPU6050_REG_ADDR_FIFO_COUNTL	    			(0x73)
#define MPU6050_REG_ADDR_FIFO_R_W		    			(0x74)
#define MPU6050_REG_ADDR_WHO_AM_I						(0x75)



/**
 * Bit Definition
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


#define ASIZE(a)										(sizeof(a) / sizeof(a[0]))

#define TIMEOUT_VAL										(200)
#define TRIALS_NUM										(0x5)
#define MPU6050_REG_VALUE_WHO_AM_I						(0x68)


typedef enum {
	INT_LEVEL_HIGH,
	INT_LEVEL_LOW
}IntLevel_t;

typedef enum {
	INT_OPEN_PUSHPULL,
	INT_OPEN_OPENDRAIN
}IntOpen_t;

typedef enum {
	FS_250,
	FS_500,
	FS_1000,
	FS_2000
}FsSel_t;

typedef enum {
	AFS_2G,
	AFS_4G,
	AFS_8G,
	AFS_16G
}AfsSel_t;

typedef enum {
	SENSOR_ERROR,
	SENSOR_OK,
}SensorStatus_t;

typedef enum {
	SleepMode_OFF,
	SleepMode_ON,
}SleepMode_t;

typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
}AxisRawVal_t;

typedef struct {
	double X;
	double Y;
	double Z;
}AxisVal_t;

typedef struct {
    double AccCo;
    double GyroCo;
}SensorCoef_t;

typedef struct {
	AxisRawVal_t AccRaw;
	AxisRawVal_t GyroRaw;
	AxisVal_t Acc;
	AxisVal_t Gyro;
	int16_t TempRaw;
	double TempC;
	SensorCoef_t SensorCoef;
}SensorHandle_t;



SensorStatus_t MPU6050_Init(SensorHandle_t *pSensor_Handle, FsSel_t FsSel, AfsSel_t AfsSel);
SensorStatus_t MPU6050_ControlStatus(void);
SensorStatus_t MPU6050_ControlId(void);
SensorStatus_t MPU6050_SleepMode(SleepMode_t SleepMode);
SensorStatus_t MPU6050_DeviceReset(void);
SensorStatus_t MPU6050_SetAccRange(SensorHandle_t *pSensor_Handle, AfsSel_t AfsSel);
SensorStatus_t MPU6050_SetGyroRange(SensorHandle_t *pSensor_Handle, FsSel_t FsSel);
SensorStatus_t MPU6050_ReadDataAll(SensorHandle_t *pSensor_Handle);
SensorStatus_t MPU6050_ReadDataAcc(SensorHandle_t *pSensor_Handle);
SensorStatus_t MPU6050_ReadDataGyro(SensorHandle_t *pSensor_Handle);
SensorStatus_t MPU6050_ReadDataTemp(SensorHandle_t *pSensor_Handle);
SensorStatus_t MPU6050_InterruptEnable(IntLevel_t IntLevel, IntOpen_t IntOpen);
SensorStatus_t MPU6050_InterruptDisable(void);
SensorStatus_t MPU6050_ReadDataIT(SensorHandle_t *pSensor_Handle);

#endif /* INC_MPU6050_H_ */
