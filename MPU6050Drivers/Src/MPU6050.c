#include "MPU6050.h"


static SensorStatus_t Sensor_UpdateReaded(SensorHandle_t *pSensor_Handle, uint8_t *pdata, MeasureType_t MeasureType);
static SensorStatus_t Sensor_VerifyId(SensorStatus_t SensorStatus, uint8_t data);
static SensorStatus_t Sensor_UpdateAccCo(SensorCoef_t *SensorCoef, AfsSel_t AfsSel);
static SensorStatus_t Sensor_UpdateGyroCo(SensorCoef_t *SensorCoef, FsSel_t FsSel);
static SensorStatus_t Sensor_ReadReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData);
static SensorStatus_t Sensor_WriteReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData);
static SensorStatus_t Sensor_IsDeviceReady(uint8_t DevAddr);
static SensorStatus_t Sensor_SampleRate(uint8_t SampleRate, uint8_t SampleDiv);
static SensorStatus_t Sensor_ReadDataAll(SensorHandle_t *pSensor_Handle);
static SensorStatus_t Sensor_ReadDataAcc(SensorHandle_t *pSensor_Handle);
static SensorStatus_t Sensor_ReadDataGyro(SensorHandle_t *pSensor_Handle);
static SensorStatus_t Sensor_ReadDataTemp(SensorHandle_t *pSensor_Handle);






/**
  * @brief  Checks the ready status of the MPU6050 sensor.
  * @retval SensorStatus_t Status of the device readiness (e.g., SENSOR_READY or SENSOR_ERROR).
  */

SensorStatus_t MPU6050_ControlStatus(void)
{
	return Sensor_IsDeviceReady(MPU6050_DEVICE_ADDR);
}


/**
  * @brief  Verifies the device ID of the MPU6050 sensor.
  * @retval SensorStatus_t Status of the verification process.
  *                        Returns SENSOR_OK if the ID matches, otherwise SENSOR_ERROR.
  */

SensorStatus_t MPU6050_ControlId(void)
{
	uint8_t pdata[1] = { 0 };

	SensorStatus_t SensorStatus;

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_WHO_AM_I, pdata, ASIZE(pdata));

	return Sensor_VerifyId(SensorStatus, pdata[0]);
}


/**
  * @brief  Reads sensor data based on the specified measurement type.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @param  MeasureType Specifies the type of measurement (e.g., ALL, ACC, GYRO, TEMP).
  * @retval SensorStatus_t Status of the data reading process.
  */

SensorStatus_t MPU6050_ReadData(SensorHandle_t *pSensor_Handle, MeasureType_t MeasureType)
{
	switch (MeasureType) {
	    case MeasureType_ALL:
	        return Sensor_ReadDataAll(pSensor_Handle);
	    case MeasureType_ACC:
	        return Sensor_ReadDataAcc(pSensor_Handle);
	    case MeasureType_GYRO:
	        return Sensor_ReadDataGyro(pSensor_Handle);
	    case MeasureType_TEMP:
	        return Sensor_ReadDataTemp(pSensor_Handle);
	    default:
	        return SENSOR_ERROR;
	}
}


/**
  * @brief  Initializes the MPU6050 sensor with specified configurations and waking up the sensor from sleep mode.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @param  FsSel Gyroscope full-scale range selection.
  * @param  AfsSel Accelerometer full-scale range selection.
  * @retval SensorStatus_t Status of the initialization process.
  */

SensorStatus_t MPU6050_Init(SensorHandle_t *pSensor_Handle, FsSel_t FsSel, AfsSel_t AfsSel)
{
	uint8_t RegVal[1] = { 0 };

	(void)MPU6050_SetGyroRange(pSensor_Handle, FsSel);

	(void)MPU6050_SetAccRange(pSensor_Handle, AfsSel);

	(void)Sensor_SampleRate(SAMPLE_RATE, SMPLRT_DIV);

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	CLEAR_BIT(RegVal[0], 0x3);

	(void)Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	return MPU6050_SleepMode(SleepMode_OFF);
}


/**
  * @brief  Configures the sleep mode of the MPU6050 sensor.
  * @param  SleepMode Specifies whether to enable or disable sleep mode.
  * @retval SensorStatus_t Status of the sleep mode configuration.
  */

SensorStatus_t MPU6050_SleepMode(SleepMode_t SleepMode)
{
	uint8_t RegVal[1] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	if (SleepMode == SleepMode_ON)
		SET_BIT(RegVal[0], MPU6050_BIT_PWR_MGMT1_SLEEP_MODE);

	else
		CLEAR_BIT(RegVal[0], MPU6050_BIT_PWR_MGMT1_SLEEP_MODE);

	return Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));
}


/**
  * @brief  Sets the accelerometer range for the MPU6050 sensor.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @param  AfsSel Accelerometer range selection.
  * @retval SensorStatus_t Status of the configuration process.
  */

SensorStatus_t MPU6050_SetAccRange(SensorHandle_t *pSensor_Handle, AfsSel_t AfsSel)
{
	uint8_t RegVal[1] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_CONFIG, RegVal, ASIZE(RegVal));

	CLEAR_BIT(RegVal[0], MPU6050_BIT_ACCEL_CONFIG_AFS_SEL);

	SET_BIT(RegVal[0], AfsSel << MPU6050_BIT_ACCEL_CONFIG_AFS_SEL_POS);

	(void)Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_CONFIG, RegVal, ASIZE(RegVal));

	return Sensor_UpdateAccCo(&pSensor_Handle->SensorCoef, AfsSel);
}


/**
  * @brief  Sets the gyroscope range for the MPU6050 sensor.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @param  FsSel Gyroscope range selection.
  * @retval SensorStatus_t Status of the configuration process.
  */

SensorStatus_t MPU6050_SetGyroRange(SensorHandle_t *pSensor_Handle, FsSel_t FsSel)
{
	uint8_t RegVal[1] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_GYRO_CONFIG, RegVal, ASIZE(RegVal));

	CLEAR_BIT(RegVal[0], MPU6050_BIT_GYRO_CONFIG_FS_SEL);

	SET_BIT(RegVal[0], FsSel << MPU6050_BIT_GYRO_CONFIG_FS_SEL_POS);

	(void)Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_GYRO_CONFIG, RegVal, ASIZE(RegVal));

	return Sensor_UpdateGyroCo(&pSensor_Handle->SensorCoef, FsSel);
}


/**
  * @brief  Configures and enables interrupts for the MPU6050 sensor.
  * @param  IntLevel Specifies the interrupt level configuration (e.g., INT_LEVEL_LOW or HIGH).
  * @param  IntOpen Specifies the interrupt pin configuration (e.g., Open-Drain or Push-Pull).
  * @retval SensorStatus_t Status of the interrupt configuration process.
  */

SensorStatus_t MPU6050_InterruptEnable(IntLevel_t IntLevel, IntOpen_t IntOpen)
{
	uint8_t pdata[1] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_PIN_CFG, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_LEVEL);
	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_OPEN);
	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_RD_CLEAR);

	if (IntLevel == INT_LEVEL_LOW)
		SET_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_LEVEL);

	if (IntOpen == INT_OPEN_OPENDRAIN)
		SET_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_OPEN);

	(void)Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_PIN_CFG, pdata, ASIZE(pdata));

	//

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_ENABLE_DATA_RDY_EN);
	SET_BIT(pdata[0], MPU6050_BIT_INT_ENABLE_DATA_RDY_EN);

	(void)Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	 /*** Read the IRQ status to clear ***/

	return Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_STATUS, pdata, ASIZE(pdata));
}


/**
  * @brief  Configures the sample rate and divider for the MPU6050 sensor.
  * @param  SampleRate Desired sample rate configuration.
  * @param  SampleDiv Divider value for the sample rate.
  * @retval SensorStatus_t Returns SENSOR_OK if the configuration is successful, otherwise SENSOR_ERROR.
  */

SensorStatus_t MPU6050_SampleRateConfig(uint8_t SampleRate, uint8_t SampleDiv)
{
	return Sensor_SampleRate(SAMPLE_RATE, SMPLRT_DIV);
}


/**
  * @brief  Disables the interrupt for data ready in the MPU6050 sensor.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

SensorStatus_t MPU6050_InterruptDisable(void)
{
	uint8_t pdata[1] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_ENABLE_DATA_RDY_EN);

	return Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));
}


/**
  * @brief  Reads all sensor data using interrupts if data is ready.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

SensorStatus_t MPU6050_ReadDataIT(SensorHandle_t *pSensor_Handle)
{
	uint8_t pdata[14] = { 0 };

	uint8_t pflag[1] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_STATUS, pflag, ASIZE(pflag));

	if (!(pflag[0] & MPU6050_BIT_INT_STATUS_DATA_RDY_INT))
		return SENSOR_ERROR;

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_XOUT_H, pdata, ASIZE(pdata));

	return Sensor_UpdateReaded(pSensor_Handle, pdata, MeasureType_ALL);
}


/**
  * @brief  Resets the MPU6050 device by writing to the power management register.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

SensorStatus_t MPU6050_ResetDevice(void)
{
	uint8_t pdata[1] = {MPU6050_BIT_PWR_MGMT_1_DEVICE_RESET};

	return Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, pdata, ASIZE(pdata));

	 HAL_Delay(50);
}


/**
  * @brief  Reads all sensor data (accelerometer, gyroscope, and temperature) from the MPU6050.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_ReadDataAll(SensorHandle_t *pSensor_Handle)
{
	uint8_t pdata[14] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_XOUT_H, pdata, ASIZE(pdata));

	return Sensor_UpdateReaded(pSensor_Handle, pdata, MeasureType_ALL);
}


/**
  * @brief  Updates the sensor handle structure with the raw and processed data.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @param  pdata Pointer to the data buffer containing raw data from the sensor.
  * @param  MeasureType Specifies the type of measurement data to update (all, accelerometer, gyroscope, or temperature).
  * @retval SensorStatus_t Returns SENSOR_OK if the data is successfully updated, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_UpdateReaded(SensorHandle_t *pSensor_Handle, uint8_t *pdata, MeasureType_t MeasureType)
{
	switch (MeasureType) {
		case MeasureType_ALL:
			pSensor_Handle->AccRaw.X = (int16_t) (pdata[0] << 8 | pdata[1]);
			pSensor_Handle->AccRaw.Y = (int16_t) (pdata[2] << 8 | pdata[3]);
			pSensor_Handle->AccRaw.Z = (int16_t) (pdata[4] << 8 | pdata[5]);

			pSensor_Handle->TempRaw = (int16_t) (pdata[6] << 8 | pdata[7]);

			pSensor_Handle->GyroRaw.X = (int16_t) (pdata[8] << 8 | pdata[9]);
			pSensor_Handle->GyroRaw.Y = (int16_t) (pdata[10] << 8 | pdata[11]);
			pSensor_Handle->GyroRaw.Z = (int16_t) (pdata[12] << 8 | pdata[13]);

			pSensor_Handle->Acc.X = pSensor_Handle->AccRaw.X
					/ pSensor_Handle->SensorCoef.AccCo;
			pSensor_Handle->Acc.Y = pSensor_Handle->AccRaw.Y
					/ pSensor_Handle->SensorCoef.AccCo;
			pSensor_Handle->Acc.Z = pSensor_Handle->AccRaw.Z
					/ pSensor_Handle->SensorCoef.AccCo;

			pSensor_Handle->TempC = (pSensor_Handle->TempRaw / 340.) + 36.53;

			pSensor_Handle->Gyro.X = pSensor_Handle->GyroRaw.X
					/ pSensor_Handle->SensorCoef.GyroCo;
			pSensor_Handle->Gyro.Y = pSensor_Handle->GyroRaw.Y
					/ pSensor_Handle->SensorCoef.GyroCo;
			pSensor_Handle->Gyro.Z = pSensor_Handle->GyroRaw.Z
					/ pSensor_Handle->SensorCoef.GyroCo;
			break;

		case MeasureType_ACC:
			pSensor_Handle->AccRaw.X = (int16_t) (pdata[0] << 8 | pdata[1]);
			pSensor_Handle->AccRaw.Y = (int16_t) (pdata[2] << 8 | pdata[3]);
			pSensor_Handle->AccRaw.Z = (int16_t) (pdata[4] << 8 | pdata[5]);

			pSensor_Handle->Acc.X = pSensor_Handle->AccRaw.X
					/ pSensor_Handle->SensorCoef.AccCo;
			pSensor_Handle->Acc.Y = pSensor_Handle->AccRaw.Y
					/ pSensor_Handle->SensorCoef.AccCo;
			pSensor_Handle->Acc.Z = pSensor_Handle->AccRaw.Z
					/ pSensor_Handle->SensorCoef.AccCo;
			break;

		case MeasureType_GYRO:
			pSensor_Handle->GyroRaw.X = (int16_t) (pdata[0] << 8 | pdata[1]);
			pSensor_Handle->GyroRaw.Y = (int16_t) (pdata[2] << 8 | pdata[3]);
			pSensor_Handle->GyroRaw.Z = (int16_t) (pdata[4] << 8 | pdata[5]);

			pSensor_Handle->Gyro.X = pSensor_Handle->GyroRaw.X
					/ pSensor_Handle->SensorCoef.GyroCo;
			pSensor_Handle->Gyro.Y = pSensor_Handle->GyroRaw.Y
					/ pSensor_Handle->SensorCoef.GyroCo;
			pSensor_Handle->Gyro.Z = pSensor_Handle->GyroRaw.Z
					/ pSensor_Handle->SensorCoef.GyroCo;
			break;

		case MeasureType_TEMP:
			pSensor_Handle->TempRaw = (int16_t) (pdata[0] << 8 | pdata[1]);
			pSensor_Handle->TempC = (pSensor_Handle->TempRaw / 340.) + 36.53;
			break;

		default:
			return SENSOR_ERROR;
	}

	return SENSOR_OK;
}


/**
  * @brief  Verifies the sensor's ID by comparing the received data with the expected ID.
  * @param  SensorStatus Current status of the sensor operation.
  * @param  data Received data to verify the sensor ID.
  * @retval SensorStatus_t Returns SENSOR_OK if ID matches, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_VerifyId(SensorStatus_t SensorStatus, uint8_t data)
{
	return (SensorStatus == SENSOR_OK && data == MPU6050_REG_VALUE_WHO_AM_I) ? SENSOR_OK : SENSOR_ERROR;
}


/**
  * @brief  Updates the accelerometer coefficient based on the selected range.
  * @param  SensorCoef Pointer to the structure holding sensor coefficients.
  * @param  AfsSel Selected accelerometer range.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR
  */

static SensorStatus_t Sensor_UpdateAccCo(SensorCoef_t *SensorCoef, AfsSel_t AfsSel)
{
	switch (AfsSel) {
	case AFS_2G:
		SensorCoef->AccCo = 16384.;
		break;
	case AFS_4G:
		SensorCoef->AccCo = 8192.;
		break;
	case AFS_8G:
		SensorCoef->AccCo = 4096.;
		break;
	case AFS_16G:
		SensorCoef->AccCo = 2048.;
		break;
	default:
		return SENSOR_ERROR;
	}

	return SENSOR_OK;
}


/**
  * @brief  Updates the gyroscope coefficient based on the selected range.
  * @param  SensorCoef Pointer to the structure holding sensor coefficients.
  * @param  FsSel Selected gyroscope range.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR
  */

static SensorStatus_t Sensor_UpdateGyroCo(SensorCoef_t *SensorCoef, FsSel_t FsSel)
{
	switch (FsSel) {
	case FS_250:
		SensorCoef->GyroCo = 131.0;
		break;
	case FS_500:
		SensorCoef->GyroCo = 65.5;
		break;
	case FS_1000:
		SensorCoef->GyroCo = 32.8;
		break;
	case FS_2000:
		SensorCoef->GyroCo = 16.4;
		break;
	default:
		return SENSOR_ERROR;
	}

	return SENSOR_OK;
}


/**
  * @brief  Reads a register value from the specified device address.
  * @param  DevAddr Device address of the sensor.
  * @param  RegAddr Register address to be read.
  * @param  pdata Pointer to the buffer to store the read data.
  * @param  SzData Size of the data to be read.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_ReadReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData)
{
	HAL_StatusTypeDef HalStatus;

	HalStatus = HAL_I2C_Mem_Read(&I2C_Handle, DevAddr, RegAddr, sizeof(RegAddr), pdata, SzData, TIMEOUT_VAL);

	return (HalStatus == HAL_OK) ? SENSOR_OK : SENSOR_ERROR;
}


/**
  * @brief  Writes data to a register at the specified device address.
  * @param  DevAddr Device address of the sensor.
  * @param  RegAddr Register address to write to.
  * @param  pdata Pointer to the data to be written.
  * @param  SzData Size of the data to be written.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */


static SensorStatus_t Sensor_WriteReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData)
{
	HAL_StatusTypeDef HalStatus;

	HalStatus = HAL_I2C_Mem_Write(&I2C_Handle, DevAddr, RegAddr, sizeof(RegAddr), pdata, SzData, TIMEOUT_VAL);

	return (HalStatus == HAL_OK) ? SENSOR_OK : SENSOR_ERROR;
}

/**
  * @brief  Checks if the device at the specified address is ready for communication.
  * @param  DevAddr Device address of the sensor.
  * @retval SensorStatus_t Returns SENSOR_OK if the device is ready, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_IsDeviceReady(uint8_t DevAddr)
{
	HAL_StatusTypeDef HalStatus;

	HalStatus = HAL_I2C_IsDeviceReady(&I2C_Handle, DevAddr, TRIALS_NUM, TIMEOUT_VAL);

	return (HalStatus == HAL_OK) ? SENSOR_OK : SENSOR_ERROR;
}


/**
  * @brief  Configures the sample rate and divider for the sensor.
  * @param  SampleRate Desired sample rate.
  * @param  SampleDiv Divider value for the sample rate.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_SampleRate(uint8_t SampleRate, uint8_t SampleDiv)
{
	uint8_t pdata[1] = { SampleDiv };

	(void)Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_SMPLRT_DIV, pdata, ASIZE(pdata));

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_CONFIG, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_CONFIG_DLPF_CFG);

	SET_BIT(pdata[0], SampleRate << MPU6050_BIT_CONFIG_DLPF_CFG_POS);

	return Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_CONFIG, pdata, ASIZE(pdata));
}


/**
  * @brief  Reads accelerometer data from the sensor and updates the sensor handle.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_ReadDataAcc(SensorHandle_t *pSensor_Handle)
{
	uint8_t pdata[6] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_XOUT_H, pdata, ASIZE(pdata));

	return Sensor_UpdateReaded(pSensor_Handle, pdata, MeasureType_ACC);
}

/**
  * @brief  Reads gyroscope data from the sensor and updates the sensor handle.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_ReadDataGyro(SensorHandle_t *pSensor_Handle)
{
	uint8_t pdata[6] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_GYRO_XOUT_H, pdata, ASIZE(pdata));

	return Sensor_UpdateReaded(pSensor_Handle, pdata, MeasureType_GYRO);
}

/**
  * @brief  Reads temperature data from the sensor and updates the sensor handle.
  * @param  pSensor_Handle Pointer to the sensor handle structure.
  * @retval SensorStatus_t Returns SENSOR_OK if the operation is successful, otherwise SENSOR_ERROR.
  */

static SensorStatus_t Sensor_ReadDataTemp(SensorHandle_t *pSensor_Handle)
{
	uint8_t pdata[2] = { 0 };

	(void)Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_TEMP_OUT_H, pdata, ASIZE(pdata));

	return Sensor_UpdateReaded(pSensor_Handle, pdata, MeasureType_TEMP);
}
