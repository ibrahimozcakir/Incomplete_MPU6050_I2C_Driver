#include "MPU6050.h"


static SensorStatus_t Sensor_UpdateReaded(SensorHandle_t *pSensor_Handle, uint8_t *pdata);
static SensorStatus_t Sensor_VerifyId(SensorStatus_t SensorStatus, uint8_t data);
static SensorStatus_t Sensor_UpdateAccCo(SensorCoef_t *SensorCoef, AfsSel_t AfsSel);
static SensorStatus_t Sensor_UpdateGyroCo(SensorCoef_t *SensorCoef, FsSel_t FsSel);
static SensorStatus_t Sensor_ReadReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData);
static SensorStatus_t Sensor_WriteReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData);
static SensorStatus_t Sensor_IsDeviceReady(uint8_t DevAddr);
static SensorStatus_t Sensor_SampleRate(uint8_t SampleRate, uint8_t SampleDiv);


SensorStatus_t MPU6050_ControlStatus(void)
{
	return Sensor_IsDeviceReady(MPU6050_DEVICE_ADDR);
}

SensorStatus_t MPU6050_ControlId(void)
{
	uint8_t pdata[1] = { 0 };

	SensorStatus_t SensorStatus;

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_WHO_AM_I, pdata, ASIZE(pdata));

	if (Sensor_VerifyId(SensorStatus, pdata[0]) == SENSOR_OK)
		return SensorStatus;

	return SensorStatus;
}

SensorStatus_t MPU6050_Init(SensorHandle_t *pSensor_Handle, FsSel_t FsSel, AfsSel_t AfsSel)
{
	SensorStatus_t SensorStatus = SENSOR_OK;

	uint8_t RegVal[1] = { 0 };

	SensorStatus = MPU6050_SetGyroRange(pSensor_Handle, FsSel);

	SensorStatus = MPU6050_SetAccRange(pSensor_Handle, AfsSel);

	SensorStatus = Sensor_SampleRate(SAMPLE_RATE, SMPLRT_DIV);

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	CLEAR_BIT(RegVal[0], 0x3);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	SensorStatus = MPU6050_SleepMode(SleepMode_OFF);

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;
}

SensorStatus_t MPU6050_SleepMode(SleepMode_t SleepMode)
{
	SensorStatus_t SensorStatus;

	uint8_t RegVal[1] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	if (SleepMode == SleepMode_ON)
		SET_BIT(RegVal[0], MPU6050_BIT_PWR_MGMT1_SLEEP_MODE);

	else
		CLEAR_BIT(RegVal[0], MPU6050_BIT_PWR_MGMT1_SLEEP_MODE);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, RegVal, ASIZE(RegVal));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;
}

SensorStatus_t MPU6050_SetAccRange(SensorHandle_t *pSensor_Handle, AfsSel_t AfsSel)
{
	SensorStatus_t SensorStatus = SENSOR_OK;

	uint8_t RegVal[1] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_CONFIG, RegVal, ASIZE(RegVal));

	CLEAR_BIT(RegVal[0], MPU6050_BIT_ACCEL_CONFIG_AFS_SEL);

	SET_BIT(RegVal[0], AfsSel << MPU6050_BIT_ACCEL_CONFIG_AFS_SEL_POS);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_CONFIG, RegVal, ASIZE(RegVal));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return Sensor_UpdateAccCo(&pSensor_Handle->SensorCoef, AfsSel);
}

SensorStatus_t MPU6050_SetGyroRange(SensorHandle_t *pSensor_Handle, FsSel_t FsSel)
{
	SensorStatus_t SensorStatus;

	uint8_t RegVal[1] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_GYRO_CONFIG, RegVal, ASIZE(RegVal));

	CLEAR_BIT(RegVal[0], MPU6050_BIT_GYRO_CONFIG_FS_SEL);

	SET_BIT(RegVal[0], FsSel << MPU6050_BIT_GYRO_CONFIG_FS_SEL_POS);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_GYRO_CONFIG, RegVal, ASIZE(RegVal));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return Sensor_UpdateGyroCo(&pSensor_Handle->SensorCoef, FsSel);
}

SensorStatus_t MPU6050_ReadDataAll(SensorHandle_t *pSensor_Handle)
{
	SensorStatus_t SensorStatus;

	uint8_t pdata[14] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_XOUT_H, pdata, ASIZE(pdata));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return Sensor_UpdateReaded(pSensor_Handle, pdata);
}

static SensorStatus_t Sensor_UpdateReaded(SensorHandle_t *pSensor_Handle, uint8_t *pdata)
{
	pSensor_Handle->AccRaw.X = (int16_t)(pdata[0] << 8 | pdata[1]);
	pSensor_Handle->AccRaw.Y = (int16_t)(pdata[2] << 8 | pdata[3]);
	pSensor_Handle->AccRaw.Z = (int16_t)(pdata[4] << 8 | pdata[5]);

	pSensor_Handle->TempRaw = (int16_t)(pdata[6] << 8 | pdata[7]);

	pSensor_Handle->GyroRaw.X = (int16_t)(pdata[8] << 8 | pdata[9]);
	pSensor_Handle->GyroRaw.Y = (int16_t)(pdata[10] << 8 | pdata[11]);
	pSensor_Handle->GyroRaw.Z = (int16_t)(pdata[12] << 8 | pdata[13]);

	pSensor_Handle->Acc.X = pSensor_Handle->AccRaw.X / pSensor_Handle->SensorCoef.AccCo;
	pSensor_Handle->Acc.Y = pSensor_Handle->AccRaw.Y / pSensor_Handle->SensorCoef.AccCo;
	pSensor_Handle->Acc.Z = pSensor_Handle->AccRaw.Z / pSensor_Handle->SensorCoef.AccCo;

	pSensor_Handle->TempC = (pSensor_Handle->TempRaw / 340.) + 36.53;

	pSensor_Handle->Gyro.X = pSensor_Handle->GyroRaw.X / pSensor_Handle->SensorCoef.GyroCo;
	pSensor_Handle->Gyro.Y = pSensor_Handle->GyroRaw.Y / pSensor_Handle->SensorCoef.GyroCo;
	pSensor_Handle->Gyro.Z = pSensor_Handle->GyroRaw.Y / pSensor_Handle->SensorCoef.GyroCo;

	return SENSOR_OK;
}

static SensorStatus_t Sensor_VerifyId(SensorStatus_t SensorStatus, uint8_t data)
{
	if (SensorStatus == SENSOR_OK && data == MPU6050_REG_VALUE_WHO_AM_I)
		return SensorStatus;

	return SensorStatus;
}

static SensorStatus_t Sensor_UpdateAccCo(SensorCoef_t *SensorCoef, AfsSel_t AfsSel)
{
	SensorStatus_t SensorStatus = SENSOR_OK;

	switch (AfsSel) {

		case AFS_2G:  SensorCoef->AccCo = 16384.; break;
		case AFS_4G:  SensorCoef->AccCo = 8192.; break;
		case AFS_8G:  SensorCoef->AccCo = 4096.; break;
		case AFS_16G: SensorCoef->AccCo = 2048.; break;
		default: SensorStatus = SENSOR_ERROR;
	}

	return SensorStatus;
}

static SensorStatus_t Sensor_UpdateGyroCo(SensorCoef_t *SensorCoef, FsSel_t FsSel)
{
	SensorStatus_t SensorStatus = SENSOR_OK;

	switch (FsSel) {

	case FS_250  : SensorCoef->GyroCo = 131.0; break;
	case FS_500  : SensorCoef->GyroCo = 65.5;  break;
	case FS_1000 : SensorCoef->GyroCo = 32.8;  break;
	case FS_2000 : SensorCoef->GyroCo = 16.4;  break;
	default : SensorStatus = SENSOR_ERROR;

	}

	return SensorStatus;
}

SensorStatus_t MPU6050_InterruptEnable(IntLevel_t IntLevel, IntOpen_t IntOpen)
{
	SensorStatus_t SensorStatus;

	uint8_t pdata[1] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_PIN_CFG, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_LEVEL);
	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_OPEN);
	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_RD_CLEAR);

	if (IntLevel == INT_LEVEL_LOW)
		SET_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_LEVEL);

	if (IntOpen == INT_OPEN_OPENDRAIN)
		SET_BIT(pdata[0], MPU6050_BIT_INT_PIN_CFG_INT_OPEN);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_PIN_CFG, pdata, ASIZE(pdata));

	//

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_ENABLE_DATA_RDY_EN);
	SET_BIT(pdata[0], MPU6050_BIT_INT_ENABLE_DATA_RDY_EN);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	 /*** Read the IRQ status to clear ***/

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_STATUS, pdata, ASIZE(pdata));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;
}

SensorStatus_t MPU6050_InterruptDisable(void)
{
	SensorStatus_t SensorStatus;

	uint8_t pdata[1] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_INT_ENABLE_DATA_RDY_EN);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_ENABLE, pdata, ASIZE(pdata));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;
}

SensorStatus_t MPU6050_ReadDataIT(SensorHandle_t *pSensor_Handle)
{
	SensorStatus_t SensorStatus;

	uint8_t pdata[14] = { 0 };

	uint8_t pflag[1] = { 0 };

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_INT_STATUS, pflag, ASIZE(pflag));

	if (!(pflag[0] & MPU6050_BIT_INT_STATUS_DATA_RDY_INT))
		return SENSOR_ERROR;

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_ACCEL_XOUT_H, pdata, ASIZE(pdata));

	Sensor_UpdateReaded(pSensor_Handle, pdata);

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;

}


SensorStatus_t MPU6050_DeviceReset(void)
{
	SensorStatus_t SensorStatus;

	uint8_t pdata[1] = {MPU6050_BIT_PWR_MGMT_1_DEVICE_RESET};

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_PWR_MGMT_1, pdata, ASIZE(pdata));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;
}

static SensorStatus_t Sensor_ReadReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData)
{
	HAL_StatusTypeDef HalStatus;

	HalStatus = HAL_I2C_Mem_Read(&I2C_Handle, DevAddr, RegAddr, sizeof(RegAddr), pdata, SzData, TIMEOUT_VAL);

	return (HalStatus == HAL_OK) ? SENSOR_OK : SENSOR_ERROR;
}


static SensorStatus_t Sensor_WriteReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pdata, uint16_t SzData)
{
	HAL_StatusTypeDef HalStatus;

	HalStatus = HAL_I2C_Mem_Write(&I2C_Handle, DevAddr, RegAddr, sizeof(RegAddr), pdata, SzData, TIMEOUT_VAL);

	return (HalStatus == HAL_OK) ? SENSOR_OK : SENSOR_ERROR;
}


static SensorStatus_t Sensor_IsDeviceReady(uint8_t DevAddr)
{
	HAL_StatusTypeDef HalStatus;

	HalStatus = HAL_I2C_IsDeviceReady(&I2C_Handle, DevAddr, TRIALS_NUM, TIMEOUT_VAL);

	return (HalStatus == HAL_OK) ? SENSOR_OK : SENSOR_ERROR;
}

static SensorStatus_t Sensor_SampleRate(uint8_t SampleRate, uint8_t SampleDiv)
{
	SensorStatus_t SensorStatus;

	uint8_t pdata[1] = { SampleDiv };

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_SMPLRT_DIV, pdata, ASIZE(pdata));

	SensorStatus = Sensor_ReadReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_CONFIG, pdata, ASIZE(pdata));

	CLEAR_BIT(pdata[0], MPU6050_BIT_CONFIG_DLPF_CFG);

	SET_BIT(pdata[0], SampleRate << MPU6050_BIT_CONFIG_DLPF_CFG_POS);

	SensorStatus = Sensor_WriteReg(MPU6050_DEVICE_ADDR, MPU6050_REG_ADDR_CONFIG, pdata, ASIZE(pdata));

	if (SensorStatus == SENSOR_ERROR)
		return SensorStatus;

	return SensorStatus;
}

SensorStatus_t MPU6050_ReadDataAcc(SensorHandle_t *pSensor_Handle)
{
	return SENSOR_OK;
}
SensorStatus_t MPU6050_ReadDataGyro(SensorHandle_t *pSensor_Handle)
{
	return SENSOR_OK;
}
SensorStatus_t MPU6050_ReadDataTemp(SensorHandle_t *pSensor_Handle)
{
	return SENSOR_OK;
}
