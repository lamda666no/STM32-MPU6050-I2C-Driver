/*
 * mpu6050.c
 *
 * Created on: 3 Nov 2026
 * Author: Lambda
 *
 * STM32G4 MPU6050 driver using  sensordriver.
 */

#include "mpu6050.h"
#include "i2c.h"     /* &hi2c1: the bus used by sensordriver.c */
#include "gpio.h"    /* Optional testLed_Pin / testLed_GPIO_Port */
#include <stddef.h>  /* NULL */

#define MPU6050_CONFIG_RANGE_MASK       (0x03U)
#define MPU6050_MEASUREMENT_BYTES       (14U)

static sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor,
		afs_sel_e accRange);
static sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor,
		fs_sel_e gyroRange);

/* The shared sensordriver uses hi2c1, so refuse an unsupported handle. */
static uint8_t MPU6050_is_valid_bus(I2C_HandleTypeDef *hi2c) {
	return (uint8_t) ((hi2c != NULL) && (hi2c == &hi2c1));
}

/* Original user API: test device and ID; 0 on success, 1 on failure. */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
	sensor_status_e status = SENSOR_ERROR;

	if (MPU6050_is_valid_bus(hi2c) != 0U) {
		status = MPU6050_test_sensor();
	}

#if defined(testLed_GPIO_Port) && defined(testLed_Pin)
	HAL_GPIO_WritePin(testLed_GPIO_Port, testLed_Pin,
			(status == SENSOR_OK) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

	return (status == SENSOR_OK) ? 0U : 1U;
}

/* Original user API: return WHO_AM_I, not HAL_OK and not an unused buffer. */
uint8_t MPU6050_READ_ID(I2C_HandleTypeDef *hi2c) {
	uint8_t id = 0U;

	if (MPU6050_is_valid_bus(hi2c) == 0U) {
		return 0U;
	}

	if (sensor_read_bytes(MPU6050_I2C_ADDRESS, MPU_REG_WHO_AM_I, &id, 1U)
			!= SENSOR_OK) {
		return 0U;
	}

	return id;
}

sensor_status_e MPU6050_test_sensor(void) {
	if (sensor_test_device(MPU6050_I2C_ADDRESS) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	if (MPU6050_READ_ID(&hi2c1) != MPU6050_WHO_AM_I_VALUE) {
		return SENSOR_ERROR;
	}

	return SENSOR_OK;
}

/* Full setup: check chip, wake it, set gyro and accelerometer ranges. */
sensor_status_e MPU6050_initialize(SensorData_t *pSensor, fs_sel_e gyroConfig,
		afs_sel_e accConfig) {
	if (pSensor == NULL || (unsigned int) gyroConfig > (unsigned int) FS_2000
			|| (unsigned int) accConfig > (unsigned int) AFS_16G) {
		return SENSOR_ERROR;
	}

	if (MPU6050_test_sensor() != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	if (MPU6050_set_sleep_mode(SLEEPMODE_OFF) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	HAL_Delay(100U); /* Allow the sensor clock/measurements to settle. */

	if (MPU6050_set_gyro_range(pSensor, gyroConfig) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	if (MPU6050_set_acc_range(pSensor, accConfig) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	return SENSOR_OK;
}

/* Set bits [4:3] in GYRO_CONFIG; do not disturb other bits. */
static sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor,
		fs_sel_e gyroRange) {
	uint8_t configReg = 0U;
	double coefficient = 0.0;
	const uint8_t mask = (uint8_t) (MPU6050_CONFIG_RANGE_MASK <<
	MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION);

	if (pSensor == NULL) {
		return SENSOR_ERROR;
	}

	switch (gyroRange) {
	case FS_250:
		coefficient = 131.0;
		break;
	case FS_500:
		coefficient = 65.5;
		break;
	case FS_1000:
		coefficient = 32.8;
		break;
	case FS_2000:
		coefficient = 16.4;
		break;
	default:
		return SENSOR_ERROR;
	}

	/* Read via status-returning API: zero is a valid register value. */
	if (sensor_read_bytes(MPU6050_I2C_ADDRESS, MPU_REG_GYRO_CONFIG, &configReg,
			1U) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	configReg = (uint8_t) ((configReg & (uint8_t) ~mask)
			| ((uint8_t) gyroRange <<
			MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION));

	if (sensor_write_register_8bit(MPU6050_I2C_ADDRESS,
	MPU_REG_GYRO_CONFIG, configReg) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	pSensor->gyro_co = coefficient;
	return SENSOR_OK;
}

/* Set bits [4:3] in ACCEL_CONFIG; do not disturb other bits. */
static sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor,
		afs_sel_e accRange) {
	uint8_t configReg = 0U;
	double coefficient = 0.0;
	const uint8_t mask = (uint8_t) (MPU6050_CONFIG_RANGE_MASK <<
	MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION);

	if (pSensor == NULL) {
		return SENSOR_ERROR;
	}

	switch (accRange) {
	case AFS_2G:
		coefficient = 16384.0;
		break;
	case AFS_4G:
		coefficient = 8192.0;
		break;
	case AFS_8G:
		coefficient = 4096.0;
		break;
	case AFS_16G:
		coefficient = 2048.0;
		break;
	default:
		return SENSOR_ERROR;
	}

	if (sensor_read_bytes(MPU6050_I2C_ADDRESS, MPU_REG_ACCEL_CONFIG, &configReg,
			1U) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	configReg = (uint8_t) ((configReg & (uint8_t) ~mask)
			| ((uint8_t) accRange <<
			MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION));

	if (sensor_write_register_8bit(MPU6050_I2C_ADDRESS,
	MPU_REG_ACCEL_CONFIG, configReg) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	pSensor->acc_co = coefficient;
	return SENSOR_OK;
}

sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode) {
	uint8_t powerReg = 0U;
	const uint8_t sleepMask = (uint8_t) (1U << MPU_BIT_PWR_MGMT_1_SLEEP_MODE);

	if ((sleepmode != SLEEPMODE_OFF) && (sleepmode != SLEEPMODE_ON)) {
		return SENSOR_ERROR;
	}

	if (sensor_read_bytes(MPU6050_I2C_ADDRESS, MPU_REG_PWR_MGMT_1, &powerReg,
			1U) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	if (sleepmode == SLEEPMODE_ON) {
		powerReg |= sleepMask;
	} else {
		powerReg &= (uint8_t) ~sleepMask;
	}

	return sensor_write_register_8bit(MPU6050_I2C_ADDRESS,
	MPU_REG_PWR_MGMT_1, powerReg);
}

/* Read ACCEL XYZ, TEMP, GYRO XYZ in one transaction (MSB first). */
sensor_status_e MPU6050_read_data(SensorData_t *pSensorData) {
	uint8_t buffer[MPU6050_MEASUREMENT_BYTES] = { 0U };

	if (pSensorData == NULL || pSensorData->acc_co == 0.0
			|| pSensorData->gyro_co == 0.0) {
		return SENSOR_ERROR;
	}

	if (sensor_read_bytes(MPU6050_I2C_ADDRESS, MPU_REG_ACCEL_XOUT_H, buffer,
			(uint8_t) sizeof(buffer)) != SENSOR_OK) {
		return SENSOR_ERROR;
	}

	/* Never use sensor_read_register_16_bit() here: that API is little-endian. */
	pSensorData->accRaw.X = (int16_t) bytesToUint16_BigEndian(&buffer[0]);
	pSensorData->accRaw.Y = (int16_t) bytesToUint16_BigEndian(&buffer[2]);
	pSensorData->accRaw.Z = (int16_t) bytesToUint16_BigEndian(&buffer[4]);
	pSensorData->tempRaw = (int16_t) bytesToUint16_BigEndian(&buffer[6]);
	pSensorData->gyroRaw.X = (int16_t) bytesToUint16_BigEndian(&buffer[8]);
	pSensorData->gyroRaw.Y = (int16_t) bytesToUint16_BigEndian(&buffer[10]);
	pSensorData->gyroRaw.Z = (int16_t) bytesToUint16_BigEndian(&buffer[12]);

	pSensorData->acc.X = pSensorData->accRaw.X / pSensorData->acc_co;
	pSensorData->acc.Y = pSensorData->accRaw.Y / pSensorData->acc_co;
	pSensorData->acc.Z = pSensorData->accRaw.Z / pSensorData->acc_co;

	pSensorData->gyro.X = pSensorData->gyroRaw.X / pSensorData->gyro_co;
	pSensorData->gyro.Y = pSensorData->gyroRaw.Y / pSensorData->gyro_co;
	pSensorData->gyro.Z = pSensorData->gyroRaw.Z / pSensorData->gyro_co;

	pSensorData->temperatureC = (pSensorData->tempRaw / 333.87) + 21.0;
	return SENSOR_OK;
}
