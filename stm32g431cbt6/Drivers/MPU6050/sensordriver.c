/*
 * sensordriver.c
 *
 * Created on: 4 Nov 2026
 * Author: Lambda
 *
 * Adapted to STM32G4 HAL from the supplied reference library.
 */

#include "sensordriver.h"
#include "i2c.h"                 /* hi2c1 and STM32 HAL declarations */
#include <stddef.h>              /* NULL */

#define I2C_TIMEOUT                 (100U)
#define I2C_NUMBER_OF_TRIALS        (4U)
#define I2C_REG_ADD_SIZE_1_BYTE     (I2C_MEMADD_SIZE_8BIT)
#define I2C_DATA_SIZE_1_BYTE        (1U)
#define I2C_DATA_SIZE_2_BYTE        (2U)

sensor_status_e sensor_test_device(uint8_t chipAdd)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(&hi2c1, chipAdd,
                                   I2C_NUMBER_OF_TRIALS, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return SENSOR_ERROR;
    }

    return SENSOR_OK;
}

/* 8-bit register address + 8-bit data. */
uint8_t sensor_read_register_8_bit(uint8_t chipAdd, uint8_t regAdd)
{
    uint8_t pdata = 0;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, chipAdd, regAdd,
                               I2C_REG_ADD_SIZE_1_BYTE,
                               &pdata, I2C_DATA_SIZE_1_BYTE, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return 0U;
    }

    return pdata;
}

/* 8-bit register address + 16-bit data (little-endian byte order). */
uint16_t sensor_read_register_16_bit(uint8_t chipAdd, uint8_t regAdd)
{
    uint8_t pdata[I2C_DATA_SIZE_2_BYTE] = {0U};
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, chipAdd, regAdd,
                               I2C_REG_ADD_SIZE_1_BYTE,
                               pdata, I2C_DATA_SIZE_2_BYTE, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return 0U;
    }

    return bytesToUint16_LittleEndian(pdata);
}

/* 8-bit register address + 8-bit data. */
sensor_status_e sensor_write_register_8bit(uint8_t chipAdd,
                                            uint8_t regAdd,
                                            uint8_t data)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c1, chipAdd, regAdd,
                                I2C_REG_ADD_SIZE_1_BYTE,
                                &data, I2C_DATA_SIZE_1_BYTE, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return SENSOR_ERROR;
    }

    return SENSOR_OK;
}

/* 8-bit register address + 16-bit data (big-endian byte order). */
sensor_status_e sensor_write_register_16bit(uint8_t chipAdd,
                                             uint8_t regAdd,
                                             uint16_t data)
{
    uint8_t pdata[I2C_DATA_SIZE_2_BYTE];
    HAL_StatusTypeDef status;

    pdata[0] = (uint8_t)(data >> 8);     /* High byte */
    pdata[1] = (uint8_t)(data & 0xFFU);  /* Low byte  */

    status = HAL_I2C_Mem_Write(&hi2c1, chipAdd, regAdd,
                                I2C_REG_ADD_SIZE_1_BYTE,
                                pdata, I2C_DATA_SIZE_2_BYTE, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return SENSOR_ERROR;
    }

    return SENSOR_OK;
}

sensor_status_e sensor_read_bytes(uint8_t chipAdd, uint8_t regAdd,
                                  uint8_t *pBuffer, uint8_t size)
{
    HAL_StatusTypeDef status;

    if ((pBuffer == NULL) || (size == 0U)) {
        return SENSOR_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c1, chipAdd, regAdd,
                               I2C_REG_ADD_SIZE_1_BYTE,
                               pBuffer, size, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return SENSOR_ERROR;
    }

    return SENSOR_OK;
}

sensor_status_e sensor_write_bytes(uint8_t chipAdd, uint8_t regAdd,
                                   uint8_t *pBuffer, uint8_t size)
{
    HAL_StatusTypeDef status;

    if ((pBuffer == NULL) || (size == 0U)) {
        return SENSOR_ERROR;
    }

    status = HAL_I2C_Mem_Write(&hi2c1, chipAdd, regAdd,
                                I2C_REG_ADD_SIZE_1_BYTE,
                                pBuffer, size, I2C_TIMEOUT);

    if (status != HAL_OK) {
        return SENSOR_ERROR;
    }

    return SENSOR_OK;
}

uint16_t bytesToUint16_LittleEndian(uint8_t *pData)
{
    return (uint16_t)(((uint16_t)pData[1] << 8) | pData[0]);
}

uint16_t bytesToUint16_BigEndian(uint8_t *pData)
{
    return (uint16_t)(((uint16_t)pData[0] << 8) | pData[1]);
}
