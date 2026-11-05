/*
 * sensordriver.h
 *
 * Created on: 4 Nov 2026
 * Author: Lambda
 *
 * General I2C sensor access for STM32 HAL.
 * All register addresses in this version are 8-bit.
 */

#ifndef SENSORDRIVER_H_
#define SENSORDRIVER_H_

#include <stdint.h>

typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERROR
} sensor_status_e;

/* chipAdd is the HAL-formatted I2C address (7-bit address << 1). */
sensor_status_e sensor_test_device(uint8_t chipAdd);

/* 8-bit register address + 8-bit data. */
uint8_t sensor_read_register_8_bit(uint8_t chipAdd, uint8_t regAdd);
sensor_status_e sensor_write_register_8bit(uint8_t chipAdd, uint8_t regAdd,
                                           uint8_t data);

/* 8-bit register address + 16-bit data.
 * Read follows the original library's LITTLE-ENDIAN byte order.
 * Write follows the original library's BIG-ENDIAN byte order.
 * Check the sensor datasheet; these are not universally interchangeable.
 */
uint16_t sensor_read_register_16_bit(uint8_t chipAdd, uint8_t regAdd);
sensor_status_e sensor_write_register_16bit(uint8_t chipAdd, uint8_t regAdd,
                                            uint16_t data);

/* Read or write a sequence of bytes beginning at an 8-bit register address. */
sensor_status_e sensor_read_bytes(uint8_t chipAdd, uint8_t regAdd,
                                  uint8_t *pBuffer, uint8_t size);
sensor_status_e sensor_write_bytes(uint8_t chipAdd, uint8_t regAdd,
                                   uint8_t *pBuffer, uint8_t size);

/* pData must point to at least two bytes. */
uint16_t bytesToUint16_LittleEndian(uint8_t *pData);
uint16_t bytesToUint16_BigEndian(uint8_t *pData);

#endif /* SENSORDRIVER_H_ */
