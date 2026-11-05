/*
 * mpu6050.h
 *
 * Created on: 3 Nov 2026
 * Author: Lambda
 *
 * STM32G4 MPU6050 driver using  sensordriver.
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "sensordriver.h"

/* HAL expects the 7-bit I2C address shifted left by one bit. */
#define MPU6050_I2C_ADRESS_AD0          (0x68U << 1)  /* AD0 = GND */
#define MPU6050_I2C_ADRESS_AD1          (0x69U << 1)  /* AD0 = VCC */

/* Select the actual hardware address here (AD0 LOW by default). */
#ifndef MPU6050_I2C_ADDRESS
#define MPU6050_I2C_ADDRESS            MPU6050_I2C_ADRESS_AD0
#endif

#define MPU6050_WHO_AM_I_VALUE          (0x70U)

/* MPU6050 register addresses (all are 8-bit). */
#define MPU_REG_SELF_TEST_X             (13U)
#define MPU_REG_SELF_TEST_Y             (14U)
#define MPU_REG_SELF_TEST_Z             (15U)
#define MPU_REG_SELF_TEST_A             (16U)
#define MPU_REG_SMPLRT_DIV              (25U)
#define MPU_REG_CONFIG                  (26U)
#define MPU_REG_GYRO_CONFIG             (27U)
#define MPU_REG_ACCEL_CONFIG            (28U)
#define MPU_REG_ACCEL_XOUT_H            (59U)
#define MPU_REG_ACCEL_XOUT_L            (60U)
#define MPU_REG_ACCEL_YOUT_H            (61U)
#define MPU_REG_ACCEL_YOUT_L            (62U)
#define MPU_REG_ACCEL_ZOUT_H            (63U)
#define MPU_REG_ACCEL_ZOUT_L            (64U)
#define MPU_REG_ACCEL_TEMP_OUT_H        (65U)
#define MPU_REG_ACCEL_TEMP_OUT_L        (66U)
#define MPU_REG_GYRO_XOUT_H             (67U)
#define MPU_REG_GYRO_XOUT_L             (68U)
#define MPU_REG_GYRO_YOUT_H             (69U)
#define MPU_REG_GYRO_YOUT_L             (70U)
#define MPU_REG_GYRO_ZOUT_H             (71U)
#define MPU_REG_GYRO_ZOUT_L             (72U)
#define MPU_REG_USER_CTRL               (106U)
#define MPU_REG_PWR_MGMT_1              (107U)
#define MPU_REG_PWR_MGMT_2              (108U)
#define MPU_REG_FIFO_COUNTH             (114U)
#define MPU_REG_FIFO_COUNTL             (115U)
#define MPU_REG_FIFO_R_W                (116U)
#define MPU_REG_WHO_AM_I                (117U)

#define MPU_BIT_PWR_MGMT_1_SLEEP_MODE                 (6U)
#define MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION (3U)
#define MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION   (3U)

typedef enum {
    FS_250 = 0,
    FS_500,
    FS_1000,
    FS_2000
} fs_sel_e;

typedef enum {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
} afs_sel_e;

typedef enum {
    SLEEPMODE_OFF = 0,
    SLEEPMODE_ON
} sleepmode_e;

typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} AxisRawVal_t;

typedef struct {
    double X;
    double Y;
    double Z;
} AxisVal_t;

typedef struct {
    AxisRawVal_t accRaw;
    AxisRawVal_t gyroRaw;
    AxisVal_t acc;         /* g */
    AxisVal_t gyro;        /* degrees/second */
    double acc_co;        /* LSB/g */
    double gyro_co;       /* LSB/(degrees/second) */
    int16_t tempRaw;      /* Added: raw temperature */
    double temperatureC;  /* Added: Celsius */
} SensorData_t;

/* Keep the user's original API. Only &hi2c1 is supported by sensordriver. */
/* MPU6050_Init: 0 = success, 1 = error; checks ACK and WHO_AM_I. */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
/* Returns WHO_AM_I (normally 0x68); returns 0 on an I2C error. */
uint8_t MPU6050_READ_ID(I2C_HandleTypeDef *hi2c);

/* Full configuration/measurement API adapted from the reference. */
sensor_status_e MPU6050_test_sensor(void);
sensor_status_e MPU6050_initialize(SensorData_t *pSensor,
                                    fs_sel_e gyroConfig,
                                    afs_sel_e accConfig);
sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode);
sensor_status_e MPU6050_read_data(SensorData_t *pSensorData);

#endif /* INC_MPU6050_H_ */
