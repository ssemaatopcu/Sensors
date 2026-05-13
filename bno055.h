/*
 * bno055.h
 *
 *  Created on: 11 Ara 2022
 *      Author: sematopcu
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "i2c_interface.h"
#include "rov.h"
#include "stdint.h"
#include "string.h"


#define BNO055_TASK_PERIOD			(10) /* ms */
#define BNO055_HAL_I2C_ADDR			(BNO055_CHIP_I2C_ADDR << 1)

typedef struct __attribute__((packed)){
	int16_t X;
	int16_t Y;
	int16_t Z;
}tAccel;

typedef struct __attribute__((packed)){
	int16_t X;
	int16_t Y;
	int16_t Z;
}tAccelFloat;

typedef struct __attribute__((packed)){
	int16_t X;
	int16_t Y;
	int16_t Z;
}tGyro;

typedef struct __attribute__((packed)){
	float X;
	float Y;
	float Z;
}tGyroFloat;

/*
 * Order is extremely important since the variables are not assigned instead they are written through pointer operations.
 * Accel and gyro should be ordered as XYZ while euler is HRP.
 */

typedef struct __attribute__((packed)){
	int16_t H;
	int16_t R;
	int16_t P;
}tEuler;

typedef struct __attribute__((packed)){
	float H;
	float R;
	float P;
}tEulerFloat;



/* Defines of BNO055 addresses */
#define	BNO055_CHIP_I2C_ADDR		(0x28)
#define BNO055_CHIP_ID_ADDR         (0x00)

#define BNO055_TEMP_ADDR            (0x34)
#define BNO055_GYR_ID_ADDR          (0x03)
#define BNO055_MAG_ID_ADDR          (0x02)
#define BNO055_ACC_ID_ADDR          (0x01)

/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR			(0X07)

/* Page ID */
#define BNO055_PAGE_ZERO			(0X00)
#define BNO055_PAGE_ONE				(0X01)

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR		(0X3B)
#define BNO055_DATA_SELECT_ADDR		(0X3C)

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR		(0X3D)
#define BNO055_PWR_MODE_ADDR		(0X3E)

#define BNO055_SYS_TRIGGER_ADDR		(0X3F)

#define BNO055_CLCK_SOURCE_POS		(7)
#define BNO055_INT_CLCK_BIT_VAL		(0)
#define BNO055_EXT_CLCK_BIT_VAL		(1)
#define BNO055_USE_EXT_CLCK			(BNO055_EXT_CLCK_BIT_VAL << BNO055_CLCK_SOURCE_POS)

/* Operation mode settings*/
/* Redefined in respective enum*/
//#define BNO055_OPERATION_MODE_NDOF                 (0X0C)

/* Power mode*/
/* Redefined in respective enum*/
//#define BNO055_POWER_MODE_NORMAL                   (0X00)

/*Accel division factor*/
#define BNO055_ACCEL_DIV_MSQ                       (100.0)
#define BNO055_ACCEL_DIV_MG                        (1)

/*Mag division factor*/
#define BNO055_MAG_DIV_UT                          (16.0)

/*Gyro division factor*/
#define BNO055_GYRO_DIV_DPS                        (16.0)
#define BNO055_GYRO_DIV_RPS                        (900.0)

/*Euler division factor*/
#define BNO055_EULER_DIV_DEG                       (16.0)
#define BNO055_EULER_DIV_RAD                       (900.0)

/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BNO055_ACCEL_XYZ_DATA_SIZE                 (6)
#define BNO055_GYRO_XYZ_DATA_SIZE                  (6)
#define BNO055_EULER_HRP_DATA_SIZE                 (6)

#define BNO055_ACCEL_OFFSET_ARRAY                  (6)
#define BNO055_MAG_OFFSET_ARRAY                    (6)
#define BNO055_GYRO_OFFSET_ARRAY                   (6)
#define BNO055_ACCEL_RADIUS_ARRAY                  (2)
#define BNO055_MAG_RADIUS_ARRAY                    (2)
#define BNO055_TOTAL_CALIB_SIZE					   (BNO055_ACCEL_OFFSET_ARRAY \
													+ BNO055_MAG_OFFSET_ARRAY \
													+ BNO055_GYRO_OFFSET_ARRAY \
													+ BNO055_MAG_RADIUS_ARRAY \
													+ BNO055_MAG_RADIUS_ARRAY)

#define BNO055_SENSOR_DATA_EULER_LSB               (0)
#define BNO055_SENSOR_DATA_EULER_MSB               (1)
#define BNO055_SENSOR_DATA_EULER_HRP_H_LSB         (0)
#define BNO055_SENSOR_DATA_EULER_HRP_H_MSB         (1)
#define BNO055_SENSOR_DATA_EULER_HRP_R_LSB         (2)
#define BNO055_SENSOR_DATA_EULER_HRP_R_MSB         (3)
#define BNO055_SENSOR_DATA_EULER_HRP_P_LSB         (4)
#define BNO055_SENSOR_DATA_EULER_HRP_P_MSB         (5)

/* Register Offset Addresses */
/* Starting from ACC OFFSET LSB to MAG_RADIUS_MSG, total byte size for calibration
 * : BNO055_TOTAL_CALIB_SIZE, 22 in default*/
#define BNO055_ACC_OFFSET_X_LSB     (0x55)    //Accelerometer Offset X <7:0>

/* Data Register Address */
#define BNO055_EUL_DATA_H_LSB     	(0x1A)      //Heading Data <7:0>
#define BNO055_EUL_DATA_H_MSB      	(0x1B)      //Heading Data <15:8>
#define BNO055_EUL_DATA_R_LSB     	(0x1C)      //Roll Data <7:0>
#define BNO055_EUL_DATA_R_MSB       (0x1D)      //Roll Data <15:8>
#define BNO055_EUL_DATA_P_LSB      	(0x1E)      //Pitch Data <7:0>
#define BNO055_EUL_DATA_P_MSB      	(0x1F)      //Pitch Data <15:8>


typedef enum __attribute__((packed)) BNO055_PowerMode
{
	BNO055_POWER_MODE_NORMAL =				(0X00U),
	BNO055_POWER_MODE_LOWPOWER =			(0X01U),
	BNO055_POWER_MODE_SUSPEND =				(0X02U),
}eBNO055_PowerMode;

typedef enum __attribute__((packed)) BNO055_OperationMode
{
	BNO055_OPERATION_MODE_CONFIG =          (0X00U),
	BNO055_OPERATION_MODE_ACCONLY = 		(0X01U),
	BNO055_OPERATION_MODE_MAGONLY =			(0X02U),
	BNO055_OPERATION_MODE_GYRONLY =         (0X03U),
	BNO055_OPERATION_MODE_ACCMAG =         	(0X04U),
	BNO055_OPERATION_MODE_ACCGYRO =         (0X05U),
	BNO055_OPERATION_MODE_MAGGYRO =         (0X06U),
	BNO055_OPERATION_MODE_AMG =         	(0X07U),
	BNO055_OPERATION_MODE_IMUPLUS =         (0X08U),
	BNO055_OPERATION_MODE_COMPASS =         (0X09U),
	BNO055_OPERATION_MODE_M4G =         	(0X0AU),
	BNO055_OPERATION_MODE_NDOF_FMC_OFF =    (0X0BU),
	BNO055_OPERATION_MODE_NDOF =            (0X0CU)
}eBNO055_OperationMode;

typedef enum __attribute__((packed)) BNO055_Task_States
{
	BNO055_SET_EXT_CLCK,
	BNO055_SET_POWER_MODE,
	BNO055_SET_OPERATION_MODE,
	BNO055_GET_EULER,
	BNO055_PROCESS_EULER,

	BNO055_SET_LnRESET,
	BNO055_SET_HnRESET,
}eBNO055_Task_States;

typedef struct BNO055{
	I2C_HandleTypeDef* hi2c;
	uint32_t taskPeriod;
	uint32_t errorCount;
	eBNO055_Task_States state;
	tAccel accel;
	tGyro gyro;
	tEuler euler;
	tEulerFloat eulerf;
}tBNO055;

/* Constructor for tBNO055 structure. Must be called before any BNO related functions.*/
tI2C_Status BNO055_Init(tBNO055* bno, I2C_HandleTypeDef* hi2c);

/* Main BNO task routine */
tI2C_Status BNO055_Task(tBNO055* bno, tROV* rov);

/* BNO task routine steps in ordered form
 * Standalone call to these functions are highly discouraged.
 * */
tI2C_Status BNO055_SetPowerMode(tBNO055* bno, eBNO055_PowerMode pmode);
tI2C_Status BNO055_SetOperationMode(tBNO055* bno, eBNO055_OperationMode opmode);
tI2C_Status BNO055_SetCalibration(tBNO055* bno);
tI2C_Status BNO055_GetCalibration(tBNO055* bno);
tI2C_Status BNO055_GetEuler(tBNO055* bno);

/* BNO State update functions */
void BNO055_SetStateNext(tBNO055* bno);
void BNO055_SetStateDefault(tBNO055* bno);

#endif



