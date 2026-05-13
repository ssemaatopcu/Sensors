/*
 * MS5837.h
 *
 *  Created on: Feb 11, 2023
 *      Author: sematopcu
 */

#ifndef INC_MS5837_H_
#define INC_MS5837_H_

#include "stm32f1xx_hal.h"
#include "rov.h"
#include "i2c_interface.h"
#include <stdint.h>
#include <string.h>
#include <math.h>


#define MS5837_ADDR               (0x76)
#define MS5837_HAL_IC2_ADDR		  (0x76 << 1)

#define MS5837_CMD_RESET              (0x1E)
#define MS5837_CMD_ADC_READ           (0x00)
#define MS5837_CMD_PROM_READ          (0xA0)
#define MS5837_CMD_CONVERT_D1_8192    (0x4A)
#define MS5837_CMD_CONVERT_D2_8192    (0x5A)

typedef enum MS5837_Task_States{
	MS5837_RESET = 0,
	MS5837_CALIBRATION,
	MS5837_CONVERT_D1,
	MS5837_COLLECT_D1,
	MS5837_CONVERT_D2,
	MS5837_COLLECT_D2,
	MS5837_CALC_DEPTH
}eMS5837_Task_States;

typedef struct {
	int32_t TEMP;
	int32_t P;
	uint16_t C[7];
	uint32_t D1;
	uint32_t D2;
} tMS5837Values;


typedef struct  {
	I2C_HandleTypeDef* hi2c;
	uint32_t taskPeriod;
	uint32_t errorCount;
	eMS5837_Task_States state;
	float depth;
	uint8_t model;
	float fluidDensity;
	float temperature;
	float pressure;
	tMS5837Values val;
}tMS5837;


tI2C_Status MS5837_Task(tMS5837* ms5837, tROV* rov);
tI2C_Status MS5837_Init(tMS5837 *MS5837, I2C_HandleTypeDef* hi2c);
tI2C_Status MS5837_Reset(tMS5837* ms5837);
tI2C_Status MS5837_Calibration(tMS5837* ms5837, uint8_t i);
tI2C_Status MS5837_RequestD1(tMS5837* ms5837);
tI2C_Status MS5837_RequestD2(tMS5837* ms5837);
tI2C_Status MS5837_CollectD1(tMS5837* ms5837);
tI2C_Status MS5837_CollectD2(tMS5837* ms5837);
tI2C_Status MS5837_Depth(tMS5837 *ms5837);
tI2C_Status MS5837_30BA_Calc(tMS5837 *ms5837);
tI2C_Status MS5837_Depth(tMS5837 *ms5837);

#endif /* INC_MS5837_H_ */
