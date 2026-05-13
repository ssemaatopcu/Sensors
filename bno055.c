/*
 * bno055.c
 *
 *  Created on: 11 Ara 2022
 *      Author: sematopcu
 */

#include "bno055.h"

#define BNO_MAX_ERR		(10)



static uint32_t current_ms, prev_ms = 0;
static inline void BNO055_SetDelay(tBNO055* bno, uint32_t ms);
static tI2C_Status BNO055_ProcessEuler(tBNO055* bno);


tI2C_Status BNO055_Init(tBNO055* bno, I2C_HandleTypeDef* hi2c){
	if(bno == NULL || hi2c ==  NULL){
		return I2C_FAIL;
	}

	memset(bno, 0, sizeof((tBNO055)*bno));
	bno->taskPeriod = BNO055_TASK_PERIOD;
	bno->hi2c= hi2c;
	bno->state = 0;
	return I2C_SUCCESS;
}


tI2C_Status BNO055_SetExtClock(tBNO055* bno){
	uint8_t extClock = BNO055_USE_EXT_CLCK;
	if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 1, &extClock, 1, 2)){
		return I2C_FAIL;
	}

	return I2C_SUCCESS;
}


tI2C_Status BNO055_SetPowerMode(tBNO055* bno, eBNO055_PowerMode pmode){
	uint8_t powerMode = (uint8_t)pmode;
	if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_PWR_MODE_ADDR, 1, &powerMode, 1, 2)){
		return I2C_FAIL;
	}

	return I2C_SUCCESS;
}


tI2C_Status BNO055_SetOperationMode(tBNO055* bno, eBNO055_OperationMode opmode){
	uint8_t opMode = (uint8_t)opmode;
	if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_OPR_MODE_ADDR, 1, &opMode, 1, 2)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}

/* Skipped the definitions of these function for the time being. Will be addressed
 * if deemed necessary.
 *
I2C_Status BNO055_SetCalibration(tBNO055* bno){
	if(HAL_OK != HAL_I2C_Mem_Write_DMA(bno->i2c_if->hi2c, bno->DevAddr, BNO055_ACC_OFFSET_X_LSB, 1, bno->CalibBytes, BNO055_TOTAL_CALIB_SIZE)){
}

I2C_Status BNO055_GetCalibration(tBNO055* bno){
	if(HAL_OK != HAL_I2C_Mem_Read_DMA(bno->i2c_if->hi2c, bno->DevAddr, BNO055_ACC_OFFSET_X_LSB, 1, bno->CalibBytes, BNO055_TOTAL_CALIB_SIZE)){
}
*/

tI2C_Status BNO055_GetEuler(tBNO055* bno){
	if(HAL_OK != HAL_I2C_Mem_Read(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_EUL_DATA_H_LSB, 1, (uint8_t*)(tEuler*)(&(bno->euler)), BNO055_EULER_HRP_DATA_SIZE, 2)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}


static tI2C_Status BNO055_ProcessEuler(tBNO055* bno){
	bno->eulerf.H = (float)bno->euler.H / BNO055_EULER_DIV_DEG;
	bno->eulerf.R = (float)bno->euler.R / BNO055_EULER_DIV_DEG;
	bno->eulerf.P = (float)bno->euler.P / BNO055_EULER_DIV_DEG;
	return I2C_SUCCESS;
}


tI2C_Status BNO055_Task(tBNO055* bno, tROV* rov){

	/* Get the current tick from HAL SysTick timer update counter */
	current_ms = uwTick;

	/* Check if the the time for task to run has come */
	if(current_ms - prev_ms < bno->taskPeriod){
		return I2C_TIMEOUT;
	}
	/* Update the previously taken tick*/
	prev_ms = current_ms;

	eBNO055_Task_States state = bno->state;
	tI2C_Status ret = I2C_FAIL;
	if(bno->errorCount > BNO_MAX_ERR){
		bno->errorCount = 0;
		state = BNO055_SET_LnRESET;
	}

	if(state == BNO055_SET_EXT_CLCK){
		ret = BNO055_SetExtClock(bno);
		BNO055_SetDelay(bno, 650);
		bno->state++;
	}
	else if(state == BNO055_SET_POWER_MODE){
		ret = BNO055_SetPowerMode(bno, BNO055_POWER_MODE_NORMAL);
		BNO055_SetDelay(bno, 650);
		bno->state++;
	}
	else if(state == BNO055_SET_OPERATION_MODE){
		ret = BNO055_SetOperationMode(bno, BNO055_OPERATION_MODE_NDOF);
		BNO055_SetDelay(bno, 650);
		bno->state++;
	}
	else if(state == BNO055_GET_EULER){
		ret = BNO055_GetEuler(bno);
		BNO055_SetDelay(bno, BNO055_TASK_PERIOD);

		if (ret == I2C_SUCCESS)
			rov->telemetry.status &= (~ImuSensError);
		else{
			rov->telemetry.status |= (ImuSensError);
		}
		bno->state++;
	}
	else if(state == BNO055_PROCESS_EULER){
		ret = BNO055_ProcessEuler(bno);
		rov->telemetry.head = (int16_t)bno->eulerf.H;
		rov->telemetry.roll = (int16_t)bno->eulerf.R;
		rov->telemetry.pitch = (int16_t)bno->eulerf.P;
		bno->state = BNO055_GET_EULER;
	}

	else if(state == BNO055_SET_LnRESET){
		HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, 0);
		BNO055_SetDelay(bno, 3);
		bno->state = BNO055_SET_HnRESET;
	}
	else if(state == BNO055_SET_HnRESET){
		HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, 1);
		BNO055_SetDelay(bno, 3);
		bno->state = BNO055_SET_EXT_CLCK;
	}

	bno->errorCount += ret;


	return ret;
}


static inline void BNO055_SetDelay(tBNO055* bno, uint32_t ms){
	if(ms < 0 || ms > 1000)
		return ;
	bno->taskPeriod = ms;
}

/* BNO055 State update functions */
inline void BNO055_SetStateNext(tBNO055* bno){
	if(bno->state < BNO055_PROCESS_EULER){
		bno->state++;
	}
}

inline void BNO055_SetStateDefault(tBNO055* bno){
	bno->state = 0;
}
