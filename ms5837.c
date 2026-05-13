/*
 * MS5837.c
 *
 *  Created on: Feb 11, 2023
 *      Author: sematopcu
 */


#include "ms5837.h"


static uint32_t current_ms, prev_ms = 0;
static inline void MS5837_SetDelay(tMS5837* ms5837, uint32_t ms);
static uint8_t calib_idx = 0;
static uint8_t crc4(uint16_t n_prom[]);



tI2C_Status MS5837_Init(tMS5837 *ms5837, I2C_HandleTypeDef* hi2c){
	ms5837->fluidDensity = 1029.0f;
	ms5837->hi2c = hi2c;
	return I2C_SUCCESS;
}


tI2C_Status MS5837_30BA_Calc(tMS5837 *ms5837){
	int32_t dT, OFFi, Ti,SENSi = 0;
	int64_t SENS, OFF, OFF2, SENS2 = 0;

	dT = ms5837->val.D2 - ((uint32_t)ms5837->val.C[5] * 256l);
	OFF = (int64_t)ms5837->val.C[2] * 65536l + ((int64_t)dT * (int64_t)ms5837->val.C[4]) / 128l;
	SENS = (int64_t)ms5837->val.C[1] * 32768l + ((int64_t)dT * (int64_t)ms5837->val.C[3]) / 256l;

	ms5837->val.TEMP = 2000l + ((int64_t)dT * ms5837->val.C[6]) / 8388608LL;
	ms5837->val.P = (((int64_t)ms5837->val.D1 * SENS) / 2097152l - OFF) / 8192l;

	if((ms5837->val.TEMP/100) < 20){
		Ti = (3*(int64_t)dT*(int64_t)dT)/8589934592LL;
		OFFi = (3*(ms5837->val.TEMP-2000l)*(ms5837->val.TEMP-2000l))/2;
		SENSi = (5*(ms5837->val.TEMP-2000l)*(ms5837->val.TEMP-2000l))/8;

		if((ms5837->val.TEMP / 100) < -15){
			OFFi = OFFi + 7 * (ms5837->val.TEMP + 1500l) * (ms5837->val.TEMP + 1500l);
			SENSi = SENSi + 4 * (ms5837->val.TEMP + 1500l) * (ms5837->val.TEMP + 1500l);
		}
	}
	else{
		Ti = (2*(int64_t)dT*(int64_t)dT)/137438953472LL;
		OFFi = ((ms5837->val.TEMP-2000l)*(ms5837->val.TEMP-2000l))/16;
		SENSi = 0;
	}

	OFF2 = OFF-OFFi;
	SENS2 = SENS-SENSi;

	ms5837->val.TEMP = (ms5837->val.TEMP - Ti);

	ms5837->val.P = ((ms5837->val.D1 * SENS2) / 2097152l - OFF2)/ 8192l;

	ms5837->temperature = (float) ms5837->val.TEMP / 100.0f;             // result of temperature in deg C
	ms5837->pressure = (float) ms5837->val.P / 10.0f;                 // BAR30 result of pressure in mBar

	return I2C_SUCCESS;
}


tI2C_Status MS5837_Task(tMS5837* ms5837, tROV* rov){
	eMS5837_Task_States state = ms5837->state;
	tI2C_Status ret = I2C_SUCCESS;

		current_ms = uwTick;

		if(current_ms - prev_ms < ms5837->taskPeriod){
			return I2C_TIMEOUT;
		}
		prev_ms = current_ms;



	switch (state) {
		case MS5837_RESET:
			ret = MS5837_Reset(ms5837);
			ms5837->state = (ret == I2C_SUCCESS) ? state + 1 : state;
			ms5837->taskPeriod = 20;
			break;

		case MS5837_CALIBRATION:
			ret = MS5837_Calibration(ms5837, calib_idx);
			calib_idx = (ret == I2C_SUCCESS) ? calib_idx + 1 : calib_idx;
			if((calib_idx == 7)){
				if((ms5837->val.C[0] >> 12) == crc4(ms5837->val.C)){
					ms5837->state++;
				}
				else{
					calib_idx = 0;
				}
			}
			ms5837->taskPeriod = 20;
			break;

		case MS5837_CONVERT_D1:
			ret = MS5837_RequestD1(ms5837);
			ms5837->state = (ret == I2C_SUCCESS) ? state + 1: state;
			ms5837->taskPeriod = 20;
			break;

		case MS5837_COLLECT_D1:
			ret = MS5837_CollectD1(ms5837);
			ms5837->state = (ret == I2C_SUCCESS) ? state + 1: state;
			ms5837->taskPeriod = 1;
			break;

		case MS5837_CONVERT_D2:
			ret = MS5837_RequestD2(ms5837);
			ms5837->state = (ret == I2C_SUCCESS) ? state + 1: state;
			ms5837->taskPeriod = 20;
			break;

		case MS5837_COLLECT_D2:
			ret = MS5837_CollectD2(ms5837);
			ms5837->state = (ret == I2C_SUCCESS) ? state + 1: state;
			ms5837->taskPeriod = 1;
			break;

		case MS5837_CALC_DEPTH:
			ret = MS5837_30BA_Calc(ms5837);
			ret += MS5837_Depth(ms5837);
			rov->telemetry.depth = (int16_t)(ms5837->depth * 100);
			ms5837->state = MS5837_CONVERT_D1;
			break;

		default:
			ms5837->state = MS5837_RESET;
			break;
	}

	if (ret == I2C_SUCCESS){
		rov->telemetry.status &= (~PressSensError);
	}
	else{
		rov->telemetry.status |= (PressSensError);
	}

	ms5837->errorCount += ret;
	return ret;

}


tI2C_Status MS5837_Reset(tMS5837* ms5837){
	uint8_t resetCmd = MS5837_CMD_RESET;
	if(HAL_OK != HAL_I2C_Master_Transmit(ms5837->hi2c, MS5837_HAL_IC2_ADDR, &resetCmd, 1, 3)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}


tI2C_Status MS5837_Calibration(tMS5837* ms5837, uint8_t i){
	uint8_t tempBuffer[2] = {0};
	uint8_t calibCmd = (MS5837_CMD_PROM_READ + i * 2);
	if(HAL_OK == HAL_I2C_Master_Transmit(ms5837->hi2c, MS5837_HAL_IC2_ADDR, &calibCmd, 1, 3)){
		if(HAL_OK == HAL_I2C_Master_Receive(ms5837->hi2c, MS5837_HAL_IC2_ADDR, (uint8_t*)tempBuffer, 2, 3)){
			ms5837->val.C[i] = (uint16_t)((tempBuffer[0] << 8) | tempBuffer[1]);
			return I2C_SUCCESS;
		}
	}
	return I2C_FAIL;
}


tI2C_Status MS5837_RequestD1(tMS5837* ms5837){
	uint8_t d1Cmd = MS5837_CMD_CONVERT_D1_8192;
	if(HAL_OK != HAL_I2C_Master_Transmit(ms5837->hi2c, MS5837_HAL_IC2_ADDR, &d1Cmd, 1, 3)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}


tI2C_Status MS5837_RequestD2(tMS5837* ms5837){
	uint8_t d2Cmd = MS5837_CMD_CONVERT_D2_8192;
	if(HAL_OK != HAL_I2C_Master_Transmit(ms5837->hi2c, MS5837_HAL_IC2_ADDR, &d2Cmd, 1, 3)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}


tI2C_Status MS5837_CollectD1(tMS5837* ms5837){
	uint8_t tempBuffer[3];
	uint8_t adcCmd = MS5837_CMD_ADC_READ;
	if(HAL_OK == HAL_I2C_Master_Transmit(ms5837->hi2c, MS5837_HAL_IC2_ADDR, &adcCmd, 1, 3)){
		if(HAL_OK == HAL_I2C_Master_Receive(ms5837->hi2c, MS5837_HAL_IC2_ADDR, (uint8_t*)tempBuffer, 3, 3)){
			ms5837->val.D1= (uint32_t)((tempBuffer[2] |(tempBuffer[1] << 8) | (tempBuffer[0] << 16)));
			return I2C_SUCCESS;
		}
	}
	return I2C_FAIL;
}


tI2C_Status MS5837_CollectD2(tMS5837* ms5837){
	uint8_t tempBuffer[3];
	uint8_t adcCmd = MS5837_CMD_ADC_READ;
	if(HAL_OK == HAL_I2C_Master_Transmit(ms5837->hi2c, MS5837_HAL_IC2_ADDR, &adcCmd, 1, 3)){
		if(HAL_OK == HAL_I2C_Master_Receive(ms5837->hi2c, MS5837_HAL_IC2_ADDR, (uint8_t*)tempBuffer, 3, 3)){
			ms5837->val.D2= (uint32_t)((tempBuffer[2] |(tempBuffer[1] << 8) | (tempBuffer[0] << 16)));
			return I2C_SUCCESS;
		}
	}
	return I2C_FAIL;
}


tI2C_Status MS5837_Depth(tMS5837 *ms5837) {
	ms5837->depth = ((ms5837->pressure * 100 / 10) - 101300) / (ms5837->fluidDensity * 9.80665);
	return I2C_SUCCESS;
}


static inline void MS5837_SetDelay(tMS5837* ms5837, uint32_t ms){
	if(ms < 0 || ms > 1000)
		return ;
	ms5837->taskPeriod = ms;
}


static uint8_t crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}

