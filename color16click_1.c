/*
 * color16click.c
 *
 *  Created on: Jan 23, 2025
 *      Author: Asus
 */

#include <stdint.h>
#include <string.h>
#include "color16click_1.h"

extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef status;
USBD_StatusTypeDef status_USB;
uint8_t *pointerledtoggle;
uint32_t LastInterruptTime = 0;

uint8_t AccessBank(uint8_t BankAccessData) {

	status = HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ACCESS_BANK, 1,
			&BankAccessData, 1, 100);
	if (status != HAL_OK) {
		return 0;
	} else {
		return 1;
	}
}


void SetLDR(uint8_t led_value_mA) {

	uint8_t led_binary_value = (led_value_mA - 4) / 2;
	uint8_t led_initialize_value = (led_binary_value | 0x80);

	pointerledtoggle = &led_initialize_value;

	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, LED_REG, 1, &led_initialize_value,
			1, 100);

}
void FD_Init(float time, uint32_t GainValue)
{
	uint32_t integTime = 0;
	uint8_t integTime_LSB=0;
	uint8_t integTime_MSB=0;
	uint8_t MSB_Value=0;

		if ((time > 2.78e-6))
		{

			integTime = (uint32_t) (time * 1000000 / 2.78);
			integTime_LSB =(uint8_t)integTime&0xFF;
			integTime_MSB = (uint8_t)((integTime>>8)&0xFF);
			HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, FD_TIME, 1, &integTime_LSB, 1, 100);
			HAL_Delay(50);

		}

	uint8_t Gain_Value;
	if (2048 == (GainValue & 2048)) {
		Gain_Value = GAIN_2048X_FD;
	} else if (1024 == (GainValue & 1024)) {
		Gain_Value = GAIN_1024X_FD;
	} else if (512 == (GainValue & 512)) {
		Gain_Value = GAIN_512X_FD;
	} else if (256 == (GainValue & 256)) {
		Gain_Value = GAIN_256X_FD;
	} else if (128 == (GainValue & 128)) {
		Gain_Value = GAIN_128X_FD;
	} else if (64 == (GainValue & 64)) {
		Gain_Value = GAIN_64X_FD;
	} else if (32 == (GainValue & 32)) {
		Gain_Value = GAIN_32X_FD;
	} else if (16 == (GainValue & 16)) {
		Gain_Value = GAIN_16X_FD;
	} else if (8 == (GainValue & 8)) {
		Gain_Value = GAIN_8X_FD;
	} else if (4 == (GainValue & 4)) {
		Gain_Value = GAIN_4X_FD;
	} else if (2 == (GainValue & 2)) {
		Gain_Value = GAIN_2X_FD;
	} else if (1 == (GainValue & 1)) {
		Gain_Value = GAIN_1X_FD;
	} else if (0 == (GainValue & 0)) {
		Gain_Value = GAIN_0p5x_FD;
	}
	MSB_Value=Gain_Value|integTime_MSB;
	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, GAIN_TIME_REG_FD, 1, &MSB_Value, 1, 100);

}


void SetGain(uint32_t GainValue) {
	uint8_t Gain_Value;
	if (2048 == (GainValue & 2048)) {
		Gain_Value = GAIN_2048x;
	} else if (1024 == (GainValue & 1024)) {
		Gain_Value = GAIN_1024x;
	} else if (512 == (GainValue & 512)) {
		Gain_Value = GAIN_512x;
	} else if (256 == (GainValue & 256)) {
		Gain_Value = GAIN_256x;
	} else if (128 == (GainValue & 128)) {
		Gain_Value = GAIN_128x;
	} else if (64 == (GainValue & 64)) {
		Gain_Value = GAIN_64x;
	} else if (32 == (GainValue & 32)) {
		Gain_Value = GAIN_32x;
	} else if (16 == (GainValue & 16)) {
		Gain_Value = GAIN_16x;
	} else if (8 == (GainValue & 8)) {
		Gain_Value = GAIN_8x;
	} else if (4 == (GainValue & 4)) {
		Gain_Value = GAIN_4x;
	} else if (2 == (GainValue & 2)) {
		Gain_Value = GAIN_2x;
	} else if (1 == (GainValue & 1)) {
		Gain_Value = GAIN_1x;
	} else if (0 == (GainValue & 0)) {
		Gain_Value = GAIN_0p5x;
	}
	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, GAIN_REG, 1, &Gain_Value, 1, 100);
}

void setWtime(float time)
{
	uint8_t time_ms=time*1000;
	uint8_t TimeValue=(uint8_t)((time_ms)/2.78-1);
	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS,WTIME_REG, 1, &TimeValue, 1, 100);
}


void setIntegrationTime(float time) {
	uint32_t integTime = 0;
	uint8_t aTime = 0;
	uint16_t aStep = 1000;
	uint8_t SendDataLow, SendDataHigh;
	if ((time > 2.78e-6)) {
//	time saniye cinsinden integrason zamanı
		integTime = (uint32_t) (time * 1000000 / 2.78);
		aTime = integTime / 65536;
		aStep = integTime / (aTime + 1) - 1;
		SendDataLow = aStep & 0xFF;
		SendDataHigh = aStep >> 8 & 0xFF;
		HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ATIME_REG, 1, &aTime, 1, 100);
		HAL_Delay(50);
		HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ASTEP_REG_LOW, 1, &SendDataLow,
				1, 100);
		HAL_Delay(50);
		HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ASTEP_REG_HIGH, 1,
				&SendDataHigh, 1, 100);

		// 5 sn 28 atime 64234 astep
	}
}

uint8_t SensorInit() {
	uint8_t PowerOff = POW_OFF;
	uint8_t PowerOn = POW_ON;


	uint8_t SMUXDefault = SMUX_DEFAULT;
	uint8_t POW_SP_WEN_ON = (0x0B);

	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ENABLE_SENSOR, 1, &PowerOff, 1,
			100);
	HAL_Delay(50);
	FD_Init(0.0001,0);
	HAL_Delay(50);
	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ENABLE_SENSOR, 1, &PowerOn, 1,
			100);
	HAL_Delay(50);
	setIntegrationTime(0.1);
	HAL_Delay(50);
	setWtime(0.01);
	HAL_Delay(50);
	SetGain(32);
	HAL_Delay(50);
	SetLDR(4);
	HAL_Delay(50);
	HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, SMUX_REG, 1, &SMUXDefault, 1, 100);
	HAL_Delay(50);
	status = HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADRESS, ENABLE_SENSOR, 1,
			&POW_SP_WEN_ON, 1, 100);

	if (status != HAL_OK)

	{
		return 0;
	} else {
		return 1;
	}
}




uint8_t ReadOneData() {
	uint8_t value;
	HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADRESS, ENABLE_SENSOR, 1, &value, 1, 100);
	return value;
}

uint8_t ReadData(color16_data *data_out) {
	uint8_t data_buf[38];
	status = HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADRESS, 0X93, 1, data_buf, 38,
			100);
	data_out->sstatus = data_buf[0];
	data_out->astatus = data_buf[1];
	data_out->ch_fz = ((uint16_t) data_buf[3] << 8) | data_buf[2];
	data_out->ch_fy = ((uint16_t) data_buf[5] << 8) | data_buf[4];
	data_out->ch_fxl = ((uint16_t) data_buf[7] << 8) | data_buf[6];
	data_out->ch_nir = ((uint16_t) data_buf[9] << 8) | data_buf[8];
	data_out->ch_2x_vis_1 = ((uint16_t) data_buf[11] << 8) | data_buf[10];
	data_out->ch_fd_1 = ((uint16_t) data_buf[13] << 8) | data_buf[12];
	data_out->ch_f2 = ((uint16_t) data_buf[15] << 8) | data_buf[14];
	data_out->ch_f3 = ((uint16_t) data_buf[17] << 8) | data_buf[16];
	data_out->ch_f4 = ((uint16_t) data_buf[19] << 8) | data_buf[18];
	data_out->ch_f6 = ((uint16_t) data_buf[21] << 8) | data_buf[20];
	data_out->ch_2x_vis_2 = ((uint16_t) data_buf[23] << 8) | data_buf[22];
	data_out->ch_fd_2 = ((uint16_t) data_buf[25] << 8) | data_buf[24];
	data_out->ch_f1 = ((uint16_t) data_buf[27] << 8) | data_buf[26];
	data_out->ch_f5 = ((uint16_t) data_buf[29] << 8) | data_buf[28];
	data_out->ch_f7 = ((uint16_t) data_buf[31] << 8) | data_buf[30];
	data_out->ch_f8 = ((uint16_t) data_buf[33] << 8) | data_buf[32];
	data_out->ch_2x_vis_3 = ((uint16_t) data_buf[35] << 8) | data_buf[34];
	data_out->ch_fd_3 = ((uint16_t) data_buf[37] << 8) | data_buf[36];
	if (status != HAL_OK) {
		return 0;
	} else {
		return 1;
	}
}

uint8_t SendData(color16_data *colorSenddata) {
	char sendData[114];
	sprintf(sendData,
			"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
			colorSenddata->sstatus, colorSenddata->ch_f1, colorSenddata->ch_f2,
			colorSenddata->ch_fz, colorSenddata->ch_f3, colorSenddata->ch_f4,
			colorSenddata->ch_fy, colorSenddata->ch_f5, colorSenddata->ch_fxl,
			colorSenddata->ch_f6, colorSenddata->ch_f7, colorSenddata->ch_f8,
			colorSenddata->ch_2x_vis_1, colorSenddata->ch_2x_vis_2,
			colorSenddata->ch_2x_vis_3, colorSenddata->ch_nir,
			colorSenddata->ch_fd_1, colorSenddata->ch_fd_2,
			colorSenddata->ch_fd_3);
	status_USB = CDC_Transmit_FS((uint8_t*) sendData,
			(uint16_t) strlen(sendData));

	if (status_USB != USBD_OK) {
		return 0;
	} else {
		return 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t CurrentTime = HAL_GetTick();

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
		if ((CurrentTime - LastInterruptTime <= 200)) {
			return;
		}

		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
	}
	LastInterruptTime = CurrentTime;

}





