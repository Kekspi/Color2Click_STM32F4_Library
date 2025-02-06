/*
 * color16click_1.h
 *
 *  Created on: Jan 29, 2025
 *      Author: Asus
 */

#ifndef INC_COLOR16CLICK_1_H_
#define INC_COLOR16CLICK_1_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern I2C_HandleTypeDef hi2c1;

#define SENSOR_ADRESS     (0x39<<1)
#define ENABLE_SENSOR     (0x80)
#define POW_ON            (0x01)
#define POW_OFF           (0x00)
#define SP_EN_ON          (0x02)
#define WEN_ON            (0x08)
#define FD_EN             ()


#define ACCESS_BANK       (0xBF)  //0 to access >=0x80
//ADC Configuration
#define CFG1              (0xC6) //ADC Gain config
#define CONTROL_REG       (0xFA) //ADC Control Register
#define SW_RESET          (0x08)
//Led Setup
#define LED_REG           (0xCD)
#define LED_ON            (0X80)
#define LED_OFF           (0X00)

#define LED_DRIVE         ()
//integration time = (ATIME+1) x (ASTEP + 1) x 2.78
#define ASTEP_REG_LOW     (0xD4)
#define ASTEP_REG_HIGH    (0xD5)
#define ATIME_REG         (0x81)
#define ATIME_1_TO_255    (0x00)

#define WTIME_REG         (0x83)
#define WTIME_DEFAULT     (0x00)

#define GAIN_REG          (0xC6)
#define GAIN_0p5x         (0x00)
#define GAIN_1x           (0x01)
#define GAIN_2x           (0x02)
#define GAIN_4x           (0x03)
#define GAIN_8x           (0x04)
#define GAIN_16x          (0x05)
#define GAIN_32x          (0x06)
#define GAIN_64x          (0x07)
#define GAIN_128x         (0x08)
#define GAIN_256x         (0x09)
#define GAIN_512x         (0x0A)
#define GAIN_1024x        (0x0B)
#define GAIN_2048x        (0x0C)

#define FD_TIME           (0xE0)
#define GAIN_TIME_REG_FD  (0xE2)
#define GAIN_0p5x_FD      (0x00)
#define GAIN_1X_FD        (0x08)
#define GAIN_2X_FD        (0x10)
#define GAIN_4X_FD        (0x18)
#define GAIN_8X_FD        (0x20)
#define GAIN_16X_FD       (0x28)
#define GAIN_32X_FD       (0x30)
#define GAIN_64X_FD       (0x38)
#define GAIN_128X_FD      (0x40)
#define GAIN_256X_FD      (0x48)
#define GAIN_512X_FD      (0x50)
#define GAIN_1024X_FD     (0x58)
#define GAIN_2048X_FD     (0x60)

#define SMUX_REG          (0xD6)

#define SMUX_DEFAULT      (0x60)

// Data Registers
typedef struct{
uint8_t sstatus;
uint8_t astatus;
  uint16_t ch_fz;
  uint16_t ch_fy;
  uint16_t ch_fxl;
  uint16_t ch_nir;
  uint16_t ch_2x_vis_1;
  uint16_t ch_fd_1;
  uint16_t ch_f2;
  uint16_t ch_f3;
  uint16_t ch_f4;
  uint16_t ch_f6;
  uint16_t ch_2x_vis_2;
  uint16_t ch_fd_2;
  uint16_t ch_f1;
  uint16_t ch_f5;
  uint16_t ch_f7;
  uint16_t ch_f8;
  uint16_t ch_2x_vis_3;
  uint16_t ch_fd_3;
}color16_data;


uint8_t AccessBank(uint8_t BankAccessData);


void FD_Init(float time,uint32_t GainValue);
void SetWtime(float time_ms);
void setIntegrationTime(float time);
void SetGain(uint32_t GainValue);

uint8_t SensorInit();
uint8_t ReadOneData();
uint8_t ReadData();
void ReturnData(color16_data* colordata);
uint8_t SendData(color16_data* colorSenddata);






#endif /* INC_COLOR16CLICK_1_H_ */
