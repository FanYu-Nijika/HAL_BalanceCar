#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "./SYSTEM/delay/delay.h"

/*===========================================================================
 * Software I2C GPIO defines for MPU6050
 * ATK-F103 balance car: PB10=SCL, PB11=SDA
 *===========================================================================*/
#define I2C_SCL_Pin              GPIO_PIN_14
#define I2C_SDA_Pin              GPIO_PIN_15
#define I2C_SCL_GPIO_Port        GPIOB
#define I2C_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define MPU6050_INT_Pin          GPIO_PIN_9
#define MPU6050_INT_GPIO_Port    GPIOB

#endif /* __MAIN_H */
