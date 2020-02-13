#ifndef __SX127X_HAL_H__
#define __SX127X_HAL_H__
/*
 * LoRa sx1278/76驱动
 *
 * \version    1.0.0 
 * \date       Dec 17 2019
 * \author     Specter
 */
 
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "stdio.h"

#define DEBUG( format , ... )	printf( format , ##__VA_ARGS__ )	//打印log信息的函数，可以不设置
#define GET_TICK_COUNT() GetTick()	//获取systick的函数

void Soft_delay_ms(uint16_t time);
void SX1276HALInit( void );
uint8_t SpiInOut( uint8_t outData );
void SpiNSSEnable( GPIO_PinState status );
void SX127X_ResetPinControl( GPIO_PinState status );
GPIO_PinState SX1276ReadDio0(void);
GPIO_PinState SX1276ReadDio1(void);
//GPIO_PinState SX1276ReadDio2(void);
GPIO_PinState SX1276ReadDio3(void);
GPIO_PinState SX1276ReadDio4(void);
//GPIO_PinState SX1276ReadDio5(void);

#endif //end of __SX127X_HAL_H__
