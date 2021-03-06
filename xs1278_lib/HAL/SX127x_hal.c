#include "hal/sx127x_hal.h"
#include "main.h"
/*
 * LoRa sx1278/76驱动
 *
 * \version    1.0.0 
 * \date       Dec 17 2019
 * \author     Specter
 */

#define RESET_IOPORT                                GPIOB
#define RESET_PIN                                   GPIO_Pin_10

/*!
 * SX1276 SPI definitions
 */
#define NSS_IOPORT                                  GPIOA
#define NSS_PIN                                     GPIO_Pin_4

#define SPI_INTERFACE                               SPI1
#define SPI_CLK                                     RCC_APB2Periph_SPI1

#define SPI_PIN_SCK_PORT                            GPIOA
#define SPI_PIN_SCK_PORT_CLK                        RCC_APB2Periph_GPIOA
#define SPI_PIN_SCK                                 GPIO_Pin_5

#define SPI_PIN_MISO_PORT                           GPIOA
#define SPI_PIN_MISO_PORT_CLK                       RCC_APB2Periph_GPIOA
#define SPI_PIN_MISO                                GPIO_Pin_6

#define SPI_PIN_MOSI_PORT                           GPIOA
#define SPI_PIN_MOSI_PORT_CLK                       RCC_APB2Periph_GPIOA
#define SPI_PIN_MOSI                                GPIO_Pin_7

/*!
 * SX1276 DIO pins  I/O definitions
 */
#define DIO0_IOPORT                                 GPIOB
#define DIO0_PIN                                    GPIO_Pin_0

#define DIO1_IOPORT                                 GPIOB
#define DIO1_PIN                                    GPIO_Pin_1

#define DIO3_IOPORT                                 GPIOA
#define DIO3_PIN                                    GPIO_Pin_1

#define DIO4_IOPORT                                 GPIOA
#define DIO4_PIN                                    GPIO_Pin_0

//软件延时函数，ms级别
void Soft_delay_ms(uint16_t time)
{    
   HAL_Delay(time);
}

//spi初始化
//static void SpiInit( void )
//{
//    SPI_InitTypeDef SPI_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;

//    /* Enable peripheral clocks --------------------------------------------------*/
//    /* Enable SPIy clock and GPIO clock for SPIy */
//    RCC_APB2PeriphClockCmd( SPI_PIN_MISO_PORT_CLK | SPI_PIN_MOSI_PORT_CLK |
//                            SPI_PIN_SCK_PORT_CLK, ENABLE );
//    RCC_APB2PeriphClockCmd( SPI_CLK, ENABLE );

//    /* GPIO configuration ------------------------------------------------------*/
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

//    GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK;
//    GPIO_Init( SPI_PIN_SCK_PORT, &GPIO_InitStructure );

//    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MOSI;
//    GPIO_Init( SPI_PIN_MOSI_PORT, &GPIO_InitStructure );

//    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO;
//    GPIO_Init( SPI_PIN_MISO_PORT, &GPIO_InitStructure );

//    //禁用JTAG
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
//    GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );

//    /* SPI_INTERFACE Config -------------------------------------------------------------*/
//    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 72/8 MHz
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
//    SPI_Init( SPI_INTERFACE, &SPI_InitStructure );
//    SPI_Cmd( SPI_INTERFACE, ENABLE );
//}


//SX127X相关初始化需要设置如下内容
//SPI片选设置为输出,并初始化SPI口
//复位角初始化为输出高电平
//DI00~5为输入（暂时只用到了DIO0和DIO1，如果不用硬件超时DIO1也可以不接）
//void SX1276HALInit( void )
//{
//    GPIO_InitTypeDef GPIO_InitStructure;


//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
//                            RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    // Configure SPI-->NSS as output
//    GPIO_InitStructure.GPIO_Pin = NSS_PIN;
//    GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );
//		GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
//		SpiInit();
//	
//		//配置复位引脚
//		GPIO_InitStructure.GPIO_Pin = RESET_PIN;
//    GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
//		GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_SET );
//	
//    // Configure radio DIO as inputs
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

//    // Configure DIO0
//    GPIO_InitStructure.GPIO_Pin =  DIO0_PIN;
//    GPIO_Init( DIO0_IOPORT, &GPIO_InitStructure );
//    
//    // Configure DIO1
//    GPIO_InitStructure.GPIO_Pin =  DIO1_PIN;
//    GPIO_Init( DIO1_IOPORT, &GPIO_InitStructure );
//		
//		// Configure DIO3
//    GPIO_InitStructure.GPIO_Pin =  DIO3_PIN;
//    GPIO_Init( DIO3_IOPORT, &GPIO_InitStructure );
//		
//		// Configure DIO4
//    GPIO_InitStructure.GPIO_Pin =  DIO4_PIN;
//    GPIO_Init( DIO4_IOPORT, &GPIO_InitStructure );
//}

//spi发送和接收函数
uint8_t SpiInOut(uint8_t outData )
{
	/* Send SPIy data */
	uint8_t readData;
	HAL_SPI_TransmitReceive(&hspi1, &outData, &readData, 1, 0xffff);
	return readData;
}

//spi片选，status=0使能（NSS拉低）status=1失能（NSS拉高）
void SpiNSSEnable(GPIO_PinState status )
{
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, status);
}

//复位引脚控制status=0拉低，其他拉高
void SX127X_ResetPinControl(GPIO_PinState status ){
	HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, status);
}

//读取DIO0电平，返回值0低电平，1高电平
GPIO_PinState SX1276ReadDio0(void){
	return HAL_GPIO_ReadPin(LoRa_DIO0_GPIO_Port, LoRa_DIO0_Pin);
}


GPIO_PinState SX1276ReadDio1(void){
	return HAL_GPIO_ReadPin(LoRa_DIO1_GPIO_Port, LoRa_DIO1_Pin);
}

//uint8_t SX1276ReadDio2(void){
//	return GPIO_ReadInputDataBit( DIO2_IOPORT, DIO2_PIN );
//}

GPIO_PinState SX1276ReadDio3(void){
	return HAL_GPIO_ReadPin(LoRa_DIO3_GPIO_Port, LoRa_DIO3_Pin);
}

GPIO_PinState SX1276ReadDio4(void){
	return HAL_GPIO_ReadPin(LoRa_DIO4_GPIO_Port, LoRa_DIO4_Pin);
}

//uint8_t SX1276ReadDio5(void){
//	return GPIO_ReadInputDataBit( DIO5_IOPORT, DIO5_PIN );
//}
