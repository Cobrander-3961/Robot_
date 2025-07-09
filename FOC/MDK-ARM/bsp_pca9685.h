/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * 
 Change Logs:
 * Date           Author       Notes
 * 2024-03-22     LCKFB-LP    first version
 */

#ifndef _BSP_PCA9685_H_
#define _BSP_PCA9685_H_
#include "main.h"


//端口移植
#define RCC_PCA9685_GPIO  	RCC_AHB1Periph_GPIOA
#define PORT_PCA9685 		GPIOB

#define GPIO_SDA 			GPIO_PIN_6
#define GPIO_SCL 			GPIO_PIN_7

//设置SDA输出模式
//#define SDA_OUT()   {        \
//                        GPIO_InitTypeDef  GPIO_InitStructure; \
//                        GPIO_InitStructure.GPIO_Pin = GPIO_SDA; \
//                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
//                        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
//                        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; \
//                        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
//                        GPIO_Init(GPIOA, &GPIO_InitStructure); \
//                     }
//设置SDA输入模式
//#define SDA_IN()    {        \
//                        GPIO_InitTypeDef  GPIO_InitStructure; \
//                        GPIO_InitStructure.GPIO_Pin = GPIO_SDA; \
//                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
//                        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
//                        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; \
//                        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
//                        GPIO_Init(GPIOA, &GPIO_InitStructure); \
//                    }
//获取SDA引脚的电平变化
#define SDA_GET()       HAL_GPIO_ReadPin(GPIOB, GPIO_SDA)
//SDA与SCL输出
#define SDA(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (x?SET:RESET) )
#define SCL(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (x?SET:RESET) )

#define PCA_Addr              0x80        //IIC地址
#define PCA_Model             0x00        
#define LED0_ON_L             0x06
#define LED0_ON_H             0x07
#define LED0_OFF_L            0x08
#define LED0_OFF_H            0x09
#define PCA_Pre               0xFE        //配置频率地址

void PCA9685_Init(float hz,uint8_t angle);
void setAngle(uint8_t num,uint8_t angle);
void PCA9685_setFreq(float freq);
void PCA9685_setPWM(uint8_t num,uint32_t on,uint32_t off);

#endif
