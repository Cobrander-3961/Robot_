/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * 
 Change Logs:
 * Date           Author       Notes
 * 2024-03-22     LCKFB-LP    first version
 */

#ifndef _BSP_PCA9685_H_
#define _BSP_PCA9685_H_
#include "main.h"


//�˿���ֲ
#define RCC_PCA9685_GPIO  	RCC_AHB1Periph_GPIOA
#define PORT_PCA9685 		GPIOB

#define GPIO_SDA 			GPIO_PIN_6
#define GPIO_SCL 			GPIO_PIN_7

//����SDA���ģʽ
//#define SDA_OUT()   {        \
//                        GPIO_InitTypeDef  GPIO_InitStructure; \
//                        GPIO_InitStructure.GPIO_Pin = GPIO_SDA; \
//                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
//                        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
//                        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; \
//                        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
//                        GPIO_Init(GPIOA, &GPIO_InitStructure); \
//                     }
//����SDA����ģʽ
//#define SDA_IN()    {        \
//                        GPIO_InitTypeDef  GPIO_InitStructure; \
//                        GPIO_InitStructure.GPIO_Pin = GPIO_SDA; \
//                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
//                        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
//                        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; \
//                        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
//                        GPIO_Init(GPIOA, &GPIO_InitStructure); \
//                    }
//��ȡSDA���ŵĵ�ƽ�仯
#define SDA_GET()       HAL_GPIO_ReadPin(GPIOB, GPIO_SDA)
//SDA��SCL���
#define SDA(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (x?SET:RESET) )
#define SCL(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (x?SET:RESET) )

#define PCA_Addr              0x80        //IIC��ַ
#define PCA_Model             0x00        
#define LED0_ON_L             0x06
#define LED0_ON_H             0x07
#define LED0_OFF_L            0x08
#define LED0_OFF_H            0x09
#define PCA_Pre               0xFE        //����Ƶ�ʵ�ַ

void PCA9685_Init(float hz,uint8_t angle);
void setAngle(uint8_t num,uint8_t angle);
void PCA9685_setFreq(float freq);
void PCA9685_setPWM(uint8_t num,uint32_t on,uint32_t off);

#endif
