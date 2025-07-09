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

#include "bsp_pca9685.h"
#include "stdio.h"
//#include "board.h"
#include <math.h>

/******************************************************************
 * �� �� �� �ƣ�PCA9685_GPIO_Init
 * �� �� ˵ ����PCA9685�����ų�ʼ��
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void PCA9685_GPIO_Init(void)
{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_PCA9685_GPIO, ENABLE);

//	GPIO_InitStructure.GPIO_Pin = GPIO_SDA|GPIO_SCL;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(PORT_PCA9685, &GPIO_InitStructure);
}
#define SDA_PIN  GPIO_PIN_6
#define SDA_PORT GPIOB
void SDA_OUT()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // �������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   // 50MHz����Ӧģʽֵ 0x3��
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);

}

void SDA_IN()
{

	    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // ����ģʽ
    GPIO_InitStruct.Pull = GPIO_NOPULL;             // �������루ԭ����δ����������
    // GPIO_InitStruct.Pull = GPIO_PULLUP;          // ������������
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);

}
/******************************************************************
 * �� �� �� �ƣ�IIC_Start
 * �� �� ˵ ����IIC��ʼʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void IIC_Start(void)
{
        SDA_OUT();
        
        SDA(1);
        delay_us(5);
        SCL(1); 
        delay_us(5);
        
        SDA(0);
        delay_us(5);
        SCL(0);
        delay_us(5);
                       
}
/******************************************************************
 * �� �� �� �ƣ�IIC_Stop
 * �� �� ˵ ����IICֹͣ�ź�
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void IIC_Stop(void)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        
        SCL(1);
        delay_us(5);
        SDA(1);
        delay_us(5);
        
}

/******************************************************************
 * �� �� �� �ƣ�IIC_Send_Ack
 * �� �� ˵ ������������Ӧ����߷�Ӧ���ź�
 * �� �� �� �Σ�0����Ӧ��  1���ͷ�Ӧ��
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void IIC_Send_Ack(unsigned char ack)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        delay_us(5);
        if(!ack) SDA(0);
        else         SDA(1);
        SCL(1);
        delay_us(5);
        SCL(0);
        SDA(1);
}


/******************************************************************
 * �� �� �� �ƣ�I2C_WaitAck
 * �� �� ˵ �����ȴ��ӻ�Ӧ��
 * �� �� �� �Σ���
 * �� �� �� �أ�0��Ӧ��  1��ʱ��Ӧ��
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
unsigned char I2C_WaitAck(void)
{
        
	char ack = 0;
	unsigned char ack_flag = 10;
	SCL(0);
	SDA(1);
	SDA_IN();
	delay_us(5);
	SCL(1);
	delay_us(5);

	while( (SDA_GET()==1) && ( ack_flag ) )
	{
			ack_flag--;
			delay_us(5);
	}
	
	if( ack_flag <= 0 )
	{
			IIC_Stop();
			return 1;
	}
	else
	{
			SCL(0);
			SDA_OUT();
	}
	return ack;
}

/******************************************************************
 * �� �� �� �ƣ�Send_Byte
 * �� �� ˵ ����д��һ���ֽ�
 * �� �� �� �Σ�datҪд�˵�����
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void Send_Byte(uint8_t dat)
{
        int i = 0;
        SDA_OUT();
        SCL(0);//����ʱ�ӿ�ʼ���ݴ���
        
        for( i = 0; i < 8; i++ )
        {
                SDA( (dat & 0x80) >> 7 );
                delay_us(1);
                SCL(1);
                delay_us(5);
                SCL(0);
                delay_us(5);
                dat<<=1;
        }        
}

/******************************************************************
 * �� �� �� �ƣ�Read_Byte
 * �� �� ˵ ����IIC��ʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ�����������
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
unsigned char Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
		SCL(0);
		delay_us(5);
		SCL(1);
		delay_us(5);
		receive<<=1;
		if( SDA_GET() )
		{        
			receive|=1;   
		}
		delay_us(5); 
	}                                         
	SCL(0); 
	return receive;
}

/******************************************************************
 * �� �� �� �ƣ�PCA9685_Write
 * �� �� ˵ ������PCA9685д���������
 * �� �� �� �Σ�addrд��ļĴ�����ַ    dataд������������
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void PCA9685_Write(uint8_t addr,uint8_t data)
{
        IIC_Start();
        
        Send_Byte(PCA_Addr);
        I2C_WaitAck();
        
        Send_Byte(addr);
        I2C_WaitAck();
        
        Send_Byte(data);
        I2C_WaitAck();
        
        IIC_Stop();
}
 
/******************************************************************
 * �� �� �� �ƣ�PCA9685_Read
 * �� �� ˵ ������ȡPCA9685����
 * �� �� �� �Σ�addr��ȡ�ļĴ�����ַ
 * �� �� �� �أ���ȡ������
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
uint8_t PCA9685_Read(uint8_t addr)
{
        uint8_t data;
        
        IIC_Start();
        
        Send_Byte(PCA_Addr);
        I2C_WaitAck();
        
        Send_Byte(addr);
        I2C_WaitAck();
        
        IIC_Stop();
        
        delay_us(10);
 
        
        IIC_Start();
 
        Send_Byte(PCA_Addr|0x01);
        I2C_WaitAck();
        
        data = Read_Byte();
        IIC_Send_Ack(1);
        IIC_Stop();
        
        return data;
}
/******************************************************************
 * �� �� �� �ƣ�PCA9685_setPWM
 * �� �� ˵ �������õ�num��PWM���ţ�onĬ��Ϊ0�����ƶ����תoff�Ƕ�
 * �� �� �� �Σ�num�����õڼ��������������Χ0~15
 *              on ��Ĭ��Ϊ0
 *              off�������ת�Ƕȣ���Χ��0~180
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void PCA9685_setPWM(uint8_t num,uint32_t on,uint32_t off)
{
        IIC_Start();
        
        Send_Byte(PCA_Addr);
        I2C_WaitAck();
        
        Send_Byte(LED0_ON_L+4*num);
        I2C_WaitAck();
        
        Send_Byte(on&0xFF);
        I2C_WaitAck();
        
        Send_Byte(on>>8);
        I2C_WaitAck();
        
        Send_Byte(off&0xFF);
        I2C_WaitAck();
        
        Send_Byte(off>>8);
        I2C_WaitAck();
        
        IIC_Stop();
        
}
 
 

/******************************************************************
 * �� �� �� �ƣ�PCA9685_setFreq
 * �� �� ˵ ��������PCA9685�����Ƶ��
 * �� �� �� �Σ�freq
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע��
floor�﷨��
FLOOR(number, significance)
    Number���衣Ҫ�������ֵ�� 
    Significance���衣Ҫ���뵽�ı����� 
˵��
    ������ number �������루�ؾ���ֵ��С�ķ���Ϊ��ӽ��� significance �ı����� 
    �����һ����Ϊ����ֵ�ͣ��� FLOOR �����ش���ֵ #VALUE!�� 
    ��� number �ķ���Ϊ������ significance �ķ���Ϊ������ FLOOR �����ش���ֵ #NUM!
ʾ��
    ��ʽ                                ˵��                                                                ���
    FLOOR(3.7,2)                �� 3.7 �ؾ���ֵ��С�ķ����������룬ʹ�������ӽ��� 2 �ı���                2
    FLOOR(-2.5, -2)                �� -2.5 �ؾ���ֵ��С�ķ����������룬ʹ�������ӽ��� -2 �ı���                -2
******************************************************************/
void PCA9685_setFreq(float freq)
{
        uint8_t prescale,oldmode,newmode;
        
        double prescaleval;
        
//        freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
 
//        PCA9685���ڲ�ʱ��Ƶ����25Mhz
//        ��ʽ: presale_Volue = round( 25000000/(4096 * update_rate) ) - 1
//        round = floor();  floor����ѧ��������Ҫ���� math.h �ļ�
//        update_rate = freq;
        prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        prescale = floor(prescaleval+0.5f);    
  
        //����MODE1��ַ�ϵ����ݣ������������ݣ�
        oldmode = PCA9685_Read(PCA_Model);
        
        //��MODE1������SLEEPλ
        newmode = (oldmode&0x7F)|0x10;
        //�����ĵ�MODE1��ֵд��MODE1��ַ,ʹоƬ˯��
        PCA9685_Write(PCA_Model,newmode);
        //д�����Ǽ��������Ƶ�ʵ�ֵ  
        //PCA_Pre = presale ��ַ��0xFE�����������ֲ�����ҵ�
        PCA9685_Write(PCA_Pre,prescale);
        //���¸�λ
        PCA9685_Write(PCA_Model,oldmode);
        //�ȴ���λ���
        delay_1ms(5);            
        //����MODE1�Ĵ��������Զ�����
        PCA9685_Write(PCA_Model,oldmode|0xa1);
}
 
//
/******************************************************************
 * �� �� �� �ƣ�setAngle
 * �� �� ˵ �������ýǶ�
 * �� �� �� �Σ�numҪ���õ�PWM����     angle���õĽǶ�
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void setAngle(uint8_t num,uint8_t angle)
{
        uint32_t off = 0;
        
        off = (uint32_t)(158+angle*2.2);
        
        PCA9685_setPWM(num,0,off);
}
 
/******************************************************************
 * �� �� �� �ƣ�PCA9685_Init
 * �� �� ˵ ����PCA9685��ʼ��������PWM���Ƶ������������PWM��������Ķ���Ƕ�
 * �� �� �� �Σ�hz���õĳ�ʼƵ��  angle���õĳ�ʼ�Ƕ�
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void PCA9685_Init(float hz,uint8_t angle)
{                  
        uint32_t off = 0;
        
        PCA9685_GPIO_Init();
        
        //��MODE1��ַ��д0x00
        PCA9685_Write(PCA_Model,0x00);        //��һ���ܹؼ������û����һ��PCA9685�Ͳ�������������
        
//        pwm.setPWMFreq(SERVO_FREQ)������Ҫ������PCA9685�����Ƶ�ʣ�
//        PCA9685��16·PWM���Ƶ����һ�µģ������ǲ���ʵ�ֲ�ͬ���Ų�ͬƵ�ʵġ�
//        ������setPWMFreq���������ݣ���Ҫ�Ǹ���Ƶ�ʼ���PRE_SCALE��ֵ��
        PCA9685_setFreq(hz);
        //����Ƕ�
        off = (uint32_t)(145+angle*2.4);
        
        //����16��������off�Ƕ�
        PCA9685_setPWM(0,0,off);
        PCA9685_setPWM(1,0,off);
        PCA9685_setPWM(2,0,off);
        PCA9685_setPWM(3,0,off);
        PCA9685_setPWM(4,0,off);
        PCA9685_setPWM(5,0,off);
        PCA9685_setPWM(6,0,off);
        PCA9685_setPWM(7,0,off);
        PCA9685_setPWM(8,0,off);
        PCA9685_setPWM(9,0,off);
        PCA9685_setPWM(10,0,off);
        PCA9685_setPWM(11,0,off);
        PCA9685_setPWM(12,0,off);
        PCA9685_setPWM(13,0,off);
        PCA9685_setPWM(14,0,off);
        PCA9685_setPWM(15,0,off);
 
        delay_1ms(100);
        
}

