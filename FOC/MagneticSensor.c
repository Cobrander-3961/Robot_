
#include <foc.h>
#include <stdio.h>
#include "motor_runtime_param.h"
#include "foc.h"


#include "arm_math.h"
#include "MagneticSensor.h" 
#pragma once
long cpr;                        // ÿת�ĽǶȼ����ֱ��ʣ�Counts Per Revolution��
float full_rotation_offset;      // ȫת�Ƕ�ƫ������������չ�Ƕȷ�Χ
long angle_data, angle_data_prev;// ��ǰ��ǰһ�εĽǶ�����
unsigned long velocity_calc_timestamp; // �ٶȼ����ʱ���
float angle_prev;                // ǰһ�εĽǶ�ֵ�������ٶȼ���


// I2Cͨ�ŵ�GPIO�˿ں����Ŷ���
#define IIC_SCL_GPIO_PORT               GPIOB    // I2Cʱ����(GPIOB�˿�)
#define IIC_SCL_GPIO_PIN                GPIO_PIN_6 // I2Cʱ����(GPIOB�˿�6������)
#define IIC_SDA_GPIO_PORT               GPIOB    // I2C������(GPIOB�˿�)
#define IIC_SDA_GPIO_PIN                GPIO_PIN_7 // I2C������(GPIOB�˿�7������)

// �궨����������I2C��ʱ���ߣ�SCL��״̬���ߵ�ƽ��͵�ƽ
#define IIC_SCL(x)        do{ x ? \
                                  HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                              }while(0)       /* ����SCL��Ϊ�ߵ�ƽ��͵�ƽ */

// �궨����������I2C�������ߣ�SDA��״̬���ߵ�ƽ��͵�ƽ
#define IIC_SDA(x)        do{ x ? \
                                  HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                              }while(0)       /* ����SDA��Ϊ�ߵ�ƽ��͵�ƽ */

// �궨�����ڶ�ȡI2C�������ߣ�SDA��״̬
#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* ��ȡSDA��״̬ */

/**
 * @brief I2Cͨ�ŵ���ʱ����
 * 
 * ͨ���򵥵Ŀ�ѭ��ʵ�ִ�Լ2΢�����ʱ�����ڱ�֤I2C�źŵ��ȶ��ԡ�
 */
static void iic_delay(void)
{
    for(uint32_t j = 0; j < 10; j++); // �򵥵���ʱѭ��
}

/**
 * @brief ����I2C�����ź�
 * 
 * ����I2CЭ��Ĺ涨���Ƚ�SDA��SCL���ߣ�Ȼ������SDA��������SCL���γ�����������
 */
void iic_start(void)
{
    IIC_SDA(1);       // ��SDA������
    IIC_SCL(1);       // ��SCL������
    iic_delay();      // ��ʱ
    IIC_SDA(0);       // ��SDA�����ͣ�׼����������
    iic_delay();      // ��ʱ
    IIC_SCL(0);       // ��SCL�����ͣ���ʼ���ݴ���
    iic_delay();      // ��ʱ
}

/**
 * @brief ����I2Cֹͣ�ź�
 * 
 * ����I2CЭ��Ĺ涨���Ƚ�SDA���ͣ�Ȼ��SCL���ߣ����SDA���ߣ��γ�ֹͣ������
 */
void iic_stop(void)
{
    IIC_SDA(0);       // ��SDA������
    iic_delay();      // ��ʱ
    IIC_SCL(1);       // ��SCL������
    iic_delay();      // ��ʱ
    IIC_SDA(1);       // ��SDA�����ߣ�����ͨ��
    iic_delay();      // ��ʱ
}

/**
 * @brief ����I2Cȷ���źţ�ACK��
 * 
 * ���豸����ACK�źţ���ʾ���ն˳ɹ����յ����ݡ�
 */
void iic_ack(void)
{
    IIC_SDA(0);       // ���豸��SDA�����ͣ���ʾACK
    iic_delay();      // ��ʱ
    IIC_SCL(1);       // ��SCL�����ߣ�����ACK�ź�
    iic_delay();      // ��ʱ
    IIC_SCL(0);       // ��SCL�����ͣ����ACK����
    iic_delay();      // ��ʱ
    IIC_SDA(1);       // ���豸�ͷ�SDA��
    iic_delay();      // ��ʱ
}

/**
 * @brief ����I2C��ȷ���źţ�NACK��
 * 
 * ���豸����NACK�źţ���ʾ���ն�δ�ɹ����յ����ݻ��ٽ������ݡ�
 */
void iic_nack(void)
{
    IIC_SDA(1);       // ���豸��SDA�����ߣ���ʾNACK
    iic_delay();      // ��ʱ
    IIC_SCL(1);       // ��SCL�����ߣ�����NACK�ź�
    iic_delay();      // ��ʱ
    IIC_SCL(0);       // ��SCL�����ͣ����NACK����
    iic_delay();      // ��ʱ
}

/**
 * @brief �ȴ�I2Cȷ���ź�
 * 
 * ���豸�ڷ������ݺ󣬵ȴ����豸����ACK������ڹ涨ʱ����δ�յ�ACK������Ϊͨ��ʧ�ܡ�
 * 
 * @return uint8_t ����ͨ���Ƿ�ʧ�ܣ�1��ʾʧ�ܣ�0��ʾ�ɹ���
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0; // �ȴ�������
    uint8_t rack = 0;      // ACK״̬��־

    IIC_SDA(1);            // ���豸�ͷ�SDA�ߣ�׼������ACK
    iic_delay();           // ��ʱ
    IIC_SCL(1);            // ��SCL�����ߣ�׼����ȡACK�ź�
    iic_delay();           // ��ʱ

    // �ȴ����豸����SDA�߱�ʾACK
    while (IIC_READ_SDA)   
    {
        waittime++; // ���ӵȴ�����

        if (waittime > 250) // ����ȴ�ʱ�䳬����ֵ
        {
            iic_stop(); // ����ֹͣ�ź�
            rack = 1;   // ����ACKʧ�ܱ�־
            break;      // �˳��ȴ�ѭ��
        }
    }

    IIC_SCL(0);            // ��SCL�����ͣ����ACK��ȡ
    iic_delay();           // ��ʱ
    return rack;           // ����ACK״̬��0��ʾ�ɹ���1��ʾʧ�ܣ�
}

/**
 * @brief ��ȡI2C�ֽ�����
 * 
 * ��ȡ���豸���͵�һ���ֽ����ݣ������ݲ��������Ƿ���ACK��NACK��
 * 
 * @param ack �Ƿ���ACK�źţ�1����ACK��0����NACK��
 * @return uint8_t ��ȡ�����ֽ�����
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0; // ѭ���������ͽ������ݱ���

    for (i = 0; i < 8; i++ )    
    {
        receive <<= 1;         // ��������������һλ
        IIC_SCL(1);            // ��SCL�����ߣ�׼����ȡ����λ
        iic_delay();           // ��ʱ

        if (IIC_READ_SDA)      // ���SDA��Ϊ�ߵ�ƽ
        {
            receive++;         // ��������λΪ1���ۼӵ�����������
        }
        
        IIC_SCL(0);            // ��SCL�����ͣ�׼����ȡ��һ������λ
        iic_delay();           // ��ʱ
    }

    if (!ack)                  // �������Ҫ����ACK
    {
        iic_nack();            // ����NACK�ź�
    }
    else
    {
        iic_ack();             // ����ACK�ź�
    }

    return receive;            // ���ؽ��յ������ֽ�
}

/**
 * @brief ����I2C�ֽ�����
 * 
 * ��һ���ֽ�����ͨ��I2C���߷��͸����豸��
 * 
 * @param data Ҫ���͵��ֽ�����
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t; // ѭ��������

    for (t = 0; t < 8; t++)
    {
        // �����ݵ����λ����7λ�����͵�SDA����
        IIC_SDA((data & 0x80) >> 7);    
        iic_delay();           // ��ʱ
        IIC_SCL(1);            // ��SCL�����ߣ���������λ
        iic_delay();           // ��ʱ
        IIC_SCL(0);            // ��SCL�����ͣ�׼��������һ������λ
        data <<= 1;            // ��������һλ��׼��������һλ
    }
    IIC_SDA(1);                // �������ͺ��ͷ�SDA��
}

/**
 * @brief ��ȡAS5600�������������ֽ�����
 * 
 * ͨ��I2Cͨ�Ŷ�ȡAS5600��������ָ���Ĵ�����ַ�������ֽ����ݡ�
 * 
 * @param readAddr Ҫ��ȡ�ļĴ�����ַ
 * @return uint16_t ��ȡ����16λ����
 */
uint16_t AS5600_ReadTwoByte(uint16_t readAddr)
{
    uint16_t temp = 0xFFFF; // ��ʱ���������ڴ洢��ȡ������

    iic_start();                           // ����I2C�����ź�
    iic_send_byte((0X36 << 1) | 0x00);     // ����AS5600��д��ַ������1λ����0��ʾд������
    iic_wait_ack();                        // �ȴ�ACK�ź�
    iic_send_byte(readAddr);               // ����Ҫ��ȡ�ļĴ�����ַ
    iic_wait_ack();                        // �ȴ�ACK�ź�
    iic_start();                           // ����I2C�ظ������ź�
    iic_send_byte((0X36 << 1) | 0x01);     // ����AS5600�Ķ���ַ������1λ����1��ʾ��������
    iic_wait_ack();                        // �ȴ�ACK�ź�
    temp = iic_read_byte(1);               // ��ȡ��λ�ֽڣ�������ACK�ź�
    temp = temp << 8 | iic_read_byte(0);   // ��ȡ��λ�ֽڣ�������ACK�ź�
    iic_stop();                            // ����I2Cֹͣ�ź�
    return temp;                           // ���ض�ȡ����16λ����
}

/******************************************************************************/
#define  AS5600_Address  0x36      // AS5600��������I2C��ַ
#define  RAW_Angle_Hi    0x0C      // AS5600��������ԭʼ�Ƕȸ��ֽڼĴ�����ַ
#define  AS5600_CPR      4096       // AS5600��������ȫת������Counts Per Revolution��

/**
 * @brief ��ȡAS5600��������ԭʼ����ֵ
 * 
 * ͨ����ȡAS5600��������ԭʼ�Ƕȸ��ֽڼĴ�����0x0C������ȡ16λԭʼ����ֵ��
 * 
 * @return unsigned short ԭʼ����ֵ
 */
unsigned short I2C_getRawCount()
{
    return AS5600_ReadTwoByte(0x0C); // ��ȡAS5600��ԭʼ�Ƕ�����
}

/******************************************************************************/
/**
 * @brief ��ʼ���ű�����������
 * 
 * ��ʼ�������У�����CPR��ÿת�����ֱ��ʣ�����ȡ��ʼ�Ƕ����ݡ�����ȫתƫ�������ٶȼ���ʱ�����
 */
void MagneticSensor_Init(void)
{
    cpr = AS5600_CPR;                             // ����ÿת�����ֱ���
    angle_data = angle_data_prev = I2C_getRawCount(); // ��ȡ��ʼ�Ƕ����ݲ�����Ϊ��ǰ��ǰһ�εĽǶ�����
    full_rotation_offset = 0;                     // ��ʼ��ȫתƫ����Ϊ0
    velocity_calc_timestamp = 0;                  // ��ʼ���ٶȼ���ʱ���Ϊ0
}
/******************************************************************************/
#define _2PI 6.283185307179586476925286766559 // 2*PI
/**
 * @brief ��ȡ��ǰ���ʵ�ʽǶ�
 * 
 * ͨ����ȡAS5600�������ļ���ֵ�����㲢���ص�ǰ���ʵ�ʽǶȡ�������ȫתƫ������ȷ���Ƕȷ�Χ����������չ��
 * 
 * @return float ��ǰ��Ƕȣ����ȣ�
 */
extern float motor_logic_angle;
extern float encoder_angle;
float getAngle(void)
{
    // float d_angle; // �Ƕȱ仯ֵ

    // angle_data = I2C_getRawCount(); // ��ȡ��ǰ�ĽǶȼ���ֵ

    // // ����Ƕȱ仯�������ڼ��ȫת
    // d_angle = angle_data - angle_data_prev;

    // // ����Ƕȱ仯��������ֵ��80%��CPR�����ж�Ϊ�����������ȫת��
    // if(fabs(d_angle) > (0.8 * cpr)) 
    //     full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; // ���ݷ������ȫתƫ����

    // // ���浱ǰ�Ƕȼ���ֵ��Ϊ�´μ�����׼��
    // angle_data_prev = angle_data;

    // // ���㲢������������Ƕȣ�����ȫתƫ�����͵�ǰ����ֵ��Ӧ�ĽǶ�
    // return (full_rotation_offset + (angle_data * 1.0 / cpr) * _2PI);

    int angle_raw = I2C_getRawCount();
    encoder_angle = (angle_raw * 1.0 / cpr) * _2PI;

    static float encoder_angle_last = 0;
    /****encoder_angle_last默认值是0，不能用于计算，要先赋值一次****/
    static int once = 1;
    if (once)
    {
      once = !once;
      encoder_angle_last = encoder_angle;
    }
    /*************/
    float _encoder_angle = encoder_angle;
    // 角度差值，用于累计多圈逻辑角度
    float diff_angle = cycle_diff(_encoder_angle - encoder_angle_last, 2 * PI);
    encoder_angle_last = _encoder_angle;
    // 实现周期操作，将motor_logic_angle转到周期内
    motor_logic_angle = cycle_diff(motor_logic_angle + diff_angle, position_cycle);

}

/**
 * @brief ��ȡ��ǰ����ٶ�
 * 
 * ͨ�����㵱ǰ�Ƕ���ǰһ�νǶȵı仯����ʱ�����㲢���ص�ǰ����ٶȡ�ʹ�õ�ͨ�˲���ƽ���ٶ�ֵ��
 * 
 * @return float ��ǰ���ٶȣ�����ÿ�룩
 */
float getVelocity(void)
{
    unsigned long now_us;    // ��ǰʱ�䣨΢�룩
    float Ts, angle_now, vel; // ����ʱ�䡢��ǰ�Ƕȡ��ٶ�

    // ��ȡ��ǰ��ϵͳ�δ����ֵ
    now_us = SysTick->VAL; //_micros(); // ������Ҫ���ݾ���ϵͳ����

    // �������ʱ���룩
    if(now_us < velocity_calc_timestamp)
        Ts = (float)(velocity_calc_timestamp - now_us) / 9 * 1e-6; 
    else
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp) / 9 * 1e-6; // ����SysTick��������

    // �����޸��쳣�Ĳ���ʱ�䣨��Ϊ0�������
    if(Ts == 0 || Ts > 0.5) Ts = 1e-3; // ����Ϊ1����

    // ��ȡ��ǰ��������Ƕ�
    angle_now = getAngle();

    // �����ٶȣ��Ƕȱ仯������ʱ���
    vel = (angle_now - angle_prev) / Ts;

    // ���浱ǰ�ǶȺ�ʱ�����Ϊ�´μ�����׼��
    angle_prev = angle_now;
    velocity_calc_timestamp = now_us;

    return vel; // ���ؼ���õ����ٶ�
}
