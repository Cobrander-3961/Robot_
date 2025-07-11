#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include <stdbool.h>
#include <main.h>
#include <stdio.h>
#include <tim.h>
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define rad60 deg2rad(60)
#define SQRT3 1.73205080756887729353

arm_pid_instance_f32 pid_position;
arm_pid_instance_f32 pid_speed;
arm_pid_instance_f32 pid_torque_d;
arm_pid_instance_f32 pid_torque_q;
motor_control_context_t motor_control_context;

static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    d = min(d, 1);
    d = max(d, -1);
    q = min(q, 1);
    q = max(q, -1);
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);
    float alpha = 0;
    float beta = 0;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;

    int K = 4 * A + 2 * B + C;
    int sector = K_to_sector[K];

    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
    float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m * v[sector - 1][0] + t_n * v[(sector) % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[(sector) % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[(sector) % 6][2] + t_0 / 2;
}

// 使用__attribute__((weak))属性声明函数，表示该函数是弱符号
// 弱符号可以在链接时被其他同名的强符号替换
// 如果没有其他同名函数定义，则该函数会被保留
// __attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
// {
//     // 无限循环，函数体内部没有任何实际操作
//     // 这种通常用于占位，或者在没有实现具体功能时的占位符
//     while (1)
//         ;
// }
void set_pwm_duty(float d_u, float d_v, float d_w)
{
  d_u = min(d_u, 0.9);
  d_v = min(d_v, 0.9);
  d_w = min(d_w, 0.9);

  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);

  __enable_irq();

}
void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    // printf( "foc_forward: d_u=%.2f, d_v=%.2f, d_w=%.2f, rotor_rad=%.2f\n", d_u, d_v, d_w, rotor_rad);
    set_pwm_duty(d_u, d_v, d_w);
}

static float position_loop(float rad)
{
    float diff = cycle_diff(rad - motor_logic_angle, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

static float speed_loop(float speed_rad)
{
    float diff = speed_rad - motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}

static float torque_d_loop(float d)
{
    float diff = d - motor_i_d / MAX_CURRENT;
    return arm_pid_f32(&pid_torque_d, diff);
}

static float torque_q_loop(float q)
{
    float diff = q - motor_i_q / MAX_CURRENT;
    return arm_pid_f32(&pid_torque_q, diff);
}

void lib_position_control(float rad)
{
    float d = 0;
    float q = position_loop(rad);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_control(float speed)
{
    float d = 0;
    float q = speed_loop(speed);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_torque_control(float torque_norm_d, float torque_norm_q)
{
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad);
    torque_norm = min(fabs(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);
    lib_torque_control(0, torque_norm);
}

void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = min(fabs(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
    lib_speed_torque_control(speed_rad, max_torque_norm);
}

void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d)
{
    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;
    arm_pid_init_f32(&pid_position, false);
    arm_pid_init_f32(&pid_speed, false);
    arm_pid_init_f32(&pid_torque_d, false);
    arm_pid_init_f32(&pid_torque_q, false);
}

float cycle_diff(float diff, float cycle)
{
    if (diff > (cycle / 2))
        diff -= cycle;
    else if (diff < (-cycle / 2))
        diff += cycle;
    return diff;
}





