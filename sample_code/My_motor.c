

#include "stm32f4xx_hal.h"
#include <main.h>
#include <stdint.h>
#include <stdlib.h>
#include "tim.h"
#include "My_motor.h"

// --------------------------------------------------
// Encoder 구조체 초기화
// --------------------------------------------------
volatile __EncoderState__ en_L = {
    .prev_cnt = 0,
    .curr_cnt = 0,
    .delta_cnt = 0
};
volatile __EncoderState__ *en_p_L = &en_L;

volatile __EncoderState__ en_R = {
    .prev_cnt = 0,
    .curr_cnt = 0,
    .delta_cnt = 0
};
volatile __EncoderState__ *en_p_R = &en_R;

// --------------------------------------------------
// RPM 구조체 초기화
// --------------------------------------------------
__RPM__ rpm_L = {
    .target_rpm_cmd = 0.0f,
    .target_rpm = 0.0f,
    .meas_rpm = 0.0f
};
__RPM__ *rpm_p_L = &rpm_L;

__RPM__ rpm_R = {
    .target_rpm_cmd = 0.0f,
    .target_rpm = 0.0f,
    .meas_rpm = 0.0f
};
__RPM__ *rpm_p_R = &rpm_R;

// --------------------------------------------------
// PI 구조체 초기화
// --------------------------------------------------
__PI__ pi_L = {
    .kp = 0.8f,
    .ki = 0.2f,
    .dt = 0.01f,
    .integrator = 0.0f,
    .out_min = 0,
    .out_max = 100
};
__PI__ *pi_p_L = &pi_L;

__PI__ pi_R = {
    .kp = 0.8f,
    .ki = 0.2f,
    .dt = 0.01f,
    .integrator = 0.0f,
    .out_min = 0,
    .out_max = 100
};
__PI__ *pi_p_R = &pi_R;

// --------------------------------------------------
// 초음파 센서 상태 구조체
// echo_us : 측정된 echo pulse width (us)
// --------------------------------------------------
volatile __HC__ hc_sr = {
    .ic_rise = 0,
    .ic_fall = 0,
    .ic_state = 0,
    .echo_us = 0
};
volatile __HC__ *hc_sr_p = &hc_sr;

// --------------------------------------------------
// 모터 방향 제어
// --------------------------------------------------

// 좌/우 바퀴 정지
void Motor_Stop()
{
    HAL_GPIO_WritePin(GPIOC, LEFT_PIN1_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, LEFT_PIN2_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, RIGHT_PIN1_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, RIGHT_PIN2_Pin, 0);
}

// 전진
void Motor_Front()
{
    HAL_GPIO_WritePin(GPIOC, LEFT_PIN1_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, LEFT_PIN2_Pin, 1);
    HAL_GPIO_WritePin(GPIOC, RIGHT_PIN1_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, RIGHT_PIN2_Pin, 1);
}

// 오른쪽 회전
void Motor_Right()
{
    HAL_GPIO_WritePin(GPIOC, LEFT_PIN1_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, LEFT_PIN2_Pin, 1);
    HAL_GPIO_WritePin(GPIOC, RIGHT_PIN1_Pin, 0);
    HAL_GPIO_WritePin(GPIOC, RIGHT_PIN2_Pin, 0);
}

// --------------------------------------------------
// 일정 카운트만큼 우회전
// - TIM6 인터럽트를 잠시 멈추고 회전 수행
// - 회전 후 PI 적분값 리셋
// --------------------------------------------------
void Motor_TurnRight90(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(&htim6);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    Motor_Front();

    const int32_t  target_cnt = 1900;
    const uint32_t timeout_ms = 2000;
    uint32_t t0 = HAL_GetTick();
    uint32_t spin_guard = 0;

    Motor_SetDuty(htim, TIM_CHANNEL_1, 100);
    Motor_SetDuty(htim, TIM_CHANNEL_3, 5);

    while (1)
    {
        int32_t cnt4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        int32_t cnt5 = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);

        int32_t mag4 = (cnt4 >= 0) ? cnt4 : -cnt4;
        int32_t mag5 = (cnt5 >= 0) ? cnt5 : -cnt5;

        int32_t mag = (mag4 > mag5) ? mag4 : mag5;

        if (mag >= target_cnt) break;
        if ((HAL_GetTick() - t0) > timeout_ms) break;
        if (++spin_guard > 2000) break;

        HAL_Delay(1);
    }

    Motor_Stop();
    Motor_SetDuty(htim, TIM_CHANNEL_1, 0);
    Motor_SetDuty(htim, TIM_CHANNEL_3, 0);

    PI_Reset(pi_p_L);
    PI_Reset(pi_p_R);

    HAL_TIM_Base_Start_IT(&htim6);
}

// --------------------------------------------------
// PWM 듀티 설정
// - 0이면 정지
// - 최소 duty / 최대 duty 제한
// --------------------------------------------------
void Motor_SetDuty(TIM_HandleTypeDef *htim, uint32_t ch, uint8_t percent)
{
    if (percent == 0)
    {
        __HAL_TIM_SET_COMPARE(htim, ch, 0);
        return;
    }

    if (percent < DUTY_MIN) percent = DUTY_MIN;
    if (percent > 100)      percent = 100;

    uint32_t ccr = ((PWM_ARR + 1) * percent) / 100;
    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

// --------------------------------------------------
// Encoder 관련
// --------------------------------------------------
void Encoder_Init(TIM_HandleTypeDef *htim, volatile __EncoderState__ *st)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

    st->prev_cnt = now;
    st->curr_cnt = now;
    st->delta_cnt = 0;
}

void Encoder_Update(TIM_HandleTypeDef *htim, volatile __EncoderState__ *st)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

    st->curr_cnt = now;
    st->delta_cnt = (int16_t)(now - st->prev_cnt);
    st->prev_cnt = now;
}

// --------------------------------------------------
// 목표 RPM까지 ramp 적용
// --------------------------------------------------
void RPM_UpdateRamp(__RPM__ *r)
{
    if (r->target_rpm < r->target_rpm_cmd) r->target_rpm += ACC_STEP;
    else if (r->target_rpm > r->target_rpm_cmd) r->target_rpm -= DEC_STEP;

    if (r->target_rpm > -1.0f && r->target_rpm < 1.0f)
        r->target_rpm = 0.0f;
}

// --------------------------------------------------
// PI 제어 업데이트
// --------------------------------------------------
float PI_Update(__PI__ *pi, float target, float meas)
{
    float err = target - meas;

    pi->integrator += pi->ki * err * pi->dt;
    if (pi->integrator > pi->out_max) pi->integrator = pi->out_max;
    if (pi->integrator < pi->out_min) pi->integrator = pi->out_min;

    float out = pi->kp * err + pi->integrator;
    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}

// PI 적분값 리셋
void PI_Reset(__PI__ *pi)
{
    pi->integrator = 0.0f;
}
