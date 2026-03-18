
/*
 * main_drive.c
 * main.c 전체 코드 중 주행 관련 코드만 작성
 */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <My_uart.h>
#include <My_motor.h>

#define N 3

// --------------------------------------------------
// TIM6 : 10ms 주기 제어 루프
// - 엔코더 갱신
// - RPM 계산
// - 목표 RPM ramp 적용
// - PI 제어 후 PWM 반영
// --------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        Encoder_Update(&htim4, en_p_L);
        Encoder_Update(&htim5, en_p_R);

        float rpm_L = -dcnt_to_rpm(en_p_L->delta_cnt);
        float rpm_R = -dcnt_to_rpm(en_p_R->delta_cnt);

        rpm_p_L->meas_rpm = rpm_L;
        rpm_p_R->meas_rpm = rpm_R;

        RPM_UpdateRamp(rpm_p_L);
        RPM_UpdateRamp(rpm_p_R);

        if (rpm_p_L->target_rpm_cmd <= 0.01f && rpm_p_R->target_rpm_cmd <= 0.01f)
        {
            rpm_p_L->target_rpm = 0.0f;
            rpm_p_R->target_rpm = 0.0f;

            PI_Reset(pi_p_L);
            PI_Reset(pi_p_R);

            Motor_SetDuty(&htim3, TIM_CHANNEL_1, 0);
            Motor_SetDuty(&htim3, TIM_CHANNEL_3, 0);

            return;
        }

        float duty_L = PI_Update(&pi_L, rpm_p_L->target_rpm, rpm_p_L->meas_rpm);
        float duty_R = PI_Update(&pi_R, rpm_p_R->target_rpm, rpm_p_R->meas_rpm);

        Motor_SetDuty(&htim3, TIM_CHANNEL_1, (uint8_t)duty_L);
        Motor_SetDuty(&htim3, TIM_CHANNEL_3, (uint8_t)duty_R);
    }
}

// --------------------------------------------------
// TIM2 Input Capture
// 초음파 echo 시간 측정
// --------------------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (hc_sr_p->ic_state == 0)
    {
        hc_sr_p->ic_rise = cap;
        hc_sr_p->ic_state = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        hc_sr_p->ic_fall = cap;
        hc_sr_p->ic_state = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_RISING);

        if (hc_sr_p->ic_fall >= hc_sr_p->ic_rise)
            hc_sr_p->echo_us = hc_sr_p->ic_fall - hc_sr_p->ic_rise;
        else
            hc_sr_p->echo_us = (0xFFFF - hc_sr_p->ic_rise) + hc_sr_p->ic_fall + 1;
    }
}

// TIM7 기반 us delay
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    while (__HAL_TIM_GET_COUNTER(&htim7) < us) { }
}

// 중앙값 필터
static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

// --------------------------------------------------
// main.c 내부 while(1)에서 사용한 주행 관련 로직 발췌
// Bluetooth 명령으로 run_enable 제어 후
// 초음파 거리값 기반으로 속도/회전 결정
// --------------------------------------------------
void drive_logic()
{
    static uint8_t run_enable = 0;
    static float buf[N] = {0, 0, 0};
    static int idx = 0;

    static float prev_distance = -1.0f;
    static uint32_t last_sample_ms = 0;

    static uint32_t turn_cooldown_until = 0;
    static uint32_t next_turn_allowed_ms = 0;

    if(rFlag->rx_end_3_flag == 1)
    {
        rFlag->rx_end_3_flag = 0;

        uint8_t cmd = (uint8_t)rBuf->rx_buf_3[0];
        rBuf->rx_buf_3[0] = 0;

        if(cmd == '1')
        {
            run_enable = 1;

            hc_sr_p->ic_state = 0;
            hc_sr_p->echo_us = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);

            prev_distance = -1.0f;
            idx = 0;
            last_sample_ms = HAL_GetTick();
        }
        else if(cmd == '0')
        {
            run_enable = 0;

            prev_distance = -1.0f;
            idx = 0;
            last_sample_ms = HAL_GetTick();
        }
    }

    if(run_enable == 1)
    {
        float dist_f = prev_distance;

        if (idx > 0 && (HAL_GetTick() - last_sample_ms) > 300)
            idx = 0;

        uint32_t now = HAL_GetTick();
        uint8_t allow_jump = (now < turn_cooldown_until);

        static uint32_t last_trig_ms = 0;
        if (HAL_GetTick() - last_trig_ms < 60)
            goto AFTER_US_BLOCK;

        last_trig_ms = HAL_GetTick();

        uint32_t prev_echo = hc_sr_p->echo_us;

        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
        delay_us(10);
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

        uint32_t t0 = HAL_GetTick();
        while(hc_sr_p->echo_us == prev_echo)
        {
            if(HAL_GetTick() - t0 > 60) break;
        }
        if (hc_sr_p->echo_us == prev_echo) goto AFTER_US_BLOCK;

        float distance_cm = hc_sr_p->echo_us * 0.0343f * 0.5f;

        if(distance_cm < 10.0f || distance_cm > 300.0f)
            goto AFTER_US_BLOCK;

        if(!allow_jump)
        {
            if(prev_distance > 0.0f && fabsf(distance_cm - prev_distance) > 30.0f)
                goto AFTER_US_BLOCK;
        }

        buf[idx] = distance_cm;
        idx++;
        last_sample_ms = HAL_GetTick();

        if(idx < N) goto AFTER_US_BLOCK;
        idx = 0;

        dist_f = median3(buf[0], buf[1], buf[2]);

        if(!allow_jump)
        {
            if(prev_distance > 0.0f && fabsf(dist_f - prev_distance) > 30.0f)
                goto AFTER_US_BLOCK;
        }

        prev_distance = dist_f;

AFTER_US_BLOCK:
        if (dist_f < 0.0f) return;

        if(dist_f < 30.0f)
        {
            Motor_Stop();
            rpm_p_L->target_rpm_cmd = 0.0f;
            rpm_p_R->target_rpm_cmd = 0.0f;
        }
        else if(dist_f < 60.0f)
        {
            if (HAL_GetTick() >= next_turn_allowed_ms)
            {
                next_turn_allowed_ms = HAL_GetTick() + 500;
                Motor_TurnRight90(&htim3);

                turn_cooldown_until = HAL_GetTick() + 700;
                idx = 0;
                prev_distance = 0.0f;
                last_sample_ms = HAL_GetTick();
            }
            else
            {
                Motor_Front();
                rpm_p_L->target_rpm_cmd = 60.0f;
                rpm_p_R->target_rpm_cmd = 60.0f;
            }
        }
        else if(dist_f < 120.0f)
        {
            Motor_Front();
            rpm_p_L->target_rpm_cmd = 70.0f;
            rpm_p_R->target_rpm_cmd = 70.0f;
        }
        else
        {
            Motor_Front();
            rpm_p_L->target_rpm_cmd = 120.0f;
            rpm_p_R->target_rpm_cmd = 120.0f;
        }
    }
    else
    {
        Motor_Stop();
        rpm_p_L->target_rpm_cmd = 0;
        rpm_p_R->target_rpm_cmd = 0;
    }
}
