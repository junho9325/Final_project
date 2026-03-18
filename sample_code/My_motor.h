
#ifndef PJ_LIB_MY_MOTOR_H_
#define PJ_LIB_MY_MOTOR_H_

#include "stm32f4xx_hal.h"

// --------------------------------------------------
// PWM 설정
// --------------------------------------------------
#define PWM_ARR    4499
#define DUTY_MIN   5

// --------------------------------------------------
// Encoder / RPM 계산 관련
// ENC_CPR : 바퀴 1회전당 카운트 수
// CTRL_DT_SEC : 제어 주기 (TIM6 = 10ms)
// --------------------------------------------------
#define ENC_CPR      1431.0f
#define CTRL_DT_SEC  0.01f

// --------------------------------------------------
// 목표 RPM까지 ramp 적용 시 증가/감소량
// --------------------------------------------------
#define ACC_STEP  2.0f
#define DEC_STEP  1.0f

// --------------------------------------------------
// Encoder 상태 저장 구조체
// --------------------------------------------------
typedef struct {
    uint16_t prev_cnt;
    uint16_t curr_cnt;
    int16_t delta_cnt;
} __EncoderState__;

extern volatile __EncoderState__ en_L;
extern volatile __EncoderState__ *en_p_L;
extern volatile __EncoderState__ en_R;
extern volatile __EncoderState__ *en_p_R;

// --------------------------------------------------
// RPM 관련 구조체
// target_rpm_cmd : 최종 목표 RPM
// target_rpm     : ramp 적용 중인 현재 목표 RPM
// meas_rpm       : 측정된 실제 RPM
// --------------------------------------------------
typedef struct {
    float target_rpm_cmd;
    float target_rpm;
    float meas_rpm;
} __RPM__;

extern __RPM__ rpm_L;
extern __RPM__ *rpm_p_L;
extern __RPM__ rpm_R;
extern __RPM__ *rpm_p_R;

// --------------------------------------------------
// PI 제어 구조체
// --------------------------------------------------
typedef struct {
    float kp;
    float ki;
    float dt;

    float integrator;
    float out_min;
    float out_max;
} __PI__;

extern __PI__ pi_L;
extern __PI__ *pi_p_L;
extern __PI__ pi_R;
extern __PI__ *pi_p_R;

// --------------------------------------------------
// 초음파 센서 측정값 저장 구조체
// --------------------------------------------------
typedef struct {
    uint32_t ic_rise;
    uint32_t ic_fall;
    uint8_t  ic_state;
    uint32_t echo_us;
    uint8_t  ready;
} __HC__;

extern volatile __HC__ hc_sr;
extern volatile __HC__ *hc_sr_p;

// --------------------------------------------------
// GPIO 제어 함수
// --------------------------------------------------
void Motor_Stop(void);
void Motor_Front(void);
void Motor_Right(void);
void Motor_TurnRight90(TIM_HandleTypeDef *htim_pwm);

// --------------------------------------------------
// PWM 제어 함수
// --------------------------------------------------
void Motor_SetDuty(TIM_HandleTypeDef *htim, uint32_t ch, uint8_t percent);

// --------------------------------------------------
// Encoder 관련 함수
// --------------------------------------------------
void Encoder_Init(TIM_HandleTypeDef *htim, volatile __EncoderState__ *st);
void Encoder_Update(TIM_HandleTypeDef *htim, volatile __EncoderState__ *st);

// --------------------------------------------------
// delta count -> RPM 변환
// --------------------------------------------------
static inline float dcnt_to_rpm(int16_t dcnt)
{
    return ((float)dcnt / ENC_CPR) * (60.0f / CTRL_DT_SEC);
}

void RPM_UpdateRamp(__RPM__ *r);

// --------------------------------------------------
// PI 제어 함수
// --------------------------------------------------
float PI_Update(__PI__ *pi, float target, float meas);
void  PI_Reset(__PI__ *pi);

#endif /* PJ_LIB_MY_MOTOR_H_ */
