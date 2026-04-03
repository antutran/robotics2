/* motor.c — Driver TB6612FNG trên STM32F411 */
#include "motor.h"

static TIM_HandleTypeDef *motor_tim = NULL;

void Motor_Module_Init(TIM_HandleTypeDef *htim) {
    motor_tim = htim;
}

/* 
 * Chuyển đổi PWM từ 0..255 sang tỷ lệ của TIM1 (0..65535)
 */
static uint32_t scalePWM(uint8_t pwm) {
    return ((uint32_t)pwm * 65535) / 255;
}

/* Motor LEFT: AIN1=PB10, AIN2=PB2 | Motor RIGHT: BIN1=PB12, BIN2=PB13 */

void Motor_Forward(uint8_t pwm) {
    uint32_t val = scalePWM(pwm);
    
    /* Left: AIN1=H, AIN2=L */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, val);

    /* Right: BIN1=H, BIN2=L */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, val);
}

void Motor_Backward(uint8_t pwm) {
    uint32_t val = scalePWM(pwm);
    
    /* Left: AIN1=L, AIN2=H */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, val);

    /* Right: BIN1=L, BIN2=H */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, val);
}

void Motor_TurnLeft(uint8_t pwm) {
    uint32_t val = scalePWM(pwm);
    
    /* Left: Lùi (AIN1=L, AIN2=H) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, val);

    /* Right: Tiến (BIN1=H, BIN2=L) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, val);
}

void Motor_TurnRight(uint8_t pwm) {
    uint32_t val = scalePWM(pwm);
    
    /* Left: Tiến (AIN1=H, AIN2=L) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, val);

    /* Right: Lùi (BIN1=L, BIN2=H) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, val);
}

void Motor_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, 0);
}

void Motor_Set_Speed(int16_t left, int16_t right) {
    if (motor_tim == NULL) return;

    // LEFT MOTOR
    if (left == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, 0);
    } else if (left > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, scalePWM((uint8_t)left));
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_1, scalePWM((uint8_t)(-left)));
    }

    // RIGHT MOTOR
    if (right == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, 0);
    } else if (right > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, scalePWM((uint8_t)right));
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_tim, TIM_CHANNEL_2, scalePWM((uint8_t)(-right)));
    }
}

/* LED logic */
void LED_Run(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Green
}
void LED_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // Red
}
void LED_Turn(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Yellow
}
