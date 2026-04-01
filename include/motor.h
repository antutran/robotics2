/* motor.h — Driver TB6612FNG cho Robot STM32 */
#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 
 * Pin Map TB6612FNG:
 *   PWMA (Trái) -> PA8 (TIM1_CH1)
 *   AIN1 -> PB10
 *   AIN2 -> PB2
 *   
 *   PWMB (Phải) -> PA9 (TIM1_CH2)
 *   BIN1 -> PB12
 *   BIN2 -> PB13
 *
 *   STBY -> Nối 5V (Luôn ON)
 */

void Motor_Module_Init(TIM_HandleTypeDef *htim);

/* Motor Control API - PWM: 0 to 255 */
void Motor_Forward(uint8_t pwm);
void Motor_Backward(uint8_t pwm);
void Motor_TurnLeft(uint8_t pwm);
void Motor_TurnRight(uint8_t pwm);
void Motor_Stop(void);

/* LED helpers (PB3, PB4, PB5) */
void LED_Run(void);    /* Green */
void LED_Stop(void);   /* Red   */
void LED_Turn(void);   /* Yellow*/

#ifdef __cplusplus
}
#endif

#endif
