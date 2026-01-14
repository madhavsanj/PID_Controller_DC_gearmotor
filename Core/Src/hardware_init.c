

/*
* File: hardware_init.c
 * Project: DC Motor Speed Control Using PID (STM32F411)
 * Author: Madhav Appanaboyina
 * Revision: 1.6
 * Date: Dec 1st 2025
 *
 * Description:
 *    Contains initialization routines for all hardware peripherals
 *    used in the project:
 *    - ADC (PA4),
 *    - PWM generation via TIM1 CH1,
 *    - motor direction control (PB0/PB1)
 *    - TIM2 quadrature encoder setup.
 *
 * Notes:
 *    - Interfaces: adc_init(), adc_read(), pwm_init(), set_pwm(),
 *      motor_dir_init(), motor_forward(), encoder_init().
 *
 * References:
 *
 * 		1) "Embedded Systems Fundamentals with ARM Cortex-M based Microcontrollers: A Practical Approach"
 * 			by Alexander G Deanfor initialization of ADC,PWM
 *
 * 		2) STM32F411 Reference Manual for understanding Encoder Interface mode(12.3.16)
 */

#include "stm32f411xe.h"
#include "hardware_init.h"

/* ========================= ADC CONFIG: PA4 ========================= */

#define ADC_CHANNEL_4      4U
#define ADC_PIN            4U
#define ADC_PIN_MODE       (3U << (ADC_PIN * 2))  // Analog Mode

void adc_init(void)
{
    /* Enable GPIOA + ADC1 clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PA4 → Analog Mode */
    GPIOA->MODER |= ADC_PIN_MODE;

    /* Enable ADC module */
    ADC1->CR2 = ADC_CR2_ADON;
}

uint16_t adc_read(void)
{
    ADC1->SQR3 = ADC_CHANNEL_4;        // Select ADC Channel 4
    ADC1->CR2 |= ADC_CR2_SWSTART;      // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));  // Wait for conversion complete
    return (uint16_t)ADC1->DR;         // Return result
}

/* ========================= MOTOR DIR: PB0 / PB1 ========================= */

#define MOTOR_IN1_PIN      0U
#define MOTOR_IN2_PIN      1U

void motor_dir_init(void)
{
    /* Enable GPIOB Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB0, PB1 → Output Mode */
    GPIOB->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk);
    GPIOB->MODER |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0);
}

void motor_forward(void)
{
    /* PB0 = HIGH, PB1 = LOW sets motor in forward direction*/
    GPIOB->ODR |=  (1U << MOTOR_IN1_PIN);
    GPIOB->ODR &= ~(1U << MOTOR_IN2_PIN);
}

/* ========================= PWM CONFIG: TIM1_CH1 @ PA8 ========================= */

#define PWM_PIN            8U
#define PWM_AF_TIM1        1U  // AF1 = TIM1 on PA8

#define PWM_PSC      (16U - 1U)   // for timer clock
#define PWM_PERIOD         (1000U - 1U) // 1 kHz PWM frequency

void pwm_init(void)
{
    /* Enable GPIOA + TIM1 Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
    GPIOA->MODER |=  GPIO_MODER_MODE8_1;

    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
    GPIOA->AFR[1] |=  (PWM_AF_TIM1 << GPIO_AFRH_AFSEL8_Pos);

    /* For PWM signal timing = 1kHz*/
    TIM1->PSC = PWM_PSC;
    TIM1->ARR = PWM_PERIOD;
    TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos); // PWM Mode 1
    TIM1->CCER  = TIM_CCER_CC1E;              // Enable CH1 output
    TIM1->BDTR  = TIM_BDTR_MOE;               // Enable PWM output
    TIM1->CR1   = TIM_CR1_CEN;                // Start timer
}

void set_pwm(float duty)
{
    /* Clamp duty cycle to timer limits */
    if (duty > PWM_PERIOD) duty = PWM_PERIOD;
    if (duty < 0.0f) duty = 0.0f;

    TIM1->CCR1 = (uint16_t)duty;
}

/* ========================= ENCODER CONFIG: TIM2 QUADRATURE ========================= */

#define ENC_A_PIN          0U
#define ENC_B_PIN          1U
#define ENC_AF_TIM2        1U

void encoder_init(void)
{
    /* Enable GPIOA + TIM2 Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk);
    GPIOA->MODER |=  (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0_Msk | GPIO_AFRL_AFSEL1_Msk);
    GPIOA->AFR[0] |=  (ENC_AF_TIM2 << GPIO_AFRL_AFSEL0_Pos) |
                      (ENC_AF_TIM2 << GPIO_AFRL_AFSEL1_Pos);

    /* Quadrature Encoder Mode 3 */
    TIM2->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM2->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* Full counter range */
    TIM2->ARR = 0xFFFF;
    TIM2->CNT = 0;

    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
}
