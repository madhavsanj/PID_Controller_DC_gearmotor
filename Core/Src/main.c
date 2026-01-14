/*
 * File: main.c
 *
 * Project: DC Motor Speed Control Using PID (STM32F411)
 * Author: Madhav Appanaboyina
 * Revision: 2.0
 * Date: December 4th 2025
 *
 * Description:
 *    This file contains the main application logic for a closed-loop(PID)
 *    DC motor speed control system
 *
 *
*/
#include "stm32f411xe.h"
#include <stdint.h>
#include "uart.h"
#include "hardware_init.h"

/* ================= USER SETTINGS ================= */
#define CPR             4704.0f // Total number of counts per revolution
#define Ts              0.05f      // PID period in seconds

/* PID gains */
#define KP  1.5f
#define KI  0.60f
#define KD  0.02f

/* Integral windup limits */
#define I_MAX   400.0f
#define I_MIN  -400.0f

#define BASE_PWM 350.0f

volatile int tim3_flag = 0;

/* Timer that ensures the control loop is triggered at the same rate equal to the sampling period Ts.*/
static void tim3_for_pid(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 96000 - 1;
    TIM3->ARR = Ts*1000 - 1;  // Sampling period
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1  |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);  // Enable interrupt in NVIC
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        tim3_flag = 1;
    }
}


/* ================= MAIN ================= */
int main(void)
{
    uart_init();
    adc_init();
    pwm_init();
    motor_dir_init();
    encoder_init();
    tim3_for_pid();

    uart_print("\r\n=== SPEED CONTROL USING PID ===\r\n");

    int32_t last_cnt = 0;
    float integral = 0.0f;
    float last_rpm = 0.0f;
    float pwm = BASE_PWM;

    motor_forward();
    set_pwm(pwm);

    while (1)
    {
        if (tim3_flag)
        {
            tim3_flag = 0;

            /* Reading ADC and scaling the input rpm between 0 and 300 */
            uint16_t adc_raw = adc_read();
            float target_rpm = (adc_raw / 4095.0f) * 300.0f;

            /* Updating the encoder */
            int32_t now = TIM2->CNT;
            int32_t dif = now - last_cnt;

            /* Logic that ensures that the dif count remains correct after rollover of 16-bit timer */
            if (dif > 32767) dif -= 65536;
            else if (dif < -32768) dif += 65536;

            last_cnt = now;

            float rpm = (float)dif * 1200.0f / CPR;
            if (rpm < 0)
            	rpm = -rpm;

            /* ================= PID ================= */
            float error = target_rpm - rpm;

            /* Integral */
            integral += error * KI * Ts;
            if (integral > I_MAX) integral = I_MAX;
            if (integral < I_MIN) integral = I_MIN;

            /* Derivative */
            float derivative = (rpm - last_rpm) / Ts;

            /* PID Output */
            float upd = BASE_PWM + (KP * error) + integral - (KD * derivative);
            pwm = upd;


            set_pwm(pwm);
            last_rpm = rpm;

            /* PRINT Data */
            uart_print_num(target_rpm);
            uart_print(",");
            uart_print_num(rpm);
            uart_print("\r\n");
        }
    }
}
