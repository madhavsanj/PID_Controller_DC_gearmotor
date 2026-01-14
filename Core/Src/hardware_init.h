/*
 * File: hardware_init.h
 * Project: DC Motor Speed Control Using PID (STM32F411)
 * Author: Madhav Appanaboyina
 * Revision: 1
 * Date: Nov 29th 2025
 *
 * Purpose:
 *    Declares function prototypes for initializing ADC, PWM, motor direction,
 *    and quadrature encoder configuration.
 *
 */


#ifndef HARDWARE_INIT_H
#define HARDWARE_INIT_H

#include <stdint.h>

/* ADC */
void adc_init(void);
uint16_t adc_read(void);

/* Motor Controls */
void motor_dir_init(void);
void motor_forward(void);

/* PWM */
void pwm_init(void);
void set_pwm(float duty);

/* Encoder */
void encoder_init(void);

#endif
