/*
 * File: uart.h
 * Project: DC Motor Speed Control Using PID (STM32F411)
 * Author: Madhav Appanaboyina
 * Revision: 1.0
 * Date: Nov 29th 2025
 *
 * Purpose:
 *    Declares UART interface functions for initialization and
 *    data transmission.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_send(char c);
void uart_print(const char *s);
void uart_print_num(int32_t n);

#endif
