/*
* File: uart.c
 * Project: DC Motor Speed Control Using PID (STM32F411)
 * Author: Madhav Appanaboyina
 * Revision: 1.2
 * Date: Nov 30th 2025
 *
 * Description:
 *    Implements UART2 initialization for printing rpm values to a serial terminal
 *    Configures PA2 (TX) and PA3 (RX)
 *    as alternate function pins and sets the USART2 baud rate.
 *
 * Notes:
 *    - Provides uart_init(), uart_send(), uart_print(), and
 *      uart_print_num() interfaces to the application.
 *    - Included in Appendix C (Firmware Source Code).
*/

#include "stm32f411xe.h"
#include "uart.h"

/* UART2 pin configuration */
#define UART_TX_PIN         2
#define UART_RX_PIN         3
#define UART_AF_USART2      7

/* Baud rate configuration */
#define USART2_CLK_HZ       16000000UL   // APB1 peripheral clock
#define USART2_BAUDRATE     9600UL
#define USART2_BRR_VALUE    (USART2_CLK_HZ / USART2_BAUDRATE)

void uart_init(void)
{
    /* Enable clocks for GPIOA and USART2 */
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR  |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
    GPIOA->MODER |=  (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk);
    GPIOA->AFR[0] |=  ((UART_AF_USART2 << GPIO_AFRL_AFSEL2_Pos) | (UART_AF_USART2 << GPIO_AFRL_AFSEL3_Pos));

    /* Baud Rate */
    USART2->BRR = USART2_BRR_VALUE;

    /* Enabling TX and USART peripheral */
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void uart_send(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void uart_print(const char *s)
{
    while (*s) uart_send(*s++);
}

void uart_print_num(int32_t n)
{
    char buf[16];
    int i = 0;

    if (n == 0) {
    	uart_send('0');
    	return;
    }

    if (n < 0) {
    	uart_send('-');
    	n = -n;
    }

    while (n > 0) {
    	buf[i++] = (n % 10) + '0'; // Converting remainder to ASCII digit
    	n /= 10; // for next digit
    }
    while (i--) uart_send(buf[i]); //printing digits in the correct order
}


