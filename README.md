## EV Pedal Control System (STM32F411)

This project implements a closed-loop DC motor speed control system using an STM32F411 microcontroller.  
Motor speed is controlled using a bare-metal PID controller with real-time feedback from a quadrature encoder.

A potentiometer is used to set the target RPM, while PWM drives the motor through an L298N motor driver.  
Encoder feedback is used to calculate RPM, and the PID algorithm adjusts the PWM duty cycle to match the target speed.

The firmware is written without HAL or RTOS, using direct register-level programming.  
Motor performance (RPM vs time) is logged over UART and analyzed using a Python script.

**Key features:**
- Bare-metal PID control implementation  
- PWM motor control with encoder feedback  
- STM32 timers for PWM, encoder decoding, and control loop timing  
- Python script for RPM visualization and analysis  
