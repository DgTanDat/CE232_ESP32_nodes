#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "fan.h"

#define GPIO_PWM0A_OUT GPIO_NUM_21   // ENA
#define GPIO_IN1 GPIO_NUM_22         // IN1
#define GPIO_IN2 GPIO_NUM_23         // IN2

void mcpwm_example_gpio_initialize(void)
{
    printf("Initializing MCPWM GPIO...\n");

    // Set IN1 and IN2 as output
    gpio_pad_select_gpio(GPIO_IN1);
    gpio_set_direction(GPIO_IN1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(GPIO_IN2);
    gpio_set_direction(GPIO_IN2, GPIO_MODE_OUTPUT);
}

void fan_init(void)
{
    // 1. Initialize MCPWM GPIO
    mcpwm_example_gpio_initialize();

    // 2. Configure MCPWM unit 0 timer 0
    printf("Configuring MCPWM...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    // frequency = 1kHz
    pwm_config.cmpr_a = 0;          // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;          // duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
/*
    while (1) {
        
        printf("Rotating forward\n");
        gpio_set_level(GPIO_IN1, 1);
        gpio_set_level(GPIO_IN2, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0); // Set duty cycle to 50%
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

        vTaskDelay(2000 / portTICK_RATE_MS);

        printf("Stopping motor\n");
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0); // Stop motor
        vTaskDelay(1000 / portTICK_RATE_MS);

        printf("Rotating backward 10\n");
        gpio_set_level(GPIO_IN1, 0);
        gpio_set_level(GPIO_IN2, 1);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0); // Set duty cycle to 50%
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

        vTaskDelay(2000 / portTICK_RATE_MS);



        printf("Stopping motor\n");
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0); // Stop motor
        vTaskDelay(1000 / portTICK_RATE_MS);

        printf("Rotating backward 100\n");
        gpio_set_level(GPIO_IN1, 0);
        gpio_set_level(GPIO_IN2, 1);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100.0); // Set duty cycle to 50%
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

        vTaskDelay(2000 / portTICK_RATE_MS);
        
    }
*/
}
