#include "motor.h"

#define MOTOR_A_EN_PIN      GPIO_NUM_21   // GPIO kết nối với chân ENABLE của motor A
#define MOTOR_A_IN1_PIN     GPIO_NUM_22  // GPIO kết nối với chân IN1 của motor A
#define MOTOR_A_IN2_PIN     GPIO_NUM_23  // GPIO kết nối với chân IN2 của motor A
#define MOTOR_A_PWM_CHANNEL LEDC_CHANNEL_1 // LEDC PWM channel sử dụng cho motor A
#define MOTOR_A_PWM_FREQ    1000            // Tần số PWM cho motor A (Hz)
#define MOTOR_A_PWM_BIT     LEDC_TIMER_10_BIT // Độ phân giải PWM cho motor A

void motor_init() {
    gpio_config_t io_conf;

    // Cấu hình GPIO cho các chân điều khiển động cơ
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    // Kết nối chân ENABLE của motor A với GPIO
    io_conf.pin_bit_mask = (1ULL << MOTOR_A_EN_PIN);
    gpio_config(&io_conf);

    // Kết nối các chân IN1 và IN2 của motor A với GPIO
    io_conf.pin_bit_mask = (1ULL << MOTOR_A_IN1_PIN) | (1ULL << MOTOR_A_IN2_PIN);
    gpio_config(&io_conf);

    // Khởi tạo LEDC PWM timer cho motor A
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.duty_resolution = MOTOR_A_PWM_BIT;
    ledc_timer.freq_hz = MOTOR_A_PWM_FREQ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Kết nối PWM channel của motor A với GPIO
    ledc_channel_config_t ledc_channel;
    ledc_channel.channel = MOTOR_A_PWM_CHANNEL;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = MOTOR_A_EN_PIN;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_config(&ledc_channel);
}

void motor_set_speed(uint32_t speed) {
    // Đặt tốc độ quay cho motor A sử dụng PWM
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_A_PWM_CHANNEL, speed);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_A_PWM_CHANNEL);

}

void motor_forward() {
    // Điều khiển motor A quay theo chiều thuận
    gpio_set_level(MOTOR_A_IN1_PIN, 1);
    gpio_set_level(MOTOR_A_IN2_PIN, 0);
}

void motor_backward() {
    // Điều khiển motor A quay theo chiều nghịch
    gpio_set_level(MOTOR_A_IN1_PIN, 0);
    gpio_set_level(MOTOR_A_IN2_PIN, 1);
}

void motor_stop() {
    // Dừng động cơ bằng cách đặt cả hai chân IN1 và IN2 về mức thấp
    gpio_set_level(MOTOR_A_IN1_PIN, 0);
    gpio_set_level(MOTOR_A_IN2_PIN, 0);
}

void motor_deinit() {
    // Vô hiệu hóa PWM cho motor A
    ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_A_PWM_CHANNEL, 0);

    // Đặt các chân IN1 và IN2 của motor A về mức thấp
    gpio_set_level(MOTOR_A_IN1_PIN, 0);
    gpio_set_level(MOTOR_A_IN2_PIN, 0);

    // Cấu hình lại các chân GPIO về chế độ mặc định (input hoặc cao trở)
    gpio_reset_pin(MOTOR_A_EN_PIN);
    gpio_reset_pin(MOTOR_A_IN1_PIN);
    gpio_reset_pin(MOTOR_A_IN2_PIN);
}


/*************USAGE EXAMPLE***********************/
/*
void app_main() {
    motor_init(); // Khởi tạo driver điều khiển động cơ

    while (1) {
        // Đặt tốc độ và hướng quay của động cơ
        motor_set_speed(500); // Đặt tốc độ là 500 (giá trị từ 0 đến 1023)
        motor_forward(); // Điều khiển motor đi về phía trước
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Chờ 2 giây

        // Đặt tốc độ và hướng quay của động cơ
        motor_set_speed(800); // Đặt tốc độ là 800
        motor_backward(); // Điều khiển motor đi về phía sau
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Chờ 2 giây
    }
}
*/