#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"

// ==================== CHÂN PHẦN CỨNG ====================
#define L_PWM_PIN     10
#define L_IN1_PIN     12
#define L_IN2_PIN     13
#define R_PWM_PIN     16
#define R_IN1_PIN     14
#define R_IN2_PIN     15
#define L_ENC_B_PIN   6
#define L_ENC_A_PIN   7
#define R_ENC_A_PIN   8
#define R_ENC_B_PIN   9
#define UART_TX_PIN   17
#define UART_RX_PIN   18
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE      1024
#define COMMAND_TIMEOUT_MS 10000          

// ==================== HÌNH HỌC ====================
#define WHEEL_DIAMETER_M       0.065f
#define WHEEL_CIRCUMFERENCE_M  (WHEEL_DIAMETER_M * 3.14159265f)
#define TRACK_WIDTH_M          0.1415f
#define PULSES_PER_WHEEL_REV   224.4f

// ==================== TỐC ĐỘ MẶC ĐỊNH ====================
#define MIN_SPEED     80
#define MAX_SPEED     255
static int default_speed = 180;

// ==================== BIẾN ====================
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
static portMUX_TYPE ticks_mux = portMUX_INITIALIZER_UNLOCKED;

static float rpm_left  = 0.0f;
static float rpm_right = 0.0f;
static float robot_x = 0.0f;
static float robot_y = 0.0f;
static float robot_theta = 0.0f;

static volatile uint32_t last_command_time = 0;
static char current_cmd = 'X';
static int  last_custom_l = 0;
static int  last_custom_r = 0;

static const char *TAG = "UART_BRIDGE";

// ==================== ISR ENCODER ====================
static void IRAM_ATTR left_enc_isr_handler(void* arg) {
    if (gpio_get_level(L_ENC_B_PIN)) left_ticks++;
    else left_ticks--;
}

static void IRAM_ATTR right_enc_isr_handler(void* arg) {
    if (gpio_get_level(R_ENC_B_PIN)) right_ticks--;
    else right_ticks++;
}

// ==================== MOTOR ====================
void set_motor_speed(int left_speed, int right_speed) {
    left_speed  = left_speed  > 255 ? 255 : (left_speed  < -255 ? -255 : left_speed);
    right_speed = right_speed > 255 ? 255 : (right_speed < -255 ? -255 : right_speed);

    if (left_speed >= 0) {
        gpio_set_level(L_IN1_PIN, 1);
        gpio_set_level(L_IN2_PIN, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left_speed);
    } else {
        gpio_set_level(L_IN1_PIN, 0);
        gpio_set_level(L_IN2_PIN, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, -left_speed);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    if (right_speed >= 0) {
        gpio_set_level(R_IN1_PIN, 1);
        gpio_set_level(R_IN2_PIN, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right_speed);
    } else {
        gpio_set_level(R_IN1_PIN, 0);
        gpio_set_level(R_IN2_PIN, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, -right_speed);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void stop_robot(void) {
    set_motor_speed(0, 0);
}

void apply_current_command(void) {
    switch (current_cmd) {
        case 'W': set_motor_speed( default_speed,  default_speed); break;
        case 'S': set_motor_speed(-default_speed, -default_speed); break;
        case 'A': set_motor_speed(-default_speed,  default_speed); break;
        case 'D': set_motor_speed( default_speed, -default_speed); break;
        case 'X': set_motor_speed(0, 0); break;
        case 'V': set_motor_speed(last_custom_l, last_custom_r); break;
        default:  set_motor_speed(0, 0); break;
    }
    last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// ==================== KHỞI TẠO ====================
void init_hardware(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch_l = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = L_PWM_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config_t ch_r = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = R_PWM_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_l);
    ledc_channel_config(&ch_r);

    gpio_set_direction(L_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(L_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(R_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(R_IN2_PIN, GPIO_MODE_OUTPUT);

    gpio_config_t io = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << L_ENC_A_PIN) | (1ULL << R_ENC_A_PIN),
        .pull_up_en = 1
    };
    gpio_config(&io);
    io.intr_type = GPIO_INTR_DISABLE;
    io.pin_bit_mask = (1ULL << L_ENC_B_PIN) | (1ULL << R_ENC_B_PIN);
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(L_ENC_A_PIN, left_enc_isr_handler, NULL);
    gpio_isr_handler_add(R_ENC_A_PIN, right_enc_isr_handler, NULL);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ==================== TASK ODOMETRY ====================
static void odometry_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);
    long prev_l = 0, prev_r = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        float dt = 0.050f;

        long curr_l, curr_r;
        portENTER_CRITICAL(&ticks_mux);
        curr_l = left_ticks;
        curr_r = right_ticks;
        portEXIT_CRITICAL(&ticks_mux);

        long delta_l = curr_l - prev_l;
        long delta_r = curr_r - prev_r;
        prev_l = curr_l;
        prev_r = curr_r;

        rpm_left  = (delta_l / PULSES_PER_WHEEL_REV) * 60.0f / dt;
        rpm_right = (delta_r / PULSES_PER_WHEEL_REV) * 60.0f / dt;

        float v_l = (rpm_left  / 60.0f) * WHEEL_CIRCUMFERENCE_M;
        float v_r = (rpm_right / 60.0f) * WHEEL_CIRCUMFERENCE_M;
        float vx  = (v_l + v_r) / 2.0f;
        float w   = (v_r - v_l) / TRACK_WIDTH_M;

        robot_theta += w * dt;
        while (robot_theta >  M_PI) robot_theta -= 2.0f * M_PI;
        while (robot_theta < -M_PI) robot_theta += 2.0f * M_PI;

        robot_x += vx * cosf(robot_theta) * dt;
        robot_y += vx * sinf(robot_theta) * dt;
    }
}

// ==================== TASK NHẬN LỆNH ====================
static void uart_rx_task(void *arg)
{
    uint8_t *data = malloc(BUF_SIZE);
    char line[128];
    int pos = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(10));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                if (c == '\n' || c == '\r') {
                    if (pos > 0) {
                        line[pos] = 0;

                        if (strcmp(line, "W") == 0) {
                            current_cmd = 'W';
                            apply_current_command();
                        }
                        else if (strcmp(line, "S") == 0) {
                            current_cmd = 'S';
                            apply_current_command();
                        }
                        else if (strcmp(line, "A") == 0) {
                            current_cmd = 'A';
                            apply_current_command();
                        }
                        else if (strcmp(line, "D") == 0) {
                            current_cmd = 'D';
                            apply_current_command();
                        }
                        else if (strcmp(line, "X") == 0) {
                            current_cmd = 'X';
                            apply_current_command();
                        }
                        else if (strcmp(line, "+") == 0) {
                            default_speed += 10;
                            if (default_speed > MAX_SPEED) default_speed = MAX_SPEED;
                            apply_current_command();
                            ESP_LOGI(TAG, "DEFAULT_SPEED = %d", default_speed);
                        }
                        else if (strcmp(line, "-") == 0) {
                            default_speed -= 10;
                            if (default_speed < MIN_SPEED) default_speed = MIN_SPEED;
                            apply_current_command();
                            ESP_LOGI(TAG, "DEFAULT_SPEED = %d", default_speed);
                        }
                        else if (strcmp(line, "R") == 0) {
                            portENTER_CRITICAL(&ticks_mux);
                            left_ticks = 0;
                            right_ticks = 0;
                            portEXIT_CRITICAL(&ticks_mux);
                            robot_x = 0.0f;
                            robot_y = 0.0f;
                            robot_theta = 0.0f;
                            ESP_LOGI(TAG, "Odometry RESET");
                        }
                        else if (line[0] == 'V') {
                            int l = 0, r = 0;
                            if (sscanf(line, "V,%d,%d", &l, &r) == 2) {
                                current_cmd = 'V';
                                last_custom_l = l;
                                last_custom_r = r;
                                apply_current_command();
                            }
                        }
                        pos = 0;
                    }
                } else if (pos < (int)sizeof(line) - 1) {
                    line[pos++] = c;
                } else {
                    pos = 0;
                }
            }
        }

        // Timeout 10s
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - last_command_time) > COMMAND_TIMEOUT_MS) {
            if (current_cmd != 'X') {
                current_cmd = 'X';
                stop_robot();
                ESP_LOGW(TAG, "Timeout -> STOP");
            }
        }

        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(3));
        }
    }
    free(data);
}

// ==================== MAIN ====================
void app_main(void)
{
    init_hardware();
    stop_robot();
    last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    xTaskCreate(uart_rx_task,  "uart_rx", 4096, NULL, 15, NULL);  
    xTaskCreate(odometry_task, "odom",   4096, NULL,  8, NULL);

    ESP_LOGI(TAG, "=== UART BRIDGE + ODOMETRY READY ===");
    ESP_LOGI(TAG, "Default speed = %d | Timeout = %dms", default_speed, COMMAND_TIMEOUT_MS);

    char tx_buf[96];
    while (1) {
        int len = snprintf(tx_buf, sizeof(tx_buf),
            "E,%ld,%ld,%.1f,%.1f,%.3f,%.3f,%.1f\n",
            left_ticks, right_ticks,
            rpm_left, rpm_right,
            robot_x, robot_y, robot_theta * 57.2958f);
        uart_write_bytes(UART_PORT_NUM, tx_buf, len);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}