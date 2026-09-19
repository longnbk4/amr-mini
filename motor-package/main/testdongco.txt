#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "esp_netif.h"

// 1. ĐỊNH NGHĨA CHÂN L298N 
#define L_PWM_PIN   10  // ENA
#define L_IN1_PIN   12  // INPUT1
#define L_IN2_PIN   13  // INPUT2

#define R_PWM_PIN   16  // ENB
#define R_IN1_PIN   14  // INPUT3
#define R_IN2_PIN   15  // INPUT4

// 2. ĐỊNH NGHĨA CHÂN ENCODER (ĐÃ FIX CHUẨN)
#define L_ENC_B_PIN 6   // L ENCODER B
#define L_ENC_A_PIN 7   // L ENCODER A
#define R_ENC_A_PIN 8   // R ENCODER A
#define R_ENC_B_PIN 9   // R ENCODER B

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// Biến điều khiển
static int motor_speed = 120;
static volatile uint32_t last_command_time = 0;
static volatile bool robot_moving = false;
static const uint32_t COMMAND_TIMEOUT_MS = 400;

static const char *TAG = "ROBOT";

// ==========================================
// ISR Encoder
// ==========================================
static void IRAM_ATTR left_enc_isr_handler(void* arg) {
    if (gpio_get_level(L_ENC_B_PIN)) left_ticks++;
    else left_ticks--;
}

static void IRAM_ATTR right_enc_isr_handler(void* arg) {
    if (gpio_get_level(R_ENC_B_PIN)) right_ticks++;
    else right_ticks--;
}

// ==========================================
// Khởi tạo phần cứng
// ==========================================
void init_hardware(void) {
    // PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_L = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = L_PWM_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel_L);

    ledc_channel_config_t ledc_channel_R = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = R_PWM_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel_R);

    // Direction pins
    gpio_set_direction(L_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(L_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(R_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(R_IN2_PIN, GPIO_MODE_OUTPUT);

    // Encoder
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << L_ENC_A_PIN) | (1ULL << R_ENC_A_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << L_ENC_B_PIN) | (1ULL << R_ENC_B_PIN);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(L_ENC_A_PIN, left_enc_isr_handler, NULL);
    gpio_isr_handler_add(R_ENC_A_PIN, right_enc_isr_handler, NULL);
}

// Điều khiển motor

void set_motor_speed(int left_speed, int right_speed) {
    if (left_speed > 255) left_speed = 255;
    if (left_speed < -255) left_speed = -255;
    if (right_speed > 255) right_speed = 255;
    if (right_speed < -255) right_speed = -255;

    // Motor Trái
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

    // Motor Phải
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

    robot_moving = (left_speed != 0 || right_speed != 0);
    last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void stop_robot(void) {
    set_motor_speed(0, 0);
}

void execute_movement(const char *cmd) {
    if (strcmp(cmd, "FORWARD") == 0) {
        set_motor_speed(motor_speed, motor_speed);
    } else if (strcmp(cmd, "BACKWARD") == 0) {
        set_motor_speed(-motor_speed, -motor_speed);
    } else if (strcmp(cmd, "LEFT") == 0) {
        set_motor_speed(-motor_speed, motor_speed);
    } else if (strcmp(cmd, "RIGHT") == 0) {
        set_motor_speed(motor_speed, -motor_speed);
    } else {
        stop_robot();
    }
}

// HTML trang điều khiển
static const char CONTROL_PAGE[] =
"<!doctype html><html lang=\"vi\"><head>"
"<meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>Điều khiển robot</title>"
"<style>"
"*{box-sizing:border-box}body{margin:0;padding:24px;font-family:system-ui,Arial,sans-serif;"
"background:#121212;color:#f2f2f2;text-align:center}"
".panel{max-width:620px;margin:auto;padding:24px;background:#1e1e1e;border:1px solid #4d4d4d;border-radius:16px}"
"h1{margin-top:0}.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:440px;margin:24px auto}"
"button{min-height:82px;padding:12px;border:1px solid #777;border-radius:12px;background:#303030;color:#fff;"
"font-size:18px;cursor:pointer;touch-action:none}button:active,button.active{background:#666}"
"button.stop{background:#802323}input[type=range]{width:100%;max-width:440px}"
"kbd{display:inline-block;min-width:28px;padding:3px 7px;border-radius:5px;background:#383838;border:1px solid #666}"
".status{margin:18px 0;padding:10px;border-radius:8px;background:#292929}.note{color:#bdbdbd;font-size:14px}"
"</style></head><body><div class=\"panel\">"
"<h1>Điều khiển robot JGA</h1>"
"<p><kbd>W</kbd> tiến, <kbd>S</kbd> lùi, <kbd>A</kbd> trái, <kbd>D</kbd> phải, <kbd>Space</kbd> STOP</p>"
"<div class=\"status\">Lệnh: <strong id=\"status\">STOP</strong></div>"
"<div class=\"controls\">"
"<div></div><button data-command=\"FORWARD\">W<br>TIẾN</button><div></div>"
"<button data-command=\"LEFT\">A<br>TRÁI</button>"
"<button class=\"stop\" data-command=\"STOP\">STOP</button>"
"<button data-command=\"RIGHT\">D<br>PHẢI</button>"
"<div></div><button data-command=\"BACKWARD\">S<br>LÙI</button><div></div>"
"</div>"
"<p>Tốc độ PWM: <strong id=\"speedValue\">120</strong> / 255</p>"
"<input id=\"speed\" type=\"range\" min=\"45\" max=\"255\" value=\"120\" step=\"1\">"
"<p class=\"note\">Giữ phím/nút để chạy liên tục. Nhả ra hoặc mất kết nối → tự dừng.</p>"
"</div>"
"<script>"
"const statusText=document.getElementById('status'),speedSlider=document.getElementById('speed'),speedValue=document.getElementById('speedValue');"
"const keyboardMap={w:'FORWARD',arrowup:'FORWARD',s:'BACKWARD',arrowdown:'BACKWARD',a:'LEFT',arrowleft:'LEFT',d:'RIGHT',arrowright:'RIGHT',x:'STOP',' ':'STOP'};"
"let activeCommand=null,keepAliveTimer=null;"
"function sendCommand(c){fetch('/command?value='+encodeURIComponent(c),{cache:'no-store'}).catch(()=>{});}"
"function setStatus(c){statusText.textContent=c;}"
"function startCommand(c){if(activeCommand===c)return;stopCommand(false);activeCommand=c;setStatus(c);sendCommand(c);"
"if(c!=='STOP')keepAliveTimer=setInterval(()=>sendCommand(c),120);}"
"function stopCommand(sendStop=true){if(keepAliveTimer){clearInterval(keepAliveTimer);keepAliveTimer=null;}activeCommand=null;setStatus('STOP');if(sendStop)sendCommand('STOP');}"
"document.addEventListener('keydown',e=>{const k=e.key.toLowerCase();if(keyboardMap[k]){e.preventDefault();startCommand(keyboardMap[k]);}});"
"document.addEventListener('keyup',e=>{const k=e.key.toLowerCase();if(keyboardMap[k]){e.preventDefault();stopCommand(true);}});"
"window.addEventListener('blur',()=>stopCommand(true));"
"document.addEventListener('visibilitychange',()=>{if(document.hidden)stopCommand(true);});"
"document.querySelectorAll('button[data-command]').forEach(btn=>{"
"btn.addEventListener('pointerdown',e=>{e.preventDefault();startCommand(btn.dataset.command);btn.classList.add('active');});"
"const release=e=>{e.preventDefault();btn.classList.remove('active');stopCommand(true);};"
"btn.addEventListener('pointerup',release);btn.addEventListener('pointercancel',release);"
"btn.addEventListener('pointerleave',e=>{if(e.buttons!==0)release(e);});});"
"speedSlider.addEventListener('input',()=>{speedValue.textContent=speedSlider.value;"
"fetch('/speed?value='+speedSlider.value,{cache:'no-store'}).catch(()=>{});});"
"</script></body></html>";

// HTTP Handlers
static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CONTROL_PAGE, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t command_get_handler(httpd_req_t *req) {
    char buf[64] = {0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char value[32] = {0};
        if (httpd_query_key_value(buf, "value", value, sizeof(value)) == ESP_OK) {
            ESP_LOGI(TAG, "Command: %s", value);
            execute_movement(value);
        }
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t speed_get_handler(httpd_req_t *req) {
    char buf[64] = {0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char value[16] = {0};
        if (httpd_query_key_value(buf, "value", value, sizeof(value)) == ESP_OK) {
            int v = atoi(value);
            if (v < 45) v = 45;
            if (v > 255) v = 255;
            motor_speed = v;
            ESP_LOGI(TAG, "Speed set to %d", motor_speed);
        }
    }
    char resp[16];
    snprintf(resp, sizeof(resp), "%d", motor_speed);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// SoftAP + HTTP Server
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "JGA_Robot",
            .ssid_len = 0,
            .password = "robot1234",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started → SSID: JGA_Robot | Pass: robot1234");
    ESP_LOGI(TAG, "Mở trình duyệt: http://192.168.4.1");
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t cmd = { .uri = "/command", .method = HTTP_GET, .handler = command_get_handler };
        httpd_register_uri_handler(server, &cmd);

        httpd_uri_t spd = { .uri = "/speed", .method = HTTP_GET, .handler = speed_get_handler };
        httpd_register_uri_handler(server, &spd);

        ESP_LOGI(TAG, "HTTP server started");
    }
    return server;
}

// Task timeout an toàn
static void safety_task(void *arg) {
    while (1) {
        if (robot_moving) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_command_time > COMMAND_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Timeout → STOP");
                stop_robot();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


// MAIN

void app_main(void) {
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_hardware();
    stop_robot();

    wifi_init_softap();
    start_webserver();

    xTaskCreate(safety_task, "safety", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "=== ROBOT READY ===");
    ESP_LOGI(TAG, "Wi-Fi: JGA_Robot / robot1234");
    ESP_LOGI(TAG, "Truy cập: http://192.168.4.1");

    // In encoder mỗi giây (tùy chọn)
    while (1) {
        ESP_LOGI(TAG, "Encoder → LEFT: %ld | RIGHT: %ld | Speed: %d",
                 left_ticks, right_ticks, motor_speed);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}