/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/uart.h"
#include "senser.h"
#include "config.h"

// 新增代码：定义传感器和设备引脚
#define SENSOR_PIN  6 // 假设传感器连接到 GPIO 6
#define DEVICE_PIN  12  // 控制另一个设备的 GPIO 引脚
// UART配置（与K230通信）
#define UART_NUM       UART_NUM_1
#define UART_RX_PIN   8
#define UART_TX_PIN   9
#define BUF_SIZE       1024


// 报警开关属性定义（Alink协议）
#define ALARM_TOPIC    "/sys/a15D0zFeGtP/AHT10/thing/event/property/post"
#define ALARM_MSG      "{\"params\":{\"AlarmSwitch\":1}}"
#define RESET_MSG      "{\"params\":{\"AlarmSwitch\":0}}"

// 新增代码：GPIO 初始化函数
static void gpio_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // 配置传感器引脚为输入模式
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SENSOR_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // 配置设备引脚为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DEVICE_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

// 新增代码：GPIO 任务函数
void gpio_task(void *pvParameters)
{
    gpio_init();

    while(1) {
        if (gpio_get_level(SENSOR_PIN)) {
            // If GPIO_NUM_INPUT is high, set GPIO_NUM_OUTPUT to low
            gpio_set_level(DEVICE_PIN, 0);
        }
        else {
            // If GPIO_NUM_INPUT is low, set GPIO_NUM_OUTPUT to high
            gpio_set_level(DEVICE_PIN, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay to avoid rapid toggling
    }
}

static const char *TAG = "app_main.c";

esp_mqtt_client_handle_t client;
// 修改UART初始化函数
static void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB  // 显式指定时钟源
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0); // 修复缓冲区大小
}
// UART接收任务（监听K230的报警信号）
void uart_receive_task(void *pvParameters) {
    char rx_buffer[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t*)rx_buffer, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0) {
            rx_buffer[len] = '\0';
            if (strstr(rx_buffer, "mouse_ALARM")) {
                // 上报报警开启状态
                esp_mqtt_client_publish(client, ALARM_TOPIC, ALARM_MSG, 0, 1, 0);
                ESP_LOGI(TAG, "AlarmSwitch=1 sent");

                // 模拟报警结束后重置（根据实际需求调整）
                vTaskDelay(pdMS_TO_TICKS(50000));
                esp_mqtt_client_publish(client, ALARM_TOPIC, RESET_MSG, 0, 1, 0);
                ESP_LOGI(TAG, "AlarmSwitch=0 sent");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // msg_id = esp_mqtt_client_publish(client, g_mqtt_topic_pub, "Device online.", 0, 1, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_SUB, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        printf("Recieve message: %.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    // char last_will[25] = {0};
    // sprintf(last_will, "Device %s is offline", g_chipid);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .credentials.username = "AHT10&a15D0zFeGtP",
        .credentials.authentication.password = "1c842e35aeb48a1d1e8f48be6f86fc6c72d8dd1f8456a7a2b55e905fe48f6c64",
        .credentials.client_id = "a15D0zFeGtP.AHT10|securemode=2,signmethod=hmacsha256,timestamp=1746279845557|",
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .session.keepalive = 120,
        .session.disable_clean_session = false,
        // .session.last_will.topic = "/test/offline",
        // .session.last_will.msg = last_will,
        // .session.last_will.qos = 1,
        // .session.last_will.retain = false,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    // uint8_t mac[6];
    // esp_efuse_mac_get_default(mac);
    // sprintf(g_chipid, "%02x%02x%02x", mac[3], mac[4], mac[5]);
    // ESP_LOGI(TAG, "ESP32-C3 Chipid: %s", g_chipid);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // 初始化UART和MQTT
    uart_init();
    mqtt_app_start();

 
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(senser_task_run, "senser_task", 4096, NULL, 10, NULL);
    // 创建UART接收任务
    xTaskCreate(uart_receive_task, "uart_task", 4096, NULL, 5, NULL);
    // 新增代码：创建 GPIO 任务
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 5, NULL);

    while (1) {
        SLEEP_MS(100000);
    }
}