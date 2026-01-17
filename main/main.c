/**
 * @file main.c
 * @brief 蓝牙模块测试程序
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "Bluetooth.h"
#include "Serial.h"  // 引入串口库

#define TAG "BLE_TEST"
#define BUTTON_GPIO 0  // BOOT按键

// 按键状态
static bool last_button_state = false;

/**
 * @brief 连接状态改变回调函数
 */
void ble_connection_callback(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "✓ BLE设备已连接!");
        ESP_LOGI(TAG, "  当前MTU: %d 字节", BLE_GetMTU());
        ESP_LOGI(TAG, "  有效载荷: %d 字节", BLE_GetMTU() - 3);
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "✗ BLE设备已断开连接");
        ESP_LOGI(TAG, "========================================");
    }
}

/**
 * @brief 数据接收回调函数
 */
void ble_data_received_callback(ble_service_id_t service_id, uint8_t *data, uint16_t len) {
    // 处理SPP透传数据
    if (service_id == BLE_SERVICE_SPP) {
        // 将接收到的BLE数据通过串口转发
        // 并在数据末尾添加换行符，方便观察
        Serial_SendArray(data, len);
        Serial_Printf("\n");
        return;
    }

    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "收到数据:");
    ESP_LOGI(TAG, "  服务: %s", service_id == BLE_SERVICE_A ? "Service A (传感器)" : "Service B (聊天)");
    ESP_LOGI(TAG, "  长度: %d 字节", len);
    
    // 尝试将数据作为字符串打印
    char str_buf[256] = {0};
    size_t copy_len = (len < sizeof(str_buf) - 1) ? len : (sizeof(str_buf) - 1);
    memcpy(str_buf, data, copy_len);
    str_buf[copy_len] = '\0';
    ESP_LOGI(TAG, "  内容(字符串): %s", str_buf);
    
    // 打印十六进制数据
    ESP_LOGI(TAG, "  内容(HEX):");
    ESP_LOG_BUFFER_HEX(TAG, data, len);
    ESP_LOGI(TAG, "----------------------------------------");
    
    // 根据不同服务做不同处理
    if (service_id == BLE_SERVICE_A) {
        // 服务A: 传感器控制 - 回显数据
        char response[200];
        snprintf(response, sizeof(response), "ESP32收到传感器数据: %.100s", str_buf);
        BLE_SendString(BLE_SERVICE_A, response);
        ESP_LOGI(TAG, "已发送回应到Service A");
        
    } else if (service_id == BLE_SERVICE_B) {
        // 服务B: 聊天 - 简单回复
        char response[200];
        snprintf(response, sizeof(response), "Hello! 收到你的消息: %.100s", str_buf);
        BLE_SendString(BLE_SERVICE_B, response);
        ESP_LOGI(TAG, "已发送回应到Service B");
    }
}

/**
 * @brief 按键检测任务
 */
void button_task(void *arg) {
    uint8_t send_counter = 0;
    
    while (1) {
        bool current_state = (gpio_get_level(BUTTON_GPIO) == 0);
        
        // 检测按键按下（边沿检测）
        if (current_state && !last_button_state) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 消抖
            
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "按键按下!");
                
                if (BLE_IsConnected()) {
                    // 向服务A发送传感器数据
                    char sensor_data[64];
                    snprintf(sensor_data, sizeof(sensor_data), 
                             "传感器数据#%d: Temp=25.5°C, Hum=60%%", send_counter);
                    
                    if (BLE_SendString(BLE_SERVICE_A, sensor_data) == ESP_OK) {
                        ESP_LOGI(TAG, "✓ 已发送数据到Service A");
                        ESP_LOGI(TAG, "  内容: %s", sensor_data);
                    } else {
                        ESP_LOGE(TAG, "✗ 发送失败");
                    }
                    
                    // 延迟后向服务B发送消息
                    vTaskDelay(pdMS_TO_TICKS(100));
                    
                    char chat_msg[64];
                    snprintf(chat_msg, sizeof(chat_msg), 
                             "你好! 这是第%d条消息", send_counter);
                    
                    if (BLE_SendString(BLE_SERVICE_B, chat_msg) == ESP_OK) {
                        ESP_LOGI(TAG, "✓ 已发送数据到Service B");
                        ESP_LOGI(TAG, "  内容: %s", chat_msg);
                    } else {
                        ESP_LOGE(TAG, "✗ 发送失败");
                    }
                    
                    send_counter++;
                } else {
                    ESP_LOGW(TAG, "✗ 蓝牙未连接，无法发送数据");
                }
                ESP_LOGI(TAG, "========================================");
                
                vTaskDelay(pdMS_TO_TICKS(300)); // 防止重复触发
            }
        }
        
        last_button_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 定时发送任务（可选）
 */
void auto_send_task(void *arg) {
    uint32_t counter = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒
        
        if (BLE_IsConnected()) {
            char msg[64];
            snprintf(msg, sizeof(msg), "自动消息#%ld: 系统运行正常", counter++);
            
            if (BLE_SendString(BLE_SERVICE_B, msg) == ESP_OK) {
                ESP_LOGI(TAG, "定时发送: %s", msg);
            }
        }
    }
}

/**
 * @brief 主函数
 */
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "      ESP32 蓝牙模块测试程序");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "IDF版本: %s", esp_get_idf_version());
    
    // 初始化串口
    Serial_Init();
    ESP_LOGI(TAG, "✓ 串口初始化完成 (115200 8N1)");

    // 配置按键GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "✓ GPIO初始化完成");
    
    // 配置蓝牙
    ble_config_t ble_config = {
        .device_name = "ABCCD",
        .appearance = 0x0000, // 默认BLE
        .conn_cb = ble_connection_callback,
        .data_cb = ble_data_received_callback
    };
    
    // 初始化蓝牙
    ESP_LOGI(TAG, "正在初始化蓝牙...");
    esp_err_t ret = BLE_Init(&ble_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ 蓝牙初始化成功!");
        
        // 获取并打印MAC地址
        uint8_t mac[6];
        BLE_GetMacAddress(mac);
        ESP_LOGI(TAG, "BLE MAC地址: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "系统就绪!");
        ESP_LOGI(TAG, "等待BLE连接...");
        ESP_LOGI(TAG, "按下BOOT键(GPIO0)发送测试数据");
        ESP_LOGI(TAG, "========================================");
        
        // 创建按键检测任务
        xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
        
        // 创建定时发送任务（可选，如不需要可注释掉）
        // xTaskCreate(auto_send_task, "auto_send_task", 4096, NULL, 3, NULL);
        
    } else {
        ESP_LOGE(TAG, "✗ 蓝牙初始化失败: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "系统将重启...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    
    // 主循环 - 检查串口数据并发送到BLE
    uint32_t loop_counter = 0;
    while (1) {
        // 检查是否有串口数据包
        if (Serial_RxFlag) {
            // 通过SPP服务发送到手机
            if (BLE_IsConnected()) {
                // 使用BLE_SendData发送原始二进制数据
                BLE_SendData(BLE_SERVICE_SPP, (uint8_t*)Serial_RxPacket, Serial_RxLen);
                ESP_LOGI(TAG, "串口 -> BLE SPP: %d 字节", Serial_RxLen);
            } else {
                ESP_LOGW(TAG, "收到串口数据但BLE未连接，丢弃 %d 字节", Serial_RxLen);
            }
            // 清除标志位
            Serial_RxFlag = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 防止看门狗触发

        // 每30秒打印一次状态
        if (loop_counter++ % 3000 == 0) {
            ESP_LOGI(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "系统运行时间: %ld 秒", loop_counter / 100);
            ESP_LOGI(TAG, "连接状态: %s", BLE_IsConnected() ? "已连接" : "未连接");
            ESP_LOGI(TAG, "空闲堆: %ld 字节", esp_get_free_heap_size());
            ESP_LOGI(TAG, "----------------------------------------");
        }
    }
}
