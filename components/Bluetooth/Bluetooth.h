/**
 * @file Bluetooth.h
 * @brief ESP32 BLE GATT Server封装库
 * @author Your Name
 * @date 2024
 */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_gatts_api.h"

/* 配置参数 */
#define BLE_DEVICE_NAME         "ABCCD"  
#define BLE_SERVICE_UUID_A      0x00AA
#define BLE_CHAR_UUID_A         0xAA01
#define BLE_SERVICE_UUID_B      0x00BB
#define BLE_CHAR_UUID_B         0xBB01
#define BLE_MAX_DATA_LEN        512

/* 服务ID定义 */
typedef enum {
    BLE_SERVICE_A = 0,  // 传感器控制与读取服务
    BLE_SERVICE_B = 1,  // 聊天服务
    BLE_SERVICE_MAX
} ble_service_id_t;

/* 连接状态回调 */
typedef void (*ble_conn_callback_t)(bool connected);

/* 数据接收回调 */
typedef void (*ble_data_callback_t)(ble_service_id_t service_id, uint8_t *data, uint16_t len);

/**
 * @brief 蓝牙配置结构体
 */
typedef struct {
    const char *device_name;            // 设备名称
    ble_conn_callback_t conn_cb;        // 连接状态回调
    ble_data_callback_t data_cb;        // 数据接收回调
} ble_config_t;

/**
 * @brief 初始化蓝牙模块
 * @param config 配置参数
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_Init(ble_config_t *config);

/**
 * @brief 反初始化蓝牙模块
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_Deinit(void);

/**
 * @brief 发送数据到指定服务
 * @param service_id 服务ID
 * @param data 数据指针
 * @param len 数据长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_SendData(ble_service_id_t service_id, uint8_t *data, uint16_t len);

/**
 * @brief 发送字符串到指定服务
 * @param service_id 服务ID
 * @param str 字符串指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_SendString(ble_service_id_t service_id, const char *str);

/**
 * @brief 获取连接状态
 * @return true 已连接, false 未连接
 */
bool BLE_IsConnected(void);

/**
 * @brief 获取蓝牙MAC地址
 * @param mac 6字节MAC地址缓冲区
 */
void BLE_GetMacAddress(uint8_t *mac);

/**
 * @brief 设置设备名称
 * @param name 设备名称
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_SetDeviceName(const char *name);

/**
 * @brief 开始广播
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_StartAdvertising(void);

/**
 * @brief 停止广播
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_StopAdvertising(void);

/**
 * @brief 断开当前连接
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_Disconnect(void);

/**
 * @brief 获取当前MTU大小
 * @return MTU大小（字节）
 */
uint16_t BLE_GetMTU(void);

/**
 * @brief 设置本地MTU大小
 * @param mtu MTU大小（建议: 23-517）
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t BLE_SetLocalMTU(uint16_t mtu);

#endif /* __BLUETOOTH_H__ */