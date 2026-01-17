/**
 * @file Bluetooth.c
 * @brief ESP32 BLE GATT Server封装库实现 (修复版)
 */

#include "Bluetooth.h"
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"

#define TAG "BLE_MODULE"
#define PREPARE_BUF_MAX_SIZE 1024   // 准备写缓冲区最大值
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40    // 特征值最大长度

/* 内部状态变量 */
static bool ble_connected = false;
static uint8_t current_gatts_if;
static uint16_t current_conn_id;
static esp_bd_addr_t remote_bda;  // 添加：保存远程设备地址
static uint16_t current_mtu = 23;  // 添加：当前MTU大小，默认23
static ble_conn_callback_t conn_callback = NULL;
static ble_data_callback_t data_callback = NULL;

/* 服务配置 */
#define PROFILE_NUM 2   // 服务数量
#define PROFILE_A_APP_ID 0      // Service A应用ID
#define PROFILE_B_APP_ID 1      // Service B应用ID
#define GATTS_NUM_HANDLE_TEST 4 // 每个服务的句柄数量

/* 前向声明 */
//两个服务的事件处理函数
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param); // Service A事件处理函数
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param); // Service B事件处理函数

/* GATT Profile结构体 */
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;       // GATT回调函数
    uint16_t gatts_if;             // GATT接口
    uint16_t app_id;               // 应用ID
    uint16_t conn_id;              // 连接ID
    uint16_t service_handle;       // 服务句柄
    esp_gatt_srvc_id_t service_id; // 服务ID
    uint16_t char_handle;          // 特征句柄
    esp_bt_uuid_t char_uuid;       // 特征UUID
    esp_gatt_perm_t perm;          // 属性权限
    esp_gatt_char_prop_t property; // 特征属性
    uint16_t descr_handle;         // 描述符句柄
    esp_bt_uuid_t descr_uuid;      // 描述符UUID
};

//比如在此，有两个服务，分别用定义两个不同的Profile
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler, // Service A的回调函数
        .gatts_if = ESP_GATT_IF_NONE,              // 初始接口值
    },
    [PROFILE_B_APP_ID] = {
        .gatts_cb = gatts_profile_b_event_handler, // Service B的回调函数
        .gatts_if = ESP_GATT_IF_NONE,              // 初始接口值
    },
};

/* Prepare write环境 */
typedef struct {
    uint8_t *prepare_buf; // 准备写入缓冲区
    int prepare_len;      // 准备写入长度
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;  // Service A的准备写环境
static prepare_type_env_t b_prepare_write_env;  // Service B的准备写环境

/* 初始属性值 */
static uint8_t char1_str[] = {0x11, 0x22, 0x33};    // 特征初始值
static esp_gatt_char_prop_t a_property = 0;         // Service A特征属性
static esp_gatt_char_prop_t b_property = 0;         // Service B特征属性

static esp_attr_value_t gatts_demo_char1_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX, // 特征值最大长度
    .attr_len = sizeof(char1_str),               // 特征值当前长度
    .attr_value = char1_str,                     // 特征值指针
};

/* 广播配置 */
static uint8_t adv_config_done = 0;     // 广播配置完成标志
#define adv_config_flag (1 << 0)        // 广播数据配置标志
#define scan_rsp_config_flag (1 << 1)   // 扫描响应数据配置标志

static uint8_t adv_service_uuid128[32] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAA, 0x00, 0x00, 0x00,
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xBB, 0x00, 0x00, 0x00,
};  // 两个服务的UUID 服务A对应0xAA00，服务B对应0xBB00

//广播数据（必须有的。不管手机问不问，我都在喊。用来让手机发现设备。）
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false, // 是否为扫描响应数据
    .include_name = true,  // 是否包含设备名称
    .include_txpower = false, // 是否包含发射功率
    .min_interval = 0x0006, // 最小广播间隔 (x 1.25ms)  用于连接后的参数协商
    .max_interval = 0x0010, // 最大广播间隔 (x 1.25ms)
    .appearance = 0x00,     // 设备外观
    .manufacturer_len = 0,  // 制造商数据长度
    .p_manufacturer_data = NULL, // 制造商数据指针
    .service_data_len = 0,       // 服务数据长度
    .p_service_data = NULL,      // 服务数据指针
    .service_uuid_len = sizeof(adv_service_uuid128), // 服务UUID长度
    .p_service_uuid = adv_service_uuid128,           // 服务UUID指针
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // 广播标志
};

//扫描响应（选配的（虽然大部分时候都需要）。手机主动问我，我才发。用来给手机提供更多无法塞进广播包的信息。）
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,  // 是否为扫描响应数据
    .include_name = true,  // 是否包含设备名称
    .include_txpower = true, // 是否包含发射功率
    .appearance = 0x00,      // 设备外观
    .manufacturer_len = 0,   // 制造商数据长度
    .p_manufacturer_data = NULL, // 制造商数据指针
    .service_data_len = 0,       // 服务数据长度
    .p_service_data = NULL,      // 服务数据指针
    .service_uuid_len = sizeof(adv_service_uuid128), // 服务UUID长度
    .p_service_uuid = adv_service_uuid128,           // 服务UUID指针
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // 广播标志
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x40,    // 最小广播间隔 (x 0.625ms) 这是真实的物理发送频率。设备现在就在以这个速度广播。与adv_dat里的max/min无关。
    .adv_int_max = 0x40,    // 最大广播间隔 (x 0.625ms)
    .adv_type = ADV_TYPE_IND, // 广播类型：通用可发现
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC, // 自身地址类型：公共地址
    .channel_map = ADV_CHNL_ALL, // 广播通道映射：所有通道
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // 广播过滤策略：允许任何扫描和连接
};

/* ========== 内部函数 ========== */
//prepare_write_env 他是分配仓库的
static void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp) { // 如果客户端请求写响应（Write Request）
        if (param->write.is_prep) { // 判断是否为Prepare Write（用于长写操作，即写入数据超过MTU）
            // 1. 缓冲区管理
            if (prepare_write_env->prepare_buf == NULL) {
                // 首次收到Prepare Write，分配缓冲区
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;     // 内存不足
                }
            } else {
                // 后续收到Prepare Write，检查偏移量是否合法
                if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;   // 偏移量无效
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;  // 属性长度无效
                }
            }

            // 2. 发送响应（Prepare Write必须回复，回传收到的数据以确认）
            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                // 根据BLE协议，Prepare Write Response必须回传请求中的所有字段（句柄、偏移、数据）
                // 以供客户端校验服务器是否正确接收了数据片段，确保数据传输的可靠性
                gatt_rsp->attr_value.len = param->write.len;           // 数据长度
                gatt_rsp->attr_value.handle = param->write.handle;     // 特征句柄
                gatt_rsp->attr_value.offset = param->write.offset;     // 数据偏移量
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE; // 认证请求
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len); // 数据内容
                
                // 发送响应 服务器必须把收到的数据原样发回去作为确认。客户端对比发出去的和收到的响应，一致了才会发下一包。
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if,  //回调函数参数
                            param->write.conn_id,  // 回复给发请求的那个连接
                            param->write.trans_id, // 带着同样的订单号
                            status,                // 告诉它成功还是失败
                            gatt_rsp);             // 把收到的数据原样回显给它(因为是Prepare Write)
                if (response_err != ESP_OK) {
                    ESP_LOGE(TAG, "Send response error");
                }
                free(gatt_rsp); // 释放响应内存
            }
            if (status != ESP_GATT_OK) {    // 出现错误，直接
                return;
            }
            
            // 3. 数据缓存（将分包数据拼接到缓冲区）
            // Prepare Write请求通常用于写入超过MTU的数据，数据会被切分为多个包发送
            // 服务器需要将这些包按照偏移量缓存起来，直到收到Execute Write请求才真正写入
            if (prepare_write_env->prepare_buf) {
                memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
                prepare_write_env->prepare_len += param->write.len;
            }
        } else {
            // 普通Write Request（非长写），直接发送空响应确认
            // 客户端收到响应后才会认为写入成功
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

static void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
        // 收到 Execute Write Request，且标志为 EXEC (执行写入)
        // 此时应该将之前 Prepare Write 缓存的数据真正写入到应用层或硬件
        if (prepare_write_env->prepare_buf) {
            // 这里仅打印数据，实际应用中应在此处处理数据
            esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
        }
    } else {
        // 收到 Execute Write Request，但标志为 CANCEL (取消写入)
        // 或者收到 Cancel Write Request
        // 此时应丢弃之前缓存的所有数据
        ESP_LOGI(TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    
    // 无论执行还是取消，事务结束，必须释放缓冲区内存
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // 广播数据设置完成事件
        // 清除广播配置标志位
        adv_config_done &= (~adv_config_flag);
        // 如果广播数据和扫描响应数据都配置完成，则开始广播 //两个都要配置完！！！0->1->0
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        // 扫描响应数据设置完成事件
        // 清除扫描响应配置标志位
        adv_config_done &= (~scan_rsp_config_flag);
        // 如果广播数据和扫描响应数据都配置完成，则开始广播 //两个都要配置完！！！0->1->0
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // 广播启动完成事件
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            ESP_LOGI(TAG, "Advertising started");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        // 广播停止完成事件
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        // 连接参数更新事件
        // 当连接参数（如连接间隔、延迟、超时）发生变化时触发
        ESP_LOGI(TAG, "Update conn params: status=%d, min=%d, max=%d, conn=%d, latency=%d, timeout=%d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        // 注册应用事件 触发时机：APP 刚刚向蓝牙栈注册成功。
        ESP_LOGI(TAG, "Service A: REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        // 配置服务ID参数
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true; // 主服务
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00; // 实例ID
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16; // UUID长度
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = BLE_SERVICE_UUID_A; // 服务UUID

        // 注意：设备名称已在BLE_Init中设置，这里配置广播数据
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(ret));
        }
        adv_config_done |= adv_config_flag;
        
        // 配置扫描响应数据
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret) {
            ESP_LOGE(TAG, "Config scan rsp data failed: %s", esp_err_to_name(ret));
        }
        adv_config_done |= scan_rsp_config_flag;

        // 创建服务
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST);
        break;

    case ESP_GATTS_READ_EVT: {  //(手机来读数据)
        // 读请求事件
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.value[0] = 0x12;
        rsp.attr_value.value[1] = 0xAB;
        rsp.attr_value.len = 2;
        // 发送读响应
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        // 写请求事件
        ESP_LOGI(TAG, "Service A: GATT_WRITE_EVT, len=%d", param->write.len);
        if (!param->write.is_prep) {
            // 非Prepare Write，直接处理数据
            esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            
            // 调用数据接收回调，通知应用层 main.c中的中断回调函数处理接收到的数据 
            if (data_callback && param->write.len > 0) {
                data_callback(BLE_SERVICE_A, param->write.value, param->write.len);
            }

            // 处理CCC (Client Characteristic Configuration) 描述符写入
            // 客户端通过写入此描述符来开启/关闭 Notify 或 Indicate
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(TAG, "Service A: notify enable");
                } else if (descr_value == 0x0002) {
                    ESP_LOGI(TAG, "Service A: indicate enable");
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(TAG, "Service A: notify/indicate disable");
                }
            }
        }
        // 处理写响应和Prepare Write逻辑  这里只是答复和合并长数据，没有处理数据
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:  //(执行长写)
        // 执行写事件（结束长写）
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;

    case ESP_GATTS_MTU_EVT:
        // MTU交换事件
        ESP_LOGI(TAG, "Service A: MTU %d", param->mtu.mtu);
        current_mtu = param->mtu.mtu;  // 保存当前MTU
        break;

    case ESP_GATTS_CREATE_EVT:
        // 服务创建完成事件
        ESP_LOGI(TAG, "Service A: CREATE_SERVICE_EVT, status %d", param->create.status);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = BLE_CHAR_UUID_A;

        // 启动服务
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        
        // 添加特征
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                               &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               a_property,
                               &gatts_demo_char1_val, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        // 特征添加完成事件
        ESP_LOGI(TAG, "Service A: ADD_CHAR_EVT, status %d", param->add_char.status);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        
        // 添加描述符 (通常是CCC)
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                     &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        // 描述符添加完成事件
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(TAG, "Service A: ADD_DESCR_EVT");
        break;

    case ESP_GATTS_START_EVT:
        // 服务启动完成事件
        ESP_LOGI(TAG, "Service A: SERVICE_START_EVT");
        break;

    case ESP_GATTS_CONNECT_EVT:
        // 设备连接事件
        ESP_LOGI(TAG, "Service A: CONNECT_EVT, conn_id %d", param->connect.conn_id);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        
        // 保存远程设备地址
        memcpy(remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        
        // 更新连接参数 (请求更快的连接间隔以提高吞吐量)
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        esp_ble_gap_update_conn_params(&conn_params);

        current_gatts_if = gatts_if;
        current_conn_id = param->connect.conn_id;
        ble_connected = true;

        // 调用连接回调
        if (conn_callback) {
            conn_callback(true);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        // 设备断开连接事件
        ESP_LOGI(TAG, "Service A: DISCONNECT_EVT, reason 0x%x", param->disconnect.reason);
        // 断开后重新开始广播
        esp_ble_gap_start_advertising(&adv_params);
        ble_connected = false;

        // 调用连接回调
        if (conn_callback) {
            conn_callback(false);
        }
        break;

    case ESP_GATTS_CONF_EVT:
        // 确认事件 (Indication收到确认)
        if (param->conf.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service A: CONF_EVT error");
        }
        break;

    default:
        break;
    }
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        // 注册应用事件
        ESP_LOGI(TAG, "Service B: REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        // 配置服务ID参数
        gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true; // 主服务
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00; // 实例ID
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16; // UUID长度
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = BLE_SERVICE_UUID_B; // 服务UUID

        // 创建服务
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_TEST);
        break;

    case ESP_GATTS_READ_EVT: {
        // 读请求事件
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        // 发送读响应
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        // 写请求事件
        ESP_LOGI(TAG, "Service B: GATT_WRITE_EVT, len=%d", param->write.len);
        if (!param->write.is_prep && param->write.len > 0) {
            // 非Prepare Write，直接处理数据
            esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            
            // 调用数据接收回调，通知应用层
            if (data_callback) {
                data_callback(BLE_SERVICE_B, param->write.value, param->write.len);
            }

            // 处理CCC (Client Characteristic Configuration) 描述符写入
            // 客户端通过写入此描述符来开启/关闭 Notify 或 Indicate
            if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(TAG, "Service B: notify enable");
                } else if (descr_value == 0x0002) {
                    ESP_LOGI(TAG, "Service B: indicate enable");
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(TAG, "Service B: notify/indicate disable");
                }
            }
        }
        // 处理写响应和Prepare Write逻辑
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:
        // 执行写事件（结束长写）
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;

    case ESP_GATTS_MTU_EVT:
        // MTU交换事件
        ESP_LOGI(TAG, "Service B: MTU %d", param->mtu.mtu);
        current_mtu = param->mtu.mtu;  // 保存当前MTU
        break;

    case ESP_GATTS_CREATE_EVT:
        // 服务创建完成事件
        ESP_LOGI(TAG, "Service B: CREATE_SERVICE_EVT, status %d", param->create.status);
        gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = BLE_CHAR_UUID_B;

        // 启动服务
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
        
        // 添加特征
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_B_APP_ID].service_handle,
                               &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               b_property,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        // 特征添加完成事件
        ESP_LOGI(TAG, "Service B: ADD_CHAR_EVT, status %d", param->add_char.status);
        gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        
        // 添加描述符 (通常是CCC)
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle,
                                     &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        // 描述符添加完成事件
        gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(TAG, "Service B: ADD_DESCR_EVT");
        break;

    case ESP_GATTS_START_EVT:
        // 服务启动完成事件
        ESP_LOGI(TAG, "Service B: SERVICE_START_EVT");
        break;

    case ESP_GATTS_CONNECT_EVT:
        // 设备连接事件
        ESP_LOGI(TAG, "Service B: CONNECT_EVT, conn_id %d", param->connect.conn_id);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        // 设备断开连接事件
        ESP_LOGI(TAG, "Service B: DISCONNECT_EVT");
        break;

    case ESP_GATTS_CONF_EVT:
        // 确认事件 (Indication收到确认)
        if (param->conf.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service B: CONF_EVT error");
        }
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    // 1. 处理注册事件 (ESP_GATTS_REG_EVT)
    // 当调用 esp_ble_gatts_app_register() 时，会触发此事件
    // 每个 Application Profile 注册时都会产生一个唯一的 gatts_if (GATT Server Interface)
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            // 注册成功，将 gatts_if 保存到对应的 Profile 结构体中
            // param->reg.app_id 是我们在注册时传入的 ID (PROFILE_A_APP_ID 或 PROFILE_B_APP_ID)
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    // 2. 事件分发
    // 遍历所有 Profile，将事件分发给对应的回调函数
    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        // 过滤条件：
        // - gatts_if == ESP_GATT_IF_NONE: 初始状态或特定全局事件
        // - gatts_if == gl_profile_tab[idx].gatts_if: 事件属于当前 Profile
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
            if (gl_profile_tab[idx].gatts_cb) {
                // 调用对应 Profile 的回调函数 (gatts_profile_a_event_handler 或 gatts_profile_b_event_handler)
                gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

/* ========== 公共API实现 ========== */

esp_err_t BLE_Init(ble_config_t *config) {
    esp_err_t ret;

    // 保存回调函数
    if (config) {
        conn_callback = config->conn_cb;
        data_callback = config->data_cb;
    }

    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 释放Classic BT内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化BT控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 使能BT控制器
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Init bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 使能Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Enable bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置设备名称 (必须在协议栈初始化后设置)
    if (config && config->device_name) {
        ret = esp_ble_gap_set_device_name(config->device_name);
        if (ret) {
            ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        }
    }

    // 设置设备外观
    if (config) {
        BLE_SetAppearance(config->appearance);
    }

    // 注册GATTS回调
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register error: %s", esp_err_to_name(ret));
        return ret;
    }

    // 注册GAP回调
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register error: %s", esp_err_to_name(ret));
        return ret;
    }

    // 注册应用程序
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register error: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register error: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置MTU (增大到最大值以支持更长的数据)
    ret = esp_ble_gatt_set_local_mtu(517);  // 修改：设置为最大值517
    if (ret) {
        ESP_LOGE(TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Local MTU set to 517 bytes");
    }

    ESP_LOGI(TAG, "BLE initialized successfully");
    return ESP_OK;
}

esp_err_t BLE_Deinit(void) {
    esp_err_t ret;

    // 反向初始化过程：严格按照初始化的相反顺序释放资源
    // Init顺序: Controller Init -> Controller Enable -> Bluedroid Init -> Bluedroid Enable
    // Deinit顺序: Bluedroid Disable -> Bluedroid Deinit -> Controller Disable -> Controller Deinit

    // 1. 禁用 Bluedroid 协议栈 (对应 esp_bluedroid_enable)
    // 停止上层协议栈运行
    ret = esp_bluedroid_disable();
    if (ret) {
        ESP_LOGE(TAG, "Disable bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. 去初始化 Bluedroid 协议栈 (对应 esp_bluedroid_init)
    // 释放 Bluedroid 占用的内存资源
    ret = esp_bluedroid_deinit();
    if (ret) {
        ESP_LOGE(TAG, "Deinit bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. 禁用 BT 控制器 (对应 esp_bt_controller_enable)
    // 停止底层控制器运行
    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(TAG, "Disable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 4. 去初始化 BT 控制器 (对应 esp_bt_controller_init)
    // 释放控制器占用的内存资源，完全关闭蓝牙功能
    ret = esp_bt_controller_deinit();
    if (ret) {
        ESP_LOGE(TAG, "Deinit controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BLE deinitialized successfully");
    return ESP_OK;
}

esp_err_t BLE_SendData(ble_service_id_t service_id, uint8_t *data, uint16_t len) {
    if (!ble_connected) {
        ESP_LOGW(TAG, "BLE not connected");
        return ESP_FAIL;
    }

    if (service_id >= BLE_SERVICE_MAX) {    // 超出服务ID范围
        ESP_LOGE(TAG, "Invalid service ID");
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL || len == 0 || len > BLE_MAX_DATA_LEN) {
        ESP_LOGE(TAG, "Invalid data or length");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_ble_gatts_send_indicate(
        current_gatts_if,
        current_conn_id,
        gl_profile_tab[service_id].char_handle,
        len,
        data,
        false       // false表示使用Notify，true表示使用Indicate
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send data failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t BLE_SendString(ble_service_id_t service_id, const char *str) {
    if (str == NULL) {
        ESP_LOGE(TAG, "String is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t len = strlen(str);
    return BLE_SendData(service_id, (uint8_t *)str, len);
}

bool BLE_IsConnected(void) {
    return ble_connected;
}

void BLE_GetMacAddress(uint8_t *mac) {
    if (mac == NULL) {
        return;
    }
    const uint8_t *addr = esp_bt_dev_get_address();
    if (addr) {
        memcpy(mac, addr, 6);
    }
}

esp_err_t BLE_SetDeviceName(const char *name) {
    if (name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_ble_gap_set_device_name(name);
}

esp_err_t BLE_SetAppearance(uint16_t appearance) {
    adv_data.appearance = appearance;
    scan_rsp_data.appearance = appearance;
    
    // 如果蓝牙已初始化，立即更新广播数据
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret != ESP_OK) return ret;
        
        return esp_ble_gap_config_adv_data(&scan_rsp_data);
    }
    
    return ESP_OK;
}

esp_err_t BLE_StartAdvertising(void) {
    return esp_ble_gap_start_advertising(&adv_params);
}

esp_err_t BLE_StopAdvertising(void) {
    return esp_ble_gap_stop_advertising();
}

esp_err_t BLE_Disconnect(void) {
    if (!ble_connected) {
        ESP_LOGW(TAG, "BLE not connected");
        return ESP_FAIL;
    }
    
    // 使用 gatts_close 断开连接
    return esp_ble_gatts_close(current_gatts_if, current_conn_id);
}

uint16_t BLE_GetMTU(void) {
    return current_mtu;
}

esp_err_t BLE_SetLocalMTU(uint16_t mtu) {
    if (mtu < 23 || mtu > 517) {
        ESP_LOGE(TAG, "Invalid MTU size: %d (valid range: 23-517)", mtu);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = esp_ble_gatt_set_local_mtu(mtu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Local MTU set to %d", mtu);
    }
    return ret;
}
