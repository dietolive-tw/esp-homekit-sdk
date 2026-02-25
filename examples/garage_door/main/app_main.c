/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* HomeKit Garage Door Opener Example
 * 
 * This example demonstrates a HomeKit Garage Door Opener with three physical buttons:
 * - Open Button (GPIO_NUM_25)
 * - Stop Button (GPIO_NUM_26)
 * - Close Button (GPIO_NUM_27)
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

static const char *TAG = "HAP Garage Door";

#define GARAGE_DOOR_TASK_PRIORITY  1
#define GARAGE_DOOR_TASK_STACKSIZE 4 * 1024
#define GARAGE_DOOR_TASK_NAME      "hap_garage_door"

/* GPIO 定义 */
#define RESET_GPIO        GPIO_NUM_0   // 重置按钮 (Boot按钮)
#define OPEN_BUTTON_GPIO  GPIO_NUM_25  // 开门按钮（输入）
#define STOP_BUTTON_GPIO  GPIO_NUM_26  // 停止按钮（输入）
#define CLOSE_BUTTON_GPIO GPIO_NUM_27  // 关门按钮（输入）
#define CLOSE_SENSOR_GPIO GPIO_NUM_12  // 关门传感器（输入）
#define SENSOR_ENABLE_GPIO GPIO_NUM_23 // 传感器使能开关（输入，低电平=不检测）

/* 继电器控制 GPIO 定义 */
#define RELAY_OPEN_GPIO   GPIO_NUM_32  // 开门继电器（输出）
#define RELAY_STOP_GPIO   GPIO_NUM_33  // 停止继电器（输出）
#define RELAY_CLOSE_GPIO  GPIO_NUM_14  // 关门继电器（输出）

/* 传感器配置 */
#define CLOSE_SENSOR_ACTIVE_LEVEL  0   // 关门传感器触发电平（0=低电平触发，1=高电平触发）

/* 继电器触发时间 */
#define RELAY_TRIGGER_TIME_MS  500     // 继电器触发持续时间 0.5秒

/* 门操作超时时间 */
#define DOOR_OPEN_TIMEOUT_MS   10000   // 开门超时时间 10秒（无传感器，依赖超时）
#define DOOR_CLOSE_TIMEOUT_MS  30000   // 关门超时时间 30秒（有传感器，故障检测用）

/* 重置按钮超时设置 */
#define RESET_NETWORK_BUTTON_TIMEOUT        3   // 3秒重置网络
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10  // 10秒恢复出厂

/* 车库门状态 */
typedef enum {
    DOOR_STATE_OPEN = 0,      // 开启
    DOOR_STATE_CLOSED = 1,    // 关闭
    DOOR_STATE_OPENING = 2,   // 正在开启
    DOOR_STATE_CLOSING = 3,   // 正在关闭
    DOOR_STATE_STOPPED = 4    // 已停止
} garage_door_state_t;

/* 目标状态 */
typedef enum {
    DOOR_TARGET_OPEN = 0,     // 目标：开启
    DOOR_TARGET_CLOSED = 1    // 目标：关闭
} garage_door_target_t;

/* 超时类型 */
typedef enum {
    TIMEOUT_FOR_OPEN = 0,     // 开门超时（对应最终状态 OPEN）
    TIMEOUT_FOR_CLOSE = 1     // 关门超时（对应最终状态 CLOSED）
} door_timeout_type_t;

/* 全局变量 */
static hap_char_t *current_door_state_char = NULL;
static hap_char_t *target_door_state_char = NULL;
static garage_door_state_t current_state = DOOR_STATE_CLOSED;
static TimerHandle_t door_timeout_timer = NULL;

/* 函数声明 */
static bool is_sensor_enabled(void);
static bool is_door_closed(void);
static void stop_door_timeout_timer(void);
static void update_door_state(garage_door_state_t new_state);
static void start_door_timeout_timer(door_timeout_type_t timeout_type);

/**
 * @brief 门操作超时回调（故障检测）
 */
static void door_timeout_callback(TimerHandle_t xTimer)
{
    door_timeout_type_t timeout_type = (door_timeout_type_t)(uintptr_t)pvTimerGetTimerID(xTimer);
    
    // 超时后检查传感器状态
    if (timeout_type == TIMEOUT_FOR_CLOSE) {
        // 关门操作超时（30秒）：检查传感器
        ESP_LOGW(TAG, "Door close operation timeout (30s) - checking sensor");
        
        if (is_sensor_enabled()) {
            // 传感器已启用：检查门是否真的关闭了
            if (is_door_closed()) {
                ESP_LOGI(TAG, "Sensor confirms: Door is CLOSED (late detection)");
                current_state = DOOR_STATE_CLOSED;
            } else {
                ESP_LOGE(TAG, "FAULT: Door failed to close in 30 seconds! Setting to STOPPED");
                current_state = DOOR_STATE_STOPPED;
            }
        } else {
            // 传感器未启用：假设已完成（降级模式）
            ESP_LOGW(TAG, "Timeout - assuming door is CLOSED (no sensor)");
            current_state = DOOR_STATE_CLOSED;
        }
    } else if (timeout_type == TIMEOUT_FOR_OPEN) {
        // 开门操作超时（10秒）：无传感器，直接假设已完成
        ESP_LOGI(TAG, "Door open timeout (10s) - assuming door is OPEN");
        current_state = DOOR_STATE_OPEN;
    }
    
    // 更新到 HomeKit
    if (current_door_state_char) {
        hap_val_t val = {.u = current_state};
        hap_char_update_val(current_door_state_char, &val);
    }
}

/**
 * @brief 启动门操作超时定时器
 */
static void start_door_timeout_timer(door_timeout_type_t timeout_type)
{
    // 根据超时类型选择超时时间
    uint32_t timeout_ms;
    if (timeout_type == TIMEOUT_FOR_CLOSE) {
        timeout_ms = DOOR_CLOSE_TIMEOUT_MS;  // 关门：30秒（故障检测）
    } else {
        timeout_ms = DOOR_OPEN_TIMEOUT_MS;   // 开门：10秒（依赖超时）
    }
    
    if (door_timeout_timer == NULL) {
        door_timeout_timer = xTimerCreate(
            "DoorTimeout",                      // 定时器名称
            pdMS_TO_TICKS(timeout_ms),          // 超时时间（动态）
            pdFALSE,                             // 单次触发
            (void *)(uintptr_t)timeout_type,    // 定时器ID存储超时类型
            door_timeout_callback                // 回调函数
        );
    } else {
        // 如果定时器已存在，更新其周期和ID
        xTimerStop(door_timeout_timer, 0);
        xTimerChangePeriod(door_timeout_timer, pdMS_TO_TICKS(timeout_ms), 0);
        vTimerSetTimerID(door_timeout_timer, (void *)(uintptr_t)timeout_type);
    }
    
    if (door_timeout_timer != NULL) {
        xTimerStart(door_timeout_timer, 0);
        ESP_LOGI(TAG, "Started %d second timer for %s operation", 
                 timeout_ms / 1000,
                 (timeout_type == TIMEOUT_FOR_CLOSE) ? "CLOSE (fault detection)" : "OPEN");
    }
}

/**
 * @brief 停止门操作超时定时器
 */
static void stop_door_timeout_timer(void)
{
    if (door_timeout_timer != NULL && xTimerIsTimerActive(door_timeout_timer)) {
        xTimerStop(door_timeout_timer, 0);
        ESP_LOGI(TAG, "Stopped fault detection timer");
    }
}

/**
 * @brief 网络重置回调
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}

/**
 * @brief 恢复出厂设置回调
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * @brief 重置按钮初始化
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/**
 * @brief 检查传感器使能状态
 * @return true 传感器检测已启用, false 传感器检测已禁用
 */
static bool is_sensor_enabled(void)
{
    int enable_level = gpio_get_level(SENSOR_ENABLE_GPIO);
    return (enable_level == 1);  // 高电平=启用，低电平=禁用
}

/**
 * @brief 检查关门传感器状态
 * @return true 门已关闭, false 门未关闭
 */
static bool is_door_closed(void)
{
    // 如果传感器检测被禁用，返回 false（假设门未关闭）
    if (!is_sensor_enabled()) {
        return false;
    }
    
    int sensor_level = gpio_get_level(CLOSE_SENSOR_GPIO);
    return (sensor_level == CLOSE_SENSOR_ACTIVE_LEVEL);
}

/**
 * @brief 初始化传感器使能 GPIO
 */
static void sensor_enable_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // 启用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << SENSOR_ENABLE_GPIO)
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Sensor enable switch initialized on GPIO %d (low=disabled, high=enabled)", 
             SENSOR_ENABLE_GPIO);
    
    // 打印初始状态
    if (is_sensor_enabled()) {
        ESP_LOGI(TAG, "Sensor detection is ENABLED");
    } else {
        ESP_LOGI(TAG, "Sensor detection is DISABLED");
    }
}

/**
 * @brief 初始化关门传感器 GPIO
 */
static void close_sensor_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // 启用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << CLOSE_SENSOR_GPIO)
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Close sensor initialized on GPIO %d (active level: %d)", 
             CLOSE_SENSOR_GPIO, CLOSE_SENSOR_ACTIVE_LEVEL);
    
    // 打印初始状态
    if (is_door_closed()) {
        ESP_LOGI(TAG, "Initial state: Door is CLOSED");
    } else {
        ESP_LOGI(TAG, "Initial state: Door is OPEN");
    }
}

/**
 * @brief 初始化继电器 GPIO（输出模式）
 */
static void relay_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << RELAY_OPEN_GPIO) | 
                        (1ULL << RELAY_STOP_GPIO) | 
                        (1ULL << RELAY_CLOSE_GPIO)
    };
    gpio_config(&io_conf);
    
    // 初始化所有继电器为高电平（未触发 - 反向逻辑）
    gpio_set_level(RELAY_OPEN_GPIO, 1);
    gpio_set_level(RELAY_STOP_GPIO, 1);
    gpio_set_level(RELAY_CLOSE_GPIO, 1);
    
    ESP_LOGI(TAG, "Relay GPIOs initialized (inverted logic): OPEN=%d, STOP=%d, CLOSE=%d", 
             RELAY_OPEN_GPIO, RELAY_STOP_GPIO, RELAY_CLOSE_GPIO);
}

/**
 * @brief 触发继电器（低电平触发 - 反向逻辑）
 */
static void trigger_relay(gpio_num_t relay_gpio, const char* relay_name)
{
    ESP_LOGI(TAG, "Triggering %s relay (GPIO %d) for %d ms", 
             relay_name, relay_gpio, RELAY_TRIGGER_TIME_MS);
    
    gpio_set_level(relay_gpio, 0);  // 激活继电器（低电平）
    vTaskDelay(pdMS_TO_TICKS(RELAY_TRIGGER_TIME_MS));  // 保持 0.5 秒
    gpio_set_level(relay_gpio, 1);  // 关闭继电器（高电平）
    
    ESP_LOGI(TAG, "%s relay released", relay_name);
}

/**
 * @brief 更新车库门状态到 HomeKit
 */
static void update_door_state(garage_door_state_t new_state)
{
    current_state = new_state;
    if (current_door_state_char) {
        hap_val_t val = {.u = new_state};
        hap_char_update_val(current_door_state_char, &val);
        ESP_LOGI(TAG, "Door state updated to: %d", new_state);
    }
}

/**
 * @brief 开门按钮回调
 */
static void open_button_handler(void* arg)
{
    ESP_LOGI(TAG, "Open button pressed");
    
    // 设置目标状态为开启
    if (target_door_state_char) {
        hap_val_t target_val = {.u = DOOR_TARGET_OPEN};
        hap_char_update_val(target_door_state_char, &target_val);
    }
    
    // 触发开门继电器
    if (current_state != DOOR_STATE_OPEN && current_state != DOOR_STATE_OPENING) {
        update_door_state(DOOR_STATE_OPENING);
        trigger_relay(RELAY_OPEN_GPIO, "OPEN");
        
        // 启动开门超时定时器
        start_door_timeout_timer(TIMEOUT_FOR_OPEN);
    }
}

/**
 * @brief 停止按钮回调
 */
static void stop_button_handler(void* arg)
{
    ESP_LOGI(TAG, "Stop button pressed");
    
    // 如果正在移动，则停止
    if (current_state == DOOR_STATE_OPENING || current_state == DOOR_STATE_CLOSING) {
        update_door_state(DOOR_STATE_STOPPED);
        trigger_relay(RELAY_STOP_GPIO, "STOP");
        
        // 停止操作时，取消超时定时器
        stop_door_timeout_timer();
    }
}

/**
 * @brief 关门按钮回调
 */
static void close_button_handler(void* arg)
{
    ESP_LOGI(TAG, "Close button pressed");
    
    // 设置目标状态为关闭
    if (target_door_state_char) {
        hap_val_t target_val = {.u = DOOR_TARGET_CLOSED};
        hap_char_update_val(target_door_state_char, &target_val);
    }
    
    // 触发关门继电器
    if (current_state != DOOR_STATE_CLOSED && current_state != DOOR_STATE_CLOSING) {
        update_door_state(DOOR_STATE_CLOSING);
        trigger_relay(RELAY_CLOSE_GPIO, "CLOSE");
        
        // 启动关门故障检测定时器
        start_door_timeout_timer(TIMEOUT_FOR_CLOSE);
    }
}

/**
 * @brief 初始化控制按钮
 */
static void control_buttons_init(void)
{
    // 开门按钮
    button_handle_t open_btn = iot_button_create(OPEN_BUTTON_GPIO, BUTTON_ACTIVE_LOW);
    iot_button_set_evt_cb(open_btn, BUTTON_CB_RELEASE, open_button_handler, NULL);
    
    // 停止按钮
    button_handle_t stop_btn = iot_button_create(STOP_BUTTON_GPIO, BUTTON_ACTIVE_LOW);
    iot_button_set_evt_cb(stop_btn, BUTTON_CB_RELEASE, stop_button_handler, NULL);
    
    // 关门按钮
    button_handle_t close_btn = iot_button_create(CLOSE_BUTTON_GPIO, BUTTON_ACTIVE_LOW);
    iot_button_set_evt_cb(close_btn, BUTTON_CB_RELEASE, close_button_handler, NULL);
    
    ESP_LOGI(TAG, "Control buttons initialized (OPEN: GPIO%d, STOP: GPIO%d, CLOSE: GPIO%d)", 
             OPEN_BUTTON_GPIO, STOP_BUTTON_GPIO, CLOSE_BUTTON_GPIO);
}

/**
 * @brief Identify 回调
 */
static int garage_door_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/**
 * @brief 车库门写入回调 (从 HomeKit 控制)
 */
static int garage_door_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        
        // 检查是否是目标门状态特征
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
            ESP_LOGI(TAG, "Received Write. Target Door State: %u", write->val.u);
            
            hap_char_update_val(write->hc, &(write->val));
            
            // 根据目标状态执行相应操作
            if (write->val.u == DOOR_TARGET_OPEN) {
                // 开门
                if (current_state != DOOR_STATE_OPEN && current_state != DOOR_STATE_OPENING) {
                    update_door_state(DOOR_STATE_OPENING);
                    trigger_relay(RELAY_OPEN_GPIO, "OPEN");
                    // 启动开门超时定时器
                    start_door_timeout_timer(TIMEOUT_FOR_OPEN);
                }
            } else if (write->val.u == DOOR_TARGET_CLOSED) {
                // 关门
                if (current_state != DOOR_STATE_CLOSED && current_state != DOOR_STATE_CLOSING) {
                    update_door_state(DOOR_STATE_CLOSING);
                    trigger_relay(RELAY_CLOSE_GPIO, "CLOSE");
                    // 启动关门故障检测定时器
                    start_door_timeout_timer(TIMEOUT_FOR_CLOSE);
                }
            }
            
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    
    return ret;
}

/**
 * @brief 主线程
 */
static void garage_door_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* 初始化 HAP 核心 */
    hap_init(HAP_TRANSPORT_WIFI);

    /* 配置附件参数 */
    hap_acc_cfg_t cfg = {
        .name = "Esp-Garage-Door",
        .manufacturer = "Espressif",
        .model = "EspGarageDoor01",
        .serial_num = "001122334455",
        .fw_rev = "1.0.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = garage_door_identify,
        .cid = HAP_CID_GARAGE_DOOR_OPENER,
    };
    
    /* 创建附件对象 */
    accessory = hap_acc_create(&cfg);

    /* 添加产品数据 */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* 添加 Wi-Fi Transport 服务 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* 创建车库门开启器服务 */
    service = hap_serv_garage_door_opener_create(DOOR_STATE_CLOSED, DOOR_TARGET_CLOSED, false);
    
    /* 添加名称特征 */
    hap_serv_add_char(service, hap_char_name_create("My Garage Door"));

    /* 获取当前门状态和目标门状态特征的指针 */
    current_door_state_char = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_DOOR_STATE);
    target_door_state_char = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_TARGET_DOOR_STATE);

    /* 设置写入回调 */
    hap_serv_set_write_cb(service, garage_door_write);

    /* 将服务添加到附件 */
    hap_acc_add_serv(accessory, service);

    /* 将附件添加到 HomeKit 数据库 */
    hap_add_accessory(accessory);

    /* 初始化继电器 GPIO */
    relay_gpio_init();

    /* 初始化传感器使能开关 */
    sensor_enable_init();

    /* 初始化关门传感器 */
    close_sensor_init();

    /* 初始化重置按钮 */
    reset_key_init(RESET_GPIO);
    
    /* 初始化控制按钮 */
    control_buttons_init();

#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* 设置配对码 (仅用于测试) */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* 启用 MFi 认证 (仅适用于 MFi SDK 变体) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_NONE);

    /* 初始化 Wi-Fi */
    app_wifi_init();

    /* 启动 HAP 核心 */
    hap_start();
    
    /* 启动 Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    /* 主循环 - 定期检查传感器状态 */
    bool last_sensor_state = is_door_closed();
    bool last_enable_state = is_sensor_enabled();
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));  // 每500ms检查一次
        
        // 检查传感器使能状态变化
        bool current_enable_state = is_sensor_enabled();
        if (current_enable_state != last_enable_state) {
            if (current_enable_state) {
                ESP_LOGI(TAG, "Sensor detection ENABLED");
            } else {
                ESP_LOGI(TAG, "Sensor detection DISABLED");
            }
            last_enable_state = current_enable_state;
        }
        
        // 只有在传感器检测启用时才处理传感器状态
        if (current_enable_state) {
            bool current_sensor_state = is_door_closed();
            
            // 检测传感器状态变化
            if (current_sensor_state != last_sensor_state) {
                if (current_sensor_state) {
                    // 门已关闭
                    ESP_LOGI(TAG, "Sensor detected: Door CLOSED");
                    
                    // 如果当前状态是正在关闭，停止故障检测定时器
                    if (current_state == DOOR_STATE_CLOSING) {
                        stop_door_timeout_timer();  // 传感器确认，停止故障检测定时器
                    }
                    
                    // 无论当前状态如何，都更新为已关闭（支持手动关门）
                    update_door_state(DOOR_STATE_CLOSED);
                    
                    // 同时更新目标状态
                    if (target_door_state_char) {
                        hap_val_t target_val = {.u = DOOR_TARGET_CLOSED};
                        hap_char_update_val(target_door_state_char, &target_val);
                    }
                } else {
                    // 门已打开
                    ESP_LOGI(TAG, "Sensor detected: Door OPENED");
                    
                    // 如果当前状态是正在开启，停止超时定时器
                    if (current_state == DOOR_STATE_OPENING) {
                        stop_door_timeout_timer();  // 传感器确认，停止超时定时器
                    }
                    
                    // 无论当前状态如何，都更新为已打开（支持手动开门）
                    update_door_state(DOOR_STATE_OPEN);
                    
                    // 同时更新目标状态
                    if (target_door_state_char) {
                        hap_val_t target_val = {.u = DOOR_TARGET_OPEN};
                        hap_char_update_val(target_door_state_char, &target_val);
                    }
                }
                
                last_sensor_state = current_sensor_state;
            }
        }
    }
}

void app_main()
{
    /* 创建应用线程 */
    xTaskCreate(garage_door_thread_entry, GARAGE_DOOR_TASK_NAME, GARAGE_DOOR_TASK_STACKSIZE,
                NULL, GARAGE_DOOR_TASK_PRIORITY, NULL);
}
