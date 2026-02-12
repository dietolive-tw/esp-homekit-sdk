# HomeKit 车库门开关示例

这是一个基于 ESP-IDF 和 ESP HomeKit SDK 的车库门开关示例程序。

## 功能特点

- 支持 HomeKit 协议，可通过 iPhone/iPad 的"家庭" App 控制
- 物理按钮控制：开门、停止、关门
- 双向同步：物理按钮和 HomeKit App 状态实时同步
- 支持车库门状态：开启、关闭、正在开启、正在关闭、已停止

## 硬件要求

### 支持的芯片
- ESP32
- ESP32-C6 (需使用 sdkconfig.defaults.esp32c6)

### GPIO 分配

| 功能 | GPIO | 说明 |
|------|------|------|
| 重置按钮 | GPIO 0 | Boot 按钮，按住3秒重置Wi-Fi，10秒恢复出厂设置 |
| 开门按钮 | GPIO 25 | 触发开门操作 |
| 停止按钮 | GPIO 26 | 停止门的运动 |
| 关门按钮 | GPIO 27 | 触发关门操作 |

**注意**：您可以根据实际硬件修改 `main/app_main.c` 中的 GPIO 定义。

## 构建和烧录

### 1. 设置 ESP-IDF 环境

```bash
. /opt/esp-idf/export.sh
```

### 2. 配置目标芯片

ESP32:
```bash
idf.py set-target esp32
```

ESP32-C6:
```bash
idf.py set-target esp32c6
```

### 3. 构建项目

```bash
idf.py build
```

### 4. 烧录到设备

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## HomeKit 配对

### 默认配对信息
- **设置代码**: 111-22-333
- **设置 ID**: ES32

### 配对步骤
1. 打开 iPhone/iPad 上的"家庭" App
2. 点击右上角 "+" 添加配件
3. 选择"没有代码或无法扫描"
4. 在附近的配件列表中找到 "Esp-Garage-Door"
5. 输入设置代码 `111-22-333`
6. 完成配对

## 硬件集成指南

当前代码提供了基本框架，实际使用时需要添加硬件控制代码：

### 需要实现的功能

1. **电机控制** - 在 `app_main.c` 中标记为 `TODO` 的位置添加：
   - 启动电机正转（开门）
   - 启动电机反转（关门）
   - 停止电机

2. **限位开关**（推荐）：
   - 检测门完全开启
   - 检测门完全关闭
   - 防止过度运行

3. **安全传感器**（推荐）：
   - 障碍物检测
   - 自动停止和反向

### 示例硬件连接

```
ESP32        继电器/电机驱动
GPIO 32  --> 开门继电器/IN1
GPIO 33  --> 关门继电器/IN2
GPIO 34  <-- 开到位限位开关
GPIO 35  <-- 关到位限位开关
```

## 故障排除

### 编译错误

如果遇到 `nonnull-compare` 编译错误，项目的 `CMakeLists.txt` 已经添加了相应的编译选项来禁用此警告。

### Wi-Fi 连接问题

1. 按住 Boot 按钮 3 秒后释放，重置 Wi-Fi 配置
2. 重新通过 HomeKit App 配对

### 恢复出厂设置

按住 Boot 按钮 10 秒以上，设备将恢复出厂设置。

## 开发说明

### 状态机

车库门使用以下状态机：

```
CLOSED --> OPENING --> OPEN
  ^          |          |
  |          v          v
  +------- STOPPED <----+
  |                     |
  +------- CLOSING <----+
```

### 修改 GPIO

编辑 `main/app_main.c`：

```c
#define OPEN_BUTTON_GPIO  GPIO_NUM_25  // 改为您的 GPIO
#define STOP_BUTTON_GPIO  GPIO_NUM_26
#define CLOSE_BUTTON_GPIO GPIO_NUM_27
```

## 许可证

本项目遵循 ESPRESSIF MIT 许可证。

## 参考资料

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/)
- [ESP HomeKit SDK](https://github.com/espressif/esp-homekit-sdk)
- [HomeKit 配件协议规范](https://developer.apple.com/homekit/)
