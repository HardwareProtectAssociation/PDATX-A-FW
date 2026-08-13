# AGENTS.md

本文档面向 AI 编码代理与新加入的开发者，描述项目的结构、设计约定与硬性约束。

**同步要求：任何对项目的修改（代码、引脚、协议、阈值、构建方式等）必须同步更新本文档与 README.md，保持三者完全一致。**

## 项目概览

ESP32-S3 电源板固件：CH224A 通过 USB-PD 请求 28V/20V 输入，DCDC 输出 12V；ADC 监测输入电压与输出电流；LED 指示电流档位与故障；WiFi + UDP 上报采样数据。

## 目录结构

```
PDATX-A-FW/
├── docker-compose.yml            # ESP-IDF v5.5.3 编译环境（xianii/esp-idf-slim:v5.5.3）
└── pdatx_a/                      # ESP-IDF 工程根目录（容器内挂载为 /workspace）
    ├── CMakeLists.txt
    ├── sdkconfig.defaults        # 仅指定 target esp32s3
    ├── main/
    │   ├── pdatx_a.c             # 硬件层与任务：I2C/ADC/GPIO/WiFi/UDP，全部业务入口
    │   ├── power_logic.c/.h      # 纯逻辑：换算公式 + 电源状态机，无 ESP-IDF 依赖
    │   └── CMakeLists.txt
    └── test/
        └── test_power_logic.c    # 宿主机 gcc assert 自测
```

## 关键设计约定

- **power_logic 必须保持无 IDF 依赖**，以便宿主机测试。修改逻辑后必须运行并按需更新 `test/test_power_logic.c`：

  ```bash
  cd pdatx_a/test && gcc -Wall -Wextra -I../main test_power_logic.c ../main/power_logic.c -o /tmp/t_pl && /tmp/t_pl
  ```

- **PD 请求结果以 ADC 实测为准**，不依赖 CH224A I2C 状态回读；I2C 写失败不致命，状态机会自然走降级/重试路径。
- 电源状态机与 LED 任务先于 WiFi 启动，**供电逻辑不得依赖网络**。
- FAULT（运行中输入 < 16V 且持续超过 100ms）为锁存态，仅重新上电解除，代码中不得添加自动恢复。
- LED 低电平有效；DCDC enable 高电平有效，上电默认关闭。
- ACS725 零点在上电、DCDC 开启前空载校准；名义换算系数（分压比 9.33、灵敏度 264mV/A）待实机校准。

## 硬件与参数速查

- 引脚：I2C SDA=GPIO8 / SCL=GPIO9；DCDC_EN=GPIO10；电流=GPIO2(ADC1_CH1)；电压=GPIO3(ADC1_CH2)；白灯=GPIO42/41/40；红灯=GPIO39。
- CH224A：7 位地址 0x22/0x23（上电探测），电压控制寄存器 0x0A（4=20V，5=28V）。
- 阈值：28V 成功 ≥26V；DCDC 开启 >19V；欠压故障 <16V 且持续 100ms；请求稳定等待 1s；重试间隔 3s。
- 电流分档：<1A 一盏白灯；1–6A 两盏；>6A 三盏（1Hz 闪烁）。
- WiFi：SSID `PD-ATX`，密码 `octppus0`（`#define` 于 pdatx_a.c）。
- UDP：广播发现 19000，收保活 19001，保活超时 5s；采样 10ms，8 帧一批 JSON 上报。

## 构建

```bash
docker compose run --rm esp-dev bash -lc 'idf.py build'
docker compose run --rm esp-dev bash -lc 'idf.py -p /dev/ttyACM0 flash monitor'
```

首次构建会自动执行组件配置；`pdatx_a/build/`、`pdatx_a/sdkconfig` 为生成物，不入库。
