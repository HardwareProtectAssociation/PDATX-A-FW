# PDATX-A 固件

ESP32-S3 电源板固件。电源板通过 CH224A 从 USB Type-C (PD) 获取 28V/5A 输入，经 DCDC 转换输出 12V，并通过 WiFi + UDP 实时上报输入电压与输出电流。

## 硬件连接

| 功能 | 引脚 |
| --- | --- |
| CH224A SDA | GPIO8 |
| CH224A SCL | GPIO9 |
| DCDC enable（高电平有效） | GPIO10 |
| 输出电流检测（ACS725LLCTR-10AU VIOUT） | GPIO2 / ADC1_CH1 |
| 输入电压检测（1M/120k 分压） | GPIO3 / ADC1_CH2 |
| White LED1 | GPIO42 |
| White LED2 | GPIO41 |
| White LED3 | GPIO40 |
| Red LED4 | GPIO39 |

LED 均为低电平有效。

## 运行逻辑

### PD 电压请求与 DCDC 控制

上电后通过 I2C 向 CH224A 请求 28V；等待电压稳定后以 ADC 实测值判定：

- 实测 ≥ 26V：判定 28V 请求成功。
- 否则降级请求 20V，实测 > 19V 则判定成功。
- 两档均失败：红色 LED 闪烁报错，每隔 3s 从 28V 开始重试。

请求成功（实测 > 19V）后开启 DCDC 输出。输出过程中若输入电压低于 16V，则关闭输出、熄灭白色 LED、红色 LED 常亮报错；该故障为锁存态，仅重新上电解除。

### 电流指示

正常输出状态下持续检测输出电流（1Hz 闪烁）：

- < 1A：闪烁 LED1
- 1–6A：闪烁 LED1 + LED2
- \> 6A：闪烁 LED1 + LED2 + LED3

### WiFi 与 UDP 上报

上电后连接 SSID 为 `PD-ATX` 的热点（密码 `octppus0`）。协议：

1. 设备向 UDP 19000 端口广播发现包：`{"type":"esp_heartbeat","dev":"PDATX_A"}`（1Hz）。
2. 主机向设备 19001 端口回 `{"type":"heartbeat"}` 保活，超过 5s 未收到视为断开。
3. 连接期间设备向主机单播采样批次（10ms 采样、8 帧一批）：

```json
{"type":"data_batch","seq":0,"records":[
  {"ch":"INPUT_VOLTAGE","ts":1234,"max8":28.01,"cur":27.99,"min8":27.95},
  {"ch":"OUTPUT_CURRENT","ts":1234,"max8":1.23,"cur":1.20,"min8":1.18}
]}
```

`max8`/`min8` 为最近 8 个采样点的极值。电源逻辑不依赖网络，断网不影响供电。

## 编译与烧录

开发环境为 docker（ESP-IDF v5.5.3），无需本地安装工具链：

```bash
# 编译
docker compose run --rm esp-dev bash -lc 'idf.py build'

# 烧录 + 监视（端口按实际调整）
docker compose run --rm esp-dev bash -lc 'idf.py -p /dev/ttyACM0 flash monitor'
```

## 测试

核心逻辑（换算公式与电源状态机）为无 IDF 依赖的纯函数，可在宿主机直接自测：

```bash
cd pdatx_a/test
gcc -Wall -Wextra -I../main test_power_logic.c ../main/power_logic.c -o /tmp/t_pl && /tmp/t_pl
```

## 校准说明

- ACS725 零点在每次上电、DCDC 开启前空载自动校准（100 点均值，异常时回退名义零点 0.33V）。
- 分压比（9.33）与 ACS725 灵敏度（264mV/A）为名义值，首次上机建议用万用表实测校准。
