// 纯逻辑：ADC 换算 + 电源状态机。无 ESP-IDF 依赖，可在宿主机编译测试。
#ifndef POWER_LOGIC_H
#define POWER_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

// CH224A 电压控制寄存器(0x0A)取值
#define PL_REQ_NONE (-1)
#define PL_REQ_20V  4
#define PL_REQ_28V  5

// 阈值(V)
#define PL_V28_OK_TH   26.0f  // 请求28V后实测达到此值算成功
#define PL_VON_TH      19.0f  // 实测高于此值开启DCDC(20V档)
#define PL_VOFF_TH     16.0f  // 运行中低于此值锁存故障
#define PL_SETTLE_MS   1000   // 请求后等待PD协商稳定的时间
#define PL_RETRY_MS    3000   // 两档均失败后的重试间隔

typedef enum {
    PL_STATE_RETRY_WAIT = 0,  // 等待(重)发起请求；上电初始态(deadline=now立即请求)
    PL_STATE_REQ_28V,         // 已请求28V，等待电压稳定
    PL_STATE_REQ_20V,         // 已请求20V，等待电压稳定
    PL_STATE_DCDC_ON,         // 正常输出
    PL_STATE_FAULT,           // 欠压锁存故障，仅重新上电解除
} pl_state_t;

typedef struct {
    pl_state_t state;
    int64_t deadline_ms;
} pl_sm_t;

typedef struct {
    bool dcdc_en;
    int request;  // 本步需写入CH224A的档位，PL_REQ_NONE表示无
} pl_out_t;

void pl_sm_init(pl_sm_t *sm, int64_t now_ms);
pl_out_t pl_sm_step(pl_sm_t *sm, int64_t now_ms, float vin);

// 名义零点(0.1*Vcc)，实际零点应上电空载校准后传入 pl_adc_to_output_current
#define PL_ACS725_ZERO_V 0.33f

// GPIO3: 1M/120k 分压
float pl_adc_to_input_voltage(float v_adc);
// GPIO2: ACS725LLCTR-10AU, 单向0-10A, 264mV/A, zero_v为实测零点
float pl_adc_to_output_current(float v_adc, float zero_v);

// 白灯档位: <1A -> 1盏, 1~6A -> 2盏, >6A -> 3盏
int pl_current_tier(float amps);

#endif
