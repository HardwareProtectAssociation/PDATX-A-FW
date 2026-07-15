// 宿主机自测: gcc -I../main test_power_logic.c ../main/power_logic.c -o t && ./t
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "power_logic.h"

static int feq(float a, float b) { return fabsf(a - b) < 0.01f; }

int main(void)
{
    // 换算：28V 输入 -> 分压点 3.0V
    assert(feq(pl_adc_to_input_voltage(3.0f), 28.0f));
    // 电流：零点 0.33V -> 0A；0.33+2.64V -> 10A；零点以下钳到 0
    assert(feq(pl_adc_to_output_current(0.33f, PL_ACS725_ZERO_V), 0.0f));
    assert(feq(pl_adc_to_output_current(2.97f, PL_ACS725_ZERO_V), 10.0f));
    assert(feq(pl_adc_to_output_current(0.10f, PL_ACS725_ZERO_V), 0.0f));
    // 校准零点偏移
    assert(feq(pl_adc_to_output_current(0.614f, 0.35f), 1.0f));

    // 电流分档边界
    assert(pl_current_tier(0.5f) == 1);
    assert(pl_current_tier(1.0f) == 2);
    assert(pl_current_tier(6.0f) == 2);
    assert(pl_current_tier(6.1f) == 3);

    pl_sm_t sm;
    pl_out_t o;

    // 场景1: 28V 请求成功
    pl_sm_init(&sm, 0);
    o = pl_sm_step(&sm, 0, 5.0f);
    assert(o.request == PL_REQ_28V && !o.dcdc_en);
    o = pl_sm_step(&sm, 500, 27.5f);  // 稳定窗口内即达标
    assert(sm.state == PL_STATE_DCDC_ON && o.dcdc_en);

    // 场景2: 28V 失败 -> 20V 成功
    pl_sm_init(&sm, 0);
    pl_sm_step(&sm, 0, 5.0f);
    o = pl_sm_step(&sm, PL_SETTLE_MS, 20.0f);  // 20V 不到 26V，超时降级
    assert(o.request == PL_REQ_20V);
    o = pl_sm_step(&sm, PL_SETTLE_MS + 100, 20.0f);
    assert(sm.state == PL_STATE_DCDC_ON && o.dcdc_en);

    // 场景3: 两档均失败 -> 3s 重试
    pl_sm_init(&sm, 0);
    pl_sm_step(&sm, 0, 5.0f);
    pl_sm_step(&sm, PL_SETTLE_MS, 5.0f);
    o = pl_sm_step(&sm, 2 * PL_SETTLE_MS, 5.0f);
    assert(sm.state == PL_STATE_RETRY_WAIT && !o.dcdc_en);
    o = pl_sm_step(&sm, 2 * PL_SETTLE_MS + PL_RETRY_MS - 1, 5.0f);
    assert(o.request == PL_REQ_NONE);  // 未到重试时间
    o = pl_sm_step(&sm, 2 * PL_SETTLE_MS + PL_RETRY_MS, 5.0f);
    assert(o.request == PL_REQ_28V);   // 重新从 28V 开始

    // 场景4: 运行中欠压 -> 锁存故障
    pl_sm_init(&sm, 0);
    pl_sm_step(&sm, 0, 5.0f);
    pl_sm_step(&sm, 100, 28.0f);
    assert(sm.state == PL_STATE_DCDC_ON);
    o = pl_sm_step(&sm, 200, 15.9f);
    assert(sm.state == PL_STATE_FAULT && !o.dcdc_en);
    o = pl_sm_step(&sm, 100000, 28.0f);  // 电压恢复也不解除
    assert(sm.state == PL_STATE_FAULT && !o.dcdc_en && o.request == PL_REQ_NONE);

    // 边界: 19.0V 不开启(要求大于19V)，16.0V 不故障(要求低于16V)
    pl_sm_init(&sm, 0);
    pl_sm_step(&sm, 0, 5.0f);
    pl_sm_step(&sm, PL_SETTLE_MS, 19.0f);
    o = pl_sm_step(&sm, PL_SETTLE_MS + 10, 19.0f);
    assert(sm.state != PL_STATE_DCDC_ON);
    pl_sm_init(&sm, 0);
    pl_sm_step(&sm, 0, 5.0f);
    pl_sm_step(&sm, 100, 28.0f);
    o = pl_sm_step(&sm, 200, 16.0f);
    assert(sm.state == PL_STATE_DCDC_ON && o.dcdc_en);

    printf("power_logic: all tests passed\n");
    return 0;
}
