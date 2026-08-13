#include "power_logic.h"

#define DIV_RATIO      ((1000.0f + 120.0f) / 120.0f)
#define ACS725_V_PER_A 0.264f

float pl_adc_to_input_voltage(float v_adc)
{
    return v_adc * DIV_RATIO;
}

float pl_adc_to_output_current(float v_adc, float zero_v)
{
    float a = (v_adc - zero_v) / ACS725_V_PER_A;
    return (a < 0.0f) ? 0.0f : a;  // 单向传感器，负值只是零点噪声
}

int pl_current_tier(float amps)
{
    if (amps > 6.0f) return 3;
    if (amps >= 1.0f) return 2;
    return 1;
}

void pl_sm_init(pl_sm_t *sm, int64_t now_ms)
{
    sm->state = PL_STATE_RETRY_WAIT;
    sm->deadline_ms = now_ms;  // 立即发起首次请求
}

pl_out_t pl_sm_step(pl_sm_t *sm, int64_t now_ms, float vin)
{
    pl_out_t out = { .dcdc_en = false, .request = PL_REQ_NONE };

    switch (sm->state) {
    case PL_STATE_RETRY_WAIT:
        if (now_ms >= sm->deadline_ms) {
            out.request = PL_REQ_28V;
            sm->state = PL_STATE_REQ_28V;
            sm->deadline_ms = now_ms + PL_SETTLE_MS;
        }
        break;

    case PL_STATE_REQ_28V:
        if (vin >= PL_V28_OK_TH) {
            sm->state = PL_STATE_DCDC_ON;
            sm->deadline_ms = 0;  // 复用为欠压保持截止时间；0=未计时
            out.dcdc_en = true;
        } else if (now_ms >= sm->deadline_ms) {
            out.request = PL_REQ_20V;
            sm->state = PL_STATE_REQ_20V;
            sm->deadline_ms = now_ms + PL_SETTLE_MS;
        }
        break;

    case PL_STATE_REQ_20V:
        if (vin > PL_VON_TH) {
            sm->state = PL_STATE_DCDC_ON;
            sm->deadline_ms = 0;
            out.dcdc_en = true;
        } else if (now_ms >= sm->deadline_ms) {
            sm->state = PL_STATE_RETRY_WAIT;
            sm->deadline_ms = now_ms + PL_RETRY_MS;
        }
        break;

    case PL_STATE_DCDC_ON:
        if (vin < PL_VOFF_TH) {
            if (sm->deadline_ms == 0) {
                sm->deadline_ms = now_ms + PL_UV_HOLD_MS;
            } else if (now_ms >= sm->deadline_ms) {
                sm->state = PL_STATE_FAULT;  // 锁存，仅重新上电解除
                break;
            }
            out.dcdc_en = true;  // 保持期内仍输出
        } else {
            sm->deadline_ms = 0;
            out.dcdc_en = true;
        }
        break;

    case PL_STATE_FAULT:
    default:
        break;
    }

    return out;
}
