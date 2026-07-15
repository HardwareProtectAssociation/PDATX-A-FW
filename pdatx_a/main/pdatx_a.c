#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <float.h>
#include <errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "cJSON.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "power_logic.h"

#define WIFI_SSID                 "PD-ATX"
#define WIFI_PASS                 "octppus0"
#define DEVICE_NAME               "PDATX_A"

#define I2C_SDA_GPIO              8
#define I2C_SCL_GPIO              9
#define DCDC_EN_GPIO              10
#define LED1_GPIO                 42   // 白，低有效
#define LED2_GPIO                 41
#define LED3_GPIO                 40
#define LED4_GPIO                 39   // 红，低有效

#define ADC_UNIT_USED             ADC_UNIT_1
#define ADC_CH_CURRENT            ADC_CHANNEL_1   // GPIO2, ACS725
#define ADC_CH_VOLTAGE            ADC_CHANNEL_2   // GPIO3, 1M/120k 分压
#define ADC_ATTEN_USED            ADC_ATTEN_DB_12 // 28V输入时分压点约3.0V，接近量程上限

#define CH224A_REG_VOLTAGE        0x0A

#define UDP_TX_PORT               19000
#define UDP_RX_PORT               19001
#define HEARTBEAT_BROADCAST_MS    1000
#define SAMPLE_PERIOD_MS          10
#define CLIENT_TIMEOUT_MS         5000
#define LED_BLINK_PERIOD_MS       1000
#define LED_BLINK_ON_MS           500

#define BATCH_SIZE                8
#define CHANNEL_COUNT             2
#define CALIB_SAMPLES             100

#define WIFI_CONNECTED_BIT        BIT0

static const char *TAG = "PDATX_A";

typedef enum {
    CH_INPUT_VOLTAGE = 0,
    CH_OUTPUT_CURRENT,
} channel_id_t;

typedef struct {
    const char *channel;
    int64_t ts_ms;
    float cur;
    float max8;
    float min8;
} sample_record_t;

typedef struct {
    sample_record_t rec[CHANNEL_COUNT];
} sample_frame_t;

typedef struct {
    bool connected;
    struct sockaddr_in addr;
    int64_t last_keepalive_ms;
} client_state_t;

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;
static i2c_master_dev_handle_t s_ch224a_dev;
static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t s_client_mutex;
static client_state_t s_client_state = {0};
static int s_udp_sock = -1;

// LED 任务读取的共享状态（sampling 任务写入）
static volatile pl_state_t s_pl_state = PL_STATE_RETRY_WAIT;
static volatile bool s_req_failed = false;   // 20V也请求失败过，红灯闪烁直到成功
static volatile float s_iout = 0.0f;

// 发送缓存放静态区，避免任务栈溢出（同参考项目）
static char s_tx_buf[2048];
static sample_frame_t s_batch[BATCH_SIZE];
static float s_hist[CHANNEL_COUNT][BATCH_SIZE];

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

// ---------- CH224A ----------

static void init_ch224a_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // 7位地址为 0x22 或 0x23，探测确定
    uint16_t addr = 0x22;
    if (i2c_master_probe(bus, 0x22, 100) != ESP_OK) {
        if (i2c_master_probe(bus, 0x23, 100) == ESP_OK) {
            addr = 0x23;
        } else {
            ESP_LOGW(TAG, "CH224A not found at 0x22/0x23, defaulting to 0x22");
        }
    }
    ESP_LOGI(TAG, "CH224A I2C addr: 0x%02X", addr);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_ch224a_dev));
}

// 电压控制寄存器: 4=20V, 5=28V (power_logic.h 的 PL_REQ_* 与之对应)
static void ch224a_request(int code)
{
    uint8_t buf[2] = { CH224A_REG_VOLTAGE, (uint8_t)code };
    esp_err_t err = i2c_master_transmit(s_ch224a_dev, buf, sizeof(buf), 100);
    if (err != ESP_OK) {
        // 写失败不致命：状态机靠ADC实测判定，会自然走降级/重试路径
        ESP_LOGW(TAG, "CH224A write failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CH224A request code=%d (%s)", code, code == PL_REQ_28V ? "28V" : "20V");
    }
}

// ---------- GPIO / ADC ----------

static void init_gpio(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << DCDC_EN_GPIO) | (1ULL << LED1_GPIO) |
                        (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO) | (1ULL << LED4_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(DCDC_EN_GPIO, 0);  // DCDC 默认关闭
    gpio_set_level(LED1_GPIO, 1);     // LED 低有效，默认全灭
    gpio_set_level(LED2_GPIO, 1);
    gpio_set_level(LED3_GPIO, 1);
    gpio_set_level(LED4_GPIO, 1);
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_USED,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CH_VOLTAGE, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CH_CURRENT, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_USED,
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) != ESP_OK) {
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw scaling");
    }
}

// 返回引脚电压(V)
static float adc_read_volts(adc_channel_t ch)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, ch, &raw) != ESP_OK) {
        return 0.0f;
    }
    if (s_adc_cali) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) == ESP_OK) {
            return mv / 1000.0f;
        }
    }
    return (raw / 4095.0f) * 3.3f;  // ponytail: 无eFuse校准时的粗略回退
}

// ---------- LED ----------

static void task_led(void *arg)
{
    (void)arg;
    while (1) {
        int64_t now = now_ms();
        bool blink_on = ((now % LED_BLINK_PERIOD_MS) < LED_BLINK_ON_MS);
        pl_state_t st = s_pl_state;

        bool w1 = false, w2 = false, w3 = false, red = false;

        if (st == PL_STATE_FAULT) {
            red = true;  // 欠压锁存：红灯常亮，白灯全灭
        } else if (st == PL_STATE_DCDC_ON) {
            int tier = pl_current_tier(s_iout);
            w1 = blink_on;
            w2 = blink_on && (tier >= 2);
            w3 = blink_on && (tier >= 3);
        } else if (s_req_failed) {
            red = blink_on;  // 请求失败重试中
        }

        gpio_set_level(LED1_GPIO, w1 ? 0 : 1);
        gpio_set_level(LED2_GPIO, w2 ? 0 : 1);
        gpio_set_level(LED3_GPIO, w3 ? 0 : 1);
        gpio_set_level(LED4_GPIO, red ? 0 : 1);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------- WiFi / UDP ----------

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void init_wifi_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = "",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void init_udp_socket(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ESP_ERROR_CHECK(s_udp_sock < 0 ? ESP_FAIL : ESP_OK);

    int broadcast_enable = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_RX_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    ESP_ERROR_CHECK(bind(s_udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0 ? ESP_FAIL : ESP_OK);

    struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(s_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

static bool client_is_connected(void)
{
    bool connected;
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    connected = s_client_state.connected &&
                ((now_ms() - s_client_state.last_keepalive_ms) <= CLIENT_TIMEOUT_MS);
    if (s_client_state.connected && !connected) {
        s_client_state.connected = false;
    }
    xSemaphoreGive(s_client_mutex);
    return connected;
}

static bool is_valid_heartbeat_json(const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    if (root == NULL) {
        return false;
    }
    cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    bool valid = cJSON_IsString(type) && (type->valuestring != NULL) &&
                 (strcmp(type->valuestring, "heartbeat") == 0);
    cJSON_Delete(root);
    return valid;
}

static void task_net_rx(void *arg)
{
    (void)arg;
    char rx_buf[128];

    while (1) {
        struct sockaddr_in src_addr = {0};
        socklen_t addr_len = sizeof(src_addr);
        int len = recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf) - 1, 0,
                           (struct sockaddr *)&src_addr, &addr_len);
        if (len > 0) {
            rx_buf[len] = '\0';
            if (!is_valid_heartbeat_json(rx_buf)) {
                continue;
            }
            xSemaphoreTake(s_client_mutex, portMAX_DELAY);
            s_client_state.addr = src_addr;
            s_client_state.last_keepalive_ms = now_ms();
            if (!s_client_state.connected) {
                char ip_str[16] = {0};
                inet_ntoa_r(src_addr.sin_addr, ip_str, sizeof(ip_str));
                ESP_LOGI(TAG, "Client connected: %s:%u", ip_str, ntohs(src_addr.sin_port));
            }
            s_client_state.connected = true;
            xSemaphoreGive(s_client_mutex);
        } else if (len < 0) {
            int err = errno;
            if (err != EAGAIN && err != EWOULDBLOCK) {
                ESP_LOGW(TAG, "recvfrom failed, errno=%d", err);
            }
        }
    }
}

static void task_heartbeat(void *arg)
{
    (void)arg;
    const char *msg = "{\"type\":\"esp_heartbeat\",\"dev\":\"" DEVICE_NAME "\"}";

    struct sockaddr_in bcast_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_TX_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };

    while (1) {
        if (!client_is_connected()) {
            int ret = sendto(s_udp_sock, msg, strlen(msg), 0,
                             (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
            if (ret < 0) {
                ESP_LOGW(TAG, "heartbeat send failed, errno=%d", errno);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_BROADCAST_MS));
    }
}

static const char *channel_name(channel_id_t ch)
{
    return (ch == CH_INPUT_VOLTAGE) ? "INPUT_VOLTAGE" : "OUTPUT_CURRENT";
}

static void batch_send(uint32_t *seq)
{
    int off = snprintf(s_tx_buf, sizeof(s_tx_buf),
                       "{\"type\":\"data_batch\",\"seq\":%" PRIu32 ",\"records\":[", (*seq)++);
    if (off <= 0 || off >= (int)sizeof(s_tx_buf)) {
        return;
    }
    bool first = true;
    for (int i = 0; i < BATCH_SIZE; i++) {
        for (int ch = 0; ch < CHANNEL_COUNT; ch++) {
            sample_record_t *r = &s_batch[i].rec[ch];
            int n = snprintf(s_tx_buf + off, sizeof(s_tx_buf) - (size_t)off,
                             "%s{\"ch\":\"%s\",\"ts\":%" PRIi64
                             ",\"max8\":%.4f,\"cur\":%.4f,\"min8\":%.4f}",
                             first ? "" : ",", r->channel, r->ts_ms,
                             r->max8, r->cur, r->min8);
            if (n <= 0 || (off + n) >= (int)sizeof(s_tx_buf)) {
                return;
            }
            off += n;
            first = false;
        }
    }
    if ((off + 3) >= (int)sizeof(s_tx_buf)) {
        return;
    }
    s_tx_buf[off++] = ']';
    s_tx_buf[off++] = '}';
    s_tx_buf[off] = '\0';

    struct sockaddr_in dst;
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    dst = s_client_state.addr;
    xSemaphoreGive(s_client_mutex);

    if (sendto(s_udp_sock, s_tx_buf, (size_t)off, 0,
               (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        ESP_LOGW(TAG, "data batch send failed, errno=%d", errno);
    }
}

// ---------- 采样 + 电源状态机 ----------

static void task_sampling(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t seq = 0;
    uint32_t log_div = 0;
    int batch_count = 0;
    int hist_count = 0;
    int hist_write_idx = 0;
    bool cache_cleared = true;

    // 上电时 DCDC 必然关闭、输出电流为0，直接取均值做 ACS725 零点校准
    float zero_v = 0.0f;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        zero_v += adc_read_volts(ADC_CH_CURRENT);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    zero_v /= CALIB_SAMPLES;
    ESP_LOGI(TAG, "ACS725 zero calibrated: %.4fV (nominal %.2fV)", zero_v, PL_ACS725_ZERO_V);
    if (zero_v < 0.1f || zero_v > 0.6f) {
        ESP_LOGW(TAG, "zero point out of expected range, falling back to nominal");
        zero_v = PL_ACS725_ZERO_V;
    }

    pl_sm_t sm;
    pl_sm_init(&sm, now_ms());

    while (1) {
        int64_t ts = now_ms();
        float vin = pl_adc_to_input_voltage(adc_read_volts(ADC_CH_VOLTAGE));
        float iout = pl_adc_to_output_current(adc_read_volts(ADC_CH_CURRENT), zero_v);

        pl_state_t prev = sm.state;
        pl_out_t out = pl_sm_step(&sm, ts, vin);
        if (out.request != PL_REQ_NONE) {
            ch224a_request(out.request);
        }
        gpio_set_level(DCDC_EN_GPIO, out.dcdc_en ? 1 : 0);

        if (sm.state != prev) {
            ESP_LOGI(TAG, "power state %d -> %d (vin=%.2fV)", prev, sm.state, vin);
            if (sm.state == PL_STATE_RETRY_WAIT) {
                s_req_failed = true;   // 20V也失败，红灯闪烁直到请求成功
            } else if (sm.state == PL_STATE_DCDC_ON) {
                s_req_failed = false;
            } else if (sm.state == PL_STATE_FAULT) {
                ESP_LOGE(TAG, "Input undervoltage (%.2fV < %.1fV), DCDC latched OFF", vin, PL_VOFF_TH);
            }
        }
        s_pl_state = sm.state;
        s_iout = iout;

        // ---- UDP 批次上报 ----
        float values[CHANNEL_COUNT] = { vin, iout };

        for (int ch = 0; ch < CHANNEL_COUNT; ch++) {
            s_hist[ch][hist_write_idx] = values[ch];
        }
        hist_write_idx = (hist_write_idx + 1) % BATCH_SIZE;
        if (hist_count < BATCH_SIZE) {
            hist_count++;
        }

        bool connected = (s_udp_sock >= 0) && client_is_connected();
        if (!connected && !cache_cleared) {
            batch_count = 0;
            hist_count = 0;
            hist_write_idx = 0;
            seq = 0;
            cache_cleared = true;
            ESP_LOGI(TAG, "Client disconnected, TX cache cleared");
        }
        if (connected && cache_cleared) {
            cache_cleared = false;
        }

        for (int ch = 0; ch < CHANNEL_COUNT; ch++) {
            float min_v = FLT_MAX, max_v = -FLT_MAX;
            for (int i = 0; i < hist_count; i++) {
                float v = s_hist[ch][i];
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
            }
            sample_record_t *r = &s_batch[batch_count].rec[ch];
            r->channel = channel_name((channel_id_t)ch);
            r->ts_ms = ts;
            r->cur = values[ch];
            r->max8 = (hist_count > 0) ? max_v : 0.0f;
            r->min8 = (hist_count > 0) ? min_v : 0.0f;
        }
        batch_count++;

        if (batch_count >= BATCH_SIZE) {
            if (connected) {
                batch_send(&seq);
            }
            batch_count = 0;
            if (++log_div >= 25) {  // 每2s一条状态日志
                log_div = 0;
                ESP_LOGI(TAG, "vin=%.2fV iout=%.3fA state=%d", vin, iout, sm.state);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Boot: I2C(SDA%d/SCL%d) DCDC_EN(%d) ADC(V:GPIO3 I:GPIO2) LEDs(42/41/40/39)",
             I2C_SDA_GPIO, I2C_SCL_GPIO, DCDC_EN_GPIO);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();
    s_client_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((s_wifi_event_group && s_client_mutex) ? ESP_OK : ESP_FAIL);

    init_gpio();
    init_adc();
    init_ch224a_i2c();

    // 电源逻辑与LED先跑起来，不依赖网络
    xTaskCreate(task_sampling, "sampling", 6144, NULL, 5, NULL);
    xTaskCreate(task_led, "led", 2048, NULL, 3, NULL);

    init_wifi_sta();
    ESP_LOGI(TAG, "Waiting WiFi connection (ssid=%s)...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    init_udp_socket();
    xTaskCreate(task_net_rx, "net_rx", 4096, NULL, 6, NULL);
    xTaskCreate(task_heartbeat, "heartbeat", 3072, NULL, 4, NULL);
}
