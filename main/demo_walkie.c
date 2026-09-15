// main/demo_walkie.c —— ESP-NOW 离线对讲机 + 百度语音转文字。
//
// 线程模型(全部阻塞操作都在工作者任务里,遵守 AGENTS.md 的运行时约束):
//   * walkie task:主工作者。处理 ESP-NOW 接收队列(BEGIN/CHUNK/END/TEXT/PAIR),
//     组装收到的语音消息并回放;每 5 秒发一次 PAIR 心跳。回放在本任务里分块
//     解码 + 写 I2S,与接收处理交替进行。
//   * rec task:按下 OK 时创建。独占音频采集,实时 ADPCM 编码 + 分包 ESP-NOW
//     广播。松手后发 END、存消息,若 WiFi + 百度 API 配置了则调 ASR,成功后
//     广播 TEXT 包并把文字挂到本机消息上。完成后自删。
//   * key():只置 volatile 标志或创建任务,按键回调路径零阻塞。
// 消息 ID 由本机维护(s_store.next_msg_id),接收方直接使用包内 ID。
// ESP-NOW 广播地址 FF:FF:FF:FF:FF:FF,所有同频道设备均可收发。
//
// ============================================================================
// 配置区:构建前填入你的 WiFi 和百度 API 凭证。
//   * WiFi:留空则不连接,ESP-NOW 对讲照常工作,仅跳过语音转文字。
//   * 百度:API Key/Secret 留空则跳过 ASR,消息以纯语音保存。
//   * 获取百度 API Key/Secret:https://console.bce.baidu.com/ai/#/ai/speech/app/list
// ============================================================================
#define WK_WIFI_SSID        ""
#define WK_WIFI_PASS        ""
#define WK_BAIDU_API_KEY    ""
#define WK_BAIDU_API_SECRET ""

#define WK_WIFI_CONFIGURED()  (WK_WIFI_SSID[0] != '\0')
#define WK_BAIDU_CONFIGURED() (WK_BAIDU_API_KEY[0] != '\0')

#include "demo.h"
#include "demo_radio.h"
#include "walkie_model.h"
#include "bsp_audio.h"
#include "bsp_display.h"
#include "ui_pixel.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "lvgl.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

static const char *TAG = "demo_walkie";

// ---- 常量 ----

#define WK_RECV_QUEUE_LEN     16
#define WK_SEND_TIMEOUT_MS     100
#define WK_WIFI_CONN_MS       10000
#define WK_TASK_STACK         4096
#define WK_REC_STACK          8192
#define WK_STOP_TIMEOUT_MS    4000
#define WK_PAIR_INTERVAL_MS   5000
#define WK_LOOP_DELAY_MS        10
#define WK_PLAY_CHUNK           490   // 回放每次解码的样本数
#define WK_REC_BEGIN_SAMPLES    484   // 首块 = 242 packed bytes = 484 nibbles
#define WK_REC_CHUNK_SAMPLES    490   // 续块 = 245 packed bytes = 490 nibbles
#define WK_ASR_PCM_CHUNK       4096   // ASR 上传每块 4KB PCM = 2048 samples
#define WK_ASR_ADPCM_CHUNK    (WK_ASR_PCM_CHUNK / 4)  // 1024 packed bytes

#define WK_BAIDU_TOKEN_URL  "https://aip.baidubce.com/oauth/2.0/token"
#define WK_BAIDU_ASR_URL    "https://vop.baidu.com/server_api"

#define WK_NVS_NS            "walkie"
#define WK_NVS_BD_TOKEN      "bd_token"
#define WK_NVS_BD_EXP        "bd_exp"

#define WK_WIFI_CONNECTED_BIT  BIT0
#define WK_WIFI_FAIL_BIT       BIT1

// 广播地址:同频道所有 ESP-NOW 设备均接收
static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- 类型 ----

// 接收到的单个 ESP-NOW 包(从回调拷贝到队列)
typedef struct {
    uint8_t mac[6];
    uint8_t data[WK_PKT_MAX];
    int     len;
} s_recv_pkt_t;

// 正在组装的接收消息
typedef struct {
    bool             active;
    uint16_t         msg_id;
    uint8_t          sender_mac[6];
    wk_adpcm_state_t init_state;
    uint8_t         *adpcm;        // 累积的 packed nibble bytes
    uint32_t         adpcm_len;
    uint32_t         adpcm_cap;
} s_pending_t;

// LVGL 气泡引用(用于后续更新文字)
typedef struct {
    bool      used;
    uint16_t  msg_id;
    lv_obj_t *panel;
    lv_obj_t *text_label;
} s_bubble_t;

// ---- 静态状态 ----

// LVGL 对象
static lv_obj_t *s_scr;
static lv_obj_t *s_wifi_label;
static lv_obj_t *s_chat;
static lv_obj_t *s_status_label;

// FreeRTOS 同步原语
static TaskHandle_t      s_walkie_task_handle;
static TaskHandle_t      s_rec_task_handle;
static SemaphoreHandle_t s_task_done;     // 计数信号量:每个任务退出各 give 一次
static SemaphoreHandle_t s_send_sem;     // 二值信号量:ESP-NOW 发送回调
static SemaphoreHandle_t s_send_mutex;   // 互斥:串行化 ESP-NOW 发送
static QueueHandle_t     s_recv_queue;   // 接收包队列

// WiFi
static EventGroupHandle_t          s_wifi_eg;
static esp_netif_t                *s_sta_netif;
static esp_event_handler_instance_t s_wifi_handler;
static esp_event_handler_instance_t s_ip_handler;
static bool s_wifi_inited;
static bool s_wifi_started;
static bool s_espnow_inited;

// 运行时标志
static volatile bool s_run;
static volatile bool s_ptt_recording;
static volatile bool s_wifi_connected;
static volatile esp_now_send_status_t s_send_status;
static volatile bool s_replay_requested;

// 本机 MAC(用于过滤自己发的广播包)
static uint8_t s_my_mac[6];

// 消息存储 + 组装缓冲
static wk_store_t   s_store;
static s_pending_t  s_pending;

// 回放状态(仅 walkie task 访问)
static wk_msg_t *s_play_msg;
static bool      s_playing;

// LVGL 气泡引用表
static s_bubble_t s_bubbles[WK_MSG_STORE_SIZE];

// PAIR 心跳计数器
static uint32_t s_pair_counter;

// ---- 前向声明 ----

static void s_rec_task(void *arg);
static void s_walkie_task(void *arg);
static void s_add_bubble(wk_msg_t *msg);
static void s_update_bubble(wk_msg_t *msg, const char *text);
static void s_update_status(const char *text);

// ============================================================================
// 小工具
// ============================================================================

static char *s_strdup(const char *s)
{
    size_t len = strlen(s) + 1;
    char *copy = malloc(len);
    if (copy) memcpy(copy, s, len);
    return copy;
}

static void s_put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// ============================================================================
// 包构造:与 walkie_model.c 的 wk_pack_voice 格式一致
//   BEGIN payload: [1B step_idx][2B predictor LE][packed nibbles...]
//   CHUNK payload: [packed nibbles...]
//   END   payload: [2B total_pkts LE][packed nibbles...]
// ============================================================================

static uint32_t s_build_begin(uint16_t msg_id, uint16_t seq,
                              const wk_adpcm_state_t *st,
                              const uint8_t *nibbles, uint32_t nib_bytes,
                              uint8_t *out, uint32_t max)
{
    uint32_t take = nib_bytes;
    if (take > WK_NIBBLES_BEGIN) take = WK_NIBBLES_BEGIN;
    uint32_t total = (uint32_t)WK_PKT_HDR + 3 + take;
    if (total > max) return 0;

    out[0] = WK_PKT_BEGIN;
    s_put_u16le(out + 1, msg_id);
    s_put_u16le(out + 3, seq);
    out[5] = st->step_index;
    s_put_u16le(out + 6, (uint16_t)st->predictor);
    if (take > 0) memcpy(out + 8, nibbles, take);
    return total;
}

static uint32_t s_build_chunk(uint16_t msg_id, uint16_t seq,
                              const uint8_t *nibbles, uint32_t nib_bytes,
                              uint8_t *out, uint32_t max)
{
    uint32_t take = nib_bytes;
    if (take > WK_NIBBLES_CHUNK) take = WK_NIBBLES_CHUNK;
    uint32_t total = (uint32_t)WK_PKT_HDR + take;
    if (total > max) return 0;

    out[0] = WK_PKT_CHUNK;
    s_put_u16le(out + 1, msg_id);
    s_put_u16le(out + 3, seq);
    if (take > 0) memcpy(out + 5, nibbles, take);
    return total;
}

static uint32_t s_build_end(uint16_t msg_id, uint16_t seq, uint16_t total_pkts,
                            const uint8_t *nibbles, uint32_t nib_bytes,
                            uint8_t *out, uint32_t max)
{
    uint32_t take = nib_bytes;
    if (take > (uint32_t)(WK_PKT_PAYLOAD - 2)) take = (uint32_t)(WK_PKT_PAYLOAD - 2);
    uint32_t total = (uint32_t)WK_PKT_HDR + 2 + take;
    if (total > max) return 0;

    out[0] = WK_PKT_END;
    s_put_u16le(out + 1, msg_id);
    s_put_u16le(out + 3, seq);
    s_put_u16le(out + 5, total_pkts);
    if (take > 0 && nibbles) memcpy(out + 7, nibbles, take);
    return total;
}

// ============================================================================
// ESP-NOW 层
// ============================================================================

static void s_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    s_send_status = status;
    if (s_send_sem) {
        xSemaphoreGive(s_send_sem);
    }
}

static void s_espnow_recv_cb(const esp_now_recv_info_t *info,
                             const uint8_t *data, int len)
{
    if (!info || !data || len <= 0 || len > WK_PKT_MAX || !s_recv_queue) return;
    s_recv_pkt_t pkt;
    memcpy(pkt.mac, info->src_addr, 6);
    memcpy(pkt.data, data, (size_t)len);
    pkt.len = len;
    // 队列满则丢弃,不阻塞回调
    (void)xQueueSend(s_recv_queue, &pkt, 0);
}

static esp_err_t s_send_pkt(const uint8_t *mac, const uint8_t *data, size_t len)
{
    if (!s_send_sem || !s_send_mutex) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_send_mutex, portMAX_DELAY);

    // 清除可能残留的信号量
    xSemaphoreTake(s_send_sem, 0);

    esp_err_t err = esp_now_send(mac, data, len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW 发送失败: %s", esp_err_to_name(err));
        xSemaphoreGive(s_send_mutex);
        return err;
    }

    if (xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(WK_SEND_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "ESP-NOW 发送超时");
        xSemaphoreGive(s_send_mutex);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t result = (s_send_status == ESP_NOW_SEND_SUCCESS) ? ESP_OK : ESP_FAIL;
    xSemaphoreGive(s_send_mutex);
    return result;
}

static esp_err_t s_espnow_start(void)
{
    if (s_espnow_inited) return ESP_OK;

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_now_register_send_cb(s_espnow_send_cb);
    if (err != ESP_OK) goto fail;

    err = esp_now_register_recv_cb(s_espnow_recv_cb);
    if (err != ESP_OK) goto fail;

    // 添加广播 peer:所有同频道设备均可接收
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, s_broadcast_mac, 6);
    peer.channel = 0;          // 0 = 当前信道
    peer.ifidx   = WIFI_IF_STA;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) goto fail;

    s_espnow_inited = true;
    ESP_LOGI(TAG, "ESP-NOW 就绪,广播模式");
    return ESP_OK;

fail:
    esp_now_deinit();
    return err;
}

static void s_espnow_stop(void)
{
    if (s_espnow_inited) {
        esp_now_deinit();
        s_espnow_inited = false;
    }
}

// ============================================================================
// WiFi 层
// ============================================================================

static void s_wifi_event_handler(void *arg, esp_event_base_t base,
                                 int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        if (s_wifi_eg) xEventGroupSetBits(s_wifi_eg, WK_WIFI_FAIL_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_wifi_connected = true;
        if (s_wifi_eg) xEventGroupSetBits(s_wifi_eg, WK_WIFI_CONNECTED_BIT);
    }
}

static esp_err_t s_wifi_start(void)
{
    esp_err_t err = demo_radio_nvs_prepare();
    if (err != ESP_OK) return err;
    err = demo_radio_network_prepare();
    if (err != ESP_OK) return err;

    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!s_sta_netif) return ESP_ERR_NO_MEM;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) return err;
    s_wifi_inited = true;

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              s_wifi_event_handler, NULL,
                                              &s_wifi_handler);
    if (err != ESP_OK) return err;
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              s_wifi_event_handler, NULL,
                                              &s_ip_handler);
    if (err != ESP_OK) return err;

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;
    err = esp_wifi_start();
    if (err != ESP_OK) return err;
    s_wifi_started = true;

    // 读取本机 MAC(用于 ESP-NOW 回调过滤自己发的包)
    esp_wifi_get_mac(WIFI_IF_STA, s_my_mac);

    // 配置了 SSID 则连接
    if (WK_WIFI_CONFIGURED()) {
        wifi_config_t wifi_cfg = { 0 };
        strncpy((char *)wifi_cfg.sta.ssid, WK_WIFI_SSID,
                sizeof(wifi_cfg.sta.ssid) - 1);
        strncpy((char *)wifi_cfg.sta.password, WK_WIFI_PASS,
                sizeof(wifi_cfg.sta.password) - 1);
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WiFi 配置失败: %s", esp_err_to_name(err));
            return err;
        }

        s_wifi_eg = xEventGroupCreate();
        if (!s_wifi_eg) return ESP_ERR_NO_MEM;

        err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WiFi 连接失败: %s", esp_err_to_name(err));
            s_wifi_connected = false;
            return err;
        }

        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_eg,
            WK_WIFI_CONNECTED_BIT | WK_WIFI_FAIL_BIT,
            pdFALSE, pdFALSE,
            pdMS_TO_TICKS(WK_WIFI_CONN_MS));

        if (bits & WK_WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi 已连接: %s", WK_WIFI_SSID);
        } else {
            ESP_LOGW(TAG, "WiFi 连接超时或失败,ESP-NOW 对讲照常工作");
            s_wifi_connected = false;
        }
        vEventGroupDelete(s_wifi_eg);
        s_wifi_eg = NULL;
    } else {
        ESP_LOGI(TAG, "未配置 WiFi SSID,跳过连接");
        s_wifi_connected = false;
    }

    return ESP_OK;
}

static void s_wifi_stop(void)
{
    if (s_wifi_eg) {
        vEventGroupDelete(s_wifi_eg);
        s_wifi_eg = NULL;
    }
    if (s_wifi_started) {
        esp_wifi_stop();
        s_wifi_started = false;
    }
    if (s_wifi_handler) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              s_wifi_handler);
        s_wifi_handler = NULL;
    }
    if (s_ip_handler) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              s_ip_handler);
        s_ip_handler = NULL;
    }
    if (s_wifi_inited) {
        esp_wifi_deinit();
        s_wifi_inited = false;
    }
    if (s_sta_netif) {
        esp_netif_destroy_default_wifi(s_sta_netif);
        s_sta_netif = NULL;
    }
    s_wifi_connected = false;
}

// ============================================================================
// JSON 简易解析(strstr,不依赖 cJSON)
// ============================================================================

// 从 JSON 中提取 "key":"value" 的 value 部分
static bool s_json_string(const char *json, const char *key,
                          char *out, size_t max)
{
    const char *p = strstr(json, key);
    if (!p) return false;
    p += strlen(key);
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    if (*p != '"') return false;
    p++;
    const char *end = strchr(p, '"');
    if (!end) return false;
    size_t len = (size_t)(end - p);
    if (len >= max) len = max - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

// 从 JSON 中提取 "key":number 的数值
static long s_json_int(const char *json, const char *key)
{
    const char *p = strstr(json, key);
    if (!p) return 0;
    p += strlen(key);
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return atol(p);
}

// ============================================================================
// 百度 ASR 层
// ============================================================================

// 从 NVS 读取缓存的百度 token 和过期时间
static bool s_load_cached_token(char *token, size_t max, int64_t *exp)
{
    nvs_handle_t h;
    if (nvs_open(WK_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;

    bool ok = false;
    size_t tlen = max;
    if (nvs_get_str(h, WK_NVS_BD_TOKEN, token, &tlen) == ESP_OK) {
        int64_t e = 0;
        if (nvs_get_i64(h, WK_NVS_BD_EXP, &e) == ESP_OK) {
            *exp = e;
            ok = true;
        }
    }
    nvs_close(h);
    return ok;
}

// 保存 token 到 NVS
static void s_save_token(const char *token, int64_t exp)
{
    nvs_handle_t h;
    if (nvs_open(WK_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS 打开失败,百度 token 未缓存");
        return;
    }
    if (nvs_set_str(h, WK_NVS_BD_TOKEN, token) != ESP_OK ||
        nvs_set_i64(h, WK_NVS_BD_EXP, exp) != ESP_OK ||
        nvs_commit(h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS 写入失败,百度 token 未缓存");
    }
    nvs_close(h);
}

// 获取百度 access_token。返回 malloc 的 token(调用方负责 free),失败返回 NULL。
static char *s_get_baidu_token(void)
{
    if (!WK_BAIDU_CONFIGURED()) return NULL;

    // 先查 NVS 缓存
    char cached[512];
    int64_t exp = 0;
    if (s_load_cached_token(cached, sizeof(cached), &exp)) {
        int64_t now = (int64_t)time(NULL);
        if (exp > now + 60) {  // 还没过期(留 60s 余量)
            return s_strdup(cached);
        }
    }

    // 请求新 token
    char url[600];
    snprintf(url, sizeof(url),
             "%s?grant_type=client_credentials&client_id=%s&client_secret=%s",
             WK_BAIDU_TOKEN_URL, WK_BAIDU_API_KEY, WK_BAIDU_API_SECRET);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .timeout_ms = 10000,
        .buffer_size = 1024,
        .buffer_size_tx = 512,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return NULL;

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "获取百度 token 失败: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        ESP_LOGW(TAG, "百度 token HTTP %d", status);
        esp_http_client_cleanup(client);
        return NULL;
    }

    char resp[1024];
    int n = esp_http_client_read(client, resp, sizeof(resp) - 1);
    esp_http_client_cleanup(client);
    if (n <= 0) return NULL;
    resp[n] = '\0';

    char token[512];
    if (!s_json_string(resp, "\"access_token\"", token, sizeof(token))) {
        ESP_LOGW(TAG, "百度 token 响应解析失败");
        return NULL;
    }

    long expires_in = s_json_int(resp, "\"expires_in\"");
    int64_t now = (int64_t)time(NULL);
    s_save_token(token, now + (int64_t)expires_in);

    ESP_LOGI(TAG, "百度 token 获取成功,有效期 %ld 秒", expires_in);
    return s_strdup(token);
}

// 从 ASR 响应中提取 result 数组的第一个文本
static bool s_parse_asr_result(const char *resp, char *text, size_t max)
{
    const char *p = strstr(resp, "\"result\"");
    if (!p) return false;
    p = strchr(p, '[');
    if (!p) return false;
    p = strchr(p, '"');
    if (!p) return false;
    p++;
    const char *end = strchr(p, '"');
    if (!end) return false;
    size_t len = (size_t)(end - p);
    if (len == 0) return false;
    if (len >= max) len = max - 1;
    memcpy(text, p, len);
    text[len] = '\0';
    return true;
}

// 百度语音转文字。成功返回 true 并填充 text。
// 解码 ADPCM → PCM 在 4KB PCM 块内边解码边上传,避免分配完整 PCM 缓冲。
static bool s_baidu_asr(const uint8_t *adpcm, uint32_t adpcm_len,
                        const wk_adpcm_state_t *init_state,
                        char *text, uint32_t text_max)
{
    char *token = s_get_baidu_token();
    if (!token) {
        ESP_LOGW(TAG, "无法获取百度 token,跳过 ASR");
        return false;
    }

    // cuid 用本机 MAC 十六进制
    char cuid[16];
    snprintf(cuid, sizeof(cuid), "%02X%02X%02X%02X%02X%02X",
             s_my_mac[0], s_my_mac[1], s_my_mac[2],
             s_my_mac[3], s_my_mac[4], s_my_mac[5]);

    char url[700];
    snprintf(url, sizeof(url),
             "%s?cuid=%s&token=%s&dev_pid=1537",
             WK_BAIDU_ASR_URL, cuid, token);
    free(token);

    // PCM 字节数 = adpcm_len * 4 (每 packed byte = 2 nibble = 2 sample = 4B)
    int content_length = (int)(adpcm_len * 4);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .timeout_ms = 15000,
        .buffer_size = 2048,
        .buffer_size_tx = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return false;

    bool result = false;
    int16_t *pcm = NULL;

    esp_http_client_set_header(client, "Content-Type",
                               "audio/pcm;rate=16000");

    esp_err_t err = esp_http_client_open(client, content_length);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ASR HTTP 打开失败: %s", esp_err_to_name(err));
        goto done;
    }

    pcm = malloc(WK_ASR_PCM_CHUNK);
    if (!pcm) {
        ESP_LOGW(TAG, "ASR PCM 缓冲分配失败");
        goto done;
    }

    // 分块解码 ADPCM → PCM 并写入 HTTP
    {
        wk_adpcm_state_t dec = *init_state;
        const uint8_t *pos = adpcm;
        uint32_t remaining = adpcm_len;

        while (remaining > 0) {
            uint32_t chunk = remaining;
            if (chunk > WK_ASR_ADPCM_CHUNK) chunk = WK_ASR_ADPCM_CHUNK;
            uint32_t nibbles = chunk * 2;
            uint32_t decoded = wk_adpcm_decode(&dec, pos, nibbles,
                                                pcm, WK_ASR_PCM_CHUNK / 2);
            if (decoded == 0) break;
            int written = esp_http_client_write(client, (char *)pcm,
                                                (int)(decoded * 2));
            if (written < 0) {
                ESP_LOGW(TAG, "ASR HTTP 写入失败");
                goto done;
            }
            pos += chunk;
            remaining -= chunk;
        }
    }

    // 获取响应头
    int fetch_len = (int)esp_http_client_fetch_headers(client);
    (void)fetch_len;

    {
        int http_status = esp_http_client_get_status_code(client);
        if (http_status != 200) {
            ESP_LOGW(TAG, "ASR HTTP %d", http_status);
            goto done;
        }

        // 读取响应体
        char resp[2048];
        int total = 0;
        int rd;
        while (total < (int)sizeof(resp) - 1) {
            rd = esp_http_client_read(client, resp + total,
                                      (int)sizeof(resp) - 1 - total);
            if (rd <= 0) break;
            total += rd;
        }
        resp[total] = '\0';

        if (!s_parse_asr_result(resp, text, (size_t)text_max)) {
            ESP_LOGW(TAG, "ASR 响应无 result: %.200s", resp);
            goto done;
        }

        ESP_LOGI(TAG, "ASR 转写成功: %s", text);
        result = true;
    }

done:
    free(pcm);
    esp_http_client_cleanup(client);
    return result;
}

// ============================================================================
// LVGL UI(内部自取 LVGL 锁)
// ============================================================================

static void s_update_status(const char *text)
{
    if (!bsp_lvgl_lock(250)) return;
    if (s_status_label) lv_label_set_text(s_status_label, text);
    bsp_lvgl_unlock();
}

static void s_update_wifi_status(void)
{
    if (!bsp_lvgl_lock(250)) return;
    if (s_wifi_label) {
        lv_label_set_text(s_wifi_label,
            s_wifi_connected ? "WiFi: ON" : "WiFi: OFF");
    }
    bsp_lvgl_unlock();
}

// 在聊天容器里添加一条消息气泡,并滚动到底部
static void s_add_bubble(wk_msg_t *msg)
{
    if (!bsp_lvgl_lock(250)) return;

    // 查找已有气泡(同 msg_id)或空槽
    s_bubble_t *b = NULL;
    for (int i = 0; i < WK_MSG_STORE_SIZE; i++) {
        if (s_bubbles[i].used && s_bubbles[i].msg_id == msg->id) {
            b = &s_bubbles[i];
            break;
        }
    }
    if (!b) {
        for (int i = 0; i < WK_MSG_STORE_SIZE; i++) {
            if (!s_bubbles[i].used) {
                b = &s_bubbles[i];
                break;
            }
        }
    }
    if (!b) {
        // 全部占用,回收索引 0 的旧气泡
        b = &s_bubbles[0];
        if (b->panel) {
            lv_obj_delete(b->panel);
            b->panel = NULL;
            b->text_label = NULL;
        }
    }

    // 气泡面板
    lv_obj_t *panel = lv_obj_create(s_chat);
    lv_obj_set_width(panel, 184);
    lv_obj_set_height(panel, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(panel, 6, 0);
    lv_obj_set_style_border_width(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 6, 0);
    uint32_t bg = msg->is_mine ? UI_ORANGE : UI_PAPER;
    lv_obj_set_style_bg_color(panel, lv_color_hex(bg), 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);

    // 左右对齐:用 margin 推到屏幕一侧
    if (msg->is_mine) {
        lv_obj_set_style_margin_left(panel, 44, 0);
        lv_obj_set_style_margin_right(panel, 4, 0);
    } else {
        lv_obj_set_style_margin_left(panel, 4, 0);
        lv_obj_set_style_margin_right(panel, 44, 0);
    }

    // 头部:发送者简称 + 时长
    char header[64];
    if (msg->is_mine) {
        snprintf(header, sizeof(header), "me  %lu.%lus",
                 msg->duration_ms / 1000, (msg->duration_ms % 1000) / 100);
    } else {
        snprintf(header, sizeof(header), "%02X%02X  %lu.%lus",
                 msg->sender_mac[4], msg->sender_mac[5],
                 msg->duration_ms / 1000, (msg->duration_ms % 1000) / 100);
    }
    lv_obj_t *hdr = lv_label_create(panel);
    lv_obj_set_style_text_color(hdr, lv_color_hex(UI_INK), 0);
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_14, 0);
    lv_label_set_text(hdr, header);

    // 文字标签(有文字时显示,无则隐藏)
    lv_obj_t *txt = lv_label_create(panel);
    lv_obj_set_style_text_color(txt, lv_color_hex(UI_INK), 0);
    lv_obj_set_style_text_font(txt, &lv_font_montserrat_14, 0);
    lv_obj_set_width(txt, 172);
    lv_label_set_long_mode(txt, LV_LABEL_LONG_WRAP);
    if (msg->has_text && msg->text[0] != '\0') {
        lv_label_set_text(txt, msg->text);
    } else {
        lv_label_set_text(txt, "");
        lv_obj_add_flag(txt, LV_OBJ_FLAG_HIDDEN);
    }

    b->used = true;
    b->msg_id = msg->id;
    b->panel = panel;
    b->text_label = txt;

    // 滚动到底部,使最新气泡可见
    lv_obj_scroll_to_view(panel, LV_ANIM_ON);

    bsp_lvgl_unlock();
}

// 更新已有气泡的文字标签(ASR 或 TEXT 包到达时调用)
static void s_update_bubble(wk_msg_t *msg, const char *text)
{
    if (!bsp_lvgl_lock(250)) return;
    for (int i = 0; i < WK_MSG_STORE_SIZE; i++) {
        if (s_bubbles[i].used && s_bubbles[i].msg_id == msg->id) {
            if (s_bubbles[i].text_label) {
                lv_obj_remove_flag(s_bubbles[i].text_label, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text(s_bubbles[i].text_label, text);
            }
            break;
        }
    }
    bsp_lvgl_unlock();
}

// ============================================================================
// 录音任务:按下 OK 时创建,松手后结束
// ============================================================================

static void s_rec_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "录音任务启动");

    // 设置音频格式(本任务独占)
    if (bsp_audio_set_format(WK_SAMPLE_RATE, 16, 1) != ESP_OK) {
        ESP_LOGE(TAG, "录音:音频格式设置失败");
        s_ptt_recording = false;
        goto done;
    }

    uint8_t *adpcm_buf = malloc(WK_MAX_ADPCM_BYTES);
    int16_t *pcm_buf   = malloc(WK_REC_CHUNK_SAMPLES * sizeof(int16_t));
    uint8_t  pkt[WK_PKT_MAX];
    if (!adpcm_buf || !pcm_buf) {
        ESP_LOGE(TAG, "录音:内存分配失败 (C3 无 PSRAM,40KB 可能紧张)");
        free(adpcm_buf);
        free(pcm_buf);
        s_ptt_recording = false;
        goto done;
    }

    wk_adpcm_state_t enc_state;
    wk_adpcm_init(&enc_state);
    // 保存初始状态(BEGIN 包携带,解码端从此状态开始)
    wk_adpcm_state_t init_state = enc_state;

    uint32_t adpcm_offset = 0;
    uint16_t seq = 0;
    uint16_t pkt_count = 0;
    uint32_t total_samples = 0;
    uint16_t msg_id = s_store.next_msg_id;
    s_store.next_msg_id++;
    if (s_store.next_msg_id == 0) s_store.next_msg_id = 1;

    // ---- 首块:484 样本 → BEGIN ----
    if (bsp_audio_read(pcm_buf, WK_REC_BEGIN_SAMPLES * sizeof(int16_t)) != ESP_OK) {
        ESP_LOGE(TAG, "录音:首块读取失败");
        free(adpcm_buf);
        free(pcm_buf);
        s_ptt_recording = false;
        goto done;
    }
    total_samples += WK_REC_BEGIN_SAMPLES;
    {
        uint32_t packed = wk_adpcm_encode(&enc_state, pcm_buf,
                                          WK_REC_BEGIN_SAMPLES,
                                          adpcm_buf + adpcm_offset,
                                          WK_MAX_ADPCM_BYTES - adpcm_offset);
        adpcm_offset += packed;
        uint32_t plen = s_build_begin(msg_id, seq++, &init_state,
                                       adpcm_buf, adpcm_offset,
                                       pkt, WK_PKT_MAX);
        if (plen > 0) {
            s_send_pkt(s_broadcast_mac, pkt, plen);
            pkt_count++;
        }
    }

    // ---- 循环:490 样本 → CHUNK ----
    while (s_ptt_recording && s_run &&
           adpcm_offset + WK_NIBBLES_CHUNK <= WK_MAX_ADPCM_BYTES) {
        if (bsp_audio_read(pcm_buf, WK_REC_CHUNK_SAMPLES * sizeof(int16_t)) != ESP_OK) {
            break;
        }
        total_samples += WK_REC_CHUNK_SAMPLES;
        uint32_t packed = wk_adpcm_encode(&enc_state, pcm_buf,
                                          WK_REC_CHUNK_SAMPLES,
                                          adpcm_buf + adpcm_offset,
                                          WK_MAX_ADPCM_BYTES - adpcm_offset);
        if (packed == 0) break;
        // CHUNK 数据在 adpcm_buf + adpcm_offset,共 packed 字节
        uint32_t plen = s_build_chunk(msg_id, seq++,
                                       adpcm_buf + adpcm_offset, packed,
                                       pkt, WK_PKT_MAX);
        adpcm_offset += packed;
        if (plen > 0) {
            s_send_pkt(s_broadcast_mac, pkt, plen);
            pkt_count++;
        }
    }

    // ---- END 包(含总包数,无尾部音频)----
    {
        uint16_t total_pkts = (uint16_t)(pkt_count + 1);
        uint32_t plen = s_build_end(msg_id, seq, total_pkts,
                                    NULL, 0, pkt, WK_PKT_MAX);
        if (plen > 0) {
            s_send_pkt(s_broadcast_mac, pkt, plen);
        }
    }

    // 计算时长
    uint32_t duration_ms = total_samples * 1000 / WK_SAMPLE_RATE;

    // 存入消息存储
    wk_store_add(&s_store, msg_id, s_my_mac, true, duration_ms,
                 adpcm_buf, adpcm_offset, &init_state);

    ESP_LOGI(TAG, "录音完成: %lu 样本, %lu ms, %lu 包",
             total_samples, duration_ms, pkt_count + 1);

    // 更新 UI(添加气泡)
    wk_msg_t *stored = wk_store_find(&s_store, msg_id);
    if (stored) s_add_bubble(stored);

    // 如果 WiFi + 百度 API 配置了,启动 ASR
    if (s_wifi_connected && WK_BAIDU_CONFIGURED()) {
        s_update_status("转写中...");
        char text[120];
        if (s_baidu_asr(adpcm_buf, adpcm_offset, &init_state,
                        text, sizeof(text))) {
            // 挂文字到本机消息
            wk_store_set_text(&s_store, msg_id, text);
            wk_msg_t *m = wk_store_find(&s_store, msg_id);
            if (m) s_update_bubble(m, text);

            // 广播 TEXT 包给其他设备
            uint8_t text_pkt[WK_PKT_MAX];
            uint32_t tlen = wk_build_text_pkt(msg_id, text,
                                               text_pkt, WK_PKT_MAX);
            if (tlen > 0) {
                s_send_pkt(s_broadcast_mac, text_pkt, tlen);
            }
        } else {
            ESP_LOGW(TAG, "ASR 转写失败,消息以纯语音保存");
        }
    }

    s_update_status("按住 OK 说话");

    free(adpcm_buf);
    free(pcm_buf);
    s_ptt_recording = false;

done:
    if (s_task_done) xSemaphoreGive(s_task_done);
    s_rec_task_handle = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
// Walkie 主任务:处理接收队列 + 回放 + PAIR 心跳
// ============================================================================

static void s_process_begin(uint16_t msg_id, const uint8_t *mac,
                            const uint8_t *payload, uint32_t payload_len)
{
    // 已有 pending 则丢弃旧的
    if (s_pending.active) {
        free(s_pending.adpcm);
        memset(&s_pending, 0, sizeof(s_pending));
    }

    s_pending.active = true;
    s_pending.msg_id = msg_id;
    memcpy(s_pending.sender_mac, mac, 6);
    wk_pkt_begin_state(payload, payload_len, &s_pending.init_state);

    // 提取 BEGIN 包的音频部分
    uint8_t audio[WK_PKT_PAYLOAD];
    uint32_t audio_len = wk_pkt_audio_bytes(WK_PKT_BEGIN, payload, payload_len,
                                             audio, WK_PKT_PAYLOAD);

    s_pending.adpcm_cap = WK_MAX_ADPCM_BYTES;
    s_pending.adpcm = malloc(s_pending.adpcm_cap);
    if (!s_pending.adpcm) {
        ESP_LOGE(TAG, "pending ADPCM 缓冲分配失败");
        s_pending.active = false;
        return;
    }
    if (audio_len > 0) {
        memcpy(s_pending.adpcm, audio, audio_len);
    }
    s_pending.adpcm_len = audio_len;
}

static void s_process_chunk(uint16_t msg_id,
                            const uint8_t *payload, uint32_t payload_len)
{
    if (!s_pending.active || s_pending.msg_id != msg_id) return;
    if (!s_pending.adpcm) return;

    uint8_t audio[WK_PKT_PAYLOAD];
    uint32_t audio_len = wk_pkt_audio_bytes(WK_PKT_CHUNK, payload, payload_len,
                                             audio, WK_PKT_PAYLOAD);
    if (audio_len == 0) return;

    if (s_pending.adpcm_len + audio_len > s_pending.adpcm_cap) {
        ESP_LOGW(TAG, "pending ADPCM 溢出,丢弃续包");
        return;
    }
    memcpy(s_pending.adpcm + s_pending.adpcm_len, audio, audio_len);
    s_pending.adpcm_len += audio_len;
}

static void s_process_end(uint16_t msg_id,
                          const uint8_t *payload, uint32_t payload_len)
{
    if (!s_pending.active || s_pending.msg_id != msg_id) return;
    if (!s_pending.adpcm) {
        s_pending.active = false;
        return;
    }

    // END 包可能携带尾部音频
    uint8_t audio[WK_PKT_PAYLOAD];
    uint32_t audio_len = wk_pkt_audio_bytes(WK_PKT_END, payload, payload_len,
                                             audio, WK_PKT_PAYLOAD);
    if (audio_len > 0 && s_pending.adpcm_len + audio_len <= s_pending.adpcm_cap) {
        memcpy(s_pending.adpcm + s_pending.adpcm_len, audio, audio_len);
        s_pending.adpcm_len += audio_len;
    }

    // 计算时长:每 packed byte = 2 样本
    uint32_t total_samples = s_pending.adpcm_len * 2;
    uint32_t duration_ms = total_samples * 1000 / WK_SAMPLE_RATE;

    // 存入消息存储
    wk_store_add(&s_store, s_pending.msg_id, s_pending.sender_mac,
                 false, duration_ms,
                 s_pending.adpcm, s_pending.adpcm_len,
                 &s_pending.init_state);

    ESP_LOGI(TAG, "收到消息 #%lu: %lu 字节 ADPCM, %lu ms",
             s_pending.msg_id, s_pending.adpcm_len, duration_ms);

    // 更新 UI(添加气泡)
    wk_msg_t *stored = wk_store_find(&s_store, s_pending.msg_id);
    if (stored) {
        s_add_bubble(stored);
        // 触发自动回放
        s_play_msg = stored;
        s_playing = false;
        s_update_status("播放中...");
    }

    free(s_pending.adpcm);
    s_pending.adpcm = NULL;
    s_pending.active = false;
}

static void s_process_text(const uint8_t *payload, uint32_t payload_len)
{
    uint16_t ref_id = 0;
    char text[120];
    wk_pkt_text(payload, payload_len, &ref_id, text, sizeof(text));
    if (ref_id == 0 || text[0] == '\0') return;

    if (wk_store_set_text(&s_store, ref_id, text)) {
        wk_msg_t *m = wk_store_find(&s_store, ref_id);
        if (m) s_update_bubble(m, text);
        ESP_LOGI(TAG, "收到文字 #%lu: %s", ref_id, text);
    }
}

static void s_walkie_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "walkie 任务启动");

    s_recv_pkt_t pkt;
    while (s_run) {
        // ---- 处理接收队列(非阻塞) ----
        while (xQueueReceive(s_recv_queue, &pkt, 0) == pdTRUE) {
            // 过滤自己发的广播包
            if (memcmp(pkt.mac, s_my_mac, 6) == 0) continue;

            uint16_t msg_id = 0, seq = 0;
            const uint8_t *payload = NULL;
            uint32_t payload_len = 0;
            int type = wk_pkt_parse(pkt.data, (uint32_t)pkt.len,
                                    &msg_id, &seq, &payload, &payload_len);
            if (type < 0) continue;

            switch (type) {
            case WK_PKT_BEGIN:
                s_process_begin(msg_id, pkt.mac, payload, payload_len);
                break;
            case WK_PKT_CHUNK:
                s_process_chunk(msg_id, payload, payload_len);
                break;
            case WK_PKT_END:
                s_process_end(msg_id, payload, payload_len);
                break;
            case WK_PKT_TEXT:
                s_process_text(payload, payload_len);
                break;
            case WK_PKT_PAIR:
                // 配对/心跳:忽略
                break;
            default:
                break;
            }
        }

        // ---- 回放请求(DOWN 键) ----
        if (s_replay_requested && !s_rec_task_handle && !s_play_msg) {
            s_replay_requested = false;
            // 找最近一条收到的消息
            wk_msg_t *recent[3];
            int n = wk_store_recent(&s_store, recent, 3);
            for (int i = 0; i < n; i++) {
                if (!recent[i]->is_mine && recent[i]->active &&
                    recent[i]->adpcm) {
                    s_play_msg = recent[i];
                    s_playing = false;
                    s_update_status("回放中...");
                    break;
                }
            }
        }

        // ---- 音频回放(录音时不回放,避免 codec 冲突) ----
        if (s_play_msg && !s_rec_task_handle &&
            s_play_msg->active && s_play_msg->adpcm) {
            if (!s_playing) {
                // 首次播放:初始化回放状态 + 音频格式
                wk_msg_replay_start(s_play_msg);
                bsp_audio_set_format(WK_SAMPLE_RATE, 16, 1);
                bsp_audio_set_volume(80);
                s_playing = true;
            }
            int16_t pcm[WK_PLAY_CHUNK];
            uint32_t decoded = wk_msg_replay(s_play_msg, pcm, WK_PLAY_CHUNK);
            if (decoded > 0) {
                bsp_audio_write(pcm, decoded * sizeof(int16_t));
            } else {
                // 回放完毕
                s_play_msg = NULL;
                s_playing = false;
                s_update_status("按住 OK 说话");
            }
        } else if (s_play_msg && (!s_play_msg->active || !s_play_msg->adpcm)) {
            // 消息已被回收(存储环形覆盖)
            s_play_msg = NULL;
            s_playing = false;
        }

        // ---- PAIR 心跳 ----
        s_pair_counter += WK_LOOP_DELAY_MS;
        if (s_pair_counter >= WK_PAIR_INTERVAL_MS) {
            s_pair_counter = 0;
            uint8_t pair_pkt[WK_PKT_MAX];
            uint32_t plen = wk_build_pair_pkt("FoloWalkie",
                                               pair_pkt, WK_PKT_MAX);
            if (plen > 0) {
                (void)s_send_pkt(s_broadcast_mac, pair_pkt, plen);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(WK_LOOP_DELAY_MS));
    }

    // 清理 pending
    if (s_pending.active && s_pending.adpcm) {
        free(s_pending.adpcm);
        s_pending.adpcm = NULL;
    }
    s_pending.active = false;

    if (s_task_done) xSemaphoreGive(s_task_done);
    s_walkie_task_handle = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
// 生命周期:enter / exit / start / stop / key
// ============================================================================

void demo_walkie_enter(void)
{
    if (!bsp_lvgl_lock(1000)) return;

    s_scr = ui_pixel_screen_create("Walkie");

    // 顶栏(y=0, h=24, UI_INK 底)
    lv_obj_t *top = lv_obj_create(s_scr);
    lv_obj_remove_flag(top, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(top, 0, 0);
    lv_obj_set_size(top, 240, 24);
    lv_obj_set_style_radius(top, 0, 0);
    lv_obj_set_style_border_width(top, 0, 0);
    lv_obj_set_style_pad_all(top, 2, 0);
    lv_obj_set_style_bg_color(top, lv_color_hex(UI_INK), 0);

    s_wifi_label = lv_label_create(top);
    lv_obj_set_style_text_color(s_wifi_label, lv_color_hex(UI_MUTED), 0);
    lv_obj_set_style_text_font(s_wifi_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_wifi_label, LV_ALIGN_LEFT_MID, 2, 0);
    lv_label_set_text(s_wifi_label, "WiFi: ---");

    lv_obj_t *espnow_lbl = lv_label_create(top);
    lv_obj_set_style_text_color(espnow_lbl, lv_color_hex(UI_YELLOW), 0);
    lv_obj_set_style_text_font(espnow_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(espnow_lbl, LV_ALIGN_RIGHT_MID, -2, 0);
    lv_label_set_text(espnow_lbl, "ESP-NOW");

    // 聊天容器(y=24, h=232, 可滚动, flex 列布局)
    s_chat = lv_obj_create(s_scr);
    lv_obj_set_pos(s_chat, 0, 24);
    lv_obj_set_size(s_chat, 240, 232);
    lv_obj_set_style_radius(s_chat, 0, 0);
    lv_obj_set_style_border_width(s_chat, 0, 0);
    lv_obj_set_style_pad_all(s_chat, 4, 0);
    lv_obj_set_style_bg_color(s_chat, lv_color_hex(UI_SKY), 0);
    lv_obj_set_flex_flow(s_chat, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_chat, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(s_chat, LV_DIR_VER);

    // 底栏(y=256, h=64, UI_INK 底)
    lv_obj_t *bottom = lv_obj_create(s_scr);
    lv_obj_remove_flag(bottom, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(bottom, 0, 256);
    lv_obj_set_size(bottom, 240, 64);
    lv_obj_set_style_radius(bottom, 0, 0);
    lv_obj_set_style_border_width(bottom, 0, 0);
    lv_obj_set_style_pad_all(bottom, 4, 0);
    lv_obj_set_style_bg_color(bottom, lv_color_hex(UI_INK), 0);

    s_status_label = lv_label_create(bottom);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(UI_PAPER), 0);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(s_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_status_label, 224);
    lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(s_status_label, "按住 OK 说话");

    lv_screen_load(s_scr);

    bsp_lvgl_unlock();
}

void demo_walkie_exit(void)
{
    if (!bsp_lvgl_lock(500)) return;
    if (s_scr) {
        lv_obj_delete(s_scr);
        s_scr = NULL;
    }
    s_wifi_label = NULL;
    s_chat = NULL;
    s_status_label = NULL;
    memset(s_bubbles, 0, sizeof(s_bubbles));
    bsp_lvgl_unlock();
}

esp_err_t demo_walkie_start(void)
{
    if (s_walkie_task_handle) return ESP_OK;

    // 初始化状态
    memset(s_bubbles, 0, sizeof(s_bubbles));
    memset(&s_pending, 0, sizeof(s_pending));
    s_play_msg = NULL;
    s_playing = false;
    s_ptt_recording = false;
    s_replay_requested = false;
    s_pair_counter = 0;
    s_run = true;
    s_send_status = ESP_NOW_SEND_SUCCESS;

    wk_store_init(&s_store);

    // 创建同步原语
    s_task_done = xSemaphoreCreateCounting(2, 0);
    s_send_sem = xSemaphoreCreateBinary();
    s_send_mutex = xSemaphoreCreateMutex();
    s_recv_queue = xQueueCreate(WK_RECV_QUEUE_LEN, sizeof(s_recv_pkt_t));
    if (!s_task_done || !s_send_sem || !s_send_mutex || !s_recv_queue) {
        if (s_task_done)   { vSemaphoreDelete(s_task_done); s_task_done = NULL; }
        if (s_send_sem)    { vSemaphoreDelete(s_send_sem); s_send_sem = NULL; }
        if (s_send_mutex)   { vSemaphoreDelete(s_send_mutex); s_send_mutex = NULL; }
        if (s_recv_queue)   { vQueueDelete(s_recv_queue); s_recv_queue = NULL; }
        return ESP_ERR_NO_MEM;
    }

    // 初始化 WiFi(失败不阻塞,ESP-NOW 仍可工作)
    esp_err_t err = s_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi 初始化失败: %s,ESP-NOW 对讲照常工作",
                 esp_err_to_name(err));
    }
    s_update_wifi_status();

    // 初始化 ESP-NOW(失败则清理退出)
    err = s_espnow_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW 初始化失败: %s", esp_err_to_name(err));
        s_wifi_stop();
        vSemaphoreDelete(s_task_done);   s_task_done = NULL;
        vSemaphoreDelete(s_send_sem);    s_send_sem = NULL;
        vSemaphoreDelete(s_send_mutex);  s_send_mutex = NULL;
        vQueueDelete(s_recv_queue);      s_recv_queue = NULL;
        s_run = false;
        return err;
    }

    // 初始化音频(失败不阻塞,只是无法录音/回放)
    err = bsp_audio_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "音频初始化失败: %s", esp_err_to_name(err));
    }

    // 创建 walkie 主任务
    if (xTaskCreate(s_walkie_task, "walkie", WK_TASK_STACK, NULL, 5,
                    &s_walkie_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "walkie 任务创建失败");
        s_espnow_stop();
        s_wifi_stop();
        vSemaphoreDelete(s_task_done);   s_task_done = NULL;
        vSemaphoreDelete(s_send_sem);    s_send_sem = NULL;
        vSemaphoreDelete(s_send_mutex);  s_send_mutex = NULL;
        vQueueDelete(s_recv_queue);      s_recv_queue = NULL;
        s_run = false;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t demo_walkie_stop(void)
{
    s_run = false;

    // 等待任务结束
    int tasks_to_wait = 0;
    if (s_walkie_task_handle) tasks_to_wait++;
    if (s_rec_task_handle) tasks_to_wait++;

    for (int i = 0; i < tasks_to_wait; i++) {
        if (s_task_done) {
            if (xSemaphoreTake(s_task_done,
                pdMS_TO_TICKS(WK_STOP_TIMEOUT_MS)) != pdTRUE) {
                ESP_LOGW(TAG, "任务停止超时");
                break;
            }
        }
    }

    s_walkie_task_handle = NULL;
    s_rec_task_handle = NULL;

    // 清理 ESP-NOW
    s_espnow_stop();

    // 清理 WiFi
    s_wifi_stop();
    s_update_wifi_status();

    // 清理消息存储
    wk_store_free(&s_store);

    // 清理 pending
    if (s_pending.active && s_pending.adpcm) {
        free(s_pending.adpcm);
        s_pending.adpcm = NULL;
    }
    s_pending.active = false;

    // 清理同步原语
    if (s_task_done)   { vSemaphoreDelete(s_task_done);   s_task_done = NULL; }
    if (s_send_sem)    { vSemaphoreDelete(s_send_sem);    s_send_sem = NULL; }
    if (s_send_mutex)  { vSemaphoreDelete(s_send_mutex);  s_send_mutex = NULL; }
    if (s_recv_queue)  { vQueueDelete(s_recv_queue);      s_recv_queue = NULL; }

    // 清理状态
    memset(s_bubbles, 0, sizeof(s_bubbles));
    s_play_msg = NULL;
    s_playing = false;
    s_ptt_recording = false;

    return ESP_OK;
}

void demo_walkie_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    // OK 按下:开始录音(如果空闲)
    if (btn == BSP_BTN_OK && ev == BSP_BTN_PRESS) {
        if (!s_ptt_recording && !s_rec_task_handle && s_run) {
            s_ptt_recording = true;
            if (xTaskCreate(s_rec_task, "wk_rec", WK_REC_STACK, NULL, 4,
                            &s_rec_task_handle) != pdPASS) {
                ESP_LOGE(TAG, "录音任务创建失败");
                s_rec_task_handle = NULL;
                s_ptt_recording = false;
            } else {
                s_update_status("录音中... 松手发送");
            }
        }
        return;
    }

    // OK 单击(松手):停止录音
    if (btn == BSP_BTN_OK && ev == BSP_BTN_CLICK) {
        if (s_ptt_recording) {
            s_ptt_recording = false;
            // 录音任务会检测到标志变化并结束
        }
        return;
    }

    // DOWN 单击:回放最后收到的消息
    if (btn == BSP_BTN_DOWN && ev == BSP_BTN_CLICK) {
        if (!s_rec_task_handle && !s_play_msg) {
            s_replay_requested = true;
        }
        return;
    }
}
