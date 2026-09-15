// main/demo_voice_jump.c —— 声控跳跳酱:用麦克风音量控制角色升降的跑酷游戏。
//
// 线程模型(全部阻塞操作都在工作者任务里,遵守 AGENTS.md 的运行时约束):
//   * audio task:独占音频所有权。平时阻塞读 256 采样(~16ms)算 RMS,用
//     深度 1 的队列「覆盖写」发布最新音量;偶发被命令切换去播放短音效。
//     读和写不并发(音效期间暂停采集),避免喇叭串进麦克风污染音量读数。
//   * game task:33ms 定步长取最新 RMS -> 更新纯逻辑模型 -> 持 LVGL 锁渲染。
//     电量读数也在这里定期刷新(I2C 由驱动内部按总线串行化,与音频任务并发安全)。
//   * key():只置 volatile 标志,由 game task 消费,按键回调路径零阻塞。
// 最高分存 NVS(namespace "voice_jump", key "best"),失败只降级不阻塞游戏。
#include "demo.h"
#include "demo_radio.h"
#include "voice_jump_model.h"

#include "bsp_audio.h"
#include "bsp_battery.h"
#include "bsp_display.h"
#include "ui_pixel.h"

#include "lvgl.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>

static const char *TAG = "demo_voice_jump";

#define VJ_TICK_MS 33
#define VJ_MIC_CHUNK 256          // 每次读取的采样数(16ms @ 16kHz)
#define VJ_SAMPLE_RATE 16000
#define VJ_TWO_PI 6.28318530718f
#define VJ_BEEP_VOLUME 70
#define VJ_STOP_TIMEOUT_MS 2000
#define VJ_BATTERY_EVERY_TICKS 60 // 约 2 秒读一次电量(I2C 慢速外设)

// 面板与音量条几何
#define VJ_PANEL_X 28
#define VJ_PANEL_Y 100
#define VJ_PANEL_W 184
#define VJ_PANEL_H 132
#define VJ_BAR_X 6
#define VJ_BAR_Y 60
#define VJ_BAR_W 12
#define VJ_BAR_H 150

typedef enum {
    VJ_AUDIO_NONE = 0,
    VJ_AUDIO_BEEP_START, // 开局短促高音
    VJ_AUDIO_BEEP_DEATH, // 死亡下滑低音
} vj_audio_cmd_t;

static lv_obj_t *s_scr;
static lv_obj_t *s_player;
static lv_obj_t *s_obst_ui[VJ_OBST_MAX];
static lv_obj_t *s_score_label, *s_best_label, *s_battery_label;
static lv_obj_t *s_bar_fill;
static lv_obj_t *s_ready_panel, *s_ready_shadow, *s_ready_hint, *s_ready_best;
static lv_obj_t *s_dead_panel, *s_dead_shadow, *s_dead_score, *s_dead_best, *s_dead_extra;

static TaskHandle_t s_game_task, s_audio_task;
static SemaphoreHandle_t s_stopped;   // 计数信号量:每个任务退出各 give 一次
static QueueHandle_t s_level_queue;   // 深度 1,只保留最新 RMS
static volatile bool s_tasks_run;
static volatile bool s_action_start;  // OK 短按 -> 由 game task 消费
static volatile vj_audio_cmd_t s_audio_cmd;
static vj_game_t s_game;

// 渲染用的「上一次值」缓存:文本与样式只在变化时更新,避免 30fps 全量失效。
static uint32_t s_shown_score, s_shown_best;
static bool s_obst_shown[VJ_OBST_MAX];
static bool s_obst_floating[VJ_OBST_MAX];

static esp_err_t vj_tasks_stop(int tasks_running);

// ---- 小工具 ----

static lv_obj_t *vj_block(lv_obj_t *parent, int x, int y, int w, int h,
                          uint32_t color)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_radius(obj, 0, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    return obj;
}

static float vj_rms_of(const int16_t *pcm, int n)
{
    int64_t acc = 0;
    for (int i = 0; i < n; i++) acc += (int64_t)pcm[i] * pcm[i];
    return sqrtf((float)(acc / n));
}

// ---- NVS 最高分 ----

static uint32_t vj_load_best(void)
{
    if (demo_radio_nvs_prepare() != ESP_OK) return 0;
    nvs_handle_t h;
    if (nvs_open("voice_jump", NVS_READONLY, &h) != ESP_OK) return 0;
    uint32_t best = 0;
    (void)nvs_get_u32(h, "best", &best); // 键不存在时保持 0
    nvs_close(h);
    return best;
}

static void vj_save_best(uint32_t best)
{
    nvs_handle_t h;
    if (nvs_open("voice_jump", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS 打开失败,本局最高分未保存");
        return;
    }
    if (nvs_set_u32(h, "best", best) != ESP_OK || nvs_commit(h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS 写入失败,本局最高分未保存");
    }
    nvs_close(h);
}

// ---- 音频任务:采集 RMS + 偶发音效 ----

// 相位累加正弦;频率随进度从 f0 线性滑到 f1,输出固定振幅。
static void vj_play_sweep(int f0, int f1, int duration_ms)
{
    const int total = VJ_SAMPLE_RATE * duration_ms / 1000;
    int16_t *buf = malloc(VJ_MIC_CHUNK * sizeof(int16_t));
    if (!buf) return;
    bsp_audio_set_volume(VJ_BEEP_VOLUME);

    float phase = 0.0f;
    int done = 0;
    while (done < total && s_tasks_run) {
        int n = total - done;
        if (n > VJ_MIC_CHUNK) n = VJ_MIC_CHUNK;
        for (int i = 0; i < n; i++) {
            const float progress = (float)(done + i) / (float)total;
            const float f = (float)f0 + ((float)f1 - (float)f0) * progress;
            phase += VJ_TWO_PI * f / (float)VJ_SAMPLE_RATE;
            if (phase > VJ_TWO_PI) phase -= VJ_TWO_PI;
            buf[i] = (int16_t)(sinf(phase) * 6000.0f);
        }
        if (bsp_audio_write(buf, (size_t)n * sizeof(int16_t)) != ESP_OK) break;
        done += n;
    }
    free(buf);
}

static void vj_audio_task(void *arg)
{
    (void)arg;
    // 格式设置在本任务内独占完成,避免与其他任务竞争 codec 状态。
    if (bsp_audio_set_format(VJ_SAMPLE_RATE, 16, 1) != ESP_OK) {
        ESP_LOGE(TAG, "音频格式设置失败,声控不可用");
        if (bsp_lvgl_lock(500)) {
            if (s_ready_hint) lv_label_set_text(s_ready_hint, "AUDIO UNAVAILABLE");
            bsp_lvgl_unlock();
        }
        xSemaphoreGive(s_stopped);
        s_audio_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    int16_t *buf = malloc(VJ_MIC_CHUNK * sizeof(int16_t));
    if (!buf) {
        xSemaphoreGive(s_stopped);
        s_audio_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (s_tasks_run) {
        if (s_audio_cmd == VJ_AUDIO_BEEP_START) {
            s_audio_cmd = VJ_AUDIO_NONE;
            vj_play_sweep(880, 880, 90);
        } else if (s_audio_cmd == VJ_AUDIO_BEEP_DEATH) {
            s_audio_cmd = VJ_AUDIO_NONE;
            vj_play_sweep(500, 180, 350);
        } else if (bsp_audio_read(buf, VJ_MIC_CHUNK * sizeof(int16_t)) == ESP_OK) {
            const float rms = vj_rms_of(buf, VJ_MIC_CHUNK);
            xQueueOverwrite(s_level_queue, &rms); // 深度 1:永远只留最新值
        } else {
            vTaskDelay(pdMS_TO_TICKS(20)); // 读失败退避,避免忙转
        }
    }

    free(buf);
    xSemaphoreGive(s_stopped);
    s_audio_task = NULL;
    vTaskDelete(NULL);
}

// ---- 渲染(内部自取 LVGL 锁) ----

static void vj_render(int tick)
{
    if (!bsp_lvgl_lock(500)) return;

    // 玩家与障碍:位置每帧同步(LVGL 对未变化的 set_pos 会短路)
    lv_obj_set_y(s_player, (int)s_game.y);
    for (int i = 0; i < VJ_OBST_MAX; i++) {
        lv_obj_t *ui = s_obst_ui[i];
        const vj_obstacle_t *o = &s_game.obst[i];
        if (!o->active) {
            if (s_obst_shown[i]) lv_obj_add_flag(ui, LV_OBJ_FLAG_HIDDEN);
            s_obst_shown[i] = false;
            continue;
        }
        if (!s_obst_shown[i] || s_obst_floating[i] != o->floating) {
            // 颜色只在出现/种类变化时写,避免每帧触发样式失效
            lv_obj_set_style_bg_color(ui,
                lv_color_hex(o->floating ? UI_ORANGE : 0x5A6B7A), 0);
            s_obst_floating[i] = o->floating;
        }
        s_obst_shown[i] = true;
        lv_obj_remove_flag(ui, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(ui, (int)o->x, (int)o->y);
        lv_obj_set_size(ui, (int)o->w, (int)o->h);
    }

    // HUD 文本:只在数值变化时刷新
    if (s_game.score != s_shown_score) {
        s_shown_score = s_game.score;
        lv_label_set_text_fmt(s_score_label, "SCORE %lu",
                              (unsigned long)s_game.score);
    }
    if (s_game.best != s_shown_best) {
        s_shown_best = s_game.best;
        lv_label_set_text_fmt(s_best_label, "BEST %lu",
                              (unsigned long)s_game.best);
        lv_label_set_text_fmt(s_ready_best, "BEST %lu",
                              (unsigned long)s_game.best);
    }

    // 电量:约定显示在右上角,读不到时优雅隐藏
    if (tick % VJ_BATTERY_EVERY_TICKS == 0) {
        const int soc = bsp_battery_soc();
        if (soc < 0) {
            lv_obj_add_flag(s_battery_label, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(s_battery_label, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text_fmt(s_battery_label, "%d%%", soc);
        }
    }

    // 音量条:底端对齐,高度随包络
    int fill_h = (int)(s_game.level * (float)VJ_BAR_H);
    if (fill_h < 1) fill_h = 1;
    lv_obj_set_size(s_bar_fill, VJ_BAR_W - 4, fill_h);
    lv_obj_set_pos(s_bar_fill, 2, VJ_BAR_H - fill_h);
    lv_obj_set_style_bg_color(s_bar_fill,
        lv_color_hex(s_game.level > 0.8f ? UI_RED : UI_YELLOW), 0);

    // 面板可见性随状态切换(add/remove flag 幂等且廉价)
    if (s_game.state == VJ_STATE_READY) {
        lv_obj_remove_flag(s_ready_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_ready_shadow, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dead_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dead_shadow, LV_OBJ_FLAG_HIDDEN);
    } else if (s_game.state == VJ_STATE_DEAD) {
        lv_obj_add_flag(s_ready_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ready_shadow, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_dead_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_dead_shadow, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ready_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ready_shadow, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dead_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dead_shadow, LV_OBJ_FLAG_HIDDEN);
    }

    bsp_lvgl_unlock();
}

// 死亡结算文本只在状态切换的瞬间更新一次。
static void vj_update_dead_panel(void)
{
    if (!bsp_lvgl_lock(500)) return;
    lv_label_set_text_fmt(s_dead_score, "SCORE %lu", (unsigned long)s_game.score);
    lv_label_set_text_fmt(s_dead_best, "BEST %lu", (unsigned long)s_game.best);
    lv_obj_set_style_text_color(s_dead_extra,
        lv_color_hex(s_game.new_best ? UI_RED : UI_INK), 0);
    lv_label_set_text(s_dead_extra, s_game.new_best ? "NEW BEST!" : "OK: RETRY");
    bsp_lvgl_unlock();
}

// ---- 游戏任务 ----

static void vj_game_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    int tick = 0;

    while (s_tasks_run) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(VJ_TICK_MS));
        tick++;

        float rms = 0.0f;
        if (s_level_queue) xQueuePeek(s_level_queue, &rms, 0);

        if (s_action_start) {
            s_action_start = false;
            if (s_game.state == VJ_STATE_READY || s_game.state == VJ_STATE_DEAD) {
                if (s_game.state == VJ_STATE_DEAD) vj_game_reset(&s_game);
                vj_game_start(&s_game);
                s_audio_cmd = VJ_AUDIO_BEEP_START;
            }
        }

        const vj_state_t prev = s_game.state;
        vj_game_update(&s_game, VJ_TICK_MS, rms);

        if (prev == VJ_STATE_PLAYING && s_game.state == VJ_STATE_DEAD) {
            if (s_game.new_best) vj_save_best(s_game.best);
            s_audio_cmd = VJ_AUDIO_BEEP_DEATH;
            vj_update_dead_panel();
        }

        vj_render(tick);
    }

    xSemaphoreGive(s_stopped);
    s_game_task = NULL;
    vTaskDelete(NULL);
}

// ---- demo 页面接口 ----

// 圆滚滚的「跳跳酱」:橙色身体、白眼黑瞳、红腮红,全部由色块拼成。
static lv_obj_t *vj_player_create(lv_obj_t *parent)
{
    lv_obj_t *m = lv_obj_create(parent);
    lv_obj_remove_flag(m, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(m, VJ_PLAYER_X, VJ_GROUND_Y - VJ_PLAYER_H);
    lv_obj_set_size(m, VJ_PLAYER_W, VJ_PLAYER_H);
    lv_obj_set_style_bg_opa(m, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(m, 0, 0);
    lv_obj_set_style_pad_all(m, 0, 0);

    vj_block(m, 1, 5, 20, 15, UI_INK);   // 身体描边
    vj_block(m, 3, 7, 16, 11, UI_ORANGE); // 身体
    vj_block(m, 4, 3, 14, 5, UI_INK);    // 头顶描边
    vj_block(m, 5, 4, 12, 3, UI_ORANGE); // 头顶
    vj_block(m, 6, 8, 4, 4, 0xFFFFFF);   // 左眼白
    vj_block(m, 13, 8, 4, 4, 0xFFFFFF);  // 右眼白
    vj_block(m, 7, 9, 2, 2, UI_INK);     // 左瞳
    vj_block(m, 14, 9, 2, 2, UI_INK);     // 右瞳
    vj_block(m, 9, 13, 5, 2, UI_INK);    // 嘴
    vj_block(m, 3, 16, 3, 3, UI_RED);     // 左腮红
    vj_block(m, 17, 16, 3, 3, UI_RED);    // 右腮红
    vj_block(m, 2, 18, 6, 3, UI_INK);     // 左脚
    vj_block(m, 14, 18, 6, 3, UI_INK);    // 右脚
    return m;
}

static lv_obj_t *vj_centered_line(lv_obj_t *panel, const char *text,
                                   const lv_font_t *font, uint32_t color, int y)
{
    lv_obj_t *label = ui_pixel_label(panel, text, font, color);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
    return label;
}

void demo_voice_jump_enter(void)
{
    s_scr = ui_pixel_screen_create("VOICE JUMPER");

    // 场景元素
    s_player = vj_player_create(s_scr);
    for (int i = 0; i < VJ_OBST_MAX; i++) {
        lv_obj_t *ui = vj_block(s_scr, -40, -40, 10, 10, 0x5A6B7A);
        lv_obj_set_style_border_color(ui, lv_color_hex(UI_INK), 0);
        lv_obj_set_style_border_width(ui, 3, 0);
        lv_obj_add_flag(ui, LV_OBJ_FLAG_HIDDEN);
        s_obst_ui[i] = ui;
        s_obst_shown[i] = false;
        s_obst_floating[i] = false;
    }

    // HUD
    s_score_label = ui_pixel_label(s_scr, "SCORE 0", &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_set_pos(s_score_label, 8, 44);
    s_best_label = ui_pixel_label(s_scr, "BEST 0", &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_set_pos(s_best_label, 104, 44);
    s_battery_label = ui_pixel_label(s_scr, "--%", &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_align(s_battery_label, LV_ALIGN_TOP_RIGHT, -8, 44);

    // 音量条(墨色描边框 + 动态填充,底端对齐)
    lv_obj_t *frame = lv_obj_create(s_scr);
    lv_obj_remove_flag(frame, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(frame, VJ_BAR_X, VJ_BAR_Y);
    lv_obj_set_size(frame, VJ_BAR_W, VJ_BAR_H);
    lv_obj_set_style_radius(frame, 0, 0);
    lv_obj_set_style_bg_opa(frame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(frame, lv_color_hex(UI_INK), 0);
    lv_obj_set_style_border_width(frame, 2, 0);
    lv_obj_set_style_pad_all(frame, 0, 0);
    s_bar_fill = vj_block(frame, 2, VJ_BAR_H - 1, VJ_BAR_W - 4, 1, UI_YELLOW);

    // READY 面板(手动创建阴影+面板,以便一起隐藏)
    s_ready_shadow = vj_block(s_scr, VJ_PANEL_X + 5, VJ_PANEL_Y + 6,
                               VJ_PANEL_W, VJ_PANEL_H, UI_INK);
    s_ready_panel = vj_block(s_scr, VJ_PANEL_X, VJ_PANEL_Y,
                              VJ_PANEL_W, VJ_PANEL_H, UI_PAPER);
    lv_obj_set_style_border_color(s_ready_panel, lv_color_hex(UI_INK), 0);
    lv_obj_set_style_border_width(s_ready_panel, 4, 0);
    lv_obj_set_style_pad_all(s_ready_panel, 7, 0);
    vj_centered_line(s_ready_panel, "VOICE JUMPER", &lv_font_montserrat_20, UI_INK, 8);
    vj_centered_line(s_ready_panel, "SHOUT = FLY", &lv_font_montserrat_14, UI_INK, 40);
    vj_centered_line(s_ready_panel, "QUIET = FALL", &lv_font_montserrat_14, UI_INK, 60);
    s_ready_best = vj_centered_line(s_ready_panel, "BEST 0",
                                    &lv_font_montserrat_14, UI_INK, 82);
    s_ready_hint = vj_centered_line(s_ready_panel, "OK: START",
                                    &lv_font_montserrat_14, UI_RED, 104);

    // 结算面板(默认隐藏,手动创建阴影+面板)
    s_dead_shadow = vj_block(s_scr, VJ_PANEL_X + 5, VJ_PANEL_Y + 6,
                              VJ_PANEL_W, VJ_PANEL_H, UI_INK);
    s_dead_panel = vj_block(s_scr, VJ_PANEL_X, VJ_PANEL_Y,
                             VJ_PANEL_W, VJ_PANEL_H, UI_PAPER);
    lv_obj_set_style_border_color(s_dead_panel, lv_color_hex(UI_INK), 0);
    lv_obj_set_style_border_width(s_dead_panel, 4, 0);
    lv_obj_set_style_pad_all(s_dead_panel, 7, 0);
    vj_centered_line(s_dead_panel, "GAME OVER", &lv_font_montserrat_20, UI_INK, 8);
    s_dead_score = vj_centered_line(s_dead_panel, "SCORE 0",
                                    &lv_font_montserrat_14, UI_INK, 44);
    s_dead_best = vj_centered_line(s_dead_panel, "BEST 0",
                                   &lv_font_montserrat_14, UI_INK, 66);
    s_dead_extra = vj_centered_line(s_dead_panel, "OK: RETRY",
                                   &lv_font_montserrat_14, UI_RED, 92);
    lv_obj_add_flag(s_dead_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_dead_shadow, LV_OBJ_FLAG_HIDDEN);

    // 模型初始化并读入历史最高分(音频不可用时页面仍可浏览)
    vj_game_t fresh = {0};
    s_game = fresh;
    vj_game_reset(&s_game);
    s_game.best = vj_load_best();
    s_shown_score = 0xFFFFFFFFu;
    s_shown_best = 0xFFFFFFFFu;
    lv_label_set_text_fmt(s_best_label, "BEST %lu", (unsigned long)s_game.best);
    lv_label_set_text_fmt(s_ready_best, "BEST %lu", (unsigned long)s_game.best);

    lv_screen_load(s_scr);
}

static void vj_free_sync(void)
{
    if (s_stopped) {
        vSemaphoreDelete(s_stopped);
        s_stopped = NULL;
    }
    if (s_level_queue) {
        vQueueDelete(s_level_queue);
        s_level_queue = NULL;
    }
}

// tasks_running = 已成功创建、需要等它退出的任务个数。
static esp_err_t vj_tasks_stop(int tasks_running)
{
    s_tasks_run = false;
    for (int i = 0; i < tasks_running; i++) {
        if (!s_stopped) break;
        if (xSemaphoreTake(s_stopped, pdMS_TO_TICKS(VJ_STOP_TIMEOUT_MS)) != pdTRUE) {
            vj_free_sync();
            return ESP_ERR_TIMEOUT;
        }
    }
    vj_free_sync();
    return ESP_OK;
}

esp_err_t demo_voice_jump_start(void)
{
    if (s_game_task || s_audio_task) return ESP_OK;
    s_action_start = false;
    s_audio_cmd = VJ_AUDIO_NONE;
    s_tasks_run = true;

    s_stopped = xSemaphoreCreateCounting(2, 0);
    s_level_queue = xQueueCreate(1, sizeof(float));
    if (!s_stopped || !s_level_queue) {
        vj_free_sync();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(vj_audio_task, "vj_audio", 3072, NULL, 4, &s_audio_task) != pdPASS) {
        vj_free_sync();
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(vj_game_task, "vj_game", 4096, NULL, 5, &s_game_task) != pdPASS) {
        const esp_err_t err = vj_tasks_stop(1); // 回收已创建的 audio task
        return (err == ESP_OK) ? ESP_ERR_NO_MEM : err;
    }
    return ESP_OK;
}

esp_err_t demo_voice_jump_stop(void)
{
    if (!s_game_task && !s_audio_task) {
        vj_free_sync();
        return ESP_OK;
    }
    return vj_tasks_stop(2);
}

void demo_voice_jump_exit(void)
{
    if (s_scr) {
        lv_obj_delete(s_scr);
        s_scr = NULL;
    }
    s_player = NULL;
    s_score_label = s_best_label = s_battery_label = NULL;
    s_bar_fill = NULL;
    s_ready_panel = s_ready_shadow = s_ready_hint = s_ready_best = NULL;
    s_dead_panel = s_dead_shadow = s_dead_score = s_dead_best = s_dead_extra = NULL;
    for (int i = 0; i < VJ_OBST_MAX; i++) s_obst_ui[i] = NULL;
}

void demo_voice_jump_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    // 只做标志置位,真正动作在 game task 里执行,按键路径保持零阻塞。
    if (ev == BSP_BTN_CLICK && btn == BSP_BTN_OK) {
        s_action_start = true;
    }
}
