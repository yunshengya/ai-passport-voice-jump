// main/demo_music_editor.c —— 音乐编辑器:简谱作曲 + 播放。
//
// 线程模型:
//   * audio task:主工作线程。每帧渲染 PCM → 写 I2S → 持 LVGL 锁更新 UI。
//     编辑模式下也保持运行,写入音符时即时响一声做反馈。
//   * key():只置 volatile 标志,零阻塞,由 audio task 消费。
//
// 按键映射:
//   编辑模式:
//     UP   短按 = 音高 +1          长按 = 清空全部(需确认)
//     DOWN 短按 = 音高 -1          双击 = 删除最后一个
//     OK   短按 = 写入当前音符     长按 = 切换到播放模式
//   播放模式:
//     UP   短按 = BPM +30
//     DOWN 短按 = BPM -30
//     OK   短按 = 播放/暂停        长按 = 切回编辑模式
//
// NVS:保存 3 首草稿(音符 + BPM),断电不丢。
#include "demo.h"
#include "music_editor_model.h"
#include "bsp_audio.h"
#include "bsp_display.h"
#include "ui_pixel.h"

#include "lvgl.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "demo_music_editor";

#define ME_CHUNK_SAMPLES    512
#define ME_TICK_MS          32
#define ME_STOP_TIMEOUT_MS  2000
#define ME_VOLUME_PCT       80

// NVS
#define ME_NVS_NS           "mused"
#define ME_NVS_KEY_SONG(i)  ("s" #i)
#define ME_NVS_KEY_BPM(i)   ("b" #i)

// UI 几何
#define ME_TITLE_H          28
#define ME_STAFF_TOP        36
#define ME_STAFF_H          80
#define ME_STAFF_LINE_CNT   5       // 五线谱
#define ME_NOTE_DISPLAY     8       // 一屏显示 8 个音符
#define ME_BIG_NOTE_TOP     124
#define ME_BIG_NOTE_H       60
#define ME_INFO_TOP         188
#define ME_INFO_H           40
#define ME_CONTROLS_TOP     232
#define ME_CONTROLS_H       40
#define ME_SONG_TAB_TOP     276
#define ME_SONG_TAB_H       28
#define ME_HINT_TOP         306

// 音符颜色(每个音高一个颜色)
static const uint32_t s_note_colors[] = {
    UI_MUTED,    // 0 = 休止(灰色)
    0xFF6B6B,    // 1 = 哆(红)
    0xFFA94D,    // 2 = 来(橙)
    0xFFD93D,    // 3 = 咪(黄)
    0x6BCB77,    // 4 = 发(绿)
    0x4D96FF,    // 5 = 唆(蓝)
    0x9B59B6,    // 6 = 拉(紫)
    0xE056FD,    // 7 = 西(粉)
};

static const char *s_pitch_names[] = {
    "休", "哆", "来", "咪", "发", "唆", "拉", "西"
};

// ---- 按键命令 ----
typedef enum {
    ME_CMD_NONE = 0,
    ME_CMD_UP,
    ME_CMD_DOWN,
    ME_CMD_OK_CLICK,
    ME_CMD_OK_LONG,
    ME_CMD_UP_LONG,
    ME_CMD_DOWN_DOUBLE,
} me_cmd_t;

// ---- 全局状态 ----
static lv_obj_t *s_scr;
static lv_obj_t *s_staff_lines[ME_STAFF_LINE_CNT];  // 五线谱线
static lv_obj_t *s_note_dots[ME_NOTE_DISPLAY];      // 音符显示点
static lv_obj_t *s_note_labels[ME_NOTE_DISPLAY];    // 音符数字标签
static lv_obj_t *s_big_note;                        // 当前选中的大音符
static lv_obj_t *s_big_label;                       // 大音符文字
static lv_obj_t *s_info_label;                      // 信息行(BPM/长度)
static lv_obj_t *s_mode_label;                      // 模式指示
static lv_obj_t *s_play_pos_bar;                    // 播放进度条
static lv_obj_t *s_song_tabs[ME_SONG_COUNT];        // 草稿标签
static lv_obj_t *s_hint_label;                      // 操作提示

static TaskHandle_t s_audio_task_handle;
static SemaphoreHandle_t s_stopped;
static volatile bool s_run;
static volatile me_cmd_t s_pending_cmd;

static me_editor_t s_editor;

// 确认对话框状态
static bool s_confirm_clear;  // 是否在"清空确认"状态
static int  s_confirm_timer;  // 确认倒计时(帧)

// 双击检测
static uint32_t s_last_down_tick;

// === UI 工具 ===

static void s_format_info(char *buf, int buf_len)
{
    const me_song_t *s = me_current_song(&s_editor);
    snprintf(buf, buf_len, "BPM:%u  %d/%d 音符",
             s->bpm, s->count, ME_MAX_NOTES);
}

static void s_update_staff(void)
{
    if (!bsp_lvgl_lock(20)) return;

    const me_song_t *song = me_current_song(&s_editor);
    int count = song->count;
    int start = 0;

    // 如果超过一屏,滚动显示最后 8 个
    if (count > ME_NOTE_DISPLAY) {
        start = count - ME_NOTE_DISPLAY;
    }

    // 当前播放位置(播放模式下高亮)
    int play_pos = -1;
    if (me_mode(&s_editor) == ME_MODE_PLAY && me_is_playing(&s_editor)) {
        play_pos = me_play_position(&s_editor);
    }

    for (int i = 0; i < ME_NOTE_DISPLAY; i++) {
        int idx = start + i;
        if (idx < count) {
            uint8_t pitch = song->notes[idx].pitch;
            uint32_t color = s_note_colors[pitch];
            const char *name = s_pitch_names[pitch];

            lv_obj_clear_flag(s_note_dots[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_note_labels[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(s_note_dots[i], lv_color_hex(color), 0);
            lv_label_set_text(s_note_labels[i], name);

            // 播放中高亮当前音符
            if (idx == play_pos) {
                lv_obj_set_style_border_color(s_note_dots[i], lv_color_hex(UI_YELLOW), 0);
                lv_obj_set_style_border_width(s_note_dots[i], 2, 0);
            } else {
                lv_obj_set_style_border_width(s_note_dots[i], 0, 0);
            }
        } else {
            lv_obj_add_flag(s_note_dots[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_note_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    bsp_lvgl_unlock();
}

static void s_update_big_note(void)
{
    if (!bsp_lvgl_lock(20)) return;

    int pitch = me_selected_pitch(&s_editor);
    uint32_t color = s_note_colors[pitch];

    lv_obj_set_style_bg_color(s_big_note, lv_color_hex(color), 0);
    lv_label_set_text(s_big_label, s_pitch_names[pitch]);

    bsp_lvgl_unlock();
}

static void s_update_info(void)
{
    if (!bsp_lvgl_lock(20)) return;

    char buf[64];
    s_format_info(buf, sizeof buf);
    lv_label_set_text(s_info_label, buf);

    bsp_lvgl_unlock();
}

static void s_update_mode(void)
{
    if (!bsp_lvgl_lock(20)) return;

    const char *mode_str;
    uint32_t mode_color;
    if (me_mode(&s_editor) == ME_MODE_EDIT) {
        mode_str = "✎ 编辑模式";
        mode_color = UI_YELLOW;
    } else {
        mode_str = me_is_playing(&s_editor) ? "▶ 播放中" : "⏸ 已暂停";
        mode_color = UI_GRASS;
    }
    lv_label_set_text(s_mode_label, mode_str);
    lv_obj_set_style_text_color(s_mode_label, lv_color_hex(mode_color), 0);

    bsp_lvgl_unlock();
}

static void s_update_song_tabs(void)
{
    if (!bsp_lvgl_lock(20)) return;

    int cur = me_song_index(&s_editor);
    for (int i = 0; i < ME_SONG_COUNT; i++) {
        if (i == cur) {
            lv_obj_set_style_bg_color(s_song_tabs[i], lv_color_hex(UI_ORANGE), 0);
            lv_obj_set_style_text_color(s_song_tabs[i], lv_color_hex(UI_INK), 0);
        } else {
            lv_obj_set_style_bg_color(s_song_tabs[i], lv_color_hex(UI_INK), 0);
            lv_obj_set_style_text_color(s_song_tabs[i], lv_color_hex(UI_MUTED), 0);
        }
    }

    bsp_lvgl_unlock();
}

static void s_update_hint(void)
{
    if (!bsp_lvgl_lock(20)) return;

    const char *hint;
    if (s_confirm_clear) {
        hint = "再按UP确认清空 / DOWN取消";
    } else if (me_mode(&s_editor) == ME_MODE_EDIT) {
        hint = "↑↓选音  OK写入  长按OK播放";
    } else {
        hint = "↑↓调BPM  OK播放/暂停  长按OK编辑";
    }
    lv_label_set_text(s_hint_label, hint);

    bsp_lvgl_unlock();
}

static void s_update_play_progress(void)
{
    if (!bsp_lvgl_lock(20)) return;

    if (me_mode(&s_editor) == ME_MODE_PLAY) {
        int pos = me_play_position(&s_editor);
        int total = me_note_count(&s_editor);
        int w = 0;
        if (total > 0) {
            w = (pos * 200) / total;
        }
        lv_obj_set_width(s_play_pos_bar, w);
        lv_obj_clear_flag(s_play_pos_bar, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_play_pos_bar, LV_OBJ_FLAG_HIDDEN);
    }

    bsp_lvgl_unlock();
}

// === 播放即时音效(写入音符时的"叮") ===

static bool s_tone_playing;
static float s_tone_elapsed_ms;
static float s_tone_phase;
static float s_tone_freq;
static float s_tone_env_ms;

static void s_start_tone(uint8_t pitch)
{
    s_tone_playing = true;
    s_tone_elapsed_ms = 0;
    s_tone_phase = 0;
    s_tone_freq = me_pitch_freq(pitch);
    s_tone_env_ms = 0;
}

static uint32_t s_render_tone(int16_t *pcm, uint32_t max_samples)
{
    if (!s_tone_playing) return 0;

    uint32_t produced = 0;
    float sample_ms = 1000.0f / (float)ME_SAMPLE_RATE;
    #define TONE_DURATION_MS  200.0f
    #define TONE_ATTACK_MS    5.0f
    #define TONE_DECAY_MS     150.0f
    #define TONE_SUSTAIN_LVL  0.2f
    #define TONE_RELEASE_MS   20.0f

    while (produced < max_samples && s_tone_elapsed_ms < TONE_DURATION_MS) {
        // 包络
        float env;
        float remain = TONE_DURATION_MS - s_tone_elapsed_ms;
        if (remain < TONE_RELEASE_MS) {
            env = TONE_SUSTAIN_LVL * (remain / TONE_RELEASE_MS);
        } else if (s_tone_env_ms < TONE_ATTACK_MS) {
            env = s_tone_env_ms / TONE_ATTACK_MS;
        } else {
            float t_decay = s_tone_env_ms - TONE_ATTACK_MS;
            if (t_decay < TONE_DECAY_MS) {
                float p = t_decay / TONE_DECAY_MS;
                env = TONE_SUSTAIN_LVL + (1.0f - TONE_SUSTAIN_LVL) * powf(0.02f, p);
            } else {
                env = TONE_SUSTAIN_LVL;
            }
        }

        float sq = (s_tone_phase < 0.5f) ? 1.0f : -1.0f;
        float sample = sq * env * (float)ME_VOLUME_MAX;
        s_tone_phase += s_tone_freq / (float)ME_SAMPLE_RATE;
        if (s_tone_phase >= 1.0f) s_tone_phase -= 1.0f;

        if (sample > 32767.0f) sample = 32767.0f;
        if (sample < -32768.0f) sample = -32768.0f;
        pcm[produced++] = (int16_t)sample;

        s_tone_elapsed_ms += sample_ms;
        s_tone_env_ms += sample_ms;
    }

    if (s_tone_elapsed_ms >= TONE_DURATION_MS) {
        s_tone_playing = false;
    }

    return produced;
}

// === NVS ===

static void s_save_song(int idx)
{
    nvs_handle_t h;
    if (nvs_open(ME_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    const me_song_t *s = &s_editor.songs[idx];
    char key[8];

    snprintf(key, sizeof key, "s%d", idx);
    nvs_set_blob(h, key, s->notes, sizeof(me_note_t) * s->count);

    snprintf(key, sizeof key, "c%d", idx);
    nvs_set_u8(h, key, s->count);

    snprintf(key, sizeof key, "b%d", idx);
    nvs_set_u16(h, key, s->bpm);

    nvs_commit(h);
    nvs_close(h);
}

static void s_load_song(int idx)
{
    nvs_handle_t h;
    if (nvs_open(ME_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;

    me_song_t *s = &s_editor.songs[idx];
    char key[8];
    size_t len = sizeof(me_note_t) * ME_MAX_NOTES;

    snprintf(key, sizeof key, "c%d", idx);
    uint8_t count = 0;
    if (nvs_get_u8(h, key, &count) == ESP_OK && count <= ME_MAX_NOTES) {
        s->count = count;
    }

    if (s->count > 0) {
        snprintf(key, sizeof key, "s%d", idx);
        len = sizeof(me_note_t) * s->count;
        nvs_get_blob(h, key, s->notes, &len);
    }

    snprintf(key, sizeof key, "b%d", idx);
    uint16_t bpm = 0;
    if (nvs_get_u16(h, key, &bpm) == ESP_OK && bpm >= ME_BPM_MIN && bpm <= ME_BPM_MAX) {
        s->bpm = bpm;
    }

    nvs_close(h);
}

// === 命令处理 ===

static void s_handle_cmd(void)
{
    me_cmd_t cmd = s_pending_cmd;
    if (cmd == ME_CMD_NONE) return;
    s_pending_cmd = ME_CMD_NONE;

    if (me_mode(&s_editor) == ME_MODE_EDIT) {
        switch (cmd) {
        case ME_CMD_UP:
            if (s_confirm_clear) {
                // 确认清空
                me_clear_song(&s_editor);
                s_save_song(me_song_index(&s_editor));
                s_confirm_clear = false;
                s_update_staff();
                s_update_info();
                s_update_hint();
            } else {
                me_pitch_up(&s_editor);
                s_update_big_note();
                s_start_tone((uint8_t)me_selected_pitch(&s_editor));
            }
            break;
        case ME_CMD_DOWN:
            if (s_confirm_clear) {
                s_confirm_clear = false;
                s_update_hint();
            } else {
                me_pitch_down(&s_editor);
                s_update_big_note();
                s_start_tone((uint8_t)me_selected_pitch(&s_editor));
            }
            break;
        case ME_CMD_OK_CLICK:
            if (me_append_note(&s_editor)) {
                s_start_tone((uint8_t)me_selected_pitch(&s_editor));
                s_update_staff();
                s_update_info();
                s_save_song(me_song_index(&s_editor));
            }
            break;
        case ME_CMD_OK_LONG:
            me_set_mode(&s_editor, ME_MODE_PLAY);
            s_update_mode();
            s_update_hint();
            s_update_play_progress();
            break;
        case ME_CMD_UP_LONG:
            if (!s_confirm_clear && me_note_count(&s_editor) > 0) {
                s_confirm_clear = true;
                s_update_hint();
            }
            break;
        case ME_CMD_DOWN_DOUBLE:
            if (me_delete_last(&s_editor)) {
                s_update_staff();
                s_update_info();
                s_save_song(me_song_index(&s_editor));
            }
            break;
        default:
            break;
        }
    } else {
        // 播放模式
        switch (cmd) {
        case ME_CMD_UP:
            me_bpm_up(&s_editor);
            s_update_info();
            s_save_song(me_song_index(&s_editor));
            break;
        case ME_CMD_DOWN:
            me_bpm_down(&s_editor);
            s_update_info();
            s_save_song(me_song_index(&s_editor));
            break;
        case ME_CMD_OK_CLICK:
            if (me_is_playing(&s_editor)) {
                me_play(&s_editor, false);
            } else {
                me_play(&s_editor, true);
            }
            s_update_mode();
            break;
        case ME_CMD_OK_LONG:
            me_set_mode(&s_editor, ME_MODE_EDIT);
            s_update_mode();
            s_update_hint();
            s_update_play_progress();
            s_update_staff();
            break;
        default:
            break;
        }
    }
}

// === 音频任务 ===

static void s_audio_task(void *arg)
{
    (void)arg;

    if (bsp_audio_init() != ESP_OK) {
        ESP_LOGE(TAG, "bsp_audio_init failed");
        xSemaphoreGive(s_stopped);
        s_audio_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (bsp_audio_set_format(ME_SAMPLE_RATE, 16, 1) != ESP_OK) {
        ESP_LOGE(TAG, "audio format set failed");
        xSemaphoreGive(s_stopped);
        s_audio_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    bsp_audio_set_volume(ME_VOLUME_PCT);

    int16_t pcm_buf[ME_CHUNK_SAMPLES];
    TickType_t last_wake = xTaskGetTickCount();
    int last_note_count = -1;
    int last_play_pos = -1;
    bool last_playing = false;
    me_mode_t last_mode = ME_MODE_EDIT;

    while (s_run) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ME_TICK_MS));

        // 处理命令
        s_handle_cmd();

        // 清缓冲区
        memset(pcm_buf, 0, sizeof(pcm_buf));
        uint32_t total_got = 0;

        // 渲染旋律播放
        if (me_mode(&s_editor) == ME_MODE_PLAY && me_is_playing(&s_editor)) {
            uint32_t got = me_render(&s_editor, pcm_buf, ME_CHUNK_SAMPLES);
            total_got = got;
        }

        // 叠加即时音效(写入反馈)
        if (s_tone_playing) {
            int16_t tone_buf[ME_CHUNK_SAMPLES];
            memset(tone_buf, 0, sizeof(tone_buf));
            uint32_t tone_got = s_render_tone(tone_buf, ME_CHUNK_SAMPLES);
            // 混音
            for (uint32_t i = 0; i < tone_got; i++) {
                int32_t mix = (int32_t)pcm_buf[i] + (int32_t)tone_buf[i];
                if (mix > 32767) mix = 32767;
                if (mix < -32768) mix = -32768;
                pcm_buf[i] = (int16_t)mix;
            }
            if (tone_got > total_got) total_got = tone_got;
        }

        // 写音频
        if (total_got > 0) {
            bsp_audio_write(pcm_buf, total_got * sizeof(int16_t));
        }

        // UI 更新(只在变化时)
        int cur_count = me_note_count(&s_editor);
        int cur_pos = me_play_position(&s_editor);
        bool cur_playing = me_is_playing(&s_editor);
        me_mode_t cur_mode = me_mode(&s_editor);

        if (cur_count != last_note_count) {
            s_update_staff();
            s_update_info();
            last_note_count = cur_count;
        }

        if (cur_mode != last_mode || cur_playing != last_playing) {
            s_update_mode();
            s_update_play_progress();
            last_mode = cur_mode;
            last_playing = cur_playing;
        }

        if (cur_mode == ME_MODE_PLAY && cur_pos != last_play_pos) {
            s_update_staff();
            s_update_play_progress();
            last_play_pos = cur_pos;
        }
    }

    xSemaphoreGive(s_stopped);
    s_audio_task_handle = NULL;
    vTaskDelete(NULL);
}

// === UI 创建 ===

void demo_music_editor_enter(void)
{
    s_scr = ui_pixel_screen_create("Music Editor");
    lv_obj_set_style_bg_color(s_scr, lv_color_hex(UI_INK), 0);

    // ---- 五线谱 ----
    for (int i = 0; i < ME_STAFF_LINE_CNT; i++) {
        lv_obj_t *line = lv_obj_create(s_scr);
        lv_obj_remove_flag(line, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(line, 220, 1);
        lv_obj_set_pos(line, 10, ME_STAFF_TOP + 10 + i * 14);
        lv_obj_set_style_bg_color(line, lv_color_hex(UI_MUTED), 0);
        lv_obj_set_style_border_width(line, 0, 0);
        lv_obj_set_style_pad_all(line, 0, 0);
        lv_obj_set_style_radius(line, 0, 0);
        s_staff_lines[i] = line;
    }

    // ---- 音符显示点 ----
    int note_y = ME_STAFF_TOP + 40;  // 中间位置
    int note_w = 24;
    int note_h = 24;
    int note_gap = (220 - note_w * ME_NOTE_DISPLAY) / (ME_NOTE_DISPLAY + 1);

    for (int i = 0; i < ME_NOTE_DISPLAY; i++) {
        int x = 10 + note_gap + i * (note_w + note_gap);

        lv_obj_t *dot = lv_obj_create(s_scr);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(dot, note_w, note_h);
        lv_obj_set_pos(dot, x, note_y);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(UI_YELLOW), 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_pad_all(dot, 0, 0);
        s_note_dots[i] = dot;

        lv_obj_t *label = ui_pixel_label(s_scr, "1", &lv_font_montserrat_14, UI_INK);
        lv_obj_set_pos(label, x + note_w / 2 - 6, note_y + 4);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        s_note_labels[i] = label;
    }

    // ---- 大音符(当前选中) ----
    s_big_note = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_big_note, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(s_big_note, 60, 60);
    lv_obj_set_pos(s_big_note, 120 - 30, ME_BIG_NOTE_TOP);
    lv_obj_set_style_radius(s_big_note, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_big_note, lv_color_hex(s_note_colors[1]), 0);
    lv_obj_set_style_border_width(s_big_note, 2, 0);
    lv_obj_set_style_border_color(s_big_note, lv_color_hex(UI_PAPER), 0);
    lv_obj_set_style_pad_all(s_big_note, 0, 0);

    s_big_label = ui_pixel_label(s_scr, "哆", &lv_font_montserrat_20, UI_INK);
    lv_obj_set_pos(s_big_label, 120 - 20, ME_BIG_NOTE_TOP + 14);
    lv_obj_set_width(s_big_label, 40);
    lv_obj_set_style_text_align(s_big_label, LV_TEXT_ALIGN_CENTER, 0);

    // ---- 信息行 ----
    s_info_label = ui_pixel_label(s_scr, "BPM:120  0/32 音符",
                                   &lv_font_montserrat_14, UI_PAPER);
    lv_obj_set_pos(s_info_label, 10, ME_INFO_TOP + 5);
    lv_obj_set_width(s_info_label, 220);
    lv_obj_set_style_text_align(s_info_label, LV_TEXT_ALIGN_CENTER, 0);

    // 播放进度条
    s_play_pos_bar = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_play_pos_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_play_pos_bar, 20, ME_INFO_TOP + 28);
    lv_obj_set_size(s_play_pos_bar, 0, 4);
    lv_obj_set_style_bg_color(s_play_pos_bar, lv_color_hex(UI_GRASS), 0);
    lv_obj_set_style_radius(s_play_pos_bar, 2, 0);
    lv_obj_set_style_border_width(s_play_pos_bar, 0, 0);
    lv_obj_set_style_pad_all(s_play_pos_bar, 0, 0);
    lv_obj_add_flag(s_play_pos_bar, LV_OBJ_FLAG_HIDDEN);

    // ---- 模式指示 ----
    s_mode_label = ui_pixel_label(s_scr, "✎ 编辑模式",
                                   &lv_font_montserrat_14, UI_YELLOW);
    lv_obj_set_pos(s_mode_label, 10, ME_CONTROLS_TOP + 5);
    lv_obj_set_width(s_mode_label, 220);
    lv_obj_set_style_text_align(s_mode_label, LV_TEXT_ALIGN_CENTER, 0);

    // ---- 草稿标签 ----
    int tab_w = 64;
    int tab_gap = 8;
    int tab_total = tab_w * ME_SONG_COUNT + tab_gap * (ME_SONG_COUNT - 1);
    int tab_start = 120 - tab_total / 2;

    for (int i = 0; i < ME_SONG_COUNT; i++) {
        lv_obj_t *tab = lv_obj_create(s_scr);
        lv_obj_remove_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(tab, tab_w, ME_SONG_TAB_H - 4);
        lv_obj_set_pos(tab, tab_start + i * (tab_w + tab_gap), ME_SONG_TAB_TOP + 2);
        lv_obj_set_style_radius(tab, 4, 0);
        lv_obj_set_style_bg_color(tab, lv_color_hex(UI_INK), 0);
        lv_obj_set_style_border_width(tab, 0, 0);
        lv_obj_set_style_pad_all(tab, 0, 0);
        s_song_tabs[i] = tab;

        char buf[16];
        snprintf(buf, sizeof buf, "草稿 %d", i + 1);
        lv_obj_t *label = ui_pixel_label(tab, buf, &lv_font_montserrat_14, UI_MUTED);
        lv_obj_set_width(label, tab_w);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_y(label, 4);
    }

    // ---- 操作提示 ----
    s_hint_label = ui_pixel_label(s_scr, "↑↓选音  OK写入  长按OK播放",
                                   &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_pos(s_hint_label, 10, ME_HINT_TOP);
    lv_obj_set_width(s_hint_label, 220);
    lv_obj_set_style_text_align(s_hint_label, LV_TEXT_ALIGN_CENTER, 0);

    // ---- 初始化模型 ----
    me_init(&s_editor);
    s_tone_playing = false;
    s_confirm_clear = false;
    s_last_down_tick = 0;

    // 加载 NVS
    for (int i = 0; i < ME_SONG_COUNT; i++) {
        s_load_song(i);
    }

    // 初始 UI
    s_update_big_note();
    s_update_staff();
    s_update_info();
    s_update_mode();
    s_update_song_tabs();
    s_update_hint();

    lv_screen_load(s_scr);
}

void demo_music_editor_exit(void)
{
    if (s_scr) {
        lv_obj_delete(s_scr);
        s_scr = NULL;
    }
    for (int i = 0; i < ME_STAFF_LINE_CNT; i++) s_staff_lines[i] = NULL;
    for (int i = 0; i < ME_NOTE_DISPLAY; i++) {
        s_note_dots[i] = NULL;
        s_note_labels[i] = NULL;
    }
    s_big_note = NULL;
    s_big_label = NULL;
    s_info_label = NULL;
    s_mode_label = NULL;
    s_play_pos_bar = NULL;
    for (int i = 0; i < ME_SONG_COUNT; i++) s_song_tabs[i] = NULL;
    s_hint_label = NULL;
}

esp_err_t demo_music_editor_start(void)
{
    if (s_audio_task_handle) return ESP_OK;

    s_run = true;
    s_pending_cmd = ME_CMD_NONE;

    s_stopped = xSemaphoreCreateBinary();
    if (!s_stopped) {
        ESP_LOGE(TAG, "cannot create semaphore");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(s_audio_task, "me_audio", 3072, NULL, 4,
                    &s_audio_task_handle) != pdPASS) {
        vSemaphoreDelete(s_stopped);
        s_stopped = NULL;
        ESP_LOGE(TAG, "cannot create audio task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "music editor started");
    return ESP_OK;
}

esp_err_t demo_music_editor_stop(void)
{
    if (!s_audio_task_handle) {
        if (s_stopped) {
            vSemaphoreDelete(s_stopped);
            s_stopped = NULL;
        }
        return ESP_OK;
    }

    s_run = false;
    if (!s_stopped ||
        xSemaphoreTake(s_stopped, pdMS_TO_TICKS(ME_STOP_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "audio task stop timeout");
        if (s_stopped) {
            vSemaphoreDelete(s_stopped);
            s_stopped = NULL;
        }
        return ESP_ERR_TIMEOUT;
    }

    vSemaphoreDelete(s_stopped);
    s_stopped = NULL;
    ESP_LOGI(TAG, "music editor stopped");
    return ESP_OK;
}

void demo_music_editor_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    // 双击检测(DOWN 键)
    if (btn == BSP_BTN_DOWN && ev == BSP_BTN_CLICK) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - s_last_down_tick < 300) {
            s_pending_cmd = ME_CMD_DOWN_DOUBLE;
            s_last_down_tick = 0;
            return;
        }
        s_last_down_tick = now;
    }

    if (ev == BSP_BTN_CLICK) {
        if (btn == BSP_BTN_UP) {
            s_pending_cmd = ME_CMD_UP;
        } else if (btn == BSP_BTN_DOWN) {
            s_pending_cmd = ME_CMD_DOWN;
        } else if (btn == BSP_BTN_OK) {
            s_pending_cmd = ME_CMD_OK_CLICK;
        }
    } else if (ev == BSP_BTN_LONG) {
        if (btn == BSP_BTN_OK) {
            s_pending_cmd = ME_CMD_OK_LONG;
        } else if (btn == BSP_BTN_UP) {
            s_pending_cmd = ME_CMD_UP_LONG;
        }
    }
}
