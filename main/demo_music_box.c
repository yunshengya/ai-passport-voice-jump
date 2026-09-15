// main/demo_music_box.c —— 八音盒音乐播放器:方波合成 + 歌曲列表 + 播放控制。
//
// 线程模型:
//   * music task:主工作线程。每帧渲染 512 样本 PCM(~32ms)→ 写 I2S →
//     持 LVGL 锁更新 UI(进度条、当前音符、CD 旋转)。用 vTaskDelayUntil 定步长。
//   * key():只置 volatile 标志,零阻塞,由 music task 消费。
#include "demo.h"
#include "music_box_model.h"

#include "bsp_audio.h"
#include "bsp_display.h"
#include "ui_pixel.h"

#include "lvgl.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "demo_music_box";

#define MB_CHUNK_SAMPLES    512     // 每帧渲染样本数(32ms @ 16kHz)
#define MB_TICK_MS          32      // 任务周期,与样本量对齐
#define MB_STOP_TIMEOUT_MS  2000
#define MB_VOLUME_PCT       80

// ---- UI 几何 ----
#define MB_TITLE_H          28
#define MB_CD_TOP           28
#define MB_CD_H             140
#define MB_CD_CX            120
#define MB_CD_CY            (MB_CD_TOP + 70)   // 100
#define MB_CD_R             60
#define MB_INFO_TOP         168
#define MB_INFO_H           40
#define MB_PROG_TOP         208
#define MB_PROG_H           12
#define MB_HINT_TOP         220
#define MB_HINT_H           30
#define MB_LIST_TOP         250
#define MB_LIST_H           70
#define MB_LIST_ITEM_H      17      // 70 / 4 ≈ 17

// CD 颜色(紫粉渐变:外圈深紫,内圈亮粉)
#define MB_CD_OUTER         0x7B2D8E  // 深紫
#define MB_CD_INNER         0xE85CA0  // 亮粉
#define MB_CD_HOLE          0xFFFFFF  // 中心孔白色

// ---- 按键命令(由 key() 设置,music task 消费) ----
typedef enum {
    MB_CMD_NONE = 0,
    MB_CMD_PLAY_PAUSE,      // OK 单击:播放/暂停
    MB_CMD_NEXT,            // DOWN 单击:下一首
    MB_CMD_PREV,            // UP 单击:上一首
    MB_CMD_TOGGLE_LOOP,     // OK 长按:切换循环
} mb_cmd_t;

// ---- 全局状态 ----
static lv_obj_t *s_scr;
static lv_obj_t *s_cd_outer;       // CD 外圈(深紫)
static lv_obj_t *s_cd_inner;       // CD 内圈(亮粉)
static lv_obj_t *s_cd_hole;        // CD 中心孔
static lv_obj_t *s_cd_spin;        // 旋转指示弧
static lv_obj_t *s_note1, *s_note2; // 浮动音符
static lv_obj_t *s_song_name;      // 歌曲名
static lv_obj_t *s_song_artist;    // 歌手
static lv_obj_t *s_prog_bg;        // 进度条背景
static lv_obj_t *s_prog_fill;      // 进度条填充
static lv_obj_t *s_time_label;     // 时间文本
static lv_obj_t *s_loop_label;     // 循环指示
static lv_obj_t *s_list_items[MB_SONG_COUNT];  // 歌曲列表项
static lv_obj_t *s_list_names[MB_SONG_COUNT];  // 列表歌名
static lv_obj_t *s_list_artists[MB_SONG_COUNT]; // 列表歌手

static TaskHandle_t s_music_task_handle;
static SemaphoreHandle_t s_stopped;
static volatile bool s_run;
static volatile mb_cmd_t s_pending_cmd;

static mb_synth_t s_synth;
static int32_t s_cd_angle;         // CD 旋转角度(0.1 度单位,LVGL 用)

// ---- 小工具 ----

// 创建一个圆形对象
static lv_obj_t *s_create_circle(lv_obj_t *parent, int cx, int cy, int r,
                                  uint32_t color)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(obj, r * 2, r * 2);
    lv_obj_set_pos(obj, cx - r, cy - r);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    return obj;
}

// 格式化时间 "M:SS"
static void s_format_time(char *buf, int buf_len, uint32_t ms)
{
    uint32_t total_sec = ms / 1000;
    uint32_t min = total_sec / 60;
    uint32_t sec = total_sec % 60;
    snprintf(buf, buf_len, "%lu:%02lu", (unsigned long)min, (unsigned long)sec);
}

// ---- UI 更新(内部自取 LVGL 锁) ----

static void s_update_progress(uint32_t pos_ms, uint32_t total_ms)
{
    if (!bsp_lvgl_lock(20)) return;

    // 进度条宽度
    if (s_prog_bg && s_prog_fill) {
        lv_coord_t bg_w = lv_obj_get_width(s_prog_bg);
        int fill_w = 0;
        if (total_ms > 0) {
            fill_w = (int)((uint64_t)(bg_w - 4) * pos_ms / total_ms);
        }
        if (fill_w < 0) fill_w = 0;
        if (fill_w > bg_w - 4) fill_w = bg_w - 4;
        lv_obj_set_width(s_prog_fill, fill_w);
    }

    // 时间文本
    if (s_time_label) {
        char pos_str[16], total_str[16];
        s_format_time(pos_str, sizeof pos_str, pos_ms);
        s_format_time(total_str, sizeof total_str, total_ms);
        char buf[64];
        snprintf(buf, sizeof buf, "%s / %s", pos_str, total_str);
        lv_label_set_text(s_time_label, buf);
    }

    bsp_lvgl_unlock();
}

static void s_update_song_info(const mb_song_t *song)
{
    if (!bsp_lvgl_lock(20)) return;

    if (s_song_name) lv_label_set_text(s_song_name, song->name);
    if (s_song_artist) lv_label_set_text(s_song_artist, song->artist);

    // 更新列表高亮
    int cur_idx = mb_synth_song_index(&s_synth);
    for (int i = 0; i < MB_SONG_COUNT; i++) {
        if (!s_list_items[i]) continue;
        if (i == cur_idx) {
            lv_obj_set_style_bg_color(s_list_items[i], lv_color_hex(UI_ORANGE), 0);
            lv_obj_set_style_text_color(s_list_names[i], lv_color_hex(UI_INK), 0);
            lv_obj_set_style_text_color(s_list_artists[i], lv_color_hex(UI_INK), 0);
        } else {
            lv_obj_set_style_bg_color(s_list_items[i], lv_color_hex(UI_INK), 0);
            lv_obj_set_style_text_color(s_list_names[i], lv_color_hex(UI_PAPER), 0);
            lv_obj_set_style_text_color(s_list_artists[i], lv_color_hex(UI_MUTED), 0);
        }
    }

    bsp_lvgl_unlock();
}

static void s_update_cd_spin(bool playing)
{
    if (!bsp_lvgl_lock(20)) return;

    if (s_cd_spin && playing) {
        // 每帧前进约 6 度(≈ 188 度/秒,在 32ms tick 下)
        s_cd_angle += 60;  // 0.1 度单位
        if (s_cd_angle >= 3600) s_cd_angle -= 3600;
        lv_arc_set_start_angle(s_cd_spin, s_cd_angle / 10);
        lv_arc_set_end_angle(s_cd_spin, (s_cd_angle / 10) + 30);
    }

    // 音符浮动:播放时上移淡出,回到原位循环
    if (s_note1 && s_note2 && playing) {
        static int note_tick = 0;
        note_tick++;
        // 简单的上下浮动效果
        int offset = (note_tick % 60);
        if (offset > 30) offset = 60 - offset;  // 三角形波
        lv_obj_set_y(s_note1, MB_CD_TOP + 20 - offset);
        lv_obj_set_y(s_note2, MB_CD_TOP + 40 - ((offset + 15) % 30));
    }

    bsp_lvgl_unlock();
}

static void s_update_loop_label(bool loop)
{
    if (!bsp_lvgl_lock(20)) return;
    if (s_loop_label) {
        lv_label_set_text(s_loop_label, loop ? "循环:开" : "循环:关");
        lv_obj_set_style_text_color(s_loop_label,
            lv_color_hex(loop ? UI_YELLOW : UI_MUTED), 0);
    }
    bsp_lvgl_unlock();
}

// ---- 音乐任务 ----

static void s_handle_cmd(void)
{
    mb_cmd_t cmd = s_pending_cmd;
    if (cmd == MB_CMD_NONE) return;
    s_pending_cmd = MB_CMD_NONE;

    const mb_song_t *songs = mb_songs();
    int count = mb_song_count();

    switch (cmd) {
    case MB_CMD_PLAY_PAUSE: {
        bool playing = s_synth.playing;
        mb_synth_play(&s_synth, !playing);
        ESP_LOGI(TAG, "%s", !playing ? "play" : "pause");
        break;
    }
    case MB_CMD_NEXT: {
        int idx = mb_synth_next(&s_synth, songs, count);
        mb_synth_play(&s_synth, true);  // 切歌后自动播放
        ESP_LOGI(TAG, "next song: %d", idx);
        s_update_song_info(mb_song_get(idx));
        break;
    }
    case MB_CMD_PREV: {
        int idx = mb_synth_prev(&s_synth, songs, count);
        mb_synth_play(&s_synth, true);
        ESP_LOGI(TAG, "prev song: %d", idx);
        s_update_song_info(mb_song_get(idx));
        break;
    }
    case MB_CMD_TOGGLE_LOOP: {
        bool loop = !s_synth.loop;
        mb_synth_set_loop(&s_synth, loop);
        ESP_LOGI(TAG, "loop: %s", loop ? "on" : "off");
        s_update_loop_label(loop);
        break;
    }
    default:
        break;
    }
}

static void s_music_task(void *arg)
{
    (void)arg;

    // 初始化音频格式
    if (bsp_audio_init() != ESP_OK) {
        ESP_LOGE(TAG, "bsp_audio_init failed");
        xSemaphoreGive(s_stopped);
        s_music_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (bsp_audio_set_format(MB_SAMPLE_RATE, 16, 1) != ESP_OK) {
        ESP_LOGE(TAG, "audio format set failed");
        xSemaphoreGive(s_stopped);
        s_music_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    bsp_audio_set_volume(MB_VOLUME_PCT);

    int16_t pcm_buf[MB_CHUNK_SAMPLES];
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_shown_pos = 0xFFFFFFFFu;
    int last_shown_song = -1;

    while (s_run) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MB_TICK_MS));

        // 处理按键命令
        s_handle_cmd();

        // 渲染 PCM
        uint32_t got = mb_synth_render(&s_synth, pcm_buf, MB_CHUNK_SAMPLES);
        if (got > 0) {
            bsp_audio_write(pcm_buf, got * sizeof(int16_t));
        }

        // 更新 UI:进度(只在变化时刷新)
        uint32_t pos = mb_synth_position_ms(&s_synth);
        uint32_t total = s_synth.total_ms;
        if (pos != last_shown_pos && total > 0) {
            // 每 100ms 刷新一次,避免过度重绘
            if (pos / 100 != last_shown_pos / 100) {
                s_update_progress(pos, total);
                last_shown_pos = pos;
            }
        }

        // 歌曲切换时刷新信息
        int cur_song = mb_synth_song_index(&s_synth);
        if (cur_song != last_shown_song) {
            const mb_song_t *song = mb_song_get(cur_song);
            if (song) s_update_song_info(song);
            last_shown_song = cur_song;
            last_shown_pos = 0xFFFFFFFFu;  // 强制刷新进度
        }

        // CD 旋转动画
        s_update_cd_spin(s_synth.playing);

        // 非播放状态下也保持进度更新(暂停时也显示当前位置)
        if (!s_synth.playing && last_shown_pos == 0xFFFFFFFFu) {
            s_update_progress(pos, total);
            last_shown_pos = pos;
        }
    }

    xSemaphoreGive(s_stopped);
    s_music_task_handle = NULL;
    vTaskDelete(NULL);
}

// ---- demo 接口 ----

void demo_music_box_enter(void)
{
    s_scr = ui_pixel_screen_create("Music Box");

    // 背景:深色
    lv_obj_set_style_bg_color(s_scr, lv_color_hex(UI_INK), 0);

    // ---- CD 区域 ----
    // 外圈(深紫)
    s_cd_outer = s_create_circle(s_scr, MB_CD_CX, MB_CD_CY, MB_CD_R, MB_CD_OUTER);
    // 内圈(亮粉,模拟渐变)
    s_cd_inner = s_create_circle(s_scr, MB_CD_CX, MB_CD_CY, 42, MB_CD_INNER);
    // 中心孔(白)
    s_cd_hole = s_create_circle(s_scr, MB_CD_CX, MB_CD_CY, 8, MB_CD_HOLE);

    // 旋转指示弧(一段亮色弧,随播放旋转)
    s_cd_spin = lv_arc_create(s_scr);
    lv_obj_remove_flag(s_cd_spin, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(s_cd_spin, MB_CD_R * 2 - 8, MB_CD_R * 2 - 8);
    lv_obj_set_pos(s_cd_spin, MB_CD_CX - MB_CD_R + 4, MB_CD_CY - MB_CD_R + 4);
    lv_arc_set_rotation(s_cd_spin, 0);
    lv_arc_set_start_angle(s_cd_spin, 0);
    lv_arc_set_end_angle(s_cd_spin, 30);
    lv_obj_set_style_arc_color(s_cd_spin, lv_color_hex(UI_YELLOW), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(s_cd_spin, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_cd_spin, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_cd_spin, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cd_spin, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_cd_spin, 0, 0);
    lv_obj_set_style_pad_all(s_cd_spin, 0, 0);
    s_cd_angle = 0;

    // 浮动音符
    s_note1 = ui_pixel_label(s_scr, "♪", &lv_font_montserrat_20, UI_YELLOW);
    lv_obj_set_pos(s_note1, MB_CD_CX - 50, MB_CD_TOP + 20);
    s_note2 = ui_pixel_label(s_scr, "♫", &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_pos(s_note2, MB_CD_CX + 35, MB_CD_TOP + 40);

    // ---- 歌曲信息 ----
    s_song_name = ui_pixel_label(s_scr, "", &lv_font_montserrat_20, UI_PAPER);
    lv_obj_set_style_text_align(s_song_name, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_song_name, 240);
    lv_obj_set_y(s_song_name, MB_INFO_TOP + 2);

    s_song_artist = ui_pixel_label(s_scr, "", &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_style_text_align(s_song_artist, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_song_artist, 240);
    lv_obj_set_y(s_song_artist, MB_INFO_TOP + 22);

    // ---- 进度条 ----
    s_prog_bg = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_prog_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_prog_bg, 20, MB_PROG_TOP);
    lv_obj_set_size(s_prog_bg, 200, MB_PROG_H);
    lv_obj_set_style_radius(s_prog_bg, 2, 0);
    lv_obj_set_style_bg_opa(s_prog_bg, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(s_prog_bg, lv_color_hex(UI_INK), 0);
    lv_obj_set_style_border_width(s_prog_bg, 2, 0);
    lv_obj_set_style_pad_all(s_prog_bg, 0, 0);

    s_prog_fill = lv_obj_create(s_prog_bg);
    lv_obj_remove_flag(s_prog_fill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_prog_fill, 2, 2);
    lv_obj_set_size(s_prog_fill, 0, MB_PROG_H - 4);
    lv_obj_set_style_radius(s_prog_fill, 0, 0);
    lv_obj_set_style_bg_color(s_prog_fill, lv_color_hex(UI_YELLOW), 0);
    lv_obj_set_style_border_width(s_prog_fill, 0, 0);
    lv_obj_set_style_pad_all(s_prog_fill, 0, 0);

    // 时间文本
    s_time_label = ui_pixel_label(s_scr, "0:00 / 0:00", &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_style_text_align(s_time_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_time_label, 240);
    lv_obj_set_y(s_time_label, MB_PROG_TOP + MB_PROG_H + 2);

    // ---- 操作提示 ----
    lv_obj_t *hint = ui_pixel_label(s_scr, "OK 播放/暂停  UP/DOWN 切歌",
                                     &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(hint, 240);
    lv_obj_set_y(hint, MB_HINT_TOP + 8);

    // 循环指示
    s_loop_label = ui_pixel_label(s_scr, "循环:关", &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_style_text_align(s_loop_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_loop_label, 240);
    lv_obj_set_y(s_loop_label, MB_HINT_TOP + 22);

    // ---- 歌曲列表 ----
    const mb_song_t *songs = mb_songs();
    for (int i = 0; i < MB_SONG_COUNT; i++) {
        int y = MB_LIST_TOP + i * MB_LIST_ITEM_H;

        // 列表项背景
        lv_obj_t *item = lv_obj_create(s_scr);
        lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(item, 10, y);
        lv_obj_set_size(item, 220, MB_LIST_ITEM_H - 1);
        lv_obj_set_style_radius(item, 0, 0);
        lv_obj_set_style_bg_color(item, lv_color_hex(UI_INK), 0);
        lv_obj_set_style_border_width(item, 0, 0);
        lv_obj_set_style_pad_all(item, 0, 0);
        s_list_items[i] = item;

        // 歌名(左对齐)
        lv_obj_t *name = ui_pixel_label(item, songs[i].name,
                                         &lv_font_montserrat_14, UI_PAPER);
        lv_obj_set_pos(name, 8, 1);
        s_list_names[i] = name;

        // 歌手(右对齐)
        lv_obj_t *artist = ui_pixel_label(item, songs[i].artist,
                                           &lv_font_montserrat_14, UI_MUTED);
        lv_obj_align(artist, LV_ALIGN_RIGHT_MID, -8, 0);
        s_list_artists[i] = artist;
    }

    // 初始化合成器,加载第一首(不自动播放)
    mb_synth_init(&s_synth);
    mb_synth_load(&s_synth, mb_song_get(0));
    s_pending_cmd = MB_CMD_NONE;
    s_cd_angle = 0;

    // 初始 UI 状态
    s_update_song_info(mb_song_get(0));
    s_update_progress(0, mb_song_duration_ms(mb_song_get(0)));
    s_update_loop_label(false);

    lv_screen_load(s_scr);
}

void demo_music_box_exit(void)
{
    if (s_scr) {
        lv_obj_delete(s_scr);
        s_scr = NULL;
    }
    s_cd_outer = s_cd_inner = s_cd_hole = s_cd_spin = NULL;
    s_note1 = s_note2 = NULL;
    s_song_name = s_song_artist = NULL;
    s_prog_bg = s_prog_fill = NULL;
    s_time_label = s_loop_label = NULL;
    for (int i = 0; i < MB_SONG_COUNT; i++) {
        s_list_items[i] = NULL;
        s_list_names[i] = NULL;
        s_list_artists[i] = NULL;
    }
}

esp_err_t demo_music_box_start(void)
{
    if (s_music_task_handle) return ESP_OK;

    s_run = true;
    s_pending_cmd = MB_CMD_NONE;

    s_stopped = xSemaphoreCreateBinary();
    if (!s_stopped) {
        ESP_LOGE(TAG, "cannot create semaphore");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(s_music_task, "mb_music", 3072, NULL, 4,
                    &s_music_task_handle) != pdPASS) {
        vSemaphoreDelete(s_stopped);
        s_stopped = NULL;
        ESP_LOGE(TAG, "cannot create music task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "music box started");
    return ESP_OK;
}

esp_err_t demo_music_box_stop(void)
{
    if (!s_music_task_handle) {
        if (s_stopped) {
            vSemaphoreDelete(s_stopped);
            s_stopped = NULL;
        }
        return ESP_OK;
    }

    s_run = false;
    if (!s_stopped ||
        xSemaphoreTake(s_stopped, pdMS_TO_TICKS(MB_STOP_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "music task stop timeout");
        if (s_stopped) {
            vSemaphoreDelete(s_stopped);
            s_stopped = NULL;
        }
        return ESP_ERR_TIMEOUT;
    }

    vSemaphoreDelete(s_stopped);
    s_stopped = NULL;
    ESP_LOGI(TAG, "music box stopped");
    return ESP_OK;
}

void demo_music_box_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    // 只置标志,真正动作在 music task 里执行,按键路径零阻塞
    if (ev == BSP_BTN_CLICK) {
        if (btn == BSP_BTN_OK) {
            s_pending_cmd = MB_CMD_PLAY_PAUSE;
        } else if (btn == BSP_BTN_UP) {
            s_pending_cmd = MB_CMD_PREV;
        } else if (btn == BSP_BTN_DOWN) {
            s_pending_cmd = MB_CMD_NEXT;
        }
    } else if (ev == BSP_BTN_LONG && btn == BSP_BTN_OK) {
        s_pending_cmd = MB_CMD_TOGGLE_LOOP;
    }
}
