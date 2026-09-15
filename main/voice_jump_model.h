// main/voice_jump_model.h —— 声控跳跳酱的纯逻辑游戏模型。
//
// 本模块只做状态机 + 物理 + 障碍 + 音量包络,不包含任何 ESP-IDF / LVGL 依赖,
// 因此可以在主机上用 tests/test_voice_jump_model.c 直接做单元测试(见
// docs/development/ai-guide.md 的「可测试状态机与实现分离」约定)。
//
// 坐标系:屏幕坐标,左上为原点,240x320 竖屏。游戏区在标题牌下方、草地条上方,
// 与 ui_pixel_screen_create() 画出的背景对齐(草地顶 y=286,标题牌底约 y=41)。
#pragma once

#include <stdbool.h>
#include <stdint.h>

// ---- 屏幕几何(与 ui_pixel 主题的实际布局一致) ----
#define VJ_SCREEN_W 240
#define VJ_AREA_TOP 48    // 玩家允许到达的最高 y(标题牌下方留 7px 呼吸空间)
#define VJ_GROUND_Y 286   // 草地条顶边,玩家脚底落点

// ---- 物理参数(单位:像素 / 秒 / 秒) ----
// 沉默时只有重力;喊声提供向上推力。净加速度 = G - level * THRUST。
#define VJ_GRAVITY_PPS2 2600.0f
#define VJ_THRUST_PPS2 5400.0f // 满音量时净向上 2800 px/s^2
#define VJ_VY_MAX_UP 720.0f
#define VJ_VY_MAX_DOWN 760.0f

// ---- 玩家 ----
#define VJ_PLAYER_X 40
#define VJ_PLAYER_W 22
#define VJ_PLAYER_H 22

// ---- 障碍 ----
#define VJ_OBST_MAX 6        // 场上同时存在的障碍上限(渲染对象池同尺寸)
#define VJ_OBST_MIN_GAP_PX 96  // 间距随难度从宽到窄
#define VJ_OBST_MAX_GAP_PX 150
#define VJ_SPIKE_MIN_H 26     // 地面尖刺高度范围
#define VJ_SPIKE_MAX_H 72
#define VJ_BRICK_W 26         // 浮空砖块
#define VJ_BRICK_H 18

// ---- 速度曲线 ----
#define VJ_SPEED_MIN_PPS 130.0f
#define VJ_SPEED_MAX_PPS 300.0f
#define VJ_SPEED_PER_SCORE 8.0f // 每得 1 分加速 8 px/s,封顶 MAX

// ---- 音量包络 ----
// 麦克风 RMS(16bit PCM)到 0..1 「喊声等级」的映射。噪声底在 READY 阶段自动
// 采集(滚动最小值 * 系数 + 余量),开局冻结;超出 FULL_SCALE 的 RMS 视为满音量。
#define VJ_RMS_FULL_SCALE 1200.0f
#define VJ_FLOOR_MIN_RMS 40.0f
#define VJ_FLOOR_MAX_RMS 800.0f
#define VJ_FLOOR_MARGIN_RMS 60.0f
#define VJ_FLOOR_FACTOR 1.6f
#define VJ_FLOOR_WINDOW 16 // 滚动窗口的块数(每块约 16ms,共约 256ms)
#define VJ_LEVEL_RELEASE_S 1.4f // 包络下落速率(每秒),上升是瞬时的

typedef enum {
    VJ_STATE_READY = 0, // 等待开始,麦克风持续校准噪声底
    VJ_STATE_PLAYING,   // 游戏进行中
    VJ_STATE_DEAD,      // 已死亡,等待重开
} vj_state_t;

typedef struct {
    bool active;    // 槽位是否在用
    bool floating;  // 浮空砖块(true)或地面尖刺(false),仅影响外观
    bool counted;   // 是否已计分(玩家越过其右缘后置位)
    float x;        // 左边缘
    float y;        // 顶边缘
    float w, h;     // 碰撞盒(生成时已按渲染外观收缩)
} vj_obstacle_t;

typedef struct {
    vj_state_t state;

    float y;  // 玩家包围盒顶边
    float vy; // 竖直速度,向上为负

    float speed_pps;
    vj_obstacle_t obst[VJ_OBST_MAX];
    uint32_t score;
    uint32_t best;
    bool new_best; // 本局是否刷新纪录(用于结算界面)

    float level;    // 当前生效的喊声等级 0..1(包络输出)
    float floor_rms; // 冻结的噪声底;仅 READY 阶段由 floor_* 更新
    float floor_hist[VJ_FLOOR_WINDOW]; // RMS 滚动窗口
    uint8_t floor_idx;
    bool floor_full;

    uint32_t rng; // LCG 种子;测试里固定它即可复现随机序列
} vj_game_t;

// ---- 噪声底(仅 READY 阶段调用) ----
void vj_floor_reset(vj_game_t *g);
void vj_floor_feed(vj_game_t *g, float rms);
float vj_floor_value(const vj_game_t *g);

// ---- RMS -> 等级映射(纯函数,便于单测) ----
float vj_level_target(float rms, float floor_rms);

// ---- 生命周期 ----
void vj_game_reset(vj_game_t *g); // -> READY,保留 best 与噪声底校准;重播随机种子
void vj_game_start(vj_game_t *g);  // READY -> PLAYING,冻结噪声底
void vj_game_update(vj_game_t *g, uint32_t dt_ms, float rms_now);

// ---- 查询(纯函数) ----
float vj_speed_for_score(uint32_t score);
