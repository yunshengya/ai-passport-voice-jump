// main/voice_jump_model.c —— 声控跳跳酱纯逻辑实现。
//
// 设计要点:
//   * 全部状态都在 vj_game_t 里,函数无隐藏全局,便于主机测试。
//   * 定步长推进:调用方(游戏任务)按固定 ~33ms 调 vj_game_update(),
//     dt 显式传入,模型自身不读时钟。
//   * 随机数用内置 LCG,种子在结构体里,测试固定种子即可完全复现。
#include "voice_jump_model.h"

#include <math.h>

// ---- 内部工具 ----

static float vj_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint32_t vj_rand_next(vj_game_t *g)
{
    // 32bit LCG(Numerical Recipes 常数)。游戏随机性要求很低,够用且可复现。
    g->rng = g->rng * 1664525u + 1013904223u;
    return g->rng;
}

static float vj_rand_range(vj_game_t *g, float lo, float hi)
{
    return lo + (hi - lo) * ((float)(vj_rand_next(g) >> 8) / 16777216.0f);
}

// ---- 噪声底 ----

void vj_floor_reset(vj_game_t *g)
{
    for (int i = 0; i < VJ_FLOOR_WINDOW; i++) g->floor_hist[i] = 0.0f;
    g->floor_idx = 0;
    g->floor_full = false;
    g->floor_rms = VJ_FLOOR_MIN_RMS;
}

void vj_floor_feed(vj_game_t *g, float rms)
{
    // 滚动最小值:窗口未填满前不产出过小的底(开局给保守值)。
    g->floor_hist[g->floor_idx] = rms;
    g->floor_idx = (uint8_t)((g->floor_idx + 1) % VJ_FLOOR_WINDOW);
    if (g->floor_idx == 0) g->floor_full = true;
    g->floor_rms = vj_floor_value(g);
}

float vj_floor_value(const vj_game_t *g)
{
    if (!g->floor_full) return VJ_FLOOR_MIN_RMS;
    float mn = g->floor_hist[0];
    for (int i = 1; i < VJ_FLOOR_WINDOW; i++) {
        if (g->floor_hist[i] < mn) mn = g->floor_hist[i];
    }
    // 最小值乘系数加余量:短暂安静不等于底噪,留出说话/呼吸的余量。
    return vj_clampf(mn * VJ_FLOOR_FACTOR + VJ_FLOOR_MARGIN_RMS,
                     VJ_FLOOR_MIN_RMS, VJ_FLOOR_MAX_RMS);
}

// ---- RMS -> 等级 ----

float vj_level_target(float rms, float floor_rms)
{
    const float floor = vj_clampf(floor_rms, VJ_FLOOR_MIN_RMS, VJ_FLOOR_MAX_RMS);
    if (rms <= floor) return 0.0f;
    return vj_clampf((rms - floor) / (VJ_RMS_FULL_SCALE - floor), 0.0f, 1.0f);
}

// ---- 生命周期 ----

void vj_game_reset(vj_game_t *g)
{
    const uint32_t best = g->best;
    g->state = VJ_STATE_READY;
    g->y = (float)(VJ_GROUND_Y - VJ_PLAYER_H);
    g->vy = 0.0f;
    g->speed_pps = VJ_SPEED_MIN_PPS;
    for (int i = 0; i < VJ_OBST_MAX; i++) {
        g->obst[i].active = false;
    }
    g->score = 0;
    g->best = best;
    g->new_best = false;
    g->level = 0.0f;
    // 噪声底校准跨局保留:重开瞬间环境没变,沿用校准值避免开局幽灵上浮;
    // 首次使用前结构体清零,窗口未填满时 vj_floor_value() 给保守值。
    if (g->rng == 0) g->rng = 0x9E3779B9u; // 固定非零种子,LCG 不允许 0
}

void vj_game_start(vj_game_t *g)
{
    if (g->state != VJ_STATE_READY) return;
    g->state = VJ_STATE_PLAYING;
    g->floor_rms = vj_floor_value(g); // 冻结校准结果
    // 第一个障碍放屏幕右侧外,给玩家留起步距离。
    g->obst[0].active = true;
    g->obst[0].floating = false;
    g->obst[0].counted = false;
    g->obst[0].x = (float)(VJ_SCREEN_W + 60);
    g->obst[0].y = (float)(VJ_GROUND_Y - VJ_SPIKE_MIN_H);
    g->obst[0].w = 18;
    g->obst[0].h = (float)VJ_SPIKE_MIN_H;
}

// ---- 障碍生成与回收 ----

static void vj_spawn_obstacle(vj_game_t *g)
{
    int slot = -1;
    for (int i = 0; i < VJ_OBST_MAX; i++) {
        if (!g->obst[i].active) {
            slot = i;
            break;
        }
    }
    if (slot < 0) return; // 池满:极小概率,直接跳过这一帧的生成

    // 生成时机(距离控制)在 vj_game_update 里;这里只负责形状。
    vj_obstacle_t *o = &g->obst[slot];
    o->active = true;
    o->counted = false;
    o->floating = (vj_rand_next(g) >> 31) != 0; // 约 40% 浮空砖
    if (o->floating) {
        // 浮空砖要留出「从下方钻过」或「从上方飞过」的两条活路。
        o->w = (float)VJ_BRICK_W;
        o->h = (float)VJ_BRICK_H;
        o->y = vj_rand_range(g, (float)(VJ_AREA_TOP + 30),
                             (float)(VJ_GROUND_Y - 130));
    } else {
        o->w = 18;
        o->h = vj_rand_range(g, (float)VJ_SPIKE_MIN_H, (float)VJ_SPIKE_MAX_H);
        o->y = (float)VJ_GROUND_Y - o->h;
    }
    o->x = (float)VJ_SCREEN_W;
}

// ---- 主推进 ----

void vj_game_update(vj_game_t *g, uint32_t dt_ms, float rms_now)
{
    const float dt = (float)dt_ms / 1000.0f;

    // 包络:上升瞬时、下落按固定速率,给玩家「收声立刻掉、出声立刻升」的手感。
    const float target = vj_level_target(rms_now, g->floor_rms);
    if (target > g->level) {
        g->level = target;
    } else {
        g->level -= VJ_LEVEL_RELEASE_S * dt;
        if (g->level < 0.0f) g->level = 0.0f;
    }

    if (g->state == VJ_STATE_READY) {
        vj_floor_feed(g, rms_now); // 准备阶段持续校准噪声底
        return;
    }
    if (g->state != VJ_STATE_PLAYING) return;

    // 物理
    g->vy += (VJ_GRAVITY_PPS2 - g->level * VJ_THRUST_PPS2) * dt;
    g->vy = vj_clampf(g->vy, -VJ_VY_MAX_UP, VJ_VY_MAX_DOWN);
    g->y += g->vy * dt;
    if (g->y < (float)VJ_AREA_TOP) {
        g->y = (float)VJ_AREA_TOP;
        if (g->vy < 0.0f) g->vy = 0.0f;
    }
    const float ground = (float)(VJ_GROUND_Y - VJ_PLAYER_H);
    if (g->y >= ground) {
        g->y = ground;
        if (g->vy > 0.0f) g->vy = 0.0f;
    }

    // 世界推进与计分
    g->speed_pps = vj_speed_for_score(g->score);
    const float dx = g->speed_pps * dt;
    float rightmost = -1.0f;
    for (int i = 0; i < VJ_OBST_MAX; i++) {
        vj_obstacle_t *o = &g->obst[i];
        if (!o->active) continue;
        o->x -= dx;
        if (o->x + o->w < -10.0f) {
            o->active = false; // 出屏回收
            continue;
        }
        if (!o->counted && o->x + o->w < (float)VJ_PLAYER_X) {
            o->counted = true;
            g->score++;
        }
        if (o->x + o->w > rightmost) rightmost = o->x + o->w;
    }

    // 生成:最右障碍距屏幕右缘不足一个间距时补一个新障碍。
    const float diff = vj_clampf(
        (g->speed_pps - VJ_SPEED_MIN_PPS) / (VJ_SPEED_MAX_PPS - VJ_SPEED_MIN_PPS),
        0.0f, 1.0f);
    const float gap = VJ_OBST_MAX_GAP_PX
                    - (VJ_OBST_MAX_GAP_PX - VJ_OBST_MIN_GAP_PX) * diff;
    if (rightmost < (float)VJ_SCREEN_W - gap) {
        vj_spawn_obstacle(g);
    }

    // 碰撞(收缩后的玩家盒 vs 障碍盒)
    const float px0 = (float)VJ_PLAYER_X + 2.0f;
    const float px1 = (float)(VJ_PLAYER_X + VJ_PLAYER_W) - 2.0f;
    const float py0 = g->y + 2.0f;
    const float py1 = g->y + (float)VJ_PLAYER_H - 2.0f;
    for (int i = 0; i < VJ_OBST_MAX; i++) {
        const vj_obstacle_t *o = &g->obst[i];
        if (!o->active) continue;
        if (px0 < o->x + o->w - 2.0f && px1 > o->x + 2.0f
            && py0 < o->y + o->h - 2.0f && py1 > o->y + 2.0f) {
            g->state = VJ_STATE_DEAD;
            g->new_best = g->score > g->best;
            if (g->new_best) g->best = g->score;
            break;
        }
    }
}

float vj_speed_for_score(uint32_t score)
{
    const float s = VJ_SPEED_MIN_PPS + VJ_SPEED_PER_SCORE * (float)score;
    return vj_clampf(s, VJ_SPEED_MIN_PPS, VJ_SPEED_MAX_PPS);
}
