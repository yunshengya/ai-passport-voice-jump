// tests/test_voice_jump_model.c —— 声控跳跳酱纯逻辑模型的主机端单元测试。
// 编译方式与现有测试一致:cc -std=c11 -Wall -Wextra -Werror -Imain(见 tools/validate.sh)。
#include <assert.h>
#include <math.h>

#include "voice_jump_model.h"

static int near(float a, float b)
{
    return fabsf(a - b) < 0.01f;
}

static void test_floor(void)
{
    vj_game_t g = {0};
    g.rng = 1;
    vj_game_reset(&g);

    // 窗口未填满:给出保守底,避免开局误判
    assert(near(vj_floor_value(&g), VJ_FLOOR_MIN_RMS));
    for (int i = 0; i < VJ_FLOOR_WINDOW; i++) vj_floor_feed(&g, 100.0f);
    assert(near(vj_floor_value(&g), 100.0f * VJ_FLOOR_FACTOR + VJ_FLOOR_MARGIN_RMS));

    // 滚动窗口只看最近 16 块:混入更小的值后底随之下降
    for (int i = 0; i < VJ_FLOOR_WINDOW; i++) vj_floor_feed(&g, 20.0f);
    assert(near(vj_floor_value(&g), 20.0f * VJ_FLOOR_FACTOR + VJ_FLOOR_MARGIN_RMS));

    // 环境极吵:底被封顶,保证映射分母不为零
    for (int i = 0; i < VJ_FLOOR_WINDOW; i++) vj_floor_feed(&g, 1000.0f);
    assert(near(vj_floor_value(&g), VJ_FLOOR_MAX_RMS));
}

static void test_level_target(void)
{
    assert(vj_level_target(100.0f, 100.0f) == 0.0f);
    assert(vj_level_target(50.0f, 100.0f) == 0.0f);
    assert(near(vj_level_target(1200.0f, 100.0f), 1.0f));
    assert(near(vj_level_target(650.0f, 100.0f), 0.5f));
    assert(near(vj_level_target(5000.0f, 40.0f), 1.0f));
    // 底超过封顶值时按封顶算:900 底被压到 800
    assert(near(vj_level_target(900.0f, 900.0f), 0.25f));
}

static void test_envelope(void)
{
    vj_game_t g = {0};
    g.rng = 1;
    vj_game_reset(&g);

    // 上升瞬时
    vj_game_update(&g, 33, 1200.0f);
    assert(near(g.level, 1.0f));
    // 下落按固定速率:1.4/s * 0.033s ≈ 0.0462
    vj_game_update(&g, 33, 0.0f);
    assert(g.level < 1.0f && g.level > 0.9f);
    for (int i = 0; i < 30; i++) vj_game_update(&g, 33, 0.0f);
    assert(near(g.level, 0.0f));
}

static void test_physics_silence_lands(void)
{
    vj_game_t g = {0};
    g.rng = 1;
    vj_game_reset(&g);
    vj_game_start(&g);
    assert(g.state == VJ_STATE_PLAYING);

    // 沉默 = 只受重力:站在地面上保持贴地
    const float stand = (float)(VJ_GROUND_Y - VJ_PLAYER_H);
    for (int i = 0; i < 50; i++) vj_game_update(&g, 33, 0.0f);
    assert(near(g.y, stand));
    assert(near(g.vy, 0.0f));
}

static void test_physics_shout_rises_and_ceiling_clamps(void)
{
    vj_game_t g = {0};
    g.rng = 1;
    vj_game_reset(&g);
    vj_game_start(&g);

    // 满音量:净向上加速度,持续喊会离地
    const float stand = (float)(VJ_GROUND_Y - VJ_PLAYER_H);
    for (int i = 0; i < 5; i++) vj_game_update(&g, 33, 1300.0f);
    assert(g.y < stand - 5.0f);

    // 顶到天花板:位置被夹住、不再继续向上
    g.y = (float)VJ_AREA_TOP;
    g.vy = -1000.0f;
    vj_game_update(&g, 33, 1300.0f);
    assert(near(g.y, (float)VJ_AREA_TOP));
    assert(g.vy >= 0.0f);
}

static void test_speed_ramp(void)
{
    assert(near(vj_speed_for_score(0), VJ_SPEED_MIN_PPS));
    assert(near(vj_speed_for_score(10), VJ_SPEED_MIN_PPS + 10 * VJ_SPEED_PER_SCORE));
    assert(near(vj_speed_for_score(1000), VJ_SPEED_MAX_PPS)); // 封顶
}

static void test_score_and_recycle(void)
{
    vj_game_t g = {0};
    g.rng = 1;
    vj_game_reset(&g);
    vj_game_start(&g);

    // 清场,手动放一块高处的浮空砖(与地面玩家无碰撞),让它从玩家左侧通过
    for (int i = 0; i < VJ_OBST_MAX; i++) g.obst[i].active = false;
    g.obst[0].active = true;
    g.obst[0].floating = true;
    g.obst[0].counted = false;
    g.obst[0].x = 10.0f;
    g.obst[0].y = 100.0f;
    g.obst[0].w = (float)VJ_BRICK_W;
    g.obst[0].h = (float)VJ_BRICK_H;
    vj_game_update(&g, 33, 0.0f);
    assert(g.score == 1);
    assert(g.state == VJ_STATE_PLAYING);

    // 完全出屏后回收槽位
    g.obst[0].x = -60.0f;
    vj_game_update(&g, 33, 0.0f);
    assert(g.obst[0].active == false);
}

static void test_collision_death_and_best(void)
{
    vj_game_t g = {0};
    g.rng = 1;
    vj_game_reset(&g);
    vj_game_start(&g);

    // 先得分 3,再用地面尖刺撞死玩家
    g.score = 3;
    for (int i = 0; i < VJ_OBST_MAX; i++) g.obst[i].active = false;
    g.obst[0].active = true;
    g.obst[0].floating = false;
    g.obst[0].counted = false;
    g.obst[0].x = (float)(VJ_PLAYER_X + 5);
    g.obst[0].y = (float)(VJ_GROUND_Y - 40);
    g.obst[0].w = 18;
    g.obst[0].h = 40;
    vj_game_update(&g, 33, 0.0f);

    assert(g.state == VJ_STATE_DEAD);
    assert(g.best == 3);
    assert(g.new_best);

    // 死亡后世界冻结
    const float frozen_x = g.obst[0].x;
    vj_game_update(&g, 33, 0.0f);
    assert(g.state == VJ_STATE_DEAD);
    assert(near(g.obst[0].x, frozen_x));

    // 重开保留 best
    vj_game_reset(&g);
    assert(g.state == VJ_STATE_READY);
    assert(g.best == 3);
    assert(g.score == 0);
}

static void test_silence_eventually_loses(void)
{
    // 固定种子:不喊就会撞上地面障碍,确定性验证「沉默会输」
    vj_game_t g = {0};
    g.rng = 42;
    vj_game_reset(&g);
    vj_game_start(&g);
    int ticks = 0;
    while (g.state == VJ_STATE_PLAYING && ticks < 1800) {
        vj_game_update(&g, 33, 0.0f);
        ticks++;
    }
    assert(g.state == VJ_STATE_DEAD);
    // 一分钟内必须死掉,否则说明速度曲线坏了
    assert(ticks < 1800);
}

int main(void)
{
    test_floor();
    test_level_target();
    test_envelope();
    test_physics_silence_lands();
    test_physics_shout_rises_and_ceiling_clamps();
    test_speed_ramp();
    test_score_and_recycle();
    test_collision_death_and_best();
    test_silence_eventually_loses();
    return 0;
}
