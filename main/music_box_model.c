// main/music_box_model.c —— 八音盒音乐播放器纯逻辑实现。
//
// 音色设计:方波 + 快速起音 + 指数衰减 = 经典八音盒(音乐盒)音色。
//   * 方波:50% 占空比,只有奇次谐波,听起来像金属簧片。
//   * 包络:attack 5ms 冲到 1.0,decay 约 200ms 指数衰减到 0.3,
//     sustain 保持到音符结束,release 30ms 归零。
//   * 休止符:直接静音,相位继续跑但 envelope = 0。
//
// 歌曲数据:每首歌副歌主旋律的前 16~24 小节,手动扒谱。
//   * 薛之谦《演员》副歌 —— 华语芭乐代表作
//   * 薛之谦《认真的雪》副歌 —— 经典慢歌
//   * 许嵩《素颜》副歌 —— 青春回忆
//   * 许嵩《断桥残雪》副歌 —— 中国风代表作
//
// 音高用 MIDI 编号:中央 C = 60, A4 = 69 = 440Hz。
#include "music_box_model.h"

#include <math.h>
#include <string.h>

// ---- ADSR 包络参数(毫秒) ----

#define MB_ENV_ATTACK_MS    8
#define MB_ENV_DECAY_MS     180
#define MB_ENV_SUSTAIN_LVL  0.35f
#define MB_ENV_RELEASE_MS   25

// ---- MIDI → 频率 ----

float mb_note_freq(uint8_t midi_note)
{
    if (midi_note == 0 || midi_note > 127) return 0.0f;
    // A4 (MIDI 69) = 440 Hz
    // f = 440 * 2^((n - 69) / 12)
    return 440.0f * powf(2.0f, ((float)midi_note - 69.0f) / 12.0f);
}

// ---- 歌曲数据 ----

// 辅助宏:方便写谱
#define N(n, d) { .note = (n), .duration_ms = (d) }
#define R(d)    { .note = 0, .duration_ms = (d) }  // rest

// ============================================================
// 歌曲 1:薛之谦《演员》副歌(简化版,主调在 C5 附近)
//  副歌开头:"该配合你演出的我演视而不见"
//  大致旋律(简化):E5 D5 C5 A4 C5 D5 E5 D5 C5 A4...
// ============================================================
static const mb_note_t s_song_actor[] = {
    // "该配合你演出的我演视而不见"
    N(64, 250), N(62, 250), N(60, 500),  // E5 D5 C5
    N(57, 250), N(60, 250), N(62, 500),  // A4 C5 D5
    N(64, 250), N(62, 250), N(60, 250), N(57, 750), // E5 D5 C5 A4
    R(125),
    // "在逼一个最爱你的人即兴表演"
    N(60, 250), N(62, 250), N(64, 500),  // C5 D5 E5
    N(62, 250), N(60, 250), N(59, 500),  // D5 C5 B4
    N(57, 250), N(59, 250), N(60, 750),  // A4 B4 C5
    R(250),
    // "什么时候我们开始收起了底线"
    N(64, 250), N(65, 250), N(67, 500),  // E5 F5 G5
    N(65, 250), N(64, 250), N(62, 500),  // F5 E5 D5
    N(60, 250), N(59, 250), N(57, 750),  // C5 B4 A4
    R(125),
    // "顺应时代的改变看那些拙劣的表演"
    N(60, 250), N(62, 250), N(64, 500),
    N(62, 250), N(60, 250), N(59, 500),
    N(57, 250), N(55, 250), N(57, 750),  // A4 G4 A4
    R(500),
};

// ============================================================
// 歌曲 2:薛之谦《认真的雪》副歌
//  "雪下得那么深下得那么认真"
//  简化旋律:G4 A4 B4 G4 E4... (E 小调)
// ============================================================
static const mb_note_t s_song_xue[] = {
    // "雪下得那么深下得那么认真"
    N(67, 375), N(69, 125), N(67, 250), N(64, 250), // G5 A5 G5 E5
    N(62, 500),                                    // D5
    N(60, 250), N(62, 250), N(64, 500),            // C5 D5 E5
    R(125),
    // "倒映出我躺在雪中的伤痕"
    N(67, 375), N(69, 125), N(67, 250), N(64, 250),
    N(62, 250), N(60, 500),                        // D5 C5
    R(125),
    // "我并不在乎自己究竟多伤痕累累"
    N(55, 250), N(57, 250), N(60, 250), N(62, 250), // G4 A4 C5 D5
    N(64, 250), N(67, 500),                        // E5 G5
    R(125),
    // "可我在乎今后你有谁陪"
    N(60, 250), N(62, 250), N(64, 250), N(67, 250),
    N(65, 250), N(64, 750),                        // F5 E5
    R(500),
};

// ============================================================
// 歌曲 3:许嵩《素颜》副歌
//  "如果再看你一眼是否还会有感觉"
//  简化旋律:C5 D5 E5 G5 E5 D5...
// ============================================================
static const mb_note_t s_song_suyan[] = {
    // "如果再看你一眼是否还会有感觉"
    N(60, 200), N(62, 200), N(64, 200), N(67, 200),  // C5 D5 E5 G5
    N(64, 200), N(62, 200), N(60, 400),              // E5 D5 C5
    R(100),
    // "当年素面朝天要多纯洁就有多纯洁"
    N(59, 200), N(57, 200), N(55, 200), N(52, 200),  // B4 A4 G4 E4
    N(55, 200), N(57, 200), N(59, 400),              // G4 A4 B4
    R(100),
    // "不画扮熟的眼线不用抹匀粉底液"
    N(60, 200), N(62, 200), N(64, 200), N(67, 200),
    N(65, 200), N(64, 200), N(62, 400),              // F5 E5 D5
    R(100),
    // "暴雨天照逛街偷笑别人花了脸"
    N(60, 200), N(59, 200), N(57, 200), N(55, 200),
    N(57, 200), N(59, 200), N(60, 400),
    R(500),
};

// ============================================================
// 歌曲 4:许嵩《断桥残雪》副歌
//  "断桥是否下过雪我望着湖面"
//  简化旋律(中国风,五声音阶为主):E5 G5 A5 G5 E5 D5...
// ============================================================
static const mb_note_t s_song_bridge[] = {
    // "断桥是否下过雪"
    N(64, 400), N(67, 200), N(69, 400),  // E5 G5 A5
    N(67, 200), N(64, 400),              // G5 E5
    R(100),
    // "我望着湖面"
    N(62, 400), N(60, 200), N(57, 400),  // D5 C5 A4
    R(200),
    // "水中寒月如雪"
    N(64, 300), N(67, 300), N(69, 300), N(67, 300),  // E5 G5 A5 G5
    N(64, 600),                                   // E5
    R(100),
    // "指尖轻点融解"
    N(62, 300), N(60, 300), N(59, 300), N(57, 300),  // D5 C5 B4 A4
    N(55, 600),                                   // G4
    R(500),
};

// ============================================================
// 歌曲列表
// ============================================================

static const mb_song_t s_songs[MB_SONG_COUNT] = {
    {
        .name = "演员",
        .artist = "薛之谦",
        .notes = s_song_actor,
        .note_count = sizeof(s_song_actor) / sizeof(s_song_actor[0]),
    },
    {
        .name = "认真的雪",
        .artist = "薛之谦",
        .notes = s_song_xue,
        .note_count = sizeof(s_song_xue) / sizeof(s_song_xue[0]),
    },
    {
        .name = "素颜",
        .artist = "许嵩",
        .notes = s_song_suyan,
        .note_count = sizeof(s_song_suyan) / sizeof(s_song_suyan[0]),
    },
    {
        .name = "断桥残雪",
        .artist = "许嵩",
        .notes = s_song_bridge,
        .note_count = sizeof(s_song_bridge) / sizeof(s_song_bridge[0]),
    },
};

const mb_song_t *mb_songs(void) { return s_songs; }
int mb_song_count(void) { return MB_SONG_COUNT; }

const mb_song_t *mb_song_get(int idx)
{
    if (idx < 0 || idx >= MB_SONG_COUNT) return NULL;
    return &s_songs[idx];
}

uint32_t mb_song_duration_ms(const mb_song_t *s)
{
    uint32_t total = 0;
    for (uint16_t i = 0; i < s->note_count; i++) {
        total += s->notes[i].duration_ms;
    }
    return total;
}

// ---- 合成器 ----

void mb_synth_init(mb_synth_t *synth)
{
    memset(synth, 0, sizeof(*synth));
    synth->loop = false;
    synth->song_idx = -1;
}

void mb_synth_load(mb_synth_t *synth, const mb_song_t *song)
{
    // 找索引
    int idx = -1;
    const mb_song_t *all = mb_songs();
    for (int i = 0; i < MB_SONG_COUNT; i++) {
        if (&all[i] == song) { idx = i; break; }
    }
    synth->song_idx = idx;
    synth->note_idx = 0;
    synth->note_elapsed_ms = 0;
    synth->cur_note = song->notes[0].note;
    synth->phase = 0.0f;
    synth->freq_hz = mb_note_freq(song->notes[0].note);
    synth->envelope = 0.0f;
    synth->note_active = (song->notes[0].note != 0);
    synth->env_elapsed_ms = 0;
    synth->playing = false;
    synth->total_ms = mb_song_duration_ms(song);
    synth->played_ms = 0;
}

void mb_synth_play(mb_synth_t *synth, bool play)
{
    synth->playing = play;
}

void mb_synth_set_loop(mb_synth_t *synth, bool loop)
{
    synth->loop = loop;
}

static void s_advance_note(mb_synth_t *synth, const mb_note_t *notes, uint16_t note_count)
{
    synth->note_idx++;
    if (synth->note_idx >= note_count) {
        if (synth->loop) {
            synth->note_idx = 0;
            synth->played_ms = 0;
        } else {
            synth->playing = false;
            return;
        }
    }
    synth->note_elapsed_ms = 0;
    synth->cur_note = notes[synth->note_idx].note;
    synth->freq_hz = mb_note_freq(synth->cur_note);
    synth->note_active = (synth->cur_note != 0);
    synth->envelope = 0.0f;
    synth->env_elapsed_ms = 0;
}

// 计算包络(0.0 ~ 1.0)
static float s_envelope(float t_ms, float note_len_ms)
{
    // attack
    if (t_ms < MB_ENV_ATTACK_MS) {
        return t_ms / MB_ENV_ATTACK_MS;
    }
    // decay
    float t_decay = t_ms - MB_ENV_ATTACK_MS;
    if (t_decay < MB_ENV_DECAY_MS) {
        float p = t_decay / MB_ENV_DECAY_MS;
        // 指数衰减:1.0 -> sustain_level
        return MB_ENV_SUSTAIN_LVL + (1.0f - MB_ENV_SUSTAIN_LVL) * powf(0.02f, p);
    }
    // sustain
    (void)note_len_ms;
    return MB_ENV_SUSTAIN_LVL;
}

uint32_t mb_synth_render(mb_synth_t *synth, int16_t *pcm, uint32_t max_samples)
{
    if (!synth->playing) return 0;

    uint32_t produced = 0;
    const mb_song_t *song = mb_song_get(synth->song_idx);
    if (!song) return 0;

    float sample_ms = 1000.0f / (float)MB_SAMPLE_RATE;

    while (produced < max_samples) {
        // 如果当前音符已结束,前进到下一个
        if (synth->note_elapsed_ms >= song->notes[synth->note_idx].duration_ms) {
            s_advance_note(synth, song->notes, song->note_count);
            if (!synth->playing) break; // 到达结尾且不循环
            continue;
        }

        // 计算包络
        float env;
        if (!synth->note_active) {
            env = 0.0f;
        } else {
            float remain = (float)song->notes[synth->note_idx].duration_ms - synth->note_elapsed_ms;
            if (remain < (float)MB_ENV_RELEASE_MS) {
                // release 阶段
                float p = remain / (float)MB_ENV_RELEASE_MS;
                env = MB_ENV_SUSTAIN_LVL * p;
                if (env < 0.0f) env = 0.0f;
            } else {
                env = s_envelope(synth->env_elapsed_ms,
                                 (float)song->notes[synth->note_idx].duration_ms);
            }
        }

        // 生成一个样本
        float sample;
        if (synth->note_active && synth->freq_hz > 0.0f) {
            // 方波:phase < 0.5 时 +1,否则 -1
            float sq = (synth->phase < 0.5f) ? 1.0f : -1.0f;
            sample = sq * env * (float)MB_VOLUME_MAX;
            // 推进相位
            synth->phase += synth->freq_hz / (float)MB_SAMPLE_RATE;
            if (synth->phase >= 1.0f) synth->phase -= 1.0f;
        } else {
            sample = 0.0f;
            // 休止符也推进相位,保持连续性
            if (synth->freq_hz > 0.0f) {
                synth->phase += synth->freq_hz / (float)MB_SAMPLE_RATE;
                if (synth->phase >= 1.0f) synth->phase -= 1.0f;
            }
        }

        if (sample > 32767.0f) sample = 32767.0f;
        if (sample < -32768.0f) sample = -32768.0f;
        pcm[produced++] = (int16_t)sample;

        synth->note_elapsed_ms += sample_ms;
        synth->env_elapsed_ms += sample_ms;
        synth->played_ms += sample_ms;
    }

    return produced;
}

int mb_synth_song_index(mb_synth_t *synth)
{
    return synth->song_idx;
}

uint32_t mb_synth_position_ms(mb_synth_t *synth)
{
    return (uint32_t)synth->played_ms;
}

uint8_t mb_synth_current_note(mb_synth_t *synth)
{
    return synth->cur_note;
}

int mb_synth_next(mb_synth_t *synth, const mb_song_t *songs, int count)
{
    int cur = mb_synth_song_index(synth);
    int next = (cur + 1) % count;
    mb_synth_load(synth, &songs[next]);
    return next;
}

int mb_synth_prev(mb_synth_t *synth, const mb_song_t *songs, int count)
{
    int cur = mb_synth_song_index(synth);
    int prev = (cur - 1 + count) % count;
    mb_synth_load(synth, &songs[prev]);
    return prev;
}
