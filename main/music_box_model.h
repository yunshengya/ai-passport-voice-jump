// main/music_box_model.h —— 八音盒音乐播放器纯逻辑层。
//
// 设计要点:
//   * 方波 + ADSR 包络 = 经典八音盒音色,纯代码合成,零音频文件。
//   * 歌曲 = 音符数组(MIDI 音高 + 毫秒时值),播放时逐音符推进。
//   * 全部状态在结构体里,无隐藏全局,便于主机测试。
//   * 生成 16kHz 16-bit mono PCM,可直接写 I2S。
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 常量 ----

#define MB_SAMPLE_RATE      16000
#define MB_VOLUME_MAX       12000   // 最大峰值(约 -2.8dB,留余量避免削波)

// 歌曲数
#define MB_SONG_COUNT       4
// 歌名最大长度
#define MB_NAME_MAX         32
#define MB_ARTIST_MAX       16

// ---- 音符定义 ----

// 音符用 MIDI 音高表示(0~127),0 = 休止符
// A4 = 69 = 440Hz
typedef struct {
    uint8_t  note;       // MIDI 音高,0 = rest
    uint16_t duration_ms; // 持续毫秒
} mb_note_t;

typedef struct {
    char name[MB_NAME_MAX];
    char artist[MB_ARTIST_MAX];
    const mb_note_t *notes;
    uint16_t note_count;
} mb_song_t;

// ---- 合成器状态 ----

typedef struct {
    // 当前音符
    int      note_idx;       // 当前音符索引
    float    note_elapsed_ms; // 当前音符已播放毫秒(浮点,亚毫秒精度)
    uint8_t  cur_note;       // 当前 MIDI 音高(0=休止)

    // 相位累加器(方波)
    float    phase;          // 0.0 ~ 1.0
    float    freq_hz;        // 当前频率

    // ADSR 包络
    float    envelope;       // 0.0 ~ 1.0
    bool     note_active;    // 当前是否在音符发声期
    float    env_elapsed_ms; // 包络已用毫秒(浮点)

    // 播放状态
    bool     playing;
    bool     loop;           // 循环播放
    uint32_t total_ms;       // 整首总时长(毫秒)
    float    played_ms;      // 已播放毫秒(浮点,亚毫秒精度)
    int      song_idx;       // 当前歌曲索引
} mb_synth_t;

// ---- 歌曲 API ----

// 获取内置歌曲列表
const mb_song_t *mb_songs(void);
int mb_song_count(void);

// 按索引找歌曲
const mb_song_t *mb_song_get(int idx);

// 计算歌曲总时长(毫秒)
uint32_t mb_song_duration_ms(const mb_song_t *s);

// ---- 合成器 API ----

// 初始化合成器
void mb_synth_init(mb_synth_t *synth);

// 加载歌曲(从头开始)
void mb_synth_load(mb_synth_t *synth, const mb_song_t *song);

// 播放/暂停
void mb_synth_play(mb_synth_t *synth, bool play);

// 设置循环
void mb_synth_set_loop(mb_synth_t *synth, bool loop);

// 跳到下一首(返回歌曲索引)
int mb_synth_next(mb_synth_t *synth, const mb_song_t *songs, int count);
int mb_synth_prev(mb_synth_t *synth, const mb_song_t *songs, int count);

// 生成一块 PCM 样本。返回生成的样本数(到达结尾返回 0)。
// 16-bit signed mono, 16kHz。
uint32_t mb_synth_render(mb_synth_t *synth, int16_t *pcm, uint32_t max_samples);

// 当前歌曲索引
int mb_synth_song_index(mb_synth_t *synth);

// 当前播放进度(毫秒)
uint32_t mb_synth_position_ms(mb_synth_t *synth);

// 当前音符(用于可视化)
uint8_t mb_synth_current_note(mb_synth_t *synth);

// 计算 MIDI 音高对应的频率
float mb_note_freq(uint8_t midi_note);

#ifdef __cplusplus
}
#endif
