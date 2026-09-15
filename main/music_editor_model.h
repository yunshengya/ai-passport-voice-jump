// main/music_editor_model.h —— 音乐编辑器纯逻辑层。
//
// 设计要点:
//   * 旋律 = 音符数组(音高 + 时值),最多 ME_MAX_NOTES 个。
//   * 音高:1~7 对应简谱哆来咪发唆拉西(C5~B5),0 = 休止符。
//   * 时值:统一四分音符(第一版简化),每个音符时长 = 60000/BPM 毫秒。
//   * 播放时逐音符推进,方波+ADSR 合成音色,复用八音盒技术。
//   * 3 首草稿切换,全部状态在结构体里,无隐藏全局。
//   * 生成 16kHz 16-bit mono PCM。
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 常量 ----

#define ME_SAMPLE_RATE      16000
#define ME_VOLUME_MAX       12000   // 最大峰值

#define ME_MAX_NOTES        32      // 每首最多音符数
#define ME_SONG_COUNT       3       // 草稿数量

// BPM 档位
#define ME_BPM_MIN          60
#define ME_BPM_MAX          180
#define ME_BPM_DEFAULT      120
#define ME_BPM_STEP         30      // 档位步长

// ADSR 包络参数(毫秒)
#define ME_ENV_ATTACK_MS    8
#define ME_ENV_DECAY_MS     200
#define ME_ENV_SUSTAIN_LVL  0.3f
#define ME_ENV_RELEASE_MS   30

// ---- 类型 ----

// 单个音符
typedef struct {
    uint8_t pitch;    // 0=休止, 1~7=哆来咪发唆拉西 (C5~B5)
} me_note_t;

// 一首旋律
typedef struct {
    me_note_t notes[ME_MAX_NOTES];
    uint8_t   count;
    uint16_t  bpm;
} me_song_t;

// 播放模式
typedef enum {
    ME_MODE_EDIT = 0,
    ME_MODE_PLAY
} me_mode_t;

// 合成器状态
typedef struct {
    // 草稿
    me_song_t songs[ME_SONG_COUNT];
    int       song_idx;      // 当前草稿索引

    // 编辑状态
    int       selected_pitch; // 当前选中的音高(0~7,0=休止)

    // 播放状态
    me_mode_t mode;
    bool      playing;
    int       play_note_idx;  // 当前播放到第几个音符
    float     note_elapsed_ms; // 当前音符已播放毫秒
    float     phase;           // 方波相位
    float     freq_hz;         // 当前频率
    float     env_elapsed_ms;  // 包络已用毫秒
    bool      note_active;     // 当前音符是否发声

    // 播放时间(整体进度,用于 UI)
    uint32_t  played_ms;
    uint32_t  total_ms;
} me_editor_t;

// ---- API ----

// 初始化编辑器
void me_init(me_editor_t *e);

// === 草稿管理 ===

// 当前草稿索引
int me_song_index(const me_editor_t *e);

// 切换到下一首/上一首草稿
void me_song_next(me_editor_t *e);
void me_song_prev(me_editor_t *e);

// 获取当前草稿
const me_song_t *me_current_song(const me_editor_t *e);

// === 编辑 ===

// 当前选中的音高
int me_selected_pitch(const me_editor_t *e);

// 选中音高 +1 / -1 (循环 0~7)
void me_pitch_up(me_editor_t *e);
void me_pitch_down(me_editor_t *e);

// 追加一个音符(使用当前选中音高),返回是否成功(满了返回 false)
bool me_append_note(me_editor_t *e);

// 删除最后一个音符,返回是否成功(空了返回 false)
bool me_delete_last(me_editor_t *e);

// 清空当前草稿
void me_clear_song(me_editor_t *e);

// 当前草稿的音符数
int me_note_count(const me_editor_t *e);

// 当前草稿 BPM
uint16_t me_bpm(const me_editor_t *e);

// 调整 BPM(按档位)
void me_bpm_up(me_editor_t *e);
void me_bpm_down(me_editor_t *e);

// === 模式切换 ===

me_mode_t me_mode(const me_editor_t *e);
void me_set_mode(me_editor_t *e, me_mode_t mode);

// === 播放控制 ===

bool me_is_playing(const me_editor_t *e);
void me_play(me_editor_t *e, bool play);

// 当前播放进度(音符索引)
int me_play_position(const me_editor_t *e);

// === 合成 ===

// 生成 PCM 样本。返回生成的样本数(播放结束返回 0)。
uint32_t me_render(me_editor_t *e, int16_t *pcm, uint32_t max_samples);

// 播放一个单音(用于按键反馈),返回持续毫秒数
// 调用方可以直接调一次 me_render 把声音取出来写 I2S
uint32_t me_play_tone(me_editor_t *e, uint8_t pitch);

// === 工具 ===

// 简谱音高(1~7)转频率 (C5~B5)
float me_pitch_freq(uint8_t pitch);

// 计算当前草稿总时长(毫秒)
uint32_t me_song_total_ms(const me_song_t *s);

#ifdef __cplusplus
}
#endif
