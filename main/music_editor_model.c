// main/music_editor_model.c —— 音乐编辑器纯逻辑实现。
//
// 音高映射:简谱 1~7 → MIDI 60~66 (C5~B5)
//   1=哆=C5=60, 2=来=D5=62, 3=咪=E5=64,
//   4=发=F5=65, 5=唆=G5=67, 6=拉=A5=69, 7=西=B5=71
#include "music_editor_model.h"
#include <math.h>
#include <string.h>

// 简谱 1~7 对应的 MIDI 音高(C大调)
static const uint8_t s_pitch_to_midi[] = {
    0,   // 0 = 休止
    60,  // 1 = 哆 C5
    62,  // 2 = 来 D5
    64,  // 3 = 咪 E5
    65,  // 4 = 发 F5
    67,  // 5 = 唆 G5
    69,  // 6 = 拉 A5
    71,  // 7 = 西 B5
};

// === 工具函数 ===

float me_pitch_freq(uint8_t pitch)
{
    if (pitch == 0 || pitch > 7) return 0.0f;
    uint8_t midi = s_pitch_to_midi[pitch];
    // f = 440 * 2^((n - 69) / 12)
    return 440.0f * powf(2.0f, ((float)midi - 69.0f) / 12.0f);
}

uint32_t me_song_total_ms(const me_song_t *s)
{
    if (s->bpm == 0) return 0;
    // 四分音符时长 = 60000 / bpm 毫秒
    uint32_t note_ms = 60000u / (uint32_t)s->bpm;
    return note_ms * s->count;
}

// === 初始化 ===

void me_init(me_editor_t *e)
{
    memset(e, 0, sizeof(*e));
    e->selected_pitch = 1;  // 默认选"哆"
    e->mode = ME_MODE_EDIT;
    e->song_idx = 0;
    for (int i = 0; i < ME_SONG_COUNT; i++) {
        e->songs[i].bpm = ME_BPM_DEFAULT;
        e->songs[i].count = 0;
    }
}

// === 草稿管理 ===

int me_song_index(const me_editor_t *e) { return e->song_idx; }

void me_song_next(me_editor_t *e)
{
    e->song_idx = (e->song_idx + 1) % ME_SONG_COUNT;
    e->playing = false;
}

void me_song_prev(me_editor_t *e)
{
    e->song_idx = (e->song_idx - 1 + ME_SONG_COUNT) % ME_SONG_COUNT;
    e->playing = false;
}

const me_song_t *me_current_song(const me_editor_t *e)
{
    return &e->songs[e->song_idx];
}

// === 编辑 ===

int me_selected_pitch(const me_editor_t *e) { return e->selected_pitch; }

void me_pitch_up(me_editor_t *e)
{
    e->selected_pitch = (e->selected_pitch + 1) % 8;
}

void me_pitch_down(me_editor_t *e)
{
    e->selected_pitch = (e->selected_pitch - 1 + 8) % 8;
}

bool me_append_note(me_editor_t *e)
{
    me_song_t *s = &e->songs[e->song_idx];
    if (s->count >= ME_MAX_NOTES) return false;
    s->notes[s->count].pitch = (uint8_t)e->selected_pitch;
    s->count++;
    return true;
}

bool me_delete_last(me_editor_t *e)
{
    me_song_t *s = &e->songs[e->song_idx];
    if (s->count == 0) return false;
    s->count--;
    return true;
}

void me_clear_song(me_editor_t *e)
{
    me_song_t *s = &e->songs[e->song_idx];
    s->count = 0;
}

int me_note_count(const me_editor_t *e)
{
    return e->songs[e->song_idx].count;
}

uint16_t me_bpm(const me_editor_t *e)
{
    return e->songs[e->song_idx].bpm;
}

void me_bpm_up(me_editor_t *e)
{
    me_song_t *s = &e->songs[e->song_idx];
    if (s->bpm + ME_BPM_STEP <= ME_BPM_MAX) {
        s->bpm += ME_BPM_STEP;
    }
}

void me_bpm_down(me_editor_t *e)
{
    me_song_t *s = &e->songs[e->song_idx];
    if (s->bpm - ME_BPM_STEP >= ME_BPM_MIN) {
        s->bpm -= ME_BPM_STEP;
    }
}

// === 模式切换 ===

me_mode_t me_mode(const me_editor_t *e) { return e->mode; }

void me_set_mode(me_editor_t *e, me_mode_t mode)
{
    e->mode = mode;
    if (mode == ME_MODE_PLAY) {
        // 进入播放模式,重置播放进度
        e->playing = false;
        e->play_note_idx = 0;
        e->note_elapsed_ms = 0;
        e->phase = 0.0f;
        e->env_elapsed_ms = 0;
        e->played_ms = 0;
        e->total_ms = me_song_total_ms(&e->songs[e->song_idx]);
    }
}

// === 播放控制 ===

bool me_is_playing(const me_editor_t *e) { return e->playing; }

void me_play(me_editor_t *e, bool play)
{
    if (e->mode != ME_MODE_PLAY) return;
    const me_song_t *s = &e->songs[e->song_idx];

    if (play) {
        if (s->count == 0) return;  // 空曲子不播放
        // 如果已经播完了,从头开始
        if (e->play_note_idx >= s->count) {
            e->play_note_idx = 0;
            e->note_elapsed_ms = 0;
            e->played_ms = 0;
        }
        // 设置当前音符
        uint8_t pitch = s->notes[e->play_note_idx].pitch;
        e->freq_hz = me_pitch_freq(pitch);
        e->note_active = (pitch != 0);
        e->env_elapsed_ms = 0;
        e->playing = true;
    } else {
        e->playing = false;
    }
}

int me_play_position(const me_editor_t *e)
{
    return e->play_note_idx;
}

// === 合成 ===

// ADSR 包络
static float s_envelope(float t_ms, float note_len_ms)
{
    // attack
    if (t_ms < ME_ENV_ATTACK_MS) {
        return t_ms / ME_ENV_ATTACK_MS;
    }
    // decay
    float t_decay = t_ms - ME_ENV_ATTACK_MS;
    if (t_decay < ME_ENV_DECAY_MS) {
        float p = t_decay / ME_ENV_DECAY_MS;
        return ME_ENV_SUSTAIN_LVL + (1.0f - ME_ENV_SUSTAIN_LVL) * powf(0.02f, p);
    }
    // sustain
    (void)note_len_ms;
    return ME_ENV_SUSTAIN_LVL;
}

// 推进到下一个音符
static void s_advance_note(me_editor_t *e, const me_song_t *s)
{
    e->play_note_idx++;
    if (e->play_note_idx >= s->count) {
        e->playing = false;
        return;
    }
    e->note_elapsed_ms = 0;
    uint8_t pitch = s->notes[e->play_note_idx].pitch;
    e->freq_hz = me_pitch_freq(pitch);
    e->note_active = (pitch != 0);
    e->env_elapsed_ms = 0;
}

uint32_t me_render(me_editor_t *e, int16_t *pcm, uint32_t max_samples)
{
    if (!e->playing) return 0;

    const me_song_t *s = &e->songs[e->song_idx];
    if (s->count == 0) return 0;

    uint32_t produced = 0;
    float sample_ms = 1000.0f / (float)ME_SAMPLE_RATE;
    uint32_t note_dur_ms = 60000u / s->bpm;  // 四分音符时长

    while (produced < max_samples) {
        // 当前音符已结束,前进
        if (e->note_elapsed_ms >= (float)note_dur_ms) {
            s_advance_note(e, s);
            if (!e->playing) break;
            continue;
        }

        // 计算包络
        float env;
        if (!e->note_active) {
            env = 0.0f;
        } else {
            float remain = (float)note_dur_ms - e->note_elapsed_ms;
            if (remain < (float)ME_ENV_RELEASE_MS) {
                float p = remain / (float)ME_ENV_RELEASE_MS;
                env = ME_ENV_SUSTAIN_LVL * p;
                if (env < 0.0f) env = 0.0f;
            } else {
                env = s_envelope(e->env_elapsed_ms, (float)note_dur_ms);
            }
        }

        // 生成样本
        float sample;
        if (e->note_active && e->freq_hz > 0.0f) {
            float sq = (e->phase < 0.5f) ? 1.0f : -1.0f;
            sample = sq * env * (float)ME_VOLUME_MAX;
            e->phase += e->freq_hz / (float)ME_SAMPLE_RATE;
            if (e->phase >= 1.0f) e->phase -= 1.0f;
        } else {
            sample = 0.0f;
            if (e->freq_hz > 0.0f) {
                e->phase += e->freq_hz / (float)ME_SAMPLE_RATE;
                if (e->phase >= 1.0f) e->phase -= 1.0f;
            }
        }

        if (sample > 32767.0f) sample = 32767.0f;
        if (sample < -32768.0f) sample = -32768.0f;
        pcm[produced++] = (int16_t)sample;

        e->note_elapsed_ms += sample_ms;
        e->env_elapsed_ms += sample_ms;
        e->played_ms += sample_ms;
    }

    return produced;
}

// 播放一个单音(用于按键反馈),把音符设置为单音模式
uint32_t me_play_tone(me_editor_t *e, uint8_t pitch)
{
    // 用播放系统播一个短音:设置到一个虚拟的单音符"歌"
    // 简单做法:直接返回一个短音的样本数,调用方持续 render
    // 这里我们用编辑器自身的播放状态
    e->freq_hz = me_pitch_freq(pitch);
    e->note_active = (pitch != 0);
    e->phase = 0.0f;
    e->env_elapsed_ms = 0;
    e->note_elapsed_ms = 0;

    // 返回单音时长(毫秒)—— 一个短的"叮"声,200ms
    return 200;
}
