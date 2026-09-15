# Voice Jump — AI Passport Custom Play

Voice-controlled parkour game.

Built for the [FoloToy AI Passport](https://github.com/folotoy/ai-passport) ESP32-C3 firmware.

## How to Use

Select **Voice Jump** from the main menu after flashing.

## Build & Flash

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Code Structure

| File | Role |
| --- | --- |
| `main/voice_jump_model.h` | Pure-logic model header |
| `main/voice_jump_model.c` | Pure-logic model implementation |
| `main/demo_voice_jump.c` | LVGL UI + audio + task management |
| `tests/test_voice_jump_model.c` | Host-side unit tests |

Design principle: **testable pure logic separated from platform code**.
