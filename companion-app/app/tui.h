#pragma once
#include "protocol.h"

void tui_init(void);
void tui_deinit(void);

void tui_update_status(const LoraStatus* s);

void tui_log(const char* fmt, ...);

typedef void (*CmdCallback)(const char* cmd);
void tui_run(CmdCallback on_cmd);