#include "tui.h"
#include <ncurses.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>


#define LOG_LINES     6
#define CMD_MAX_LEN   128
#define COLOR_GOOD    1 
#define COLOR_WARN    2
#define COLOR_BAD     3
#define COLOR_DIM     4
#define COLOR_TITLE   5

// ── internal state ─────────────────────────────────────────────────────────

static WINDOW* win_header;
static WINDOW* win_align;
static WINDOW* win_camera;
static WINDOW* win_env;
static WINDOW* win_power;
static WINDOW* win_log;
static WINDOW* win_cmd;

static LoraStatus current_status = {0};
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;

static char log_buf[LOG_LINES][256];
static int  log_head = 0;   // circular buffer head

static char cmd_buf[CMD_MAX_LEN];
static int  cmd_pos = 0;

// ── helpers ────────────────────────────────────────────────────────────────

static void draw_box_title(WINDOW* w, const char* title) {
    box(w, 0, 0);
    wattron(w, COLOR_PAIR(COLOR_TITLE) | A_BOLD);
    mvwprintw(w, 0, 2, " %s ", title);
    wattroff(w, COLOR_PAIR(COLOR_TITLE) | A_BOLD);
}

static void indicator(WINDOW* w, int y, int x, bool on, const char* label) {
    if (on) {
        wattron(w, COLOR_PAIR(COLOR_GOOD) | A_BOLD);
        mvwprintw(w, y, x, "[*]");
        wattroff(w, COLOR_PAIR(COLOR_GOOD) | A_BOLD);
    } else {
        wattron(w, COLOR_PAIR(COLOR_DIM));
        mvwprintw(w, y, x, "[ ]");
        wattroff(w, COLOR_PAIR(COLOR_DIM));
    }
    mvwprintw(w, y, x + 2, "%s", label);
}

static void rssi_bar(WINDOW* w, int y, int x, int rssi) {
    // map -120..-40 dBm → 0..10 bars
    int bars = (rssi + 120) * 10 / 80;
    if (bars < 0) bars = 0;
    if (bars > 10) bars = 10;

    int pair = bars >= 7 ? COLOR_GOOD : bars >= 4 ? COLOR_WARN : COLOR_BAD;
    wattron(w, COLOR_PAIR(pair));
    mvwprintw(w, y, x, "[");
    for (int i = 0; i < 10; i++)
        wprintw(w, i < bars ? "#" : ".");
    wprintw(w, "] %d dBm", rssi);
    wattroff(w, COLOR_PAIR(pair));
}

// ── window layout ──────────────────────────────────────────────────────────

static void create_windows(void) {
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    //  header:  3 rows
    //  align:   4 rows
    //  camera:  3 rows
    //  env:     3 rows
    //  power:   3 rows
    //  log:     LOG_LINES + 2 rows
    //  cmd:     3 rows  (pinned to bottom)

    int y = 0;
    win_header = newwin(3,            cols, y, 0); y += 3;
    win_align  = newwin(4,            cols, y, 0); y += 4;
    win_camera = newwin(3,            cols, y, 0); y += 3;
    win_env    = newwin(3,            cols, y, 0); y += 3;
    win_power  = newwin(3,            cols, y, 0); y += 3;
    win_log    = newwin(LOG_LINES+2,  cols, y, 0); y += LOG_LINES + 2;
    win_cmd    = newwin(3,            cols, rows - 3, 0);
}
// ── draw functions ─────────────────────────────────────────────────────────

static void draw_header(const LoraStatus* s) {
    werase(win_header);
    draw_box_title(win_header, "StarTracker Companion");

    // timestamp top-right
    time_t t = time(NULL);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&t));
    mvwprintw(win_header, 0, getmaxx(win_header) - strlen(ts) - 2, "%s", ts);

    indicator(win_header, 1, 2,  s->is_tracking,  "TRACKING");
    indicator(win_header, 1, 16, s->is_on_target, "ON TARGET");
    mvwprintw(win_header, 1, 32, "GPS: %d sats", s->gps_sattelites);
    rssi_bar  (win_header, 1, 48, s->rssi);

    wnoutrefresh(win_header);
}

static void draw_align(const LoraStatus* s) {
    werase(win_align);
    draw_box_title(win_align, "Alignment");

    float delta = s->pole_az_deg - s->current_az_deg;
    int delta_color = fabsf(delta) < 1.0f ? COLOR_GOOD :
                      fabsf(delta) < 5.0f ? COLOR_WARN  : COLOR_BAD;

    mvwprintw(win_align, 1,  2, "Pole az    %7.1f°", s->pole_az_deg);
    mvwprintw(win_align, 1, 26, "Platform az  %7.1f°", s->current_az_deg);
    wattron(win_align, COLOR_PAIR(delta_color) | A_BOLD);
    mvwprintw(win_align, 1, 52, "Delta:  %+.1f°", delta);
    wattroff(win_align, COLOR_PAIR(delta_color) | A_BOLD);

    mvwprintw(win_align, 2,  2, "Alt corr   %+7.1f°", s->alt_correction_deg);

    wnoutrefresh(win_align);
}

static void draw_camera(const LoraStatus* s) {
    werase(win_camera);
    draw_box_title(win_camera, "Camera");
    mvwprintw(win_camera, 1,  2, "Az  %7.1f°", s->current_camera_az);
    mvwprintw(win_camera, 1, 22, "Alt  %7.1f°", s->current_camera_alt);
    wnoutrefresh(win_camera);
}

static void draw_env(const LoraStatus* s) {
    werase(win_env);
    draw_box_title(win_env, "Environment");
    mvwprintw(win_env, 1, 2, "Temp  %5.1f°C", s->temp);
    mvwprintw(win_env, 1, 22, "Humidity  %5.1f%%", s->hum);
    if (s->hum_warning) {
        wattron(win_env, COLOR_PAIR(COLOR_WARN) | A_BOLD);
        mvwprintw(win_env, 1, 42, "! HUMIDITY WARNING");
        wattroff(win_env, COLOR_PAIR(COLOR_WARN) | A_BOLD);
    }
    wnoutrefresh(win_env);
}

static void draw_power(const LoraStatus* s) {
    werase(win_power);
    draw_box_title(win_power, "Power");

    int batt_color = s->battery_mv > 5800 ? COLOR_GOOD :
                     s->battery_mv > 5400 ? COLOR_WARN  : COLOR_BAD;

    wattron(win_power, COLOR_PAIR(batt_color));
    mvwprintw(win_power, 1, 2,  "Battery  %5.0f mV", s->battery_mv);
    wattroff(win_power, COLOR_PAIR(batt_color));
    mvwprintw(win_power, 1, 26, "Current  %5.0f mA", s->current_ma);
    wnoutrefresh(win_power);
}

static void draw_log(void) {
    werase(win_log);
    draw_box_title(win_log, "Log");
    for (int i = 0; i < LOG_LINES; i++) {
        int idx = (log_head + i) % LOG_LINES;
        if (log_buf[idx][0])
            mvwprintw(win_log, i + 1, 2, "%s", log_buf[idx]);
    }
    wnoutrefresh(win_log);
}

static void draw_cmd(void) {
    werase(win_cmd);
    box(win_cmd, 0, 0);
    mvwprintw(win_cmd, 1, 2, "> %s", cmd_buf);
    // show cursor
    wmove(win_cmd, 1, 4 + cmd_pos);
    wnoutrefresh(win_cmd);
}

static void redraw_all(void) {
    pthread_mutex_lock(&status_mutex);
    LoraStatus s = current_status;
    pthread_mutex_unlock(&status_mutex);

    draw_header(&s);
    draw_align(&s);
    draw_camera(&s);
    draw_env(&s);
    draw_power(&s);
    draw_log();
    draw_cmd();
    doupdate();
}

// ── public API ─────────────────────────────────────────────────────────────

void tui_init(void) {
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);   // non-blocking getch
    curs_set(1);

    start_color();
    use_default_colors();
    init_pair(COLOR_GOOD,  COLOR_GREEN,   -1);
    init_pair(COLOR_WARN,  COLOR_YELLOW,  -1);
    init_pair(COLOR_BAD,   COLOR_RED,     -1);
    init_pair(COLOR_DIM,   COLOR_WHITE,   -1);
    init_pair(COLOR_TITLE, COLOR_CYAN,    -1);

    create_windows();
    memset(log_buf, 0, sizeof(log_buf));
    memset(cmd_buf, 0, sizeof(cmd_buf));
}

void tui_deinit(void) {
    delwin(win_header);
    delwin(win_align);
    delwin(win_camera);
    delwin(win_env);
    delwin(win_power);
    delwin(win_log);
    delwin(win_cmd);
    endwin();
}

void tui_update_status(const LoraStatus* s) {
    pthread_mutex_lock(&status_mutex);
    current_status = *s;
    pthread_mutex_unlock(&status_mutex);
}

void tui_log(const char* fmt, ...) {
    time_t t = time(NULL);
    struct tm* tm = localtime(&t);
    char ts[12];
    strftime(ts, sizeof(ts), "%H:%M:%S", tm);

    char msg[240];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    snprintf(log_buf[log_head], sizeof(log_buf[0]), "[%s] %s", ts, msg);
    log_head = (log_head + 1) % LOG_LINES;
}

void tui_run(CmdCallback on_cmd) {
    while (1) {
        redraw_all();

        int ch = getch();
        if (ch == ERR) {
            // no keypress — sleep 100ms then redraw (gives ~10fps)
            napms(100);
            continue;
        }

        switch (ch) {
            case '\n':
            case KEY_ENTER:
                if (cmd_pos > 0) {
                    cmd_buf[cmd_pos] = '\0';
                    tui_log("$ %s", cmd_buf);
                    if (on_cmd) on_cmd(cmd_buf);
                    memset(cmd_buf, 0, sizeof(cmd_buf));
                    cmd_pos = 0;
                }
                break;

            case KEY_BACKSPACE:
            case 127:
                if (cmd_pos > 0)
                    cmd_buf[--cmd_pos] = '\0';
                break;

            case KEY_RESIZE:
                // terminal was resized — recreate windows
                delwin(win_header); delwin(win_align);
                delwin(win_camera); delwin(win_env);
                delwin(win_power);  delwin(win_log);
                delwin(win_cmd);
                create_windows();
                break;

            default:
                if (ch >= 32 && ch < 127 && cmd_pos < CMD_MAX_LEN - 1)
                    cmd_buf[cmd_pos++] = (char)ch;
                break;
        }
    }
}