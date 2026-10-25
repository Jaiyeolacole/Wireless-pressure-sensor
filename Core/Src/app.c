/*
 * TireSense TPMS - application layer (main loop side)
 *
 *  - collects each radio burst captured by the interrupts (rf_capture)
 *  - decodes it (tpms_decode) and keeps a small table of sensors
 *  - a sensor is shown only after its ID has been received with a valid
 *    checksum TWICE (sensors repeat every packet several times), so random
 *    noise can never put made-up numbers on the screen
 *  - drives the 20x4 LCD, the LED and the USB serial log
 */
#include "app.h"
#include "cc1101.h"
#include "rf_capture.h"
#include "tpms_decode.h"
#include "lcd_i2c.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ---------------- product settings ---------------- */
#define APP_TITLE           "TireSense TPMS"   /* max 20 chars */
#define LOW_PRESSURE_KPA    180                /* alert below 1.80 bar (26 psi) */
#define HIGH_PRESSURE_KPA   350                /* alert above 3.50 bar (51 psi) */
#define STALE_MS            (15u * 60u * 1000u)/* reading older than 15 min is marked old */

/* ---------------- timing ---------------- */
#define STATUS_PERIOD_MS    5000u
#define LCD_PERIOD_MS        500u
#define RSSI_STREAM_MS       100u
#define CYCLE_MS            4000u              /* rotate between sensors if several */

#define BURST_MAX         1536u                /* pulses buffered per burst (~10 packets) */
#define MAX_SENSORS          4u
#define PULSES_PER_LINE     16u

/* ---------------- state ---------------- */
typedef struct {
    uint8_t        used;
    uint8_t        confirmed;
    uint8_t        hits;
    tpms_reading_t r;
    int16_t        rssi;
    uint32_t       last_ms;
    uint32_t       updates;
} sensor_t;

static sensor_t s_sensors[MAX_SENSORS];

static cc1101_mod_t s_mod = CC1101_MOD_FSK;
static uint8_t  s_radio_ok;
static uint8_t  s_rssi_stream;
static uint8_t  s_verbose;                     /* raw pulse dump over USB */

static uint16_t s_burst[BURST_MAX];
static uint32_t s_burst_n;
static uint8_t  s_in_frame;
static uint32_t s_line_count;

static uint32_t s_signals;                     /* radio bursts seen */
static uint32_t s_valid_frames;                /* checksum-valid packets */

static uint32_t t_status, t_lcd, t_rssi, t_led, t_cycle;
static uint8_t  s_show_idx;
static char     s_lcd_want[4][21];
static char     s_lcd_have[4][21];

/* ---------------- helpers ---------------- */

static void out(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) usb_write(buf, (uint32_t)((n < (int)sizeof buf) ? n : (int)sizeof buf - 1));
}

static const char *mod_name(void) { return (s_mod == CC1101_MOD_FSK) ? "FSK" : "OOK"; }

/* Prepare a 20-char LCD row. Actual drawing happens one row per loop pass
 * (lcd_service), so the slow I2C display never blocks the loop for long. */
static void lcd_want(uint8_t row, const char *fmt, ...)
{
    char buf[40];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    snprintf(s_lcd_want[row], sizeof s_lcd_want[row], "%-20.20s", buf);
}

static void lcd_center(uint8_t row, const char *text)
{
    size_t len = strlen(text);
    if (len > 20u) len = 20u;
    unsigned pad = (unsigned)((20u - len) / 2u);
    lcd_want(row, "%*s%s", (int)pad, "", text);
}

static void lcd_service(void)
{
    for (uint8_t row = 0; row < 4; row++) {
        if (strcmp(s_lcd_want[row], s_lcd_have[row]) != 0) {
            memcpy(s_lcd_have[row], s_lcd_want[row], sizeof s_lcd_have[row]);
            lcd_print_at(0, row, s_lcd_have[row]);
            return;                             /* one row per pass */
        }
    }
}

/* "2.31" bar from 0.1 kPa units */
static void fmt_bar(char *dst, size_t sz, int16_t kpa_x10)
{
    unsigned v = (unsigned)((kpa_x10 + 5) / 10);          /* bar * 100 */
    snprintf(dst, sz, "%u.%02u", v / 100u, v % 100u);
}

/* "33.5" psi from 0.1 kPa units (1 kPa = 0.145038 psi) */
static void fmt_psi(char *dst, size_t sz, int16_t kpa_x10)
{
    uint32_t psi_x10 = ((uint32_t)kpa_x10 * 145038u + 500000u) / 1000000u;
    snprintf(dst, sz, "%lu.%lu", (unsigned long)(psi_x10 / 10u), (unsigned long)(psi_x10 % 10u));
}

static void fmt_age(char *dst, size_t sz, uint32_t ms)
{
    uint32_t s = ms / 1000u;
    if (s < 100u)        snprintf(dst, sz, "%lus", (unsigned long)s);
    else if (s < 6000u)  snprintf(dst, sz, "%lum", (unsigned long)(s / 60u));
    else                 snprintf(dst, sz, "%luh", (unsigned long)(s / 3600u));
}

/* ---------------- radio ---------------- */

static const char *radio_err_text(cc1101_status_t st)
{
    switch (st) {
        case CC1101_TIMEOUT:     return "SPI timeout";
        case CC1101_VERIFY_FAIL: return "reg verify fail";
        case CC1101_NOT_RX:      return "not in RX";
        default:                 return "ok";
    }
}

static void radio_configure(void)
{
#if !APP_RADIO_ENABLED
    s_radio_ok = 0;
    out("# USB-ONLY TEST BUILD: radio disabled\r\n");
    return;
#endif
    rfc_stop();
    cc1101_status_t st = cc1101_config_async_rx(s_mod);
    s_radio_ok = (st == CC1101_OK);
    if (s_radio_ok) {
        rfc_start();
        out("# radio: 433.92 MHz %s receive OK\r\n", mod_name());
    } else {
        out("# radio ERROR: %s (check wiring, then press reset)\r\n", radio_err_text(st));
    }
}

static void print_help(void)
{
    out("# ---- " APP_TITLE " ----\r\n");
    out("# Decodes 433.92 MHz FSK TPMS: VDO/Continental (Hyundai/Kia, PSA, Fiat/Mazda), Renault, Ford\r\n");
    out("# keys: v=raw pulse dump  r=RSSI stream  f/o=FSK/OOK  +/-=trigger level  c=clear  ?=help\r\n");
}

/* ---------------- sensor table ---------------- */

static sensor_t *sensor_find(const tpms_reading_t *r)
{
    for (uint32_t i = 0; i < MAX_SENSORS; i++) {
        if (s_sensors[i].used && s_sensors[i].r.id == r->id &&
            strcmp(s_sensors[i].r.proto, r->proto) == 0) {
            return &s_sensors[i];
        }
    }
    return NULL;
}

static sensor_t *sensor_alloc(void)
{
    sensor_t *victim = NULL;
    for (uint32_t i = 0; i < MAX_SENSORS; i++) {
        if (!s_sensors[i].used) return &s_sensors[i];
        /* prefer to replace unconfirmed entries, then the oldest */
        if (victim == NULL ||
            (!s_sensors[i].confirmed && victim->confirmed) ||
            (s_sensors[i].confirmed == victim->confirmed && s_sensors[i].last_ms < victim->last_ms)) {
            victim = &s_sensors[i];
        }
    }
    return victim;
}

static void sensor_update(const tpms_reading_t *r, int16_t rssi, uint32_t now)
{
    sensor_t *s = sensor_find(r);
    if (s == NULL) {
        s = sensor_alloc();
        memset(s, 0, sizeof *s);
        s->used = 1;
    }
    s->r = *r;
    s->rssi = rssi;
    s->last_ms = now;
    if (s->hits < 255u) s->hits++;

    if (!s->confirmed && s->hits >= 2u) {
        s->confirmed = 1;
        out("# NEW SENSOR id=%0*lX (%s)\r\n", r->id_digits, (unsigned long)r->id, r->proto);
    }
    if (s->confirmed) s->updates++;
}

/* ---------------- burst handling ---------------- */

static void decode_buffer(int16_t rssi)
{
    tpms_reading_t r[12];
    int n = tpms_decode_burst(s_burst, s_burst_n, r, 12);
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < n; i++) {
        s_valid_frames++;
        sensor_update(&r[i], rssi, now);
    }

    /* one log line per sensor seen in this burst (packets are repeats) */
    for (int i = 0; i < n; i++) {
        int dup = 0;
        for (int j = 0; j < i; j++) {
            if (r[j].id == r[i].id && strcmp(r[j].proto, r[i].proto) == 0) { dup = 1; break; }
        }
        if (dup) continue;
        sensor_t *s = sensor_find(&r[i]);
        char bar[12], psi[12];
        fmt_bar(bar, sizeof bar, r[i].kpa_x10);
        fmt_psi(psi, sizeof psi, r[i].kpa_x10);
        out("T id=%0*lX proto=%s P=%d.%dkPa %sbar %spsi", r[i].id_digits, (unsigned long)r[i].id,
            r[i].proto, r[i].kpa_x10 / 10, r[i].kpa_x10 % 10, bar, psi);
        if (r[i].temp_valid) out(" T=%dC", r[i].temp_c);
        out(" rssi=%d %s\r\n", rssi, (s && s->confirmed) ? "OK" : "(needs 2nd packet)");
    }
    s_burst_n = 0;
}

static void on_burst_start(void)
{
    s_in_frame = 1;
    s_burst_n = 0;
    s_line_count = 0;
    s_signals++;
    if (s_verbose) {
        uint32_t t = HAL_GetTick();
        out("\r\n# BURST %lu t=%lu.%03lus\r\nP", (unsigned long)s_signals,
            (unsigned long)(t / 1000u), (unsigned long)(t % 1000u));
    }
}

static void on_pulse(uint16_t e)
{
    if (s_burst_n >= BURST_MAX) {
        decode_buffer(rfc_rssi_dbm());          /* very long burst: decode what we have */
    }
    s_burst[s_burst_n++] = e;

    if (s_verbose) {
        if (s_line_count == PULSES_PER_LINE) { usb_print("\r\nP"); s_line_count = 0; }
        out(" %c%u", (e & RFC_LEVEL_BIT) ? '+' : '-', (unsigned)(e & RFC_DUR_MASK));
        s_line_count++;
    }
}

static void on_burst_end(int16_t peak, uint16_t samples)
{
    s_in_frame = 0;
    if (s_verbose) {
        uint32_t us = (uint32_t)samples * RFC_SAMPLE_US;
        out("\r\n# END pulses=%lu peak=%ddBm dur=%lu.%lums\r\n", (unsigned long)s_burst_n, peak,
            (unsigned long)(us / 1000u), (unsigned long)((us % 1000u) / 100u));
    }
    decode_buffer(peak);
}

static void process_ring(void)
{
    uint16_t e;
    uint32_t budget = 600;
    while (budget-- && rfc_pop(&e)) {
        if (e == RFC_MARK_START) {
            on_burst_start();
        } else if (e == RFC_MARK_END) {
            uint16_t peak = 0, samples = 0;
            if (!rfc_pop(&peak) || !rfc_pop(&samples)) { peak = 0; samples = 0; }
            if (s_in_frame) on_burst_end((int16_t)peak, samples);
        } else if (s_in_frame) {
            on_pulse(e);
        }
    }
}

/* ---------------- keyboard commands from the PC ---------------- */

static void handle_commands(void)
{
    int c;
    while ((c = usb_getc()) >= 0) {
        switch (c) {
            case 'v': s_verbose = !s_verbose; out("# raw pulse dump %s\r\n", s_verbose ? "ON" : "OFF"); break;
            case 'f': s_mod = CC1101_MOD_FSK; radio_configure(); break;
            case 'o': s_mod = CC1101_MOD_OOK; radio_configure(); break;
            case '+': rfc_set_threshold_db((int8_t)(rfc_get_threshold_db() + 1));
                      out("# trigger = %d dB above noise\r\n", rfc_get_threshold_db()); break;
            case '-': rfc_set_threshold_db((int8_t)(rfc_get_threshold_db() - 1));
                      out("# trigger = %d dB above noise\r\n", rfc_get_threshold_db()); break;
            case 'r': s_rssi_stream = !s_rssi_stream;
                      out("# RSSI stream %s\r\n", s_rssi_stream ? "ON" : "OFF"); break;
            case 'c': memset(s_sensors, 0, sizeof s_sensors); s_signals = 0; s_valid_frames = 0;
                      out("# cleared\r\n"); break;
            case '?': case 'h': print_help(); break;
            default: break;
        }
    }
}

/* ---------------- screen ---------------- */

static uint32_t confirmed_count(void)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < MAX_SENSORS; i++) n += s_sensors[i].confirmed;
    return n;
}

/* the k-th confirmed sensor */
static sensor_t *confirmed_at(uint32_t k)
{
    for (uint32_t i = 0; i < MAX_SENSORS; i++) {
        if (s_sensors[i].confirmed) {
            if (k == 0u) return &s_sensors[i];
            k--;
        }
    }
    return NULL;
}

static void build_screen(uint32_t now)
{
#if !APP_RADIO_ENABLED
    lcd_want(0, "USB TEST  USB:%s", usb_link_text());
    lcd_want(1, "Radio disabled");
    lcd_want(2, "Up %lus", (unsigned long)(now / 1000u));
    lcd_want(3, "Want: USB:OK");
    return;
#endif
    if (!s_radio_ok) {
        lcd_center(0, APP_TITLE);
        lcd_want(1, "RADIO ERROR");
        lcd_want(2, "Check CC1101 wiring");
        lcd_want(3, "then press RESET");
        return;
    }

    uint32_t nconf = confirmed_count();
    if (nconf == 0u) {
        static const char spin[4] = { '|', '/', '-', '\\' };
        lcd_center(0, APP_TITLE);
        lcd_want(1, "Searching sensor %c", spin[(now / 500u) % 4u]);
        lcd_want(2, "RF%5ddBm 433.92MHz", rfc_rssi_dbm());
        lcd_want(3, "Signals:%-4lu Valid:%lu", (unsigned long)s_signals, (unsigned long)s_valid_frames);
        return;
    }

    if (s_show_idx >= nconf) s_show_idx = 0;
    sensor_t *s = confirmed_at(s_show_idx);
    if (s == NULL) return;

    int kpa = s->r.kpa_x10 / 10;
    int low = kpa < LOW_PRESSURE_KPA, high = kpa > HIGH_PRESSURE_KPA;
    uint8_t blink = (uint8_t)((now / 1000u) & 1u);

    /* row 0: title, or a flashing alert */
    if ((low || high) && blink) lcd_center(0, low ? "!! LOW PRESSURE !!" : "!! HIGH PRESSURE !!");
    else                        lcd_center(0, APP_TITLE);

    /* row 1: pressure */
    char bar[12], psi[12];
    fmt_bar(bar, sizeof bar, s->r.kpa_x10);
    fmt_psi(psi, sizeof psi, s->r.kpa_x10);
    lcd_want(1, "P:%sbar  %spsi", bar, psi);

    /* row 2: temperature + signal */
    if (s->r.temp_valid) lcd_want(2, "T:%dC  RF:%ddBm", s->r.temp_c, s->rssi);
    else                 lcd_want(2, "T:--   RF:%ddBm", s->rssi);

    /* row 3: id + age (+ sensor index if several) */
    char age[8];
    uint32_t elapsed = now - s->last_ms;
    fmt_age(age, sizeof age, elapsed);
    if (nconf > 1u) {
        lcd_want(3, "%u/%lu ID:%0*lX %s%s", (unsigned)(s_show_idx + 1u), (unsigned long)nconf,
                 s->r.id_digits, (unsigned long)s->r.id, age, elapsed > STALE_MS ? "!" : "");
    } else {
        lcd_want(3, "ID:%0*lX  %s%s", s->r.id_digits, (unsigned long)s->r.id, age,
                 elapsed > STALE_MS ? " old" : " ago");
    }
}

/* LED: slow heartbeat normally, fast blink on a pressure alert */
static void led_service(uint32_t now)
{
    uint32_t period = 500u;
    sensor_t *s = confirmed_count() ? confirmed_at(s_show_idx) : NULL;
    if (s != NULL) {
        int kpa = s->r.kpa_x10 / 10;
        if (kpa < LOW_PRESSURE_KPA || kpa > HIGH_PRESSURE_KPA) period = 100u;
    }
    if (now - t_led >= period) {
        t_led = now;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

/* ---------------- public ---------------- */

void app_init(TIM_HandleTypeDef *htim_capture, TIM_HandleTypeDef *htim_rssi)
{
    rfc_init(htim_capture, htim_rssi);
    memset(s_lcd_have, 0, sizeof s_lcd_have);
    memset(s_lcd_want, 0, sizeof s_lcd_want);
    lcd_clear();
    radio_configure();
    t_status = t_lcd = t_rssi = t_led = t_cycle = HAL_GetTick();
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();

    process_ring();
    handle_commands();
    if (usb_take_open_event()) {
        print_help();
    }

    if (!s_in_frame) {
        if (now - t_status >= STATUS_PERIOD_MS) {
            t_status = now;
            out("S t=%lus rf=%ddBm noise=%d signals=%lu valid=%lu sensors=%lu lost=%lu\r\n",
                (unsigned long)(now / 1000u), rfc_rssi_dbm(), rfc_floor_dbm(),
                (unsigned long)s_signals, (unsigned long)s_valid_frames,
                (unsigned long)confirmed_count(), (unsigned long)rfc_overflows());
        }
        if (s_rssi_stream && now - t_rssi >= RSSI_STREAM_MS) {
            t_rssi = now;
            out("R %d\r\n", rfc_rssi_dbm());
        }
    }

    if (now - t_cycle >= CYCLE_MS) {
        t_cycle = now;
        s_show_idx++;
    }
    if (now - t_lcd >= LCD_PERIOD_MS) {
        t_lcd = now;
        build_screen(now);
    }
    lcd_service();
    led_service(now);
    usb_poll();
}
