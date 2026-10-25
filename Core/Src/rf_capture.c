#include "rf_capture.h"
#include "cc1101.h"
#include "main.h"

#define RING_SIZE  2048u                    /* entries; power of two */

/* Sample period is RFC_SAMPLE_US (250 us) */
#define START_SAMPLES   2                   /* 0.5 ms above threshold -> burst start (fast: keeps most of the preamble) */
#define END_SAMPLES     40                  /* 10 ms below -> burst end (keeps a sensor's repeated packets in one burst) */
#define HYSTERESIS_DB   3
#define MAX_BURST_SAMPLES 4000              /* 1 s: longer than any TPMS transmission -> it's interference */
#define WARMUP_SAMPLES  400                 /* 100 ms: let the CC1101 AGC settle before trusting the floor */

static TIM_HandleTypeDef *s_tim_cap;
static TIM_HandleTypeDef *s_tim_rssi;

static uint16_t          s_ring[RING_SIZE];
static volatile uint32_t s_head;            /* written by ISRs */
static volatile uint32_t s_tail;            /* written by main loop */
static volatile uint32_t s_overflows;

/* RSSI / burst state (owned by TIM3 ISR) */
static volatile int16_t  s_rssi = -120;
static volatile int32_t  s_floor_q8 = 0;    /* noise floor in dBm * 256 (fixed point) */
static uint16_t          s_warmup;          /* samples left before detection is allowed */
static volatile int8_t   s_thr_db = 15;
static volatile uint8_t  s_in_burst;
static uint8_t           s_above, s_below;
static int16_t           s_peak;
static uint32_t          s_burst_samples;

/* Capture state (owned by TIM2 ISR) */
static uint16_t          s_last_cap;
static volatile uint8_t  s_skip_first;      /* first pulse after burst start is partial: drop it */

/* ---------------- ring buffer ---------------- */

/* Called from TIM2 ISR (highest priority) or with interrupts disabled */
static inline void ring_push(uint16_t v)
{
    uint32_t next = (s_head + 1u) & (RING_SIZE - 1u);
    if (next == s_tail) {
        s_overflows++;
        return;
    }
    s_ring[s_head] = v;
    s_head = next;
}

/* TIM3 runs at lower priority than TIM2, so block TIM2 while it pushes */
static void ring_push_locked(uint16_t v)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ring_push(v);
    __set_PRIMASK(primask);
}

int rfc_pop(uint16_t *out)
{
    if (s_tail == s_head) return 0;
    *out = s_ring[s_tail];
    s_tail = (s_tail + 1u) & (RING_SIZE - 1u);
    return 1;
}

/* ---------------- capture interrupt gating ----------------
 * The edge interrupt is only enabled DURING a burst. Between bursts GDO0 outputs
 * pure noise (tens of thousands of edges per second): servicing those would
 * waste CPU and delay the USB interrupt for nothing.
 */
static void capture_arm(void)
{
    TIM_TypeDef *tim = s_tim_cap->Instance;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* wait for the edge that the pin is NOT currently at */
    if (CC1101_GDO0_GPIO_Port->IDR & CC1101_GDO0_Pin) tim->CCER |= TIM_CCER_CC1P;
    else                                              tim->CCER &= ~TIM_CCER_CC1P;
    s_last_cap = (uint16_t)tim->CNT;
    s_skip_first = 1;                                 /* first pulse is partial */
    tim->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);         /* drop any stale capture */
    tim->DIER |= TIM_DIER_CC1IE;
    __set_PRIMASK(primask);
}

static void capture_disarm(void)
{
    s_tim_cap->Instance->DIER &= ~TIM_DIER_CC1IE;
}

/* ---------------- public API ---------------- */

void rfc_init(TIM_HandleTypeDef *htim_capture, TIM_HandleTypeDef *htim_rssi)
{
    s_tim_cap  = htim_capture;
    s_tim_rssi = htim_rssi;
}

void rfc_start(void)
{
    s_head = s_tail = 0;
    s_in_burst = 0;
    s_above = s_below = 0;
    s_warmup = WARMUP_SAMPLES;

    /* Start the timer + capture channel, but keep the edge interrupt OFF until a burst */
    HAL_TIM_IC_Start_IT(s_tim_cap, TIM_CHANNEL_1);
    capture_disarm();
    HAL_TIM_Base_Start_IT(s_tim_rssi);
}

void rfc_stop(void)
{
    HAL_TIM_Base_Stop_IT(s_tim_rssi);
    HAL_TIM_IC_Stop_IT(s_tim_cap, TIM_CHANNEL_1);
    s_in_burst = 0;
}

int16_t rfc_rssi_dbm(void)   { return s_rssi; }
int16_t rfc_floor_dbm(void)  { return (int16_t)(s_floor_q8 / 256); }
uint8_t rfc_in_burst(void)   { return s_in_burst; }
uint32_t rfc_overflows(void) { return s_overflows; }
int8_t rfc_get_threshold_db(void) { return s_thr_db; }

void rfc_set_threshold_db(int8_t db)
{
    if (db < 3)  db = 3;
    if (db > 40) db = 40;
    s_thr_db = db;
}

/* ---------------- TIM3: RSSI sampling + burst detection ---------------- */

static void rssi_tick(void)
{
    int16_t rssi = cc1101_read_rssi_dbm_isr();
    s_rssi = rssi;

    if (s_warmup) {
        /* Fast-tracking average while the AGC settles; no detection yet */
        if (s_warmup == WARMUP_SAMPLES) s_floor_q8 = (int32_t)rssi * 256;
        else                            s_floor_q8 += ((int32_t)rssi * 256 - s_floor_q8) / 8;
        s_warmup--;
        return;
    }

    int16_t floor_db = (int16_t)(s_floor_q8 / 256);

    if (!s_in_burst) {
        /* Track the noise floor slowly (time constant ~128 ms), only when quiet */
        s_floor_q8 += ((int32_t)rssi * 256 - s_floor_q8) / 512;

        if (rssi > floor_db + s_thr_db) {
            if (++s_above >= START_SAMPLES) {
                s_in_burst = 1;
                s_below = 0;
                s_peak = rssi;
                s_burst_samples = 0;
                ring_push_locked(RFC_MARK_START);
                capture_arm();
            }
        } else {
            s_above = 0;
        }
    } else {
        s_burst_samples++;
        if (rssi > s_peak) s_peak = rssi;

        uint8_t ended = 0;
        if (rssi < floor_db + s_thr_db - HYSTERESIS_DB) {
            if (++s_below >= END_SAMPLES) ended = 1;
        } else {
            s_below = 0;
        }

        if (s_burst_samples >= MAX_BURST_SAMPLES) {
            /* A carrier that never stops is not a TPMS sensor: accept it as the new floor */
            ended = 1;
            s_floor_q8 = (int32_t)rssi * 256;
        }

        if (ended) {
            capture_disarm();
            s_in_burst = 0;
            s_above = 0;
            uint32_t primask = __get_PRIMASK();
            __disable_irq();                       /* keep the 3 entries together */
            ring_push(RFC_MARK_END);
            ring_push((uint16_t)s_peak);
            ring_push((uint16_t)(s_burst_samples > 0xFFFF ? 0xFFFF : s_burst_samples));
            __set_PRIMASK(primask);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (s_tim_rssi != NULL && htim->Instance == s_tim_rssi->Instance) {
        rssi_tick();
    }
}

/* ---------------- TIM2 CH1: edge timestamps on GDO0 ---------------- */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (s_tim_cap == NULL || htim->Instance != s_tim_cap->Instance) return;
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    TIM_TypeDef *tim = htim->Instance;
    uint16_t cap = (uint16_t)tim->CCR1;

    /* STM32F1 timers cannot capture both edges at once, so we switch the
     * polarity after every edge.
     * CC1P = 0 -> we just caught a RISING edge  -> the pulse that ended was LOW.
     * CC1P = 1 -> we just caught a FALLING edge -> the pulse that ended was HIGH. */
    uint32_t ccer = tim->CCER;
    uint16_t prev_level_high = (ccer & TIM_CCER_CC1P) ? 1u : 0u;

    /* Choose the next edge from the ACTUAL pin level, not by blindly toggling:
     * if an edge was ever missed (noise), blind toggling would invert every
     * HIGH/LOW label from then on. Reading the pin re-syncs on every edge. */
    if (CC1101_GDO0_GPIO_Port->IDR & CC1101_GDO0_Pin) {
        ccer |= TIM_CCER_CC1P;      /* pin is high -> wait for falling */
    } else {
        ccer &= ~TIM_CCER_CC1P;     /* pin is low  -> wait for rising  */
    }
    tim->CCER = ccer;

    uint16_t dur = (uint16_t)(cap - s_last_cap);   /* 16-bit wrap-around is handled by unsigned math */
    s_last_cap = cap;

    if (!s_in_burst) return;
    if (s_skip_first) { s_skip_first = 0; return; }

    if (dur > RFC_DUR_MAX) dur = RFC_DUR_MAX;
    ring_push((uint16_t)((prev_level_high ? RFC_LEVEL_BIT : 0u) | dur));
}
