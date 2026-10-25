/*
 * Multi-protocol TPMS decoder. See tpms_decode.h for the overview.
 *
 * Pipeline for one burst:
 *   1. Estimate the chip width T (nominal 52 us) from the burst itself, so a
 *      sensor whose crystal is a little off still decodes.
 *   2. Convert pulses to chips. Edge positions are rounded on the CUMULATIVE
 *      time axis, so per-pulse timing errors do not accumulate.
 *   3. Try both polarities (we don't know which FSK tone the CC1101 maps to 1).
 *   4. Find the preamble end pattern, Manchester-decode the bits after it.
 *   5. Hand the bytes to each protocol; keep only checksum-valid frames.
 */
#include "tpms_decode.h"
#include <string.h>

#define LEVEL_BIT       0x8000u
#define DUR_MASK        0x7FFFu

#define NOMINAL_CHIP_US 52u
#define MAX_CHIPS       4096u          /* per segment */
#define MAX_FRAME_BITS  96u

/* After inversion (rtl_433 convention) the preamble ends ...1010 1010 1010 1001 */
#define PREAMBLE        0xAAA9u
#define PREAMBLE_LEN    16u

static uint8_t  s_chips[MAX_CHIPS / 8];
static uint32_t s_nchips;

/* ---------------- small helpers ---------------- */

static inline uint8_t chip_at(uint32_t i)
{
    return (uint8_t)((s_chips[i >> 3] >> (7u - (i & 7u))) & 1u);
}

static inline void chip_push(uint8_t v)
{
    if (s_nchips >= MAX_CHIPS) return;
    uint32_t i = s_nchips++;
    if (v) s_chips[i >> 3] |=  (uint8_t)(0x80u >> (i & 7u));
    else   s_chips[i >> 3] &= (uint8_t)~(0x80u >> (i & 7u));
}

static uint8_t crc8(const uint8_t *b, uint32_t n, uint8_t poly, uint8_t init)
{
    uint8_t c = init;
    for (uint32_t i = 0; i < n; i++) {
        c ^= b[i];
        for (int k = 0; k < 8; k++) {
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ poly) : (uint8_t)(c << 1);
        }
    }
    return c;
}

static uint8_t xor_bytes(const uint8_t *b, uint32_t n)
{
    uint8_t x = 0;
    for (uint32_t i = 0; i < n; i++) x ^= b[i];
    return x;
}

/* Manchester decode starting at chip 'start': pair (a,b) with a!=b gives bit b.
 * Stops at the first invalid pair. Returns number of bits decoded. */
static uint32_t manchester(uint32_t start, uint8_t *out, uint32_t max_bits)
{
    memset(out, 0, (max_bits + 7u) / 8u);
    uint32_t nb = 0;
    for (uint32_t i = start; i + 1u < s_nchips && nb < max_bits; i += 2u) {
        uint8_t a = chip_at(i), b = chip_at(i + 1u);
        if (a == b) break;
        if (b) out[nb >> 3] |= (uint8_t)(0x80u >> (nb & 7u));
        nb++;
    }
    return nb;
}

static int plausible(int kpa_x10, int temp_c, uint8_t temp_valid)
{
    if (kpa_x10 < 0 || kpa_x10 > 9000) return 0;              /* 0..900 kPa */
    if (temp_valid && (temp_c < -40 || temp_c > 125)) return 0;
    return 1;
}

/* ---------------- protocols ----------------
 * Each returns 1 and fills 'r' if the frame is valid. 'b' holds 'nb' bits. */

static int p_vdo_hyundai(const uint8_t *b, uint32_t nb, tpms_reading_t *r)
{
    if (nb < 80) return 0;
    if (crc8(b, 9, 0x07, 0xAA) != b[9]) return 0;
    r->proto = "VDO-HK";
    r->id = (uint32_t)b[1] << 24 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 8 | b[4];
    r->id_digits = 8;
    r->kpa_x10 = (int16_t)((b[6] * 1375 + 50) / 100);          /* 1.375 kPa/step */
    r->temp_c = (int16_t)(b[7] - 50);
    r->temp_valid = 1;
    return plausible(r->kpa_x10, r->temp_c, 1);
}

static int p_vdo_citroen(const uint8_t *b, uint32_t nb, tpms_reading_t *r)
{
    if (nb < 80) return 0;
    if (xor_bytes(&b[1], 9) != 0) return 0;
    if (b[6] == 0 || b[7] == 0) return 0;
    r->proto = "VDO-PSA";
    r->id = (uint32_t)b[1] << 24 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 8 | b[4];
    r->id_digits = 8;
    r->kpa_x10 = (int16_t)((b[6] * 1364 + 50) / 100);          /* 1.364 kPa/step */
    r->temp_c = (int16_t)(b[7] - 50);
    r->temp_valid = 1;
    return plausible(r->kpa_x10, r->temp_c, 1);
}

static int p_renault(const uint8_t *b, uint32_t nb, tpms_reading_t *r)
{
    if (nb < 72) return 0;
    if (crc8(b, 8, 0x07, 0x00) != b[8]) return 0;
    r->proto = "Renault";
    r->id = (uint32_t)b[5] << 16 | (uint32_t)b[4] << 8 | b[3];  /* little-endian */
    r->id_digits = 6;
    int raw = ((b[0] & 0x03) << 8) | b[1];
    r->kpa_x10 = (int16_t)((raw * 75 + 5) / 10);               /* 0.75 kPa/step */
    r->temp_c = (int16_t)(b[2] - 30);
    r->temp_valid = 1;
    return plausible(r->kpa_x10, r->temp_c, 1);
}

static int p_ford(const uint8_t *b, uint32_t nb, tpms_reading_t *r)
{
    if (nb < 64) return 0;
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++) sum = (uint8_t)(sum + b[i]);
    if (sum != b[7]) return 0;
    uint8_t mode = b[6] & 0x4C;
    if (mode != 0x08 && mode != 0x04 && mode != 0x44) return 0;   /* learn / rest / moving */
    if (b[6] & 0x90) return 0;
    r->proto = "Ford";
    r->id = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
    r->id_digits = 8;
    int psi_x4 = ((b[6] & 0x20) << 3) | b[4];
    r->kpa_x10 = (int16_t)((psi_x4 * 68948 + 2000) / 4000);    /* PSI*4 -> 0.1 kPa (6.8948 kPa/PSI) */
    r->temp_valid = (b[5] & 0x80) ? 0 : 1;
    r->temp_c = (int16_t)((b[5] & 0x7F) - 56);
    return plausible(r->kpa_x10, r->temp_c, r->temp_valid);
}

static int p_vdo_tg1c(const uint8_t *b, uint32_t nb, tpms_reading_t *r)
{
    if (nb < 72) return 0;
    if (xor_bytes(b, 9) != 0) return 0;
    r->proto = "VDO-TG1C";
    r->id = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
    r->id_digits = 8;
    r->kpa_x10 = (int16_t)((b[5] * 138 + 5) / 10);             /* 1.38 kPa/step */
    r->temp_c = (int16_t)(b[6] - 50);
    r->temp_valid = 1;
    return plausible(r->kpa_x10, r->temp_c, 1);
}

typedef int (*proto_fn)(const uint8_t *, uint32_t, tpms_reading_t *);
static const proto_fn s_protocols[] = {
    p_vdo_hyundai, p_renault, p_ford, p_vdo_citroen, p_vdo_tg1c,
};

/* ---------------- search one chip segment ---------------- */

static int search_segment(tpms_reading_t *out, int n_out, int max_out)
{
    uint8_t  frame[MAX_FRAME_BITS / 8];
    uint16_t window = 0;

    for (uint32_t i = 0; i < s_nchips && n_out < max_out; i++) {
        window = (uint16_t)((window << 1) | chip_at(i));
        if (i + 1u < PREAMBLE_LEN || window != PREAMBLE) continue;

        uint32_t nb = manchester(i + 1u, frame, MAX_FRAME_BITS);
        if (nb < 64) continue;

        for (uint32_t p = 0; p < sizeof s_protocols / sizeof s_protocols[0]; p++) {
            tpms_reading_t r;
            memset(&r, 0, sizeof r);
            if (s_protocols[p](frame, nb, &r)) {
                out[n_out++] = r;
                break;                      /* one protocol per frame */
            }
        }
    }
    return n_out;
}

/* ---------------- chip width estimate ---------------- */

/* ---------------- clock recovery ----------------
 *
 * 1. Find a PREAMBLE RUN at pulse level: >= MIN_RUN consecutive pulses that are
 *    each ~1 chip long (alternating 1010...). No exact clock is needed for this.
 * 2. Initial chip width T = run duration / run length.
 * 3. Refine T by least squares on the edge positions, over a window that grows
 *    1.5x per step (each step stays inside the +/-half-chip lock range).
 * 4. Convert the packet to chips with the refined T and search for frames.
 * Integer only (Cortex-M3 has no FPU): times in Q8 (1/256 us).
 */

#define MIN_RUN        10u
#define T_MIN_Q8       (20u << 8)   /* plausible chip width range: 20..120 us */
#define T_MAX_Q8       (120u << 8)
#define MAX_EDGES      320u      /* per packet attempt (~2 packets of edges) */

static uint16_t s_t[MAX_EDGES + 1];   /* edge times from the run start, in us (max 65 ms) */
#define TQ8(j) ((uint32_t)s_t[(j)] << 8)  /* same, in Q8 */

/* Least-squares line fit  t = a + p*T  over edges with t <= limit, where
 * p = round((t - a)/T) uses the previous estimate. Fitting the intercept 'a'
 * as well matters: the first edge has jitter too, and forcing the line through
 * it biases T. Updates *T and *a (both Q8). */
static void fit_line(uint32_t m, uint32_t *T, int32_t *a, uint32_t limit_q8)
{
    /* Per-edge maths in 32 bit (hardware divide on Cortex-M3); only the
     * accumulators and the final solve are 64 bit. */
    int64_t n = 0, sp = 0, st = 0, spp = 0, spt = 0;
    const int32_t Ti = (int32_t)*T, ai = *a;
    for (uint32_t j = 0; j <= m && TQ8(j) <= limit_q8; j++) {
        int32_t t = (int32_t)TQ8(j);
        int32_t p = (t - ai + Ti / 2) / Ti;
        n++; sp += p; st += t; spp += (int64_t)p * p; spt += (int64_t)p * t;
    }
    int64_t den = n * spp - sp * sp;
    if (n < 4 || den <= 0) return;
    int64_t Tn = (n * spt - sp * st) / den;
    if (Tn < (int64_t)T_MIN_Q8 || Tn > (int64_t)T_MAX_Q8) return;   /* never accept an impossible clock */
    *T = (uint32_t)Tn;
    *a = (int32_t)((st - Tn * sp) / n);
}

/* Decode one candidate packet starting at pulse index 'start' */
static int try_packet(const uint16_t *pulses, uint32_t n, uint32_t start, uint32_t run_len,
                      tpms_reading_t *out, int n_out, int max_out)
{
    /* edge times relative to the start of the run */
    uint32_t m = 0, t = 0;
    for (uint32_t i = start; i < n && m < MAX_EDGES; i++) {
        uint32_t d = pulses[i] & DUR_MASK;
        if (d > 8u * NOMINAL_CHIP_US * 2u) break;          /* gap: end of packet */
        t += d;
        if (t > 65000u) break;
        s_t[++m] = (uint16_t)t;
    }
    s_t[0] = 0;
    if (m < 40u) return n_out;
    if (run_len > m) run_len = m;     /* noise can look like a very long "preamble" */

    uint32_t T = TQ8(run_len) / run_len;                    /* initial estimate from the run */
    if (T < T_MIN_Q8 || T > T_MAX_Q8) return n_out;
    int32_t  a = 0;                                         /* intercept, Q8 */
    fit_line(m, &T, &a, TQ8(run_len));                      /* refine on the run itself */
    uint32_t span = run_len;
    for (int it = 0; it < 16; it++) {                       /* grow window 1.5x per step */
        span = span * 3u / 2u;
        uint32_t limit = span * T;
        if (limit >= TQ8(m)) break;
        fit_line(m, &T, &a, limit);
    }
    fit_line(m, &T, &a, TQ8(m));
    fit_line(m, &T, &a, TQ8(m));                            /* second pass: positions now final */

    for (uint8_t inv = 0; inv < 2u && n_out < max_out; inv++) {
        s_nchips = 0;
        uint32_t prev = 0;
        for (uint32_t j = 1; j <= m; j++) {
            int32_t tj = (int32_t)TQ8(j) - a;
            uint32_t pos = (tj <= 0) ? 0u : (uint32_t)((tj + (int32_t)(T / 2u)) / (int32_t)T);
            uint32_t k = pos - prev;
            if (k == 0u) continue;                          /* glitch */
            if (k > 4u) k = 4u;
            uint8_t level = (uint8_t)((((pulses[start + j - 1u] & LEVEL_BIT) ? 1u : 0u)) ^ inv);
            for (uint32_t c = 0; c < k; c++) chip_push(level);
            prev = pos;
        }
        n_out = search_segment(out, n_out, max_out);
    }
    (void)run_len;
    return n_out;
}

/* ---------------- public ---------------- */

#define MAX_ATTEMPTS 48u          /* bounds CPU time on long noise bursts */

int tpms_decode_burst(const uint16_t *pulses, uint32_t n, tpms_reading_t *out, int max_out)
{
    int n_out = 0;
    uint32_t attempts = 0;

    /* A preamble run = consecutive pulses of about the same length (one chip each).
     * Each pulse is judged against the run's running average, so one jittery
     * pulse does not break it, and any chip width from ~25 to ~80 us works. */
    uint32_t i = 0;
    while (i < n && attempts < MAX_ATTEMPTS && n_out < max_out) {
        uint32_t d0 = pulses[i] & DUR_MASK;
        if (d0 < 20u || d0 > 90u) { i++; continue; }
        uint32_t sum = d0, j = i + 1u;
        while (j < n) {
            uint32_t d = pulses[j] & DUR_MASK;
            uint32_t avg = sum / (j - i);
            if (d * 2u < avg || d * 2u > avg * 3u) break;      /* outside 0.5x .. 1.5x */
            sum += d;
            j++;
        }
        uint32_t run = j - i;
        if (run >= MIN_RUN) {
            attempts++;
            n_out = try_packet(pulses, n, i, run, out, n_out, max_out);
            i = j;
        } else {
            i++;
        }
    }
    return n_out;
}
