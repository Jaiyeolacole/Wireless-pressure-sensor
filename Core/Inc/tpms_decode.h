#ifndef TPMS_DECODE_H
#define TPMS_DECODE_H

/*
 * Multi-protocol 433.92 MHz TPMS decoder.
 *
 * Input : one radio burst as captured by rf_capture (pulse entries: bit15 = level,
 *         bits14..0 = duration in microseconds).
 * Output: zero or more readings whose checksum/CRC validated.
 *
 * Supported formats (all FSK, Manchester, ~9.6 kbit/s = 52 us chips,
 * preamble 55 55 .. 56). Frame layouts as documented by the open-source
 * rtl_433 project (github.com/merbanan/rtl_433), re-implemented here:
 *   - VDO (Hyundai/Kia/Continental TG1C-type)  CRC-8 poly 0x07 init 0xAA
 *   - VDO (Citroen/Peugeot/Mitsubishi)          XOR checksum
 *   - Renault/Dacia                             CRC-8 poly 0x07 init 0x00
 *   - Ford (Continental)                        8-bit sum + flag check
 *   - VDO TG1C (Fiat/Abarth/Mazda)              XOR checksum + range check
 *
 * NOTE: none of these is confirmed to be the format of a given aftermarket
 * sensor. A reading is only produced when a checksum validates.
 */

#include <stdint.h>

typedef struct {
    const char *proto;          /* short protocol name */
    uint32_t    id;             /* sensor ID */
    uint8_t     id_digits;      /* hex digits to print (6 or 8) */
    int16_t     kpa_x10;        /* pressure in 0.1 kPa */
    int16_t     temp_c;         /* temperature in deg C */
    uint8_t     temp_valid;
} tpms_reading_t;

/* Returns number of readings written to out[] (max 'max_out'). */
int tpms_decode_burst(const uint16_t *pulses, uint32_t n, tpms_reading_t *out, int max_out);

#endif /* TPMS_DECODE_H */
