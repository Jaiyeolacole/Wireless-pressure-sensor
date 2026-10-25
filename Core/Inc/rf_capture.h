#ifndef RF_CAPTURE_H
#define RF_CAPTURE_H

/*
 * Interrupt side of the receiver.
 *
 *  TIM3 (every 250 us): reads RSSI from the CC1101 and decides "is something
 *        transmitting right now?" by comparing against a slowly-tracked noise floor.
 *  TIM2 CH1 (PA0 = CC1101 GDO0): input capture. Every edge of the demodulated
 *        data is timestamped by hardware with 1 us resolution. While a burst is
 *        active, each pulse (level + duration) is pushed into a ring buffer.
 *
 * The main loop pops entries from the ring buffer with rfc_pop().
 */

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Ring entries */
#define RFC_MARK_START   0xFFFFu  /* a burst began */
#define RFC_MARK_END     0xFFFEu  /* a burst ended; followed by 2 entries: peak dBm (int16), duration in samples (RFC_SAMPLE_US each) */
#define RFC_LEVEL_BIT    0x8000u  /* pulse entry: bit15 = level (1 = high), bits14..0 = duration in us */
#define RFC_DUR_MASK     0x7FFFu
#define RFC_DUR_MAX      0x7FE0u  /* longer pulses are clamped */

#define RFC_SAMPLE_US    250u     /* RSSI sampling period (must match TIM3: 1 MHz / 250) */

void     rfc_init(TIM_HandleTypeDef *htim_capture, TIM_HandleTypeDef *htim_rssi);
void     rfc_start(void);
void     rfc_stop(void);

int      rfc_pop(uint16_t *out);       /* 1 = got an entry, 0 = empty */

int16_t  rfc_rssi_dbm(void);           /* latest RSSI sample */
int16_t  rfc_floor_dbm(void);          /* tracked noise floor */
uint8_t  rfc_in_burst(void);
uint32_t rfc_overflows(void);          /* pulses lost because main loop was too slow */

void     rfc_set_threshold_db(int8_t db);   /* burst = RSSI > floor + threshold */
int8_t   rfc_get_threshold_db(void);

#endif /* RF_CAPTURE_H */
