#ifndef CC1101_H
#define CC1101_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ---- Configuration registers (read/write, 0x00-0x2E) ---- */
#define CC1101_IOCFG2    0x00  /* GDO2 pin function */
#define CC1101_IOCFG1    0x01  /* GDO1 pin function (shared with MISO) */
#define CC1101_IOCFG0    0x02  /* GDO0 pin function */
#define CC1101_FIFOTHR   0x03
#define CC1101_SYNC1     0x04
#define CC1101_SYNC0     0x05
#define CC1101_PKTLEN    0x06
#define CC1101_PKTCTRL1  0x07
#define CC1101_PKTCTRL0  0x08  /* packet format: we use ASYNC SERIAL */
#define CC1101_ADDR      0x09
#define CC1101_CHANNR    0x0A
#define CC1101_FSCTRL1   0x0B  /* IF frequency */
#define CC1101_FSCTRL0   0x0C
#define CC1101_FREQ2     0x0D  /* carrier frequency, 3 bytes */
#define CC1101_FREQ1     0x0E
#define CC1101_FREQ0     0x0F
#define CC1101_MDMCFG4   0x10  /* RX filter bandwidth + data rate exponent */
#define CC1101_MDMCFG3   0x11  /* data rate mantissa */
#define CC1101_MDMCFG2   0x12  /* modulation format, sync mode */
#define CC1101_MDMCFG1   0x13
#define CC1101_MDMCFG0   0x14
#define CC1101_DEVIATN   0x15  /* expected FSK deviation */
#define CC1101_MCSM2     0x16
#define CC1101_MCSM1     0x17
#define CC1101_MCSM0     0x18
#define CC1101_FOCCFG    0x19
#define CC1101_BSCFG     0x1A
#define CC1101_AGCCTRL2  0x1B
#define CC1101_AGCCTRL1  0x1C
#define CC1101_AGCCTRL0  0x1D
#define CC1101_FREND1    0x21
#define CC1101_FREND0    0x22
#define CC1101_FSCAL3    0x23
#define CC1101_FSCAL2    0x24
#define CC1101_FSCAL1    0x25
#define CC1101_FSCAL0    0x26
#define CC1101_TEST2     0x2C
#define CC1101_TEST1     0x2D
#define CC1101_TEST0     0x2E

/* ---- Command strobes (single byte, write) ---- */
#define CC1101_SRES      0x30  /* Reset chip */
#define CC1101_SCAL      0x33  /* Calibrate synthesizer */
#define CC1101_SRX       0x34  /* Enter RX */
#define CC1101_SIDLE     0x36  /* Go to IDLE */
#define CC1101_SFRX      0x3A  /* Flush RX FIFO */

/* ---- Status registers (0x30-0x3D) ----
 * These addresses overlap the command strobes above.
 * The chip tells them apart by the BURST bit: to READ a status
 * register you must send (addr | 0xC0). Sending 0x30 alone = SRES!
 */
#define CC1101_PARTNUM   0x30  /* Expect 0x00 */
#define CC1101_VERSION   0x31  /* Expect 0x14 (clones: 0x04) */
#define CC1101_RSSI      0x34
#define CC1101_MARCSTATE 0x35  /* main radio state machine; 0x0D = RX */

#define CC1101_MARCSTATE_RX  0x0D

typedef enum {
    CC1101_OK = 0,
    CC1101_TIMEOUT,
    CC1101_VERIFY_FAIL,   /* a register read back differently than written */
    CC1101_NOT_RX         /* chip did not reach RX state */
} cc1101_status_t;

typedef enum {
    CC1101_MOD_FSK = 0,   /* 2-FSK: most 433 MHz TPMS sensors */
    CC1101_MOD_OOK        /* on-off keying: some cheaper sensors */
} cc1101_mod_t;

cc1101_status_t cc1101_init(SPI_HandleTypeDef *hspi);
cc1101_status_t cc1101_reset(void);
cc1101_status_t cc1101_read_status_reg(uint8_t addr, uint8_t *out);
cc1101_status_t cc1101_read_reg(uint8_t addr, uint8_t *out);
cc1101_status_t cc1101_write_reg(uint8_t addr, uint8_t value);
cc1101_status_t cc1101_strobe(uint8_t cmd);

/* Configure 433.92 MHz, raw asynchronous receive. Demodulated data comes out on GDO0. */
cc1101_status_t cc1101_config_async_rx(cc1101_mod_t mod);

/* Interrupt-safe RSSI read (no HAL_GetTick, no HAL SPI calls). Returns dBm. */
int16_t cc1101_read_rssi_dbm_isr(void);

/* Convert raw RSSI register value to dBm (datasheet section 17.3, offset 74 dB) */
int16_t cc1101_rssi_to_dbm(uint8_t raw);

#endif /* CC1101_H */
