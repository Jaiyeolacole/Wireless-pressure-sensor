#include "cc1101.h"
#include "main.h"   /* CC1101_CS_Pin / CC1101_CS_GPIO_Port come from the CubeMX user label */

/* SPI header byte: bit7 = R/W (1 = read), bit6 = burst, bits5:0 = address */
#define HDR_READ_SINGLE  0x80
#define HDR_READ_BURST   0xC0

/* MISO of SPI1 is PA6. We read it as a plain input to see the CC1101 "ready" signal. */
#define MISO_PORT  GPIOA
#define MISO_PIN   GPIO_PIN_6

#define READY_TIMEOUT_MS 10

static SPI_HandleTypeDef *s_hspi;

static inline void cs_low(void)  { HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_SET); }

/*
 * After CSn goes low, the CC1101 holds SO (our MISO) HIGH until its crystal
 * oscillator is stable, then pulls it LOW = "ready". Never clock data before this.
 * On STM32F1 the input data register reads the pin even in alternate-function mode.
 */
static cc1101_status_t wait_ready(void)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(MISO_PORT, MISO_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > READY_TIMEOUT_MS) {
            return CC1101_TIMEOUT;
        }
    }
    return CC1101_OK;
}

static uint8_t spi_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(s_hspi, &tx, &rx, 1, 10);
    return rx;
}

cc1101_status_t cc1101_init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;
    return cc1101_reset();
}

/*
 * Manual power-on reset sequence (CC1101 datasheet, section "Power-On Start-Up Sequence"):
 * CSn high -> low -> high, hold high >= 40 us, CSn low, wait SO low,
 * send SRES, wait SO low again (reset finished), CSn high.
 */
cc1101_status_t cc1101_reset(void)
{
    cs_high(); HAL_Delay(1);
    cs_low();  HAL_Delay(1);
    cs_high(); HAL_Delay(1);   /* 1 ms >> 40 us, fine for a one-time reset */

    cs_low();
    if (wait_ready() != CC1101_OK) { cs_high(); return CC1101_TIMEOUT; }

    spi_xfer(CC1101_SRES);

    if (wait_ready() != CC1101_OK) { cs_high(); return CC1101_TIMEOUT; }
    cs_high();
    return CC1101_OK;
}

cc1101_status_t cc1101_read_status_reg(uint8_t addr, uint8_t *out)
{
    cs_low();
    if (wait_ready() != CC1101_OK) { cs_high(); return CC1101_TIMEOUT; }
    spi_xfer(addr | HDR_READ_BURST);   /* burst bit selects status reg, not strobe */
    *out = spi_xfer(0x00);             /* dummy byte clocks the value out */
    cs_high();
    return CC1101_OK;
}

cc1101_status_t cc1101_read_reg(uint8_t addr, uint8_t *out)
{
    cs_low();
    if (wait_ready() != CC1101_OK) { cs_high(); return CC1101_TIMEOUT; }
    spi_xfer(addr | HDR_READ_SINGLE);
    *out = spi_xfer(0x00);
    cs_high();
    return CC1101_OK;
}

cc1101_status_t cc1101_write_reg(uint8_t addr, uint8_t value)
{
    cs_low();
    if (wait_ready() != CC1101_OK) { cs_high(); return CC1101_TIMEOUT; }
    spi_xfer(addr);        /* R/W bit = 0 -> write */
    spi_xfer(value);
    cs_high();
    return CC1101_OK;
}

cc1101_status_t cc1101_strobe(uint8_t cmd)
{
    cs_low();
    if (wait_ready() != CC1101_OK) { cs_high(); return CC1101_TIMEOUT; }
    spi_xfer(cmd);
    cs_high();
    return CC1101_OK;
}

/* ===================================================================
 *  RF configuration: 433.92 MHz, raw asynchronous receive
 * =================================================================== */

typedef struct { uint8_t addr; uint8_t val; } reg_t;

/*
 * Values assume the module's 26 MHz crystal.
 * Where a value differs between FSK and OOK, the OOK value is patched in below.
 */
static const reg_t s_async_rx_regs[] = {
    { CC1101_IOCFG2,   0x0E },  /* GDO2 = carrier sense (high while RSSI above threshold) */
    { CC1101_IOCFG1,   0x2E },  /* GDO1 = high impedance (it shares the MISO pin) */
    { CC1101_IOCFG0,   0x0D },  /* GDO0 = demodulated data, asynchronous: THIS is what we capture */
    { CC1101_FIFOTHR,  0x47 },
    { CC1101_PKTCTRL1, 0x00 },
    { CC1101_PKTCTRL0, 0x32 },  /* async serial mode, infinite length, no CRC, no whitening */
    { CC1101_FSCTRL1,  0x08 },  /* IF = 203 kHz */
    { CC1101_FSCTRL0,  0x00 },
    { CC1101_FREQ2,    0x10 },  /* 433.92 MHz = 0x10B071 * 26 MHz / 2^16 */
    { CC1101_FREQ1,    0xB0 },
    { CC1101_FREQ0,    0x71 },
    { CC1101_MDMCFG4,  0x5A },  /* RX bandwidth 325 kHz (wide: cheap sensors are often off-frequency), DRATE_E=10 */
    { CC1101_MDMCFG3,  0x83 },  /* data rate ~38.4 kBaud: sets demodulator sampling, ~2x a typical TPMS chip rate */
    { CC1101_MDMCFG2,  0x00 },  /* 2-FSK, no preamble/sync detection (we want everything) */
    { CC1101_MDMCFG1,  0x02 },
    { CC1101_MDMCFG0,  0xF8 },
    { CC1101_DEVIATN,  0x47 },  /* expect +/-47.6 kHz FSK deviation */
    { CC1101_MCSM2,    0x07 },
    { CC1101_MCSM1,    0x3C },  /* stay in RX */
    { CC1101_MCSM0,    0x18 },  /* auto-calibrate when going IDLE -> RX */
    { CC1101_FOCCFG,   0x16 },
    { CC1101_BSCFG,    0x6C },
    { CC1101_AGCCTRL2, 0x43 },
    { CC1101_AGCCTRL1, 0x40 },
    { CC1101_AGCCTRL0, 0x91 },
    { CC1101_FREND1,   0x56 },
    { CC1101_FREND0,   0x10 },
    { CC1101_FSCAL3,   0xE9 },
    { CC1101_FSCAL2,   0x2A },
    { CC1101_FSCAL1,   0x00 },
    { CC1101_FSCAL0,   0x1F },
    { CC1101_TEST2,    0x81 },
    { CC1101_TEST1,    0x35 },
    { CC1101_TEST0,    0x09 },
};

static uint8_t patched_value(const reg_t *r, cc1101_mod_t mod)
{
    if (mod == CC1101_MOD_OOK) {
        switch (r->addr) {
            case CC1101_MDMCFG2:  return 0x30;  /* ASK/OOK */
            case CC1101_AGCCTRL2: return 0x03;  /* AGC settings recommended for OOK */
            case CC1101_AGCCTRL1: return 0x00;
            case CC1101_AGCCTRL0: return 0x92;
            case CC1101_FREND0:   return 0x11;
            default: break;
        }
    }
    return r->val;
}

cc1101_status_t cc1101_config_async_rx(cc1101_mod_t mod)
{
    const uint32_t n = sizeof(s_async_rx_regs) / sizeof(s_async_rx_regs[0]);

    if (cc1101_strobe(CC1101_SIDLE) != CC1101_OK) return CC1101_TIMEOUT;

    for (uint32_t i = 0; i < n; i++) {
        if (cc1101_write_reg(s_async_rx_regs[i].addr, patched_value(&s_async_rx_regs[i], mod)) != CC1101_OK) {
            return CC1101_TIMEOUT;
        }
    }

    /* Read every register back: catches bad wiring/noise that SPI-ID test alone can miss */
    for (uint32_t i = 0; i < n; i++) {
        uint8_t v;
        if (cc1101_read_reg(s_async_rx_regs[i].addr, &v) != CC1101_OK) return CC1101_TIMEOUT;
        if (v != patched_value(&s_async_rx_regs[i], mod)) return CC1101_VERIFY_FAIL;
    }

    cc1101_strobe(CC1101_SFRX);
    cc1101_strobe(CC1101_SRX);       /* calibrates (MCSM0) then enters RX */

    /* Wait until the radio state machine reports RX */
    uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t st;
        if (cc1101_read_status_reg(CC1101_MARCSTATE, &st) != CC1101_OK) return CC1101_TIMEOUT;
        if ((st & 0x1F) == CC1101_MARCSTATE_RX) break;
        if (HAL_GetTick() - start > 20) return CC1101_NOT_RX;
    }
    return CC1101_OK;
}

/* ===================================================================
 *  Interrupt-safe RSSI read
 *  Talks to SPI1 registers directly: HAL SPI functions use HAL_GetTick()
 *  for timeouts, and the tick does not advance inside a higher-priority ISR.
 * =================================================================== */

static inline uint8_t spi_xfer_raw(uint8_t tx)
{
    SPI_TypeDef *spi = s_hspi->Instance;
    uint32_t guard = 1000;
    while (!(spi->SR & SPI_SR_TXE) && --guard) { }
    *(volatile uint8_t *)&spi->DR = tx;
    guard = 1000;
    while (!(spi->SR & SPI_SR_RXNE) && --guard) { }
    return *(volatile uint8_t *)&spi->DR;
}

int16_t cc1101_rssi_to_dbm(uint8_t raw)
{
    int16_t v = (raw >= 128) ? (int16_t)raw - 256 : (int16_t)raw;
    return (int16_t)(v / 2 - 74);
}

static uint8_t read_rssi_raw_once(void)
{
    SPI_TypeDef *spi = s_hspi->Instance;

    cs_low();
    uint32_t guard = 2000;                         /* chip is awake in RX: MISO goes low at once */
    while (HAL_GPIO_ReadPin(MISO_PORT, MISO_PIN) == GPIO_PIN_SET && --guard) { }

    (void)spi_xfer_raw(CC1101_RSSI | HDR_READ_BURST);
    uint8_t raw = spi_xfer_raw(0x00);

    guard = 1000;
    while ((spi->SR & SPI_SR_BSY) && --guard) { }
    cs_high();
    return raw;
}

int16_t cc1101_read_rssi_dbm_isr(void)
{
    s_hspi->Instance->CR1 |= SPI_CR1_SPE;          /* make sure SPI is enabled */

    /* CC1101 errata: a status register that changes during the SPI read can be
     * returned corrupted. Read until two consecutive values agree (max 4 tries). */
    uint8_t a = read_rssi_raw_once();
    for (int i = 0; i < 3; i++) {
        uint8_t b = read_rssi_raw_once();
        if (a == b) break;
        a = b;
    }
    return cc1101_rssi_to_dbm(a);
}
