#ifndef SX126X_H
#define SX126X_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * The commands of the SX126x family (SX1261, SX1262, SX1268) of Semtech as they go over the SPI: one function per command, no hardware in
 * it. The hardware is the chip_t: `transfer` sends the bytes of a command and receives what the chip answers (the chip select is the job
 * of whoever implements it), `wait_busy` waits until the BUSY line is low (the chip is ready for a command).
 */

/* The commands (datasheet, table 11-1 .. 11-4) */
#define SX126X_CMD_SET_SLEEP                0x84
#define SX126X_CMD_SET_STANDBY              0x80
#define SX126X_CMD_SET_TX                   0x83
#define SX126X_CMD_SET_RX                   0x82
#define SX126X_CMD_SET_CAD                  0xC5
#define SX126X_CMD_SET_PACKET_TYPE          0x8A
#define SX126X_CMD_SET_RF_FREQUENCY         0x86
#define SX126X_CMD_SET_PA_CONFIG            0x95
#define SX126X_CMD_SET_TX_PARAMS            0x8E
#define SX126X_CMD_SET_BUFFER_BASE          0x8F
#define SX126X_CMD_SET_MODULATION_PARAMS    0x8B
#define SX126X_CMD_SET_PACKET_PARAMS        0x8C
#define SX126X_CMD_SET_CAD_PARAMS           0x88
#define SX126X_CMD_SET_DIO_IRQ_PARAMS       0x08
#define SX126X_CMD_GET_IRQ_STATUS           0x12
#define SX126X_CMD_CLEAR_IRQ_STATUS         0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH    0x9D
#define SX126X_CMD_SET_DIO3_AS_TCXO         0x97
#define SX126X_CMD_CALIBRATE                0x89
#define SX126X_CMD_CALIBRATE_IMAGE          0x98
#define SX126X_CMD_SET_REGULATOR_MODE       0x96
#define SX126X_CMD_WRITE_REGISTER           0x0D
#define SX126X_CMD_READ_REGISTER            0x1D
#define SX126X_CMD_WRITE_BUFFER             0x0E
#define SX126X_CMD_READ_BUFFER              0x1E
#define SX126X_CMD_GET_STATUS               0xC0
#define SX126X_CMD_GET_RX_BUFFER_STATUS     0x13
#define SX126X_CMD_GET_PACKET_STATUS        0x14
#define SX126X_CMD_GET_RSSI_INST            0x15
#define SX126X_CMD_GET_DEVICE_ERRORS        0x17
#define SX126X_CMD_CLEAR_DEVICE_ERRORS      0x07
#define SX126X_CMD_SET_LORA_SYMB_TIMEOUT    0xA0

/* The registers */
#define SX126X_REG_LORA_SYNC_WORD           0x0740      /* 2 bytes */
#define SX126X_REG_RX_GAIN                  0x08AC      /* 0x94: power saving, 0x96: boosted */
#define SX126X_REG_OCP                      0x08E7      /* the over current protection, 2.5 mA per step */

#define SX126X_RX_GAIN_BOOSTED              0x96
#define SX126X_SYNC_WORD_PRIVATE            0x1424
#define SX126X_SYNC_WORD_PUBLIC             0x3444

/* The bits of the interrupts */
#define SX126X_IRQ_TX_DONE                  (1u << 0)
#define SX126X_IRQ_RX_DONE                  (1u << 1)
#define SX126X_IRQ_PREAMBLE_DETECTED        (1u << 2)
#define SX126X_IRQ_SYNC_WORD_VALID          (1u << 3)
#define SX126X_IRQ_HEADER_VALID             (1u << 4)
#define SX126X_IRQ_HEADER_ERR               (1u << 5)
#define SX126X_IRQ_CRC_ERR                  (1u << 6)
#define SX126X_IRQ_CAD_DONE                 (1u << 7)
#define SX126X_IRQ_CAD_DETECTED             (1u << 8)
#define SX126X_IRQ_TIMEOUT                  (1u << 9)
#define SX126X_IRQ_ALL                      0x03FFu

/* The mode of the chip in the status byte (bits 6:4) and of the command (bits 3:1) */
#define SX126X_STATUS_MODE(status)          (((status) >> 4) & 0x7u)
#define SX126X_MODE_STBY_RC                 2
#define SX126X_MODE_STBY_XOSC               3
#define SX126X_MODE_FS                      4
#define SX126X_MODE_RX                      5
#define SX126X_MODE_TX                      6
#define SX126X_STATUS_COMMAND(status)       (((status) >> 1) & 0x7u)
#define SX126X_COMMAND_TIMEOUT              4
#define SX126X_COMMAND_ERROR                5
#define SX126X_COMMAND_FAILED               6

#define SX126X_XTAL_HZ                      32000000u
#define SX126X_MAX_PAYLOAD                  255
#define SX126X_TX_POWER_MIN_DBM             (-9)
#define SX126X_TX_POWER_MAX_DBM             22

typedef struct sx126x_chip sx126x_chip_t;

struct sx126x_chip
{
    /** Sends `size` bytes and receives as many; 0 or a negative errno. The chip select is done by whoever implements it. */
    int (*transfer)(void *user, const uint8_t *tx, uint8_t *rx, size_t size);
    /** Waits until BUSY is low; 0 or -ETIMEDOUT. */
    int (*wait_busy)(void *user);
    void *user;
};

typedef struct
{
    uint32_t    frequency_hz;
    uint32_t    bandwidth_hz;
    uint8_t     spreading_factor;
    uint8_t     coding_rate;            /* 5..8 */
    int8_t      tx_power_dbm;
    bool        crc;
    uint16_t    preamble_symbols;
    uint16_t    sync_word;
    bool        implicit_header;
    bool        iq_invert;
    uint8_t     payload_length;
} sx126x_lora_t;

/** The code of the bandwidth for the chip (the nearest it has not above the request; 0xFF: a request below the least); sets *hz to the bandwidth taken. */
uint8_t sx126x_bandwidth_code(uint32_t hz, uint32_t *taken_hz);

/** The frequency register of a carrier: hz * 2^25 / 32 MHz. */
uint32_t sx126x_frequency_register(uint32_t hz);

/** Is a low data rate optimization needed (a symbol longer than 16.38 ms). */
bool sx126x_low_data_rate(uint8_t spreading_factor, uint32_t bandwidth_hz);

/** The time a frame of `length` bytes is in the air, in microseconds (the formula of the datasheet, 6.1.4). */
uint32_t sx126x_airtime_us(const sx126x_lora_t *lora, uint8_t length);

/** The checks of the configuration the chip can take: 0 or -EINVAL. */
int sx126x_check_lora(const sx126x_lora_t *lora);

/* One function per command: 0 or a negative errno; `status` (may be NULL) gets the status byte the chip answers with */
int sx126x_set_standby(sx126x_chip_t *chip, bool xosc);
int sx126x_set_sleep(sx126x_chip_t *chip, bool warm_start);
int sx126x_set_regulator_dcdc(sx126x_chip_t *chip);
int sx126x_set_dio3_tcxo(sx126x_chip_t *chip, uint8_t voltage_code, uint32_t delay_15_625us);
int sx126x_set_dio2_rf_switch(sx126x_chip_t *chip, bool on);
int sx126x_calibrate_all(sx126x_chip_t *chip);
int sx126x_calibrate_image(sx126x_chip_t *chip, uint32_t frequency_hz);
int sx126x_set_packet_type_lora(sx126x_chip_t *chip);
int sx126x_set_frequency(sx126x_chip_t *chip, uint32_t hz);
int sx126x_set_pa(sx126x_chip_t *chip, int8_t power_dbm);
int sx126x_set_buffer_base(sx126x_chip_t *chip, uint8_t tx, uint8_t rx);
int sx126x_set_modulation(sx126x_chip_t *chip, const sx126x_lora_t *lora);
int sx126x_set_packet(sx126x_chip_t *chip, const sx126x_lora_t *lora, uint8_t payload_length);
int sx126x_set_sync_word(sx126x_chip_t *chip, uint16_t sync_word);
int sx126x_set_irq(sx126x_chip_t *chip, uint16_t mask, uint16_t dio1);
int sx126x_get_irq(sx126x_chip_t *chip, uint16_t *irq);
int sx126x_clear_irq(sx126x_chip_t *chip, uint16_t mask);
int sx126x_write_buffer(sx126x_chip_t *chip, uint8_t offset, const uint8_t *data, size_t size);
int sx126x_read_buffer(sx126x_chip_t *chip, uint8_t offset, uint8_t *data, size_t size);
int sx126x_set_tx(sx126x_chip_t *chip, uint32_t timeout_ms);
int sx126x_set_rx_continuous(sx126x_chip_t *chip);
int sx126x_set_cad(sx126x_chip_t *chip, uint8_t spreading_factor, uint8_t symbols);
int sx126x_get_status(sx126x_chip_t *chip, uint8_t *status);
int sx126x_get_rx_buffer_status(sx126x_chip_t *chip, uint8_t *length, uint8_t *start);
/** rssi_dbm of the last frame, snr in quarters of dB. */
int sx126x_get_packet_status(sx126x_chip_t *chip, int16_t *rssi_dbm, int16_t *snr_dbx4);
/** The instantaneous strength on the channel in dBm. */
int sx126x_get_rssi_inst(sx126x_chip_t *chip, int16_t *rssi_dbm);
int sx126x_get_device_errors(sx126x_chip_t *chip, uint16_t *errors);
int sx126x_write_register(sx126x_chip_t *chip, uint16_t address, const uint8_t *data, size_t size);
int sx126x_read_register(sx126x_chip_t *chip, uint16_t address, uint8_t *data, size_t size);

#endif
