#include "sx126x.h"
#include <errno.h>
#include <string.h>

/* 64-bit / 32-bit division by shifting: a module has no libgcc */
static uint64_t div64(uint64_t n, uint32_t d)
{
    uint64_t q = 0, r = 0;
    for (int i = 63; i >= 0; i--)
    {
        r = (r << 1) | ((n >> i) & 1u);
        if (r >= d)
        {
            r -= d;
            q |= (uint64_t)1 << i;
        }
    }
    return q;
}

/* ---- The pure functions ---- */

uint8_t sx126x_bandwidth_code(uint32_t hz, uint32_t *taken_hz)
{
    /* the bandwidths of the chip: code, Hz (the nominal values 7.8 kHz .. 500 kHz) */
    static const struct { uint8_t code; uint32_t hz; } table[] = {
        { 0x00, 7810 },   { 0x08, 10420 },  { 0x01, 15630 },  { 0x09, 20830 },  { 0x02, 31250 },
        { 0x0A, 41670 },  { 0x03, 62500 },  { 0x04, 125000 }, { 0x05, 250000 }, { 0x06, 500000 },
    };
    uint8_t code = 0xFF;
    uint32_t taken = 0;
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++)
    {
        /* a request of 7800 is the 7.81 kHz: the nominal values are rounded, so allow the 1 % below */
        if (hz + hz / 100u >= table[i].hz)
        {
            code = table[i].code;
            taken = table[i].hz;
        }
    }
    if (taken_hz != NULL)
        *taken_hz = taken;
    return code;
}

uint32_t sx126x_frequency_register(uint32_t hz)
{
    return (uint32_t)div64((uint64_t)hz << 25, SX126X_XTAL_HZ);
}

bool sx126x_low_data_rate(uint8_t spreading_factor, uint32_t bandwidth_hz)
{
    /* the symbol (2^SF / BW) longer than 16.38 ms: 2^SF * 1000000 > 16380 * BW */
    return ((uint64_t)1 << spreading_factor) * 1000000u > (uint64_t)16380u * bandwidth_hz;
}

uint32_t sx126x_airtime_us(const sx126x_lora_t *lora, uint8_t length)
{
    uint32_t bw = 0;
    (void)sx126x_bandwidth_code(lora->bandwidth_hz, &bw);
    if (bw == 0)
        bw = lora->bandwidth_hz != 0 ? lora->bandwidth_hz : 125000u;
    uint32_t sf = lora->spreading_factor;
    bool de = sx126x_low_data_rate((uint8_t)sf, bw);
    uint32_t symbol_ns = (uint32_t)div64(((uint64_t)1 << sf) * 1000000000u, bw);
    uint32_t preamble_x4 = 4u * lora->preamble_symbols + (sf < 7 ? 25u : 17u);
    int32_t n = 8 * (int32_t)length - 4 * (int32_t)sf + (sf >= 7 ? 8 : 0) + (lora->crc ? 16 : 0) + (lora->implicit_header ? 0 : 20);
    uint32_t divisor = 4u * (sf - (de ? 2u : 0u));
    uint32_t blocks = n > 0 ? ((uint32_t)n + divisor - 1u) / divisor : 0u;
    uint32_t symbols = 8u + blocks * lora->coding_rate;
    uint64_t total_x4 = (uint64_t)preamble_x4 + 4u * (uint64_t)symbols;
    return (uint32_t)div64(div64(total_x4 * symbol_ns, 4u), 1000u);
}

int sx126x_check_lora(const sx126x_lora_t *lora)
{
    uint32_t taken = 0;
    if (lora == NULL || lora->spreading_factor < 5 || lora->spreading_factor > 12 || lora->coding_rate < 5 || lora->coding_rate > 8 ||
        lora->tx_power_dbm < SX126X_TX_POWER_MIN_DBM || lora->tx_power_dbm > SX126X_TX_POWER_MAX_DBM ||
        sx126x_bandwidth_code(lora->bandwidth_hz, &taken) == 0xFF || lora->preamble_symbols < 1 || lora->frequency_hz < 150000000u ||
        lora->frequency_hz > 960000000u)
        return -EINVAL;
    return 0;
}

/* ---- The commands ---- */

/* One exchange: the chip is waited for until it is ready, then the bytes go */
static int exchange(sx126x_chip_t *chip, const uint8_t *tx, uint8_t *rx, size_t size)
{
    if (chip == NULL || chip->transfer == NULL)
        return -EINVAL;
    if (chip->wait_busy != NULL)
    {
        int ret = chip->wait_busy(chip->user);
        if (ret != 0)
            return ret;
    }
    return chip->transfer(chip->user, tx, rx, size);
}

/* A command with arguments and no answer */
static int command(sx126x_chip_t *chip, uint8_t opcode, const uint8_t *args, size_t count)
{
    uint8_t tx[12];
    if (count > sizeof(tx) - 1u)
        return -EINVAL;
    tx[0] = opcode;
    if (count != 0)
        memcpy(tx + 1, args, count);
    return exchange(chip, tx, NULL, count + 1u);
}

/* A command that answers: after the opcode (and `skip` bytes of its own) the chip sends the status and then `count` bytes */
static int query(sx126x_chip_t *chip, uint8_t opcode, uint8_t *out, size_t count, uint8_t *status)
{
    uint8_t tx[8] = { opcode, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t rx[8];
    if (count > sizeof(tx) - 2u)
        return -EINVAL;
    int ret = exchange(chip, tx, rx, count + 2u);
    if (ret != 0)
        return ret;
    if (status != NULL)
        *status = rx[1];
    if (out != NULL && count != 0)
        memcpy(out, rx + 2, count);
    return 0;
}

int sx126x_set_standby(sx126x_chip_t *chip, bool xosc)
{
    uint8_t arg = xosc ? 1 : 0;
    return command(chip, SX126X_CMD_SET_STANDBY, &arg, 1);
}

int sx126x_set_sleep(sx126x_chip_t *chip, bool warm_start)
{
    uint8_t arg = warm_start ? 0x04 : 0x00;
    return command(chip, SX126X_CMD_SET_SLEEP, &arg, 1);
}

int sx126x_set_regulator_dcdc(sx126x_chip_t *chip)
{
    uint8_t arg = 1;
    return command(chip, SX126X_CMD_SET_REGULATOR_MODE, &arg, 1);
}

int sx126x_set_dio3_tcxo(sx126x_chip_t *chip, uint8_t voltage_code, uint32_t delay_15_625us)
{
    uint8_t args[4] = { voltage_code, (uint8_t)(delay_15_625us >> 16), (uint8_t)(delay_15_625us >> 8), (uint8_t)delay_15_625us };
    return command(chip, SX126X_CMD_SET_DIO3_AS_TCXO, args, sizeof(args));
}

int sx126x_set_dio2_rf_switch(sx126x_chip_t *chip, bool on)
{
    uint8_t arg = on ? 1 : 0;
    return command(chip, SX126X_CMD_SET_DIO2_AS_RF_SWITCH, &arg, 1);
}

int sx126x_calibrate_all(sx126x_chip_t *chip)
{
    uint8_t arg = 0x7F;
    return command(chip, SX126X_CMD_CALIBRATE, &arg, 1);
}

int sx126x_calibrate_image(sx126x_chip_t *chip, uint32_t frequency_hz)
{
    uint8_t args[2];
    if (frequency_hz <= 440000000u)
    {
        args[0] = 0x6B; args[1] = 0x6F;
    }
    else if (frequency_hz <= 510000000u)
    {
        args[0] = 0x75; args[1] = 0x81;
    }
    else if (frequency_hz <= 787000000u)
    {
        args[0] = 0xC1; args[1] = 0xC5;
    }
    else if (frequency_hz <= 900000000u)
    {
        args[0] = 0xD7; args[1] = 0xDB;
    }
    else
    {
        args[0] = 0xE1; args[1] = 0xE9;
    }
    return command(chip, SX126X_CMD_CALIBRATE_IMAGE, args, sizeof(args));
}

int sx126x_set_packet_type_lora(sx126x_chip_t *chip)
{
    uint8_t arg = 1;
    return command(chip, SX126X_CMD_SET_PACKET_TYPE, &arg, 1);
}

int sx126x_set_frequency(sx126x_chip_t *chip, uint32_t hz)
{
    uint32_t reg = sx126x_frequency_register(hz);
    uint8_t args[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
    return command(chip, SX126X_CMD_SET_RF_FREQUENCY, args, sizeof(args));
}

int sx126x_set_pa(sx126x_chip_t *chip, int8_t power_dbm)
{
    /* the SX1262: the high power amplifier (paDutyCycle 4, hpMax 7, device 0, paLut 1) gives up to +22 dBm; the power is set directly */
    uint8_t pa[4] = { 0x04, 0x07, 0x00, 0x01 };
    int ret = command(chip, SX126X_CMD_SET_PA_CONFIG, pa, sizeof(pa));
    if (ret != 0)
        return ret;
    uint8_t tx[2] = { (uint8_t)power_dbm, 0x04 };       /* ramp time 200 us */
    return command(chip, SX126X_CMD_SET_TX_PARAMS, tx, sizeof(tx));
}

int sx126x_set_buffer_base(sx126x_chip_t *chip, uint8_t tx, uint8_t rx)
{
    uint8_t args[2] = { tx, rx };
    return command(chip, SX126X_CMD_SET_BUFFER_BASE, args, sizeof(args));
}

int sx126x_set_modulation(sx126x_chip_t *chip, const sx126x_lora_t *lora)
{
    uint32_t taken = 0;
    uint8_t code = sx126x_bandwidth_code(lora->bandwidth_hz, &taken);
    if (code == 0xFF)
        return -EINVAL;
    uint8_t args[4] = { lora->spreading_factor, code, (uint8_t)(lora->coding_rate - 4u), sx126x_low_data_rate(lora->spreading_factor, taken) ? 1u : 0u };
    return command(chip, SX126X_CMD_SET_MODULATION_PARAMS, args, sizeof(args));
}

int sx126x_set_packet(sx126x_chip_t *chip, const sx126x_lora_t *lora, uint8_t payload_length)
{
    uint8_t args[6] = { (uint8_t)(lora->preamble_symbols >> 8), (uint8_t)lora->preamble_symbols, lora->implicit_header ? 1u : 0u, payload_length,
                        lora->crc ? 1u : 0u, lora->iq_invert ? 1u : 0u };
    return command(chip, SX126X_CMD_SET_PACKET_PARAMS, args, sizeof(args));
}

int sx126x_set_sync_word(sx126x_chip_t *chip, uint16_t sync_word)
{
    uint8_t data[2] = { (uint8_t)(sync_word >> 8), (uint8_t)sync_word };
    return sx126x_write_register(chip, SX126X_REG_LORA_SYNC_WORD, data, sizeof(data));
}

int sx126x_set_irq(sx126x_chip_t *chip, uint16_t mask, uint16_t dio1)
{
    uint8_t args[8] = { (uint8_t)(mask >> 8), (uint8_t)mask, (uint8_t)(dio1 >> 8), (uint8_t)dio1, 0, 0, 0, 0 };
    return command(chip, SX126X_CMD_SET_DIO_IRQ_PARAMS, args, sizeof(args));
}

int sx126x_get_irq(sx126x_chip_t *chip, uint16_t *irq)
{
    uint8_t out[2];
    int ret = query(chip, SX126X_CMD_GET_IRQ_STATUS, out, sizeof(out), NULL);
    if (ret == 0)
        *irq = (uint16_t)((out[0] << 8) | out[1]);
    return ret;
}

int sx126x_clear_irq(sx126x_chip_t *chip, uint16_t mask)
{
    uint8_t args[2] = { (uint8_t)(mask >> 8), (uint8_t)mask };
    return command(chip, SX126X_CMD_CLEAR_IRQ_STATUS, args, sizeof(args));
}

int sx126x_write_buffer(sx126x_chip_t *chip, uint8_t offset, const uint8_t *data, size_t size)
{
    uint8_t tx[2 + SX126X_MAX_PAYLOAD];
    if (size > SX126X_MAX_PAYLOAD || (size != 0 && data == NULL))
        return -EINVAL;
    tx[0] = SX126X_CMD_WRITE_BUFFER;
    tx[1] = offset;
    if (size != 0)
        memcpy(tx + 2, data, size);
    return exchange(chip, tx, NULL, size + 2u);
}

int sx126x_read_buffer(sx126x_chip_t *chip, uint8_t offset, uint8_t *data, size_t size)
{
    uint8_t tx[3 + SX126X_MAX_PAYLOAD];
    uint8_t rx[3 + SX126X_MAX_PAYLOAD];
    if (size > SX126X_MAX_PAYLOAD || (size != 0 && data == NULL))
        return -EINVAL;
    memset(tx, 0, size + 3u);
    tx[0] = SX126X_CMD_READ_BUFFER;
    tx[1] = offset;
    int ret = exchange(chip, tx, rx, size + 3u);        /* [opcode][offset][status][data...] answers as [..][..][nop][data...] */
    if (ret == 0 && size != 0)
        memcpy(data, rx + 3, size);
    return ret;
}

int sx126x_set_tx(sx126x_chip_t *chip, uint32_t timeout_ms)
{
    uint32_t units = timeout_ms * 64u;                  /* 15.625 us */
    if (timeout_ms > 0x3FFFFFu / 64u)
        units = 0;                                      /* longer than the chip counts: no timeout of its own */
    uint8_t args[3] = { (uint8_t)(units >> 16), (uint8_t)(units >> 8), (uint8_t)units };
    return command(chip, SX126X_CMD_SET_TX, args, sizeof(args));
}

int sx126x_set_rx_continuous(sx126x_chip_t *chip)
{
    uint8_t args[3] = { 0xFF, 0xFF, 0xFF };
    return command(chip, SX126X_CMD_SET_RX, args, sizeof(args));
}

int sx126x_set_cad(sx126x_chip_t *chip, uint8_t spreading_factor, uint8_t symbols)
{
    /* symbols: 0 (1 symbol), 1 (2), 2 (4), 3 (8), 4 (16); the detection thresholds are those of Semtech's application note */
    uint8_t peak = spreading_factor <= 8 ? 22u : (spreading_factor == 9 ? 23u : (spreading_factor == 10 ? 24u : (spreading_factor == 11 ? 25u : 28u)));
    uint8_t args[7] = { symbols, peak, 10, 0, 0, 0, 0 };        /* exit mode 0: CAD only; no timeout */
    int ret = command(chip, SX126X_CMD_SET_CAD_PARAMS, args, sizeof(args));
    if (ret != 0)
        return ret;
    return command(chip, SX126X_CMD_SET_CAD, NULL, 0);
}

int sx126x_get_status(sx126x_chip_t *chip, uint8_t *status)
{
    uint8_t tx[2] = { SX126X_CMD_GET_STATUS, 0 };
    uint8_t rx[2];
    int ret = exchange(chip, tx, rx, sizeof(tx));
    if (ret == 0)
        *status = rx[1];
    return ret;
}

int sx126x_get_rx_buffer_status(sx126x_chip_t *chip, uint8_t *length, uint8_t *start)
{
    uint8_t out[2];
    int ret = query(chip, SX126X_CMD_GET_RX_BUFFER_STATUS, out, sizeof(out), NULL);
    if (ret == 0)
    {
        *length = out[0];
        *start = out[1];
    }
    return ret;
}

int sx126x_get_packet_status(sx126x_chip_t *chip, int16_t *rssi_dbm, int16_t *snr_dbx4)
{
    uint8_t out[3];
    int ret = query(chip, SX126X_CMD_GET_PACKET_STATUS, out, sizeof(out), NULL);
    if (ret == 0)
    {
        *rssi_dbm = (int16_t)(-(int)out[0] / 2);
        *snr_dbx4 = (int16_t)(int8_t)out[1];
    }
    return ret;
}

int sx126x_get_rssi_inst(sx126x_chip_t *chip, int16_t *rssi_dbm)
{
    uint8_t out;
    int ret = query(chip, SX126X_CMD_GET_RSSI_INST, &out, 1, NULL);
    if (ret == 0)
        *rssi_dbm = (int16_t)(-(int)out / 2);
    return ret;
}

int sx126x_get_device_errors(sx126x_chip_t *chip, uint16_t *errors)
{
    uint8_t out[2];
    int ret = query(chip, SX126X_CMD_GET_DEVICE_ERRORS, out, sizeof(out), NULL);
    if (ret == 0)
        *errors = (uint16_t)((out[0] << 8) | out[1]);
    return ret;
}

int sx126x_write_register(sx126x_chip_t *chip, uint16_t address, const uint8_t *data, size_t size)
{
    uint8_t tx[3 + 4];
    if (size > 4 || (size != 0 && data == NULL))
        return -EINVAL;
    tx[0] = SX126X_CMD_WRITE_REGISTER;
    tx[1] = (uint8_t)(address >> 8);
    tx[2] = (uint8_t)address;
    memcpy(tx + 3, data, size);
    return exchange(chip, tx, NULL, size + 3u);
}

int sx126x_read_register(sx126x_chip_t *chip, uint16_t address, uint8_t *data, size_t size)
{
    uint8_t tx[4 + 4] = { SX126X_CMD_READ_REGISTER, (uint8_t)(address >> 8), (uint8_t)address, 0, 0, 0, 0, 0 };
    uint8_t rx[4 + 4];
    if (size > 4 || (size != 0 && data == NULL))
        return -EINVAL;
    int ret = exchange(chip, tx, rx, size + 4u);
    if (ret == 0 && size != 0)
        memcpy(data, rx + 4, size);
    return ret;
}
