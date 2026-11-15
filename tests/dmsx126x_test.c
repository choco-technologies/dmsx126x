#define DMOD_ENABLE_REGISTRATION ON
#include "dmod_test.h"
#include "sx126x.h"
#include <errno.h>
#include <string.h>

/* The commands of the chip: what goes over the SPI for each of them, and what is made of the answers. No hardware: the chip is a recorder. */

typedef struct
{
    uint8_t  sent[300];
    size_t   sent_size;
    uint8_t  answer[16];            /* what the chip says back (rx[i] = answer[i]) */
    unsigned exchanges;
    unsigned busy_waits;
} mock_t;

static int mock_transfer(void *user, const uint8_t *tx, uint8_t *rx, size_t size)
{
    mock_t *m = user;
    m->exchanges++;
    m->sent_size = size < sizeof(m->sent) ? size : sizeof(m->sent);
    memcpy(m->sent, tx, m->sent_size);
    if (rx != NULL)
    {
        memset(rx, 0, size);
        memcpy(rx, m->answer, size < sizeof(m->answer) ? size : sizeof(m->answer));
    }
    return 0;
}

static int mock_wait_busy(void *user)
{
    ((mock_t *)user)->busy_waits++;
    return 0;
}

static sx126x_chip_t make_chip(mock_t *m)
{
    memset(m, 0, sizeof(*m));
    sx126x_chip_t chip = { mock_transfer, mock_wait_busy, m };
    return chip;
}

#define EXPECT_SENT(m, ...)                                                                       \
    do {                                                                                          \
        static const uint8_t want[] = { __VA_ARGS__ };                                            \
        DMOD_TEST_EXPECT_EQ((int)(m).sent_size, (int)sizeof(want));                               \
        DMOD_TEST_EXPECT_TRUE(memcmp((m).sent, want, sizeof(want)) == 0);                         \
    } while (0)

static sx126x_lora_t meshcore_like(void)
{
    sx126x_lora_t lora = { .frequency_hz = 869618000u, .bandwidth_hz = 62500u, .spreading_factor = 8, .coding_rate = 8, .tx_power_dbm = 22,
                           .crc = true, .preamble_symbols = 16, .sync_word = SX126X_SYNC_WORD_PRIVATE, .implicit_header = false, .iq_invert = false,
                           .payload_length = 0 };
    return lora;
}

DMOD_TEST_STEP(frequency_register)
{
    DMOD_TEST_EXPECT_EQ((int)sx126x_frequency_register(868000000u), 0x36400000);
    DMOD_TEST_EXPECT_EQ((int)sx126x_frequency_register(915000000u), 0x39300000);
    DMOD_TEST_EXPECT_EQ((int)sx126x_frequency_register(433000000u), 0x1B100000);
    mock_t m;
    sx126x_chip_t chip = make_chip(&m);
    DMOD_TEST_EXPECT_EQ(sx126x_set_frequency(&chip, 868000000u), 0);
    EXPECT_SENT(m, 0x86, 0x36, 0x40, 0x00, 0x00);
    DMOD_TEST_EXPECT_EQ((int)m.busy_waits, 1);
}

DMOD_TEST_STEP(bandwidth_codes)
{
    uint32_t taken = 0;
    DMOD_TEST_EXPECT_EQ(sx126x_bandwidth_code(125000u, &taken), 0x04);
    DMOD_TEST_EXPECT_EQ((int)taken, 125000);
    DMOD_TEST_EXPECT_EQ(sx126x_bandwidth_code(62500u, &taken), 0x03);
    DMOD_TEST_EXPECT_EQ(sx126x_bandwidth_code(500000u, &taken), 0x06);
    DMOD_TEST_EXPECT_EQ(sx126x_bandwidth_code(7800u, &taken), 0x00);        /* the nominal 7.8 kHz is the 7.81 */
    DMOD_TEST_EXPECT_EQ(sx126x_bandwidth_code(300000u, &taken), 0x05);      /* the nearest not above */
    DMOD_TEST_EXPECT_EQ((int)taken, 250000);
    DMOD_TEST_EXPECT_EQ(sx126x_bandwidth_code(5000u, &taken), 0xFF);
}

DMOD_TEST_STEP(low_data_rate_optimization)
{
    DMOD_TEST_EXPECT_TRUE(sx126x_low_data_rate(12, 125000u));
    DMOD_TEST_EXPECT_TRUE(sx126x_low_data_rate(11, 125000u));               /* 16.4 ms */
    DMOD_TEST_EXPECT_TRUE(!sx126x_low_data_rate(10, 125000u));
    DMOD_TEST_EXPECT_TRUE(!sx126x_low_data_rate(11, 250000u));
    DMOD_TEST_EXPECT_TRUE(!sx126x_low_data_rate(7, 125000u));
}

DMOD_TEST_STEP(air_time_of_a_frame)
{
    sx126x_lora_t lora = { .frequency_hz = 868000000u, .bandwidth_hz = 125000u, .spreading_factor = 7, .coding_rate = 5, .tx_power_dbm = 14,
                           .crc = true, .preamble_symbols = 8, .sync_word = SX126X_SYNC_WORD_PUBLIC };
    /* the calculator of Semtech: 10 bytes, SF7, 125 kHz, 4/5, 8 symbols of the preamble, CRC, explicit header: 41.22 ms */
    DMOD_TEST_EXPECT_EQ((int)sx126x_airtime_us(&lora, 10), 41216);
    /* SF12: 991.23 ms (the low data rate optimization is on) */
    lora.spreading_factor = 12;
    DMOD_TEST_EXPECT_EQ((int)sx126x_airtime_us(&lora, 10), 991232);
    /* a longer frame takes longer */
    DMOD_TEST_EXPECT_TRUE(sx126x_airtime_us(&lora, 100) > sx126x_airtime_us(&lora, 10));
}

DMOD_TEST_STEP(configuration_is_checked)
{
    sx126x_lora_t lora = meshcore_like();
    DMOD_TEST_EXPECT_EQ(sx126x_check_lora(&lora), 0);
    lora.spreading_factor = 13;
    DMOD_TEST_EXPECT_EQ(sx126x_check_lora(&lora), -EINVAL);
    lora = meshcore_like(); lora.coding_rate = 4;
    DMOD_TEST_EXPECT_EQ(sx126x_check_lora(&lora), -EINVAL);
    lora = meshcore_like(); lora.tx_power_dbm = 23;
    DMOD_TEST_EXPECT_EQ(sx126x_check_lora(&lora), -EINVAL);
    lora = meshcore_like(); lora.bandwidth_hz = 1000;
    DMOD_TEST_EXPECT_EQ(sx126x_check_lora(&lora), -EINVAL);
    lora = meshcore_like(); lora.frequency_hz = 2400000000u;
    DMOD_TEST_EXPECT_EQ(sx126x_check_lora(&lora), -EINVAL);
}

DMOD_TEST_STEP(modulation_and_packet_commands)
{
    mock_t m;
    sx126x_chip_t chip = make_chip(&m);
    sx126x_lora_t lora = meshcore_like();
    DMOD_TEST_EXPECT_EQ(sx126x_set_modulation(&chip, &lora), 0);
    EXPECT_SENT(m, 0x8B, 8, 0x03, 4, 0);                                     /* SF8, 62.5 kHz, 4/8, no LDRO */
    lora.spreading_factor = 12; lora.bandwidth_hz = 125000u; lora.coding_rate = 5;
    DMOD_TEST_EXPECT_EQ(sx126x_set_modulation(&chip, &lora), 0);
    EXPECT_SENT(m, 0x8B, 12, 0x04, 1, 1);                                    /* SF12, 125 kHz, 4/5, LDRO */
    lora = meshcore_like();
    DMOD_TEST_EXPECT_EQ(sx126x_set_packet(&chip, &lora, 255), 0);
    EXPECT_SENT(m, 0x8C, 0, 16, 0, 255, 1, 0);
    lora.implicit_header = true; lora.crc = false; lora.iq_invert = true;
    DMOD_TEST_EXPECT_EQ(sx126x_set_packet(&chip, &lora, 20), 0);
    EXPECT_SENT(m, 0x8C, 0, 16, 1, 20, 0, 1);
}

DMOD_TEST_STEP(power_sync_word_and_irq)
{
    mock_t m;
    sx126x_chip_t chip = make_chip(&m);
    DMOD_TEST_EXPECT_EQ(sx126x_set_pa(&chip, 14), 0);
    EXPECT_SENT(m, 0x8E, 14, 0x04);                                          /* the last of the two commands */
    DMOD_TEST_EXPECT_EQ((int)m.exchanges, 2);
    DMOD_TEST_EXPECT_EQ(sx126x_set_pa(&chip, -9), 0);
    EXPECT_SENT(m, 0x8E, 0xF7, 0x04);
    DMOD_TEST_EXPECT_EQ(sx126x_set_sync_word(&chip, SX126X_SYNC_WORD_PRIVATE), 0);
    EXPECT_SENT(m, 0x0D, 0x07, 0x40, 0x14, 0x24);
    DMOD_TEST_EXPECT_EQ(sx126x_set_irq(&chip, SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE),
                        0);
    EXPECT_SENT(m, 0x08, 0x02, 0x03, 0x00, 0x03, 0, 0, 0, 0);
    DMOD_TEST_EXPECT_EQ(sx126x_clear_irq(&chip, SX126X_IRQ_ALL), 0);
    EXPECT_SENT(m, 0x02, 0x03, 0xFF);
}

DMOD_TEST_STEP(transmit_and_receive_commands)
{
    mock_t m;
    sx126x_chip_t chip = make_chip(&m);
    DMOD_TEST_EXPECT_EQ(sx126x_set_tx(&chip, 1000), 0);
    EXPECT_SENT(m, 0x83, 0x00, 0xFA, 0x00);                                  /* 1 s = 64000 units of 15.625 us */
    DMOD_TEST_EXPECT_EQ(sx126x_set_rx_continuous(&chip), 0);
    EXPECT_SENT(m, 0x82, 0xFF, 0xFF, 0xFF);
    DMOD_TEST_EXPECT_EQ(sx126x_set_standby(&chip, false), 0);
    EXPECT_SENT(m, 0x80, 0x00);
    DMOD_TEST_EXPECT_EQ(sx126x_set_buffer_base(&chip, 0, 0), 0);
    EXPECT_SENT(m, 0x8F, 0, 0);
    const uint8_t frame[3] = { 0xDE, 0xAD, 0x01 };
    DMOD_TEST_EXPECT_EQ(sx126x_write_buffer(&chip, 0, frame, sizeof(frame)), 0);
    EXPECT_SENT(m, 0x0E, 0x00, 0xDE, 0xAD, 0x01);
    DMOD_TEST_EXPECT_EQ(sx126x_write_buffer(&chip, 0, frame, 256), -EINVAL);
    DMOD_TEST_EXPECT_EQ(sx126x_set_cad(&chip, 8, 2), 0);
    EXPECT_SENT(m, 0xC5);
    DMOD_TEST_EXPECT_EQ((int)m.exchanges, 7);                         /* (the CAD is two commands: its parameters and the start) */
}

DMOD_TEST_STEP(answers_of_the_chip)
{
    mock_t m;
    sx126x_chip_t chip = make_chip(&m);
    uint16_t irq = 0;
    m.answer[1] = 0xA2; m.answer[2] = 0x02; m.answer[3] = 0x42;             /* the status byte, then the interrupts */
    DMOD_TEST_EXPECT_EQ(sx126x_get_irq(&chip, &irq), 0);
    EXPECT_SENT(m, 0x12, 0, 0, 0);
    DMOD_TEST_EXPECT_EQ((int)irq, 0x0242);
    uint8_t length = 0, start = 0;
    memset(m.answer, 0, sizeof(m.answer)); m.answer[2] = 37; m.answer[3] = 4;
    DMOD_TEST_EXPECT_EQ(sx126x_get_rx_buffer_status(&chip, &length, &start), 0);
    DMOD_TEST_EXPECT_EQ((int)length, 37);
    DMOD_TEST_EXPECT_EQ((int)start, 4);
    int16_t rssi = 0, snr = 0;
    memset(m.answer, 0, sizeof(m.answer)); m.answer[2] = 130; m.answer[3] = (uint8_t)(int8_t)-22;   /* -65 dBm, -5.5 dB */
    DMOD_TEST_EXPECT_EQ(sx126x_get_packet_status(&chip, &rssi, &snr), 0);
    DMOD_TEST_EXPECT_EQ((int)rssi, -65);
    DMOD_TEST_EXPECT_EQ((int)snr, -22);
    memset(m.answer, 0, sizeof(m.answer)); m.answer[2] = 200;
    DMOD_TEST_EXPECT_EQ(sx126x_get_rssi_inst(&chip, &rssi), 0);
    DMOD_TEST_EXPECT_EQ((int)rssi, -100);
    uint8_t status = 0;
    memset(m.answer, 0, sizeof(m.answer)); m.answer[1] = 0x6C;
    DMOD_TEST_EXPECT_EQ(sx126x_get_status(&chip, &status), 0);
    DMOD_TEST_EXPECT_EQ((int)SX126X_STATUS_MODE(status), SX126X_MODE_TX);
    uint8_t data[3];
    memset(m.answer, 0, sizeof(m.answer)); m.answer[3] = 1; m.answer[4] = 2; m.answer[5] = 3;
    DMOD_TEST_EXPECT_EQ(sx126x_read_buffer(&chip, 4, data, sizeof(data)), 0);
    EXPECT_SENT(m, 0x1E, 4, 0, 0, 0, 0);
    DMOD_TEST_EXPECT_TRUE(data[0] == 1 && data[1] == 2 && data[2] == 3);
    uint8_t reg[2];
    memset(m.answer, 0, sizeof(m.answer)); m.answer[4] = 0x14; m.answer[5] = 0x24;
    DMOD_TEST_EXPECT_EQ(sx126x_read_register(&chip, SX126X_REG_LORA_SYNC_WORD, reg, sizeof(reg)), 0);
    EXPECT_SENT(m, 0x1D, 0x07, 0x40, 0, 0, 0);
    DMOD_TEST_EXPECT_TRUE(reg[0] == 0x14 && reg[1] == 0x24);
}

DMOD_TEST_STEP(the_chip_is_waited_for_and_errors_pass)
{
    mock_t m;
    sx126x_chip_t chip = make_chip(&m);
    DMOD_TEST_EXPECT_EQ(sx126x_set_standby(&chip, true), 0);
    DMOD_TEST_EXPECT_EQ((int)m.busy_waits, 1);
    chip.wait_busy = NULL;                                                   /* a board without the BUSY line */
    DMOD_TEST_EXPECT_EQ(sx126x_set_standby(&chip, true), 0);
    chip.transfer = NULL;
    DMOD_TEST_EXPECT_EQ(sx126x_set_standby(&chip, true), -EINVAL);
}
