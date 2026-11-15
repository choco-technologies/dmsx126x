#define DMOD_ENABLE_REGISTRATION    ON
#include "dmod.h"
#include "sx126x.h"
#include "dmdrvi.h"
#include "dmini.h"
#include "dmosi.h"
#include "dmspi_types.h"
#include "dmgpio_types.h"
#include <errno.h>
#include <string.h>

/*
 * dmsx126x: the driver of the SX1262 LoRa radio of Semtech (SX1261/SX1268 are the same on the bus) - the radio of the LILYGO T-Deck Pro. Its node
 * follows the LoRa interface of dmdrvi (DMDRVI_IOCTL_LORA_*): the modulation, a frame sent, a frame received, a look at the channel. A mesh
 * protocol (dmmeshcore) is a client of the node.
 *
 * The wiring is done by dmdevfs: the radio is in the friends_group of the board's SPI bus (on the T-Deck Pro the bus is shared with the
 * e-paper and the SD card), and the friend_role says who is who:
 *
 *   spi         the dmspi node of the bus (required): the transfers go with dmspi_ioctl_cmd_transfer_cs - framed by the chip select of the
 *               radio, not by the one of the friends_group, and held on the bus from the selection to the deselection
 *   lora_cs     the chip select of the radio, a dmgpio output (required)
 *   lora_reset  the reset line, low = reset (required)
 *   lora_busy   the BUSY line, a dmgpio input: high while the chip works on a command (required)
 *   lora_dio1   the line of the interrupt DIO1, a dmgpio input (required)
 *   lora_power  the power of the radio, a dmgpio output (optional: the board's configuration may keep it on)
 *
 * The radio listens whenever it does not send. A thread looks at DIO1 (every POLL_MS) and takes the frames that come into a ring of a few; the
 * clients take them with RECV. SEND stops the listening for the time of the transmission. The chip is reset and configured when all the
 * friends are there (in whatever order they come), and again when a transmission does not end.
 */

#define CONTEXT_MAGIC       0x4C523236U             /* "LR26" */
#define POLL_MS             3U
#define BUSY_TIMEOUT_MS     1000U
#define RESET_PULSE_MS      2U
#define RING_FRAMES         4U
#define TCXO_DELAY_UNITS    320U                    /* 5 ms in units of 15.625 us */

typedef struct
{
    char   *path;                                   /**< Node of the friend (NULL: not there) */
    void   *file;                                   /**< Kept open once used */
} node_t;

typedef struct
{
    uint8_t length;
    uint8_t crc_error;
    int16_t rssi_dbm;
    int16_t snr_dbx4;
    uint8_t data[SX126X_MAX_PAYLOAD];
} frame_t;

struct dmdrvi_context
{
    uint32_t            magic;
    node_t              spi, cs, reset, busy, dio1, power;
    dmosi_mutex_t       lock;                       /**< The chip, the state, the ring */
    dmosi_semaphore_t   frames;                     /**< One count per frame in the ring */
    dmosi_thread_t      worker;
    volatile bool       stopping;
    bool                ready;                      /**< The chip is configured and listens (or sends) */
    bool                transmitting;
    sx126x_chip_t       chip;
    sx126x_lora_t       lora;
    uint16_t            tcxo_mv;
    bool                dio2_rf_switch;
    uint8_t             ocp_step;                   /**< Over current protection in units of 2.5 mA */
    frame_t             ring[RING_FRAMES];
    uint8_t             ring_head, ring_count;
    uint32_t            frames_sent, frames_received, crc_errors, tx_timeouts;
    int                 start_error;                /**< Why the last start of the chip failed (0: it did not) */
    uint8_t             start_stage;                /**< How far it got */
    uint8_t             start_status;               /**< The status byte of the chip at the end of the start */
    unsigned            starts;
    int16_t             last_rssi_dbm, last_snr_dbx4;
    char                report[480];
};

static bool is_valid_context(dmdrvi_context_t context)
{
    return context != NULL && context->magic == CONTEXT_MAGIC;
}

/* ---- The nodes of the friends ---- */

static void node_close(node_t *node)
{
    if (node->file != NULL)
        Dmod_FileClose(node->file);
    node->file = NULL;
}

static void node_forget(node_t *node)
{
    node_close(node);
    Dmod_Free(node->path);
    node->path = NULL;
}

static void *node_file(node_t *node)
{
    if (node->file == NULL && node->path != NULL)
        node->file = Dmod_FileOpen(node->path, "r+");
    return node->file;
}

static int gpio_set(node_t *node, bool high)
{
    dmgpio_pins_state_t state = high ? dmgpio_pins_state_all_high : dmgpio_pins_state_all_low;
    void *file = node_file(node);
    if (file == NULL)
        return -ENODEV;
    if (Dmod_Ioctl(file, dmgpio_ioctl_cmd_set_pins_state, &state) != 0)
    {
        node_close(node);
        return -EIO;
    }
    return 0;
}

/* The level of a dmgpio input: 1 high, 0 low, a negative errno when the node cannot be read */
static int gpio_get(node_t *node)
{
    dmgpio_pins_mask_t high = 0;
    void *file = node_file(node);
    if (file == NULL)
        return -ENODEV;
    if (Dmod_Ioctl(file, dmgpio_ioctl_cmd_get_high_pins_state, &high) != 0)
    {
        node_close(node);
        return -EIO;
    }
    return high != 0 ? 1 : 0;
}

/* ---- The bus: the chip of sx126x.c ---- */

static int chip_transfer(void *user, const uint8_t *tx, uint8_t *rx, size_t size)
{
    dmdrvi_context_t context = user;
    void *file = node_file(&context->spi);
    if (file == NULL || context->cs.path == NULL)
        return -ENODEV;
    dmspi_transfer_cs_t transfer = { .tx = tx, .rx = rx, .size = size, .chip_select = context->cs.path };
    int ret = Dmod_Ioctl(file, dmspi_ioctl_cmd_transfer_cs, &transfer);
    if (ret != 0)
        node_close(&context->spi);                      /* (a handle that failed is opened again) */
    return ret == 0 ? 0 : (ret < 0 ? ret : -EIO);
}

static int chip_wait_busy(void *user)
{
    dmdrvi_context_t context = user;
    for (uint32_t waited = 0; waited < BUSY_TIMEOUT_MS; waited++)
    {
        int level = gpio_get(&context->busy);
        if (level < 0)
            return level;
        if (level == 0)
            return 0;
        dmosi_thread_sleep(1);
    }
    return -ETIMEDOUT;
}

static bool friends_there(dmdrvi_context_t context)
{
    return context->spi.path != NULL && context->cs.path != NULL && context->reset.path != NULL && context->busy.path != NULL &&
           context->dio1.path != NULL;
}

/* ---- The chip ---- */

/* The modulation and what goes with it: the lock is held, the chip is in standby */
static int apply_lora(dmdrvi_context_t context)
{
    sx126x_chip_t *chip = &context->chip;
    const sx126x_lora_t *lora = &context->lora;
    int ret = sx126x_set_packet_type_lora(chip);
    if (ret == 0) ret = sx126x_calibrate_image(chip, lora->frequency_hz);
    if (ret == 0) ret = sx126x_set_frequency(chip, lora->frequency_hz);
    if (ret == 0) ret = sx126x_set_pa(chip, lora->tx_power_dbm);
    if (ret == 0) ret = sx126x_set_modulation(chip, lora);
    if (ret == 0) ret = sx126x_set_packet(chip, lora, lora->implicit_header ? lora->payload_length : SX126X_MAX_PAYLOAD);
    if (ret == 0) ret = sx126x_set_sync_word(chip, lora->sync_word);
    return ret;
}

static int start_listening(dmdrvi_context_t context)
{
    int ret = sx126x_set_buffer_base(&context->chip, 0, 0);
    if (ret == 0) ret = sx126x_set_irq(&context->chip, SX126X_IRQ_ALL, SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
    if (ret == 0) ret = sx126x_clear_irq(&context->chip, SX126X_IRQ_ALL);
    if (ret == 0) ret = sx126x_set_rx_continuous(&context->chip);
    return ret;
}

/* The chip from the reset to the listening: the lock is held */
static int chip_start(dmdrvi_context_t context)
{
    sx126x_chip_t *chip = &context->chip;
    context->ready = false;
    context->starts++;
    context->start_stage = 1;
    if (context->power.path != NULL)
        (void)gpio_set(&context->power, true);
    int ret = gpio_set(&context->reset, false);
    if (ret != 0)
        return context->start_error = ret;
    dmosi_thread_sleep(RESET_PULSE_MS);
    ret = gpio_set(&context->reset, true);
    if (ret != 0)
        return context->start_error = ret;
    dmosi_thread_sleep(RESET_PULSE_MS);
    context->start_stage = 2;
    ret = chip_wait_busy(context);
    if (ret == 0) { context->start_stage = 3; ret = sx126x_set_standby(chip, false); }
    if (ret == 0 && context->tcxo_mv != 0)
    {
        /* the code of the voltage of the TCXO: 1.6, 1.7, 1.8, 2.2, 2.4, 2.7, 3.0, 3.3 V */
        uint8_t code = context->tcxo_mv <= 1600 ? 0 : context->tcxo_mv <= 1700 ? 1 : context->tcxo_mv <= 1800 ? 2 : context->tcxo_mv <= 2200 ? 3 :
                       context->tcxo_mv <= 2400 ? 4 : context->tcxo_mv <= 2700 ? 5 : context->tcxo_mv <= 3000 ? 6 : 7;
        ret = sx126x_set_dio3_tcxo(chip, code, TCXO_DELAY_UNITS);
        dmosi_thread_sleep(10);
    }
    if (ret == 0) { context->start_stage = 4; ret = sx126x_calibrate_all(chip); }
    dmosi_thread_sleep(5);
    if (ret == 0) ret = sx126x_set_standby(chip, false);
    if (ret == 0) ret = sx126x_set_regulator_dcdc(chip);
    if (ret == 0) ret = sx126x_set_dio2_rf_switch(chip, context->dio2_rf_switch);
    if (ret == 0) { context->start_stage = 5; ret = apply_lora(context); }
    if (ret == 0)
    {
        uint8_t ocp = context->ocp_step;
        ret = sx126x_write_register(chip, SX126X_REG_OCP, &ocp, 1);
    }
    if (ret == 0) ret = start_listening(context);
    if (ret == 0)
    {
        uint8_t status = 0;
        context->start_stage = 6;
        ret = sx126x_get_status(chip, &status);
        context->start_status = status;
        if (ret == 0 && SX126X_STATUS_MODE(status) != SX126X_MODE_RX)
            ret = -EIO;                                 /* the chip is not there, or does not do what it is told */
    }
    context->ready = ret == 0;
    context->start_error = ret;
    return ret;
}

/* A frame the chip has received: the lock is held */
static void take_frame(dmdrvi_context_t context, uint16_t irq)
{
    uint8_t length = 0, start = 0;
    if ((irq & SX126X_IRQ_CRC_ERR) != 0)
    {
        context->crc_errors++;
        if (context->lora.crc)
            return;                                     /* the chip hands it over, the interface does not */
    }
    if (sx126x_get_rx_buffer_status(&context->chip, &length, &start) != 0 || length == 0)
        return;
    frame_t *frame = &context->ring[(context->ring_head + context->ring_count) % RING_FRAMES];
    if (context->ring_count == RING_FRAMES)
    {
        context->ring_head = (uint8_t)((context->ring_head + 1U) % RING_FRAMES);       /* the oldest is lost */
        context->ring_count--;
        frame = &context->ring[(context->ring_head + context->ring_count) % RING_FRAMES];
        (void)dmosi_semaphore_wait(context->frames, 1, 0);
    }
    if (sx126x_read_buffer(&context->chip, start, frame->data, length) != 0)
        return;
    frame->length = length;
    frame->crc_error = (irq & SX126X_IRQ_CRC_ERR) != 0 ? 1U : 0U;
    (void)sx126x_get_packet_status(&context->chip, &frame->rssi_dbm, &frame->snr_dbx4);
    context->last_rssi_dbm = frame->rssi_dbm;
    context->last_snr_dbx4 = frame->snr_dbx4;
    context->frames_received++;
    context->ring_count++;
    (void)dmosi_semaphore_post(context->frames, 1);
}

/* One look at the chip while it listens: the lock is held */
static void service_locked(dmdrvi_context_t context)
{
    uint16_t irq = 0;
    if (context->transmitting)
        return;
    if (!context->ready)
    {
        if (friends_there(context))
            (void)chip_start(context);
        return;
    }
    int level = gpio_get(&context->dio1);
    if (level != 1)
        return;
    if (sx126x_get_irq(&context->chip, &irq) != 0)
    {
        context->ready = false;                         /* the chip does not answer: it is started again */
        return;
    }
    (void)sx126x_clear_irq(&context->chip, irq);
    if ((irq & SX126X_IRQ_RX_DONE) != 0)
        take_frame(context, irq);
}

static void worker_main(void *arg)
{
    dmdrvi_context_t context = arg;
    while (!context->stopping)
    {
        dmosi_mutex_lock(context->lock);
        service_locked(context);
        dmosi_mutex_unlock(context->lock);
        dmosi_thread_sleep(context->ready ? POLL_MS : 200U);
    }
}

/* ---- The commands of the interface ---- */

static void lora_to_config(const sx126x_lora_t *lora, dmdrvi_lora_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->frequency_hz = lora->frequency_hz;
    config->bandwidth_hz = lora->bandwidth_hz;
    config->spreading_factor = lora->spreading_factor;
    config->coding_rate = lora->coding_rate;
    config->tx_power_dbm = lora->tx_power_dbm;
    config->crc = lora->crc ? 1U : 0U;
    config->preamble_symbols = lora->preamble_symbols;
    config->sync_word = lora->sync_word;
    config->implicit_header = lora->implicit_header ? 1U : 0U;
    config->iq_invert = lora->iq_invert ? 1U : 0U;
    config->payload_length = lora->payload_length;
}

static void config_to_lora(const dmdrvi_lora_config_t *config, sx126x_lora_t *lora)
{
    lora->frequency_hz = config->frequency_hz;
    lora->bandwidth_hz = config->bandwidth_hz;
    lora->spreading_factor = config->spreading_factor;
    lora->coding_rate = config->coding_rate;
    lora->tx_power_dbm = config->tx_power_dbm;
    lora->crc = config->crc != 0;
    lora->preamble_symbols = config->preamble_symbols;
    lora->sync_word = config->sync_word;
    lora->implicit_header = config->implicit_header != 0;
    lora->iq_invert = config->iq_invert != 0;
    lora->payload_length = config->payload_length;
}

static int get_info(dmdrvi_context_t context, dmdrvi_lora_info_t *info)
{
    (void)context;
    memset(info, 0, sizeof(*info));
    Dmod_SnPrintf(info->driver, sizeof(info->driver), "dmsx126x");
    Dmod_SnPrintf(info->chip, sizeof(info->chip), "SX1262");
    info->capabilities = DMDRVI_LORA_CAP_LORA | DMDRVI_LORA_CAP_CAD | DMDRVI_LORA_CAP_IQ_INVERT;
    info->frequency_min_hz = 150000000U;
    info->frequency_max_hz = 960000000U;
    info->tx_power_min_dbm = SX126X_TX_POWER_MIN_DBM;
    info->tx_power_max_dbm = SX126X_TX_POWER_MAX_DBM;
    return 0;
}

static int get_config(dmdrvi_context_t context, dmdrvi_lora_config_t *config)
{
    dmosi_mutex_lock(context->lock);
    lora_to_config(&context->lora, config);
    dmosi_mutex_unlock(context->lock);
    return 0;
}

static int set_config(dmdrvi_context_t context, const dmdrvi_lora_config_t *config)
{
    sx126x_lora_t lora = context->lora;
    config_to_lora(config, &lora);
    if (sx126x_check_lora(&lora) != 0 || (lora.implicit_header && lora.payload_length == 0))
        return -EINVAL;
    dmosi_mutex_lock(context->lock);
    int ret = 0;
    if (context->transmitting)
        ret = -EBUSY;
    else
    {
        context->lora = lora;
        if (context->ready)
        {
            ret = sx126x_set_standby(&context->chip, false);
            if (ret == 0) ret = apply_lora(context);
            if (ret == 0) ret = start_listening(context);
            if (ret != 0)
                context->ready = false;                 /* the thread starts the chip again, with the new configuration */
        }
    }
    dmosi_mutex_unlock(context->lock);
    return ret;
}

static int send_frame(dmdrvi_context_t context, dmdrvi_lora_frame_t *frame)
{
    if (frame->length == 0 || frame->length > DMDRVI_LORA_MAX_PAYLOAD)
        return -EINVAL;
    uint32_t timeout = frame->timeout_ms != 0 ? frame->timeout_ms : 5000U;
    dmosi_mutex_lock(context->lock);
    if (context->transmitting)
    {
        dmosi_mutex_unlock(context->lock);
        return -EBUSY;
    }
    if (!context->ready)
    {
        dmosi_mutex_unlock(context->lock);
        return -ENODEV;
    }
    sx126x_chip_t *chip = &context->chip;
    uint8_t length = (uint8_t)frame->length;
    int ret = sx126x_set_standby(chip, false);
    if (ret == 0) ret = sx126x_set_packet(chip, &context->lora, context->lora.implicit_header ? context->lora.payload_length : length);
    if (ret == 0) ret = sx126x_set_buffer_base(chip, 0, 0);
    if (ret == 0) ret = sx126x_write_buffer(chip, 0, frame->data, length);
    if (ret == 0) ret = sx126x_clear_irq(chip, SX126X_IRQ_ALL);
    if (ret == 0) ret = sx126x_set_tx(chip, timeout);
    if (ret != 0)
    {
        context->ready = false;
        dmosi_mutex_unlock(context->lock);
        return -EIO;
    }
    context->transmitting = true;
    dmosi_mutex_unlock(context->lock);

    /* The thread leaves the chip alone meanwhile: the interrupt is looked at here. The loop leaves with the lock held. */
    uint32_t started = dmosi_get_tick_count();
    bool done = false, failed = false;
    while (true)
    {
        dmosi_thread_sleep(POLL_MS);
        dmosi_mutex_lock(context->lock);
        uint16_t irq = 0;
        if (gpio_get(&context->dio1) == 1 && sx126x_get_irq(chip, &irq) == 0)
        {
            (void)sx126x_clear_irq(chip, irq);
            if ((irq & SX126X_IRQ_TX_DONE) != 0)
                done = true;
            else if ((irq & SX126X_IRQ_TIMEOUT) != 0)
                failed = true;
        }
        if (done || failed)
            break;
        dmosi_mutex_unlock(context->lock);
        if ((uint32_t)(dmosi_get_tick_count() - started) > timeout + 200U)
        {
            dmosi_mutex_lock(context->lock);
            failed = true;
            break;
        }
    }
    context->transmitting = false;
    if (done)
    {
        context->frames_sent++;
        frame->airtime_us = sx126x_airtime_us(&context->lora, length);
        ret = start_listening(context);
        if (ret != 0)
            context->ready = false;                     /* the thread starts the chip again */
    }
    else
    {
        context->tx_timeouts++;
        context->ready = false;                         /* the chip is reset and configured again by the thread */
        ret = -ETIMEDOUT;
    }
    dmosi_mutex_unlock(context->lock);
    return ret;
}

static int receive_frame(dmdrvi_context_t context, dmdrvi_lora_frame_t *frame)
{
    uint32_t timeout = frame->timeout_ms;
    if (dmosi_semaphore_wait(context->frames, 1, timeout == 0 ? 0 : (int32_t)timeout) != 0)
        return timeout == 0 ? -EAGAIN : -ETIMEDOUT;
    dmosi_mutex_lock(context->lock);
    if (context->ring_count == 0)
    {
        dmosi_mutex_unlock(context->lock);
        return -EAGAIN;
    }
    const frame_t *held = &context->ring[context->ring_head];
    memset(frame, 0, sizeof(*frame));
    frame->length = held->length;
    frame->rssi_dbm = held->rssi_dbm;
    frame->snr_dbx4 = held->snr_dbx4;
    frame->crc_error = held->crc_error;
    memcpy(frame->data, held->data, held->length);
    context->ring_head = (uint8_t)((context->ring_head + 1U) % RING_FRAMES);
    context->ring_count--;
    dmosi_mutex_unlock(context->lock);
    return 0;
}

static int channel_activity(dmdrvi_context_t context, dmdrvi_lora_cad_t *cad)
{
    dmosi_mutex_lock(context->lock);
    if (context->transmitting || !context->ready)
    {
        dmosi_mutex_unlock(context->lock);
        return context->transmitting ? -EBUSY : -ENODEV;
    }
    sx126x_chip_t *chip = &context->chip;
    context->transmitting = true;                       /* (the thread leaves the chip alone, as it does while a frame is sent) */
    int ret = sx126x_set_standby(chip, false);
    if (ret == 0) ret = sx126x_set_irq(chip, SX126X_IRQ_ALL, SX126X_IRQ_CAD_DONE);
    if (ret == 0) ret = sx126x_clear_irq(chip, SX126X_IRQ_ALL);
    if (ret == 0) ret = sx126x_set_cad(chip, context->lora.spreading_factor, 2);
    uint16_t irq = 0;
    uint32_t timeout = cad->timeout_ms != 0 ? cad->timeout_ms : 200U;
    bool done = false;
    for (uint32_t waited = 0; ret == 0 && !done && waited < timeout; waited += POLL_MS)
    {
        dmosi_mutex_unlock(context->lock);
        dmosi_thread_sleep(POLL_MS);
        dmosi_mutex_lock(context->lock);
        if (gpio_get(&context->dio1) == 1 && sx126x_get_irq(chip, &irq) == 0 && (irq & SX126X_IRQ_CAD_DONE) != 0)
            done = true;
    }
    cad->busy = (done && (irq & SX126X_IRQ_CAD_DETECTED) != 0) ? 1U : 0U;
    context->transmitting = false;
    int back = start_listening(context);
    if (ret == 0 && !done)
        ret = -ETIMEDOUT;
    if (back != 0)
        context->ready = false;
    dmosi_mutex_unlock(context->lock);
    return ret;
}

static int get_status(dmdrvi_context_t context, dmdrvi_lora_status_t *status)
{
    memset(status, 0, sizeof(*status));
    dmosi_mutex_lock(context->lock);
    status->state = !context->ready ? DMDRVI_LORA_STATE_UNKNOWN : context->transmitting ? DMDRVI_LORA_STATE_TX : DMDRVI_LORA_STATE_RX;
    status->last_rssi_dbm = context->last_rssi_dbm;
    status->last_snr_dbx4 = context->last_snr_dbx4;
    status->frames_sent = context->frames_sent;
    status->frames_received = context->frames_received;
    status->crc_errors = context->crc_errors;
    status->tx_timeouts = context->tx_timeouts;
    if (context->ready && !context->transmitting)
    {
        int16_t rssi = 0;
        if (sx126x_get_rssi_inst(&context->chip, &rssi) == 0)
            status->channel_rssi_dbm = rssi;
    }
    dmosi_mutex_unlock(context->lock);
    return 0;
}

/* ---- The driver interface ---- */

static void read_lora_config(dmini_context_t config, sx126x_lora_t *lora)
{
    /* the defaults of a mesh of MeshCore on 868 MHz (EU narrow); the section of the board overrides them */
    lora->frequency_hz = (uint32_t)dmini_get_int(config, NULL, "frequency_hz", 869618000);
    lora->bandwidth_hz = (uint32_t)dmini_get_int(config, NULL, "bandwidth_hz", 62500);
    lora->spreading_factor = (uint8_t)dmini_get_int(config, NULL, "spreading_factor", 8);
    lora->coding_rate = (uint8_t)dmini_get_int(config, NULL, "coding_rate", 8);
    lora->tx_power_dbm = (int8_t)dmini_get_int(config, NULL, "tx_power_dbm", 14);
    lora->crc = dmini_get_int(config, NULL, "crc", 1) != 0;
    lora->preamble_symbols = (uint16_t)dmini_get_int(config, NULL, "preamble_symbols", 16);
    lora->sync_word = (uint16_t)dmini_get_int(config, NULL, "sync_word", SX126X_SYNC_WORD_PRIVATE);
    lora->implicit_header = false;
    lora->iq_invert = false;
    lora->payload_length = 0;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, dmdrvi_context_t, _create, ( dmini_context_t config, dmdrvi_dev_num_t* dev_num ))
{
    if (config == NULL || dev_num == NULL)
        return NULL;
    dmdrvi_context_t context = Dmod_Malloc(sizeof(struct dmdrvi_context));
    if (context == NULL)
        return NULL;
    memset(context, 0, sizeof(*context));
    context->magic = CONTEXT_MAGIC;
    read_lora_config(config, &context->lora);
    context->tcxo_mv = (uint16_t)dmini_get_int(config, NULL, "tcxo_mv", 0);
    context->dio2_rf_switch = dmini_get_int(config, NULL, "dio2_rf_switch", 1) != 0;
    context->ocp_step = (uint8_t)(dmini_get_int(config, NULL, "current_limit_ma", 140) * 2 / 5);
    if (sx126x_check_lora(&context->lora) != 0)
    {
        DMOD_LOG_ERROR("SX126x: the configuration of the modulation is not valid\n");
        context->magic = 0;
        Dmod_Free(context);
        return NULL;
    }
    context->lock = dmosi_mutex_create(false);
    context->frames = dmosi_semaphore_create(0, RING_FRAMES + 1U);
    if (context->lock == NULL || context->frames == NULL)
    {
        if (context->lock != NULL)
            dmosi_mutex_destroy(context->lock);
        if (context->frames != NULL)
            dmosi_semaphore_destroy(context->frames);
        context->magic = 0;
        Dmod_Free(context);
        return NULL;
    }
    context->chip.transfer = chip_transfer;
    context->chip.wait_busy = chip_wait_busy;
    context->chip.user = context;
    /* The node is named after the section of the board's ini ([lora]: /dev/lora, the name clients look for - dmdevfs fills alt_name in) */
    memset(dev_num, 0, sizeof(*dev_num));
    dev_num->flags = DMDRVI_NUM_MAJOR | DMDRVI_NUM_ALT_NAME;
    dev_num->major = (dmdrvi_dev_id_t)dmini_get_int(config, NULL, "instance", 0);
    return context;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, void, _free, ( dmdrvi_context_t context ))
{
    if (!is_valid_context(context))
        return;
    if (context->worker != NULL)
    {
        context->stopping = true;
        (void)dmosi_thread_join(context->worker);
        dmosi_thread_destroy(context->worker);
        context->worker = NULL;
    }
    /* The nodes of the friends are not touched: dmdevfs tears them down before this call */
    Dmod_Free(context->spi.path);
    Dmod_Free(context->cs.path);
    Dmod_Free(context->reset.path);
    Dmod_Free(context->busy.path);
    Dmod_Free(context->dio1.path);
    Dmod_Free(context->power.path);
    dmosi_mutex_destroy(context->lock);
    dmosi_semaphore_destroy(context->frames);
    context->magic = 0;
    Dmod_Free(context);
}

static void try_start(dmdrvi_context_t context)
{
    if (context->worker != NULL || !friends_there(context))
        return;
    context->worker = dmosi_thread_create(worker_main, context, 2, 4096, "lora", dmosi_process_current());
    if (context->worker == NULL)
        DMOD_LOG_ERROR("SX126x: no thread\n");
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, void, _friend_changed, ( dmdrvi_context_t context, const dmdrvi_friend_info_t* info ))
{
    if (!is_valid_context(context) || info == NULL || info->friend_role == NULL)
        return;
    node_t *node = strcmp(info->friend_role, "spi") == 0 ? &context->spi
                 : strcmp(info->friend_role, "lora_cs") == 0 ? &context->cs
                 : strcmp(info->friend_role, "lora_reset") == 0 ? &context->reset
                 : strcmp(info->friend_role, "lora_busy") == 0 ? &context->busy
                 : strcmp(info->friend_role, "lora_dio1") == 0 ? &context->dio1
                 : strcmp(info->friend_role, "lora_power") == 0 ? &context->power : NULL;
    if (node == NULL)
        return;
    dmosi_mutex_lock(context->lock);
    node_forget(node);
    context->ready = false;
    if (info->state == dmdrvi_dev_state_ready && info->node_path != NULL)
    {
        node->path = Dmod_StrDup(info->node_path);
        if (node->path == NULL)
            DMOD_LOG_ERROR("SX126x: out of memory for the path of a friend\n");
    }
    dmosi_mutex_unlock(context->lock);
    try_start(context);
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, void*, _open, ( dmdrvi_context_t context, int flags, const dmdrvi_dev_num_t* dev_num ))
{
    return is_valid_context(context) ? context : NULL;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, void, _close, ( dmdrvi_context_t context, void* handle ))
{
}

/* The text of read(): the radio as lines "key: value" (cat /dev/lora), for a shell */
static void make_report(dmdrvi_context_t context)
{
    dmdrvi_lora_status_t s;
    dmdrvi_lora_config_t c;
    (void)get_status(context, &s);
    (void)get_config(context, &c);
    Dmod_SnPrintf(context->report, sizeof(context->report),
                  "chip: SX1262 (%s)\nfrequency: %u Hz\nbandwidth: %u Hz  sf: %u  cr: 4/%u  power: %d dBm  preamble: %u  sync: 0x%04X  crc: %u\n"
                  "sent: %u  received: %u  crc errors: %u  tx timeouts: %u\nlast frame: %d dBm, snr %d.%02u dB\nchannel: %d dBm\nstart: %u times, stage %u, error %d, status 0x%02X\nlines: busy %d, dio1 %d\n",
                  s.state == DMDRVI_LORA_STATE_UNKNOWN ? "not ready" : s.state == DMDRVI_LORA_STATE_TX ? "sending" : "listening",
                  (unsigned)c.frequency_hz, (unsigned)c.bandwidth_hz, (unsigned)c.spreading_factor, (unsigned)c.coding_rate, (int)c.tx_power_dbm,
                  (unsigned)c.preamble_symbols, (unsigned)c.sync_word, (unsigned)c.crc, (unsigned)s.frames_sent, (unsigned)s.frames_received,
                  (unsigned)s.crc_errors, (unsigned)s.tx_timeouts, (int)s.last_rssi_dbm, (int)(s.last_snr_dbx4 / 4),
                  (unsigned)((s.last_snr_dbx4 < 0 ? -s.last_snr_dbx4 : s.last_snr_dbx4) % 4) * 25U, (int)s.channel_rssi_dbm,
                  context->starts, (unsigned)context->start_stage, context->start_error, (unsigned)context->start_status,
                  gpio_get(&context->busy), gpio_get(&context->dio1));
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, dmdrvi_ssize_t, _read, ( dmdrvi_context_t context, void* handle, void* buffer, size_t size, dmdrvi_offset_t offset ))
{
    if (!is_valid_context(context) || (buffer == NULL && size != 0) || offset < 0)
        return -EINVAL;
    if (offset == 0)
        make_report(context);
    size_t length = strlen(context->report);
    if ((size_t)offset >= length)
        return 0;
    size_t count = length - (size_t)offset < size ? length - (size_t)offset : size;
    memcpy(buffer, context->report + offset, count);
    return (dmdrvi_ssize_t)count;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, dmdrvi_ssize_t, _write, ( dmdrvi_context_t context, void* handle, const void* buffer, size_t size, dmdrvi_offset_t offset ))
{
    return -ENOTSUP;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, int, _ioctl, ( dmdrvi_context_t context, void* handle, int command, void* arg ))
{
    if (!is_valid_context(context))
        return -EINVAL;
    if (arg == NULL && command >= DMDRVI_IOCTL_LORA_GET_INFO && command <= DMDRVI_IOCTL_LORA_GET_STATUS)
        return -EINVAL;
    switch (command)
    {
        case DMDRVI_IOCTL_LORA_GET_INFO:    return get_info(context, (dmdrvi_lora_info_t *)arg);
        case DMDRVI_IOCTL_LORA_GET_CONFIG:  return get_config(context, (dmdrvi_lora_config_t *)arg);
        case DMDRVI_IOCTL_LORA_SET_CONFIG:  return set_config(context, (const dmdrvi_lora_config_t *)arg);
        case DMDRVI_IOCTL_LORA_SEND:        return send_frame(context, (dmdrvi_lora_frame_t *)arg);
        case DMDRVI_IOCTL_LORA_RECV:        return receive_frame(context, (dmdrvi_lora_frame_t *)arg);
        case DMDRVI_IOCTL_LORA_CAD:         return channel_activity(context, (dmdrvi_lora_cad_t *)arg);
        case DMDRVI_IOCTL_LORA_GET_STATUS:  return get_status(context, (dmdrvi_lora_status_t *)arg);
        case DMDRVI_IOCTL_LORA_AIRTIME:
        {
            dmdrvi_lora_airtime_t *airtime = arg;
            dmosi_mutex_lock(context->lock);
            airtime->airtime_us = sx126x_airtime_us(&context->lora, (uint8_t)(airtime->length > 255 ? 255 : airtime->length));
            dmosi_mutex_unlock(context->lock);
            return 0;
        }
        default:
            return -ENOTTY;
    }
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, int, _flush, ( dmdrvi_context_t context, void* handle ))
{
    return 0;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmsx126x, int, _stat, ( dmdrvi_context_t context, const char* path, dmdrvi_stat_t* stat ))
{
    if (!is_valid_context(context) || stat == NULL)
        return -EINVAL;
    stat->size = 0;
    stat->mode = 0666;
    return 0;
}

int dmod_init(const Dmod_Config_t *Config)
{
    (void)Config;
    return 0;
}

int dmod_deinit(void)
{
    return 0;
}
