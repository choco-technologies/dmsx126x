#define DMOD_ENABLE_REGISTRATION ON
#include "dmod.h"
#include "dmdrvi_ioctl.h"
#include <errno.h>
#include <string.h>

/* lora [-d NODE] info | status | config | set KEY=VALUE... | rx [SECONDS] | tx HEX | cad | airtime BYTES */

static bool parse_number(const char *text, int64_t *value)
{
    bool negative = text[0] == '-';
    const char *c = text + (negative ? 1 : 0);
    int base = 10;
    if (c[0] == '0' && (c[1] == 'x' || c[1] == 'X'))
    {
        base = 16;
        c += 2;
    }
    if (*c == '\0')
        return false;
    int64_t result = 0;
    for (; *c != '\0'; c++)
    {
        int digit = (*c >= '0' && *c <= '9') ? *c - '0' : (*c >= 'a' && *c <= 'f') ? *c - 'a' + 10 : (*c >= 'A' && *c <= 'F') ? *c - 'A' + 10 : 99;
        if (digit >= base)
            return false;
        result = result * base + digit;
    }
    *value = negative ? -result : result;
    return true;
}

static void print_config(const dmdrvi_lora_config_t *c)
{
    Dmod_Printf("frequency=%u bandwidth=%u sf=%u cr=%u power=%d crc=%u preamble=%u sync=0x%04X implicit=%u iq=%u\n", (unsigned)c->frequency_hz,
                (unsigned)c->bandwidth_hz, (unsigned)c->spreading_factor, (unsigned)c->coding_rate, (int)c->tx_power_dbm, (unsigned)c->crc,
                (unsigned)c->preamble_symbols, (unsigned)c->sync_word, (unsigned)c->implicit_header, (unsigned)c->iq_invert);
}

static int set_key(dmdrvi_lora_config_t *c, const char *argument)
{
    const char *equal = strchr(argument, '=');
    int64_t v = 0;
    if (equal == NULL || !parse_number(equal + 1, &v))
        return -EINVAL;
    size_t length = (size_t)(equal - argument);
    if (length == 9 && strncmp(argument, "frequency", 9) == 0) c->frequency_hz = (uint32_t)v;
    else if (length == 9 && strncmp(argument, "bandwidth", 9) == 0) c->bandwidth_hz = (uint32_t)v;
    else if (length == 2 && strncmp(argument, "sf", 2) == 0) c->spreading_factor = (uint8_t)v;
    else if (length == 2 && strncmp(argument, "cr", 2) == 0) c->coding_rate = (uint8_t)v;
    else if (length == 5 && strncmp(argument, "power", 5) == 0) c->tx_power_dbm = (int8_t)v;
    else if (length == 3 && strncmp(argument, "crc", 3) == 0) c->crc = (uint8_t)v;
    else if (length == 8 && strncmp(argument, "preamble", 8) == 0) c->preamble_symbols = (uint16_t)v;
    else if (length == 4 && strncmp(argument, "sync", 4) == 0) c->sync_word = (uint16_t)v;
    else if (length == 2 && strncmp(argument, "iq", 2) == 0) c->iq_invert = (uint8_t)v;
    else return -EINVAL;
    return 0;
}

static void print_frame(const dmdrvi_lora_frame_t *f)
{
    Dmod_Printf("%u bytes  %d dBm  snr %d.%02u dB%s  ", (unsigned)f->length, (int)f->rssi_dbm, (int)(f->snr_dbx4 / 4),
                (unsigned)((f->snr_dbx4 < 0 ? -f->snr_dbx4 : f->snr_dbx4) % 4) * 25U, f->crc_error ? "  CRC ERROR" : "");
    for (unsigned i = 0; i < f->length && i < 48; i++)
        Dmod_Printf("%02x", (unsigned)f->data[i]);
    Dmod_Printf(f->length > 48 ? "...\n" : "\n");
}

static int receive(void *file, int64_t seconds)
{
    unsigned count = 0;
    for (int64_t t = 0; t < seconds; t++)
    {
        for (;;)
        {
            dmdrvi_lora_frame_t frame;
            memset(&frame, 0, sizeof(frame));
            frame.timeout_ms = t == 0 && count == 0 ? 1000U : 1000U;
            int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_RECV, &frame);
            if (ret == -ETIMEDOUT || ret == -EAGAIN)
                break;
            if (ret != 0)
                return ret;
            print_frame(&frame);
            count++;
            break;
        }
    }
    Dmod_Printf("%u frames in %d s\n", count, (int)seconds);
    return 0;
}

static int hex_digit(char c)
{
    return (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
}

static int execute(void *file, int argc, char **argv)
{
    const char *action = argc == 0 ? "status" : argv[0];
    if (strcmp(action, "info") == 0)
    {
        dmdrvi_lora_info_t info;
        int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_GET_INFO, &info);
        if (ret == 0)
            Dmod_Printf("%s %s: %u..%u Hz, %d..%d dBm, capabilities 0x%X\n", info.driver, info.chip, (unsigned)info.frequency_min_hz,
                        (unsigned)info.frequency_max_hz, (int)info.tx_power_min_dbm, (int)info.tx_power_max_dbm, (unsigned)info.capabilities);
        return ret;
    }
    if (strcmp(action, "status") == 0)
    {
        dmdrvi_lora_status_t s;
        int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_GET_STATUS, &s);
        if (ret == 0)
            Dmod_Printf("state=%u sent=%u received=%u crc_errors=%u tx_timeouts=%u last=%d dBm snr_x4=%d channel=%d dBm\n", (unsigned)s.state,
                        (unsigned)s.frames_sent, (unsigned)s.frames_received, (unsigned)s.crc_errors, (unsigned)s.tx_timeouts, (int)s.last_rssi_dbm,
                        (int)s.last_snr_dbx4, (int)s.channel_rssi_dbm);
        return ret;
    }
    if (strcmp(action, "config") == 0 || strcmp(action, "set") == 0)
    {
        dmdrvi_lora_config_t config;
        int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_GET_CONFIG, &config);
        if (ret == 0 && action[0] == 's')
        {
            for (int i = 1; i < argc && ret == 0; i++)
                ret = set_key(&config, argv[i]);
            if (ret == 0)
                ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_SET_CONFIG, &config);
            if (ret == 0)
                ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_GET_CONFIG, &config);
        }
        if (ret == 0)
            print_config(&config);
        return ret;
    }
    if (strcmp(action, "rx") == 0)
    {
        int64_t seconds = 10;
        if (argc > 1 && !parse_number(argv[1], &seconds))
            return -EINVAL;
        return receive(file, seconds);
    }
    if (strcmp(action, "tx") == 0 && argc == 2)
    {
        dmdrvi_lora_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        size_t digits = strlen(argv[1]);
        if (digits == 0 || digits % 2 != 0 || digits / 2 > DMDRVI_LORA_MAX_PAYLOAD)
            return -EINVAL;
        for (size_t i = 0; i < digits / 2; i++)
        {
            int hi = hex_digit(argv[1][2 * i]), lo = hex_digit(argv[1][2 * i + 1]);
            if (hi < 0 || lo < 0)
                return -EINVAL;
            frame.data[i] = (uint8_t)(hi * 16 + lo);
        }
        frame.length = (uint16_t)(digits / 2);
        frame.timeout_ms = 5000;
        int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_SEND, &frame);
        if (ret == 0)
            Dmod_Printf("sent %u bytes, %u us in the air\n", (unsigned)frame.length, (unsigned)frame.airtime_us);
        return ret;
    }
    if (strcmp(action, "cad") == 0)
    {
        dmdrvi_lora_cad_t cad = { .timeout_ms = 500 };
        int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_CAD, &cad);
        if (ret == 0)
            Dmod_Printf("channel %s\n", cad.busy ? "busy" : "free");
        return ret;
    }
    if (strcmp(action, "airtime") == 0 && argc == 2)
    {
        int64_t bytes = 0;
        dmdrvi_lora_airtime_t airtime = { 0 };
        if (!parse_number(argv[1], &bytes) || bytes < 1 || bytes > 255)
            return -EINVAL;
        airtime.length = (uint16_t)bytes;
        int ret = Dmod_Ioctl(file, DMDRVI_IOCTL_LORA_AIRTIME, &airtime);
        if (ret == 0)
            Dmod_Printf("%u bytes: %u us\n", (unsigned)airtime.length, (unsigned)airtime.airtime_us);
        return ret;
    }
    return -EINVAL;
}

int main(int argc, char *argv[])
{
    const char *path = "/dev/lora";
    int first = 1;
    if (argc > 2 && strcmp(argv[1], "-d") == 0)
    {
        path = argv[2];
        first = 3;
    }
    void *file = Dmod_FileOpen(path, "r+");
    if (file == NULL)
    {
        Dmod_Printf("lora: cannot open %s\n", path);
        return 1;
    }
    int ret = execute(file, argc - first, argv + first);
    Dmod_FileClose(file);
    if (ret != 0)
        Dmod_Printf("lora: error %d\nusage: lora [-d NODE] info | status | config | set KEY=VALUE... | rx [SECONDS] | tx HEX | cad | airtime BYTES\n"
                    "keys: frequency bandwidth sf cr power crc preamble sync iq\n", ret);
    return ret == 0 ? 0 : 1;
}
