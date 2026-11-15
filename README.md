# dmsx126x

The driver of the **SX1262** LoRa radio of Semtech (SX1261/SX1268 are the same on the bus) for DMOD - the radio of the LILYGO T-Deck Pro. The node
(`/dev/lora`) follows the **LoRa interface of dmdrvi** (`DMDRVI_IOCTL_LORA_*`, the 0xA00 range): the modulation, a frame sent, a frame received
(with its RSSI and SNR), a look at the channel (CAD, listen before talk), the air time of a frame. A mesh protocol (dmmeshcore) is a client of the node and
knows nothing about the chip.

```
cat /dev/lora
chip: SX1262 (listening)
frequency: 869618000 Hz
bandwidth: 62500 Hz  sf: 8  cr: 4/8  power: 14 dBm  preamble: 16  sync: 0x1424  crc: 1
sent: 0  received: 3  crc errors: 0  tx timeouts: 0
last frame: -71 dBm, snr 6.25 dB
channel: -112 dBm
```

## How it works

* `src/sx126x.c` - the commands of the chip, one function per command, no hardware in it (tested on a PC: `tests/`): what goes over the SPI for each
  of them, the frequency register, the bandwidth codes, the low data rate optimization, the **air time** of a frame (the formula of the datasheet).
* `src/dmsx126x.c` - the driver. The radio is in the `friends_group` of the board's SPI bus; the roles of the friends:

  | friend_role | what |
  |-------------|------|
  | `spi` | the dmspi node of the bus (required) |
  | `lora_cs` | the chip select of the radio, a dmgpio output (required) |
  | `lora_reset` | the reset line (required) |
  | `lora_busy` | BUSY, a dmgpio input (required) |
  | `lora_dio1` | the interrupt line DIO1, a dmgpio input (required) |
  | `lora_power` | the power of the radio, a dmgpio output (optional) |

  The bus can be shared with other devices (on the T-Deck Pro: the e-paper and the SD card): the transfers go with `dmspi_ioctl_cmd_transfer_cs` of
  dmspi (the chip select of the radio frames the transfer, the bus is held from the selection to the deselection).
* The radio listens whenever it does not send. A thread looks at DIO1 and takes the frames into a ring of four; `RECV` gives them in order. `SEND` stops the
  listening for the time of the transmission and returns to it. A transmission that does not end resets the chip and configures it again.
* The configuration (`configs/board/t-deck-pro/lora.ini`): the carrier, the bandwidth, the spreading factor, the coding rate, the power, the CRC, the
  preamble, the sync word, `tcxo_mv` (the TCXO on DIO3), `dio2_rf_switch`, `current_limit_ma`. The defaults are those of a MeshCore mesh on 868 MHz (EU narrow:
  869.618 MHz, 62.5 kHz, SF8, 4/8, sync word 0x1424); a protocol that sets its own (`DMDRVI_IOCTL_LORA_SET_CONFIG`) overrides them.

## Use

```c
dmdrvi_lora_frame_t frame = { .timeout_ms = 1000 };
void *lora = Dmod_FileOpen("/dev/lora", "r+");
if (Dmod_Ioctl(lora, DMDRVI_IOCTL_LORA_RECV, &frame) == 0)
    Dmod_Printf("%u bytes, %d dBm\n", frame.length, frame.rssi_dbm);
```

The duty cycle of a mesh: `DMDRVI_IOCTL_LORA_AIRTIME` says how long a frame of a given length is in the air with the configuration now.

## Build and test

```sh
cmake -S . -B build -DDMOD_TOOLS_NAME=arch/x86_64
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Needs dmdrvi with the LoRa interface (choco-technologies/dmdrvi#42) and dmspi with `dmspi_ioctl_cmd_transfer_cs` (choco-technologies/dmspi#12).

[MIT license](LICENSE)
