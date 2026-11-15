# dmsx126x API

The module has no API of its own: its node (`/dev/lora`) is the LoRa interface of dmdrvi (`DMDRVI_IOCTL_LORA_*`, see `docs/dmdrvi.md` of dmdrvi).
The commands of the chip are in `src/sx126x.h` (private: one function per command of the datasheet, tested in `tests/`).
