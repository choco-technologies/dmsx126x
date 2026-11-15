#ifndef DMSX126X_H
#define DMSX126X_H

/**
 * @file dmsx126x.h
 * @brief dmsx126x: the driver of the SX1262 LoRa radio. It has no functions to call: its node (/dev/lora - the name of the section of the ini; without
 *        one /dev/dmsx126x<instance>) is the LoRa interface of dmdrvi (DMDRVI_IOCTL_LORA_*, dmdrvi_ioctl.h), and cat of the node gives the state as text.
 *
 * What a board's ini file (configs/board/<board>/lora.ini) can set in the section of the driver:
 *   instance           the number of the node (0)
 *   frequency_hz       the carrier (869618000)
 *   bandwidth_hz       7800 .. 500000 (62500)
 *   spreading_factor   5..12 (8)
 *   coding_rate        5..8, the denominator of 4/5 .. 4/8 (8)
 *   tx_power_dbm       -9..22 (14)
 *   crc                1: the frames have a CRC (1)
 *   preamble_symbols   (16)
 *   sync_word          decimal: 5156 = 0x1424 (private), 13380 = 0x3444 (public)
 *   tcxo_mv            the voltage of the TCXO on DIO3, 0: none (0)
 *   dio2_rf_switch     1: DIO2 controls the antenna switch (1)
 *   current_limit_ma   the over current protection (140)
 * and the roles of the friends: spi (the dmspi node of the bus), lora_cs, lora_reset, lora_busy, lora_dio1, lora_power (optional).
 */

#define DMSX126X_DRIVER_NAME        "dmsx126x"

#endif
