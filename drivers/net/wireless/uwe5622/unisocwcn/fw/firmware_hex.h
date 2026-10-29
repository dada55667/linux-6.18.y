#ifndef _FIRMWARE_HEX_H
#define _FIRMWARE_HEX_H
/*
 * Embedded hex firmware is not shipped in this in-tree copy.
 * Load wcnmodem.bin from /lib/firmware (see UNISOC_FW_PATH_CONFIG).
 * Do not enable CONFIG_WCN_DOWNLOAD_FIRMWARE_FROM_HEX without restoring
 * wcnmodem.bin.hex next to this header.
 */
#if defined(CONFIG_WCN_DOWNLOAD_FIRMWARE_FROM_HEX)
#error "unisocwcn: embedded wcnmodem.bin.hex not packaged; use /lib/firmware"
#endif
#endif /* _FIRMWARE_HEX_H */
