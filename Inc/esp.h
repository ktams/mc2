/**
 * @file esp.h
 *
 * @author Andi
 * @date   07.04.2020
 */

/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2020 Tams Elektronik GmbH and Andreas Kretzer
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ESP_H__
#define __ESP_H__

#define BOOT_ADDR				0x00000000				///< where to flash the bootloader
#define PARTITION_ADDR			0x00008000				///< where to flash the partition table
#define APPLICATION_ADDR		0x00010000				///< where to flash the application code
#define ESP_DIR					"/esp/"
#define BOOTLOADER_FILE			"bootloader.bin"
#define PARTITION_FILE			"partitions_singleapp.bin"
#define APPLICATION_FILE		"mc2wlan.bin"

#define SLIP_BLOCK				0xC0					///< the character that starts and ends a block on the line
#define SLIP_ESCAPE				0xDB					///< the SLIP_BLOCK and SLIP_ESCAPE must be escaped if part of the data stream
#define SLIP_ESC_BLOCK			0xDC					///< 0xDB + 0xDC are used to encode a single 0xC0 in the data stream
#define SLIP_ESC_ESCAPE			0xDD					///< 0xDB + 0xDD are used to encode a single 0xDB in the data stream

#define BLOCKSIZE_FLASH			0x400
#define SECTOR_SIZE				0x1000
#define SECTORS_PER_BLOCK		16
#define BOOT_PACKET_SIZE		(BLOCKSIZE_FLASH + 4 * sizeof(uint32_t))

// the command opcodes of the boot loader (ESP8266 factory boot loader)
#define ESPBL_FLASH_BEGIN		0x02
#define ESPBL_FLASH_DATA		0x03
#define ESPBL_FLASH_END			0x04
#define ESPBL_MEM_BEGIN			0x05
#define ESPBL_MEM_END			0x06
#define ESPBL_MEM_DATA			0x07
#define ESPBL_SYNC				0x08
#define ESPBL_WRITE_REG			0x09
#define ESPBL_READ_REG			0x0A

// the extended command opcodes of the boot loader (ESP32 factory boot loader)
#define ESPBL_SPI_SET_PARAMS	0x0B
#define ESPBL_SPI_ATTACH		0x0D
#define ESPBL_CHANGE_BAUDRATE	0x0F
#define ESPBL_FLASH_DEFL_BEGIN	0x10
#define ESPBL_FLASH_DEFL_DATA	0x11
#define ESPBL_FLASH_DEFL_END	0x12
#define ESPBL_FLASH_MD5			0x13

// the extended command opcodes of the software loader (see https://github.com/espressif/esptool/wiki)
#define ESPBL_ERASE_FLASH		0xD0
#define ESPBL_ERASE_REGION		0xD1
#define ESPBL_READ_FLSH			0xD2
#define ESPBL_RUN_USER_CODE		0xD3

struct bootpacket {
	unsigned		dir:8;							///< direction 0: request to module, 1: answer from module
	unsigned		cmd:8;							///< the command for this block
	unsigned		size:16;						///< the (data-)size of this packet
	uint32_t		chk;							///< a one byte XOR chksum for packets containing FLASH or RAM data
	uint8_t			data[BOOT_PACKET_SIZE];			///< the data payload of "size" elements (if any)
};

enum slipstat {
	SLIP_STARTUP = 0,			///< SLIP did not start yet, try to debug-print any output from ESP-01
	SLIP_IDLE,					///< we are outside a block, wait for / send a block start character
	SLIP_TRANSFER,				///< receive / send block data
	SLIP_ESCSTATE,				///< we have received / transmitted an escape character (SLIP_ESCAPE)
	SLIP_COMPLETE,				///< the received block is complete / transfer is complete (send terminating block char), wake up thread
};

#endif /* __ESP_H__ */
