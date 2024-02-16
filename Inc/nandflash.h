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

#ifndef __NANDFLASH_H__
#define __NANDFLASH_H__

#include "yaffs_guts.h"

/*
 * Prototypes of the NAND driver
 */
int nand_write_chunk (struct yaffs_dev *dev, int nand_chunk, const u8 *data, int data_len, const u8 *oob, int oob_len);
int nand_read_chunk (struct yaffs_dev *dev, int nand_chunk, u8 *data, int data_len, u8 *oob, int oob_len, enum yaffs_ecc_result *ecc_result);
int nand_erase (struct yaffs_dev *dev, int block_no);
int nand_mark_bad (struct yaffs_dev *dev, int block_no);
int nand_check_bad (struct yaffs_dev *dev, int block_no);
int nand_initialise (struct yaffs_dev *dev);
int nand_deinitialise (struct yaffs_dev *dev);

#endif /* __NANDFLASH_H__ */
