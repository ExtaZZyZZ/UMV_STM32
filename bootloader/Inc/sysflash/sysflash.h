/*
 * sysflash.h
 *
 *  Created on: 19 нояб. 2026 г.
 *      Author: i.yakovlev
 */

#ifndef MCUBOOT_CONFIG_SYSFLASH_SYSFLASH_H_
#define MCUBOOT_CONFIG_SYSFLASH_SYSFLASH_H_

#define 	BOARD_FLASH_BASE	0x08000000u
#define 	BOARD_PAGE_SIZE	0x800u
#define 	BOARD_SLOT0_OFF		0xC000u
#define 	BOARD_SLOT0_SIZE	0x34000u
#define 	BOARD_SLOT1_OFF		0x40000u
#define 	BOARD_SLOT1_SIZE	0x33800u
#define 	BOARD_CALIB_OFF		0x7F800u
#define FLASH_AREA_IMAGE_PRIMARY(i)   (((i) == 0) ? 1 : 0xFF)
#define FLASH_AREA_IMAGE_SECONDARY(i) (((i) == 0) ? 2 : 0xFF)




_Static_assert(BOARD_SLOT0_OFF + BOARD_SLOT0_SIZE == BOARD_SLOT1_OFF,
               "Slot boundary mismatch");
_Static_assert(BOARD_FLASH_BASE + BOARD_CALIB_OFF == 0x0807F800u,
               "Encoder calibration address changed");
_Static_assert(BOARD_SLOT1_OFF <= BOARD_CALIB_OFF &&
               BOARD_SLOT1_SIZE <= BOARD_CALIB_OFF - BOARD_SLOT1_OFF,
               "Secondary slot overlaps encoder calibration");
#endif /* MCUBOOT_CONFIG_SYSFLASH_SYSFLASH_H_ */
