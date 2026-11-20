/*
 * mcuboot_config.h
 *
 *  Created on: 19 нояб. 2026 г.
 *      Author: i.yakovlev
 */

#ifndef MCUBOOT_CONFIG_MCUBOOT_CONFIG_H_
#define MCUBOOT_CONFIG_MCUBOOT_CONFIG_H_
#pragma once
#define MCUBOOT_IMAGE_NUMBER 1
#define MCUBOOT_SWAP_USING_MOVE 1
/* TinyCrypt provides SHA-256 for image integrity. */
#define MCUBOOT_USE_TINYCRYPT 1
#define MCUBOOT_VALIDATE_PRIMARY_SLOT 1
#define MCUBOOT_MAX_IMG_SECTORS 104
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS 1

void boot_watchdog_feed(void);
#define MCUBOOT_WATCHDOG_FEED() boot_watchdog_feed()
#endif /* MCUBOOT_CONFIG_MCUBOOT_CONFIG_H_ */
