/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Turing's mx6 based boards
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6TURING_CONFIG_H
#define __MX6TURING_CONFIG_H

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE		5650
#define CONFIG_MXC_UART_BASE	UART5_BASE
#define CONFIG_BAUDRATE         115200
#define CONFIG_CONSOLE_DEV		"ttymxc4"
#define CONFIG_MMCROOT1			"/dev/mmcblk0p2"
#define CONFIG_MMCROOT2			"/dev/mmcblk3p2"
#define CONFIG_MMCROOT			CONFIG_MMCROOT1

#include "mx6turing_common.h"

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_MMC_ENV_DEV1		0	/* 0: SDHC1; 1:SDHC4 */
#define CONFIG_SYS_MMC_ENV_DEV2		1	/* 0: SDHC1; 1:SDHC4 */
#define CONFIG_SYS_MMC_ENV_DEV		CONFIG_SYS_MMC_ENV_DEV1
#define CONFIG_SYS_MMC_ENV_PART     0   /* user partition */

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif

#endif                         /* __MX6TURING_CONFIG_H */
