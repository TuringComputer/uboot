/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Turing's mx6 based boards
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6TURING_CONFIG_H
#define __MX6TURING_CONFIG_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include "mx6_common.h"

#define CONFIG_IMX_THERMAL

#ifdef CONFIG_SPL
#define CONFIG_SPL_FS_LOAD_ARGS_NAME    "args"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME  "uImage"
#define CONFIG_SYS_SPL_ARGS_ADDR       0x18000000
#define CONFIG_CMD_SPL_WRITE_SIZE      (128 * SZ_1K)
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR  0x800   /* 1MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS (CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR        0x1000  /* 2MB */
#include "imx6_spl.h"
#endif

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SYS_GENERIC_BOARD

#define CONFIG_SYS_MALLOC_LEN			(10 * SZ_1M)
#define CONFIG_MXC_GPIO
#define CONFIG_MXC_UART
#define CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP

#define CONFIG_CMD_PCI
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif

#ifdef CONFIG_CMD_SF
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  	100000

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_USDHC_NUM		2
#define MMCROOT1					"/dev/mmcblk0p2"
#define MMCROOT2					"/dev/mmcblk3p2"
#define CONFIG_MMCROOT				MMCROOT1

#define CONFIG_BOUNCE_BUFFER
#define CONFIG_RBTREE
#define CONFIG_LZO

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* USB */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC					(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS					0
#define CONFIG_USB_MAX_CONTROLLER_COUNT			2
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_USB_HOST_ETHER
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_SMSC95XX

#define CONFIG_LOADADDR				0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

#define CONFIG_MXC_UART_BASE		UART5_BASE
#define CONSOLE_DEV			"ttymxc4"

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=zImage\0" 																								\
	"splashpos=m,m\0"																								\
	"panel=atm0700l6bt\0"																							\
	"fdtfile=undefined\0" 																							\
	"fdt_addr_r=0x18000000\0" 																						\
	"boot_fdt=try\0" 																								\
	"ip_dyn=yes\0" 																									\
	"console=" CONSOLE_DEV "\0" 																				    \
	"bootm_size=0x10000000\0" 																						\
	"mmcdev=" __stringify(SYS_MMC_ENV_DEV1) "\0"																    \
	"mmcpart=1\0" 																									\
	"mmcroot=" MMCROOT1 " rootwait rw\0" 																	        \
	"mmcargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} soc=$board_rev root=${mmcroot} fbcon=map:<02>\0" 	\
	"loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" 										\
	"bootscript=echo Running bootscript from mmc ...; source\0" 													\
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" 											\
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr_r} ${fdtfile}\0" 											\
	"mmcboot=echo Booting from mmc ...; " 																			\
		"run mmcargs; " 																							\
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " 												\
			"if run loadfdt; then " 																				\
				"bootz ${loadaddr} - ${fdt_addr_r}; " 																\
			"else " 																								\
				"if test ${boot_fdt} = try; then " 																	\
					"bootz; " 																						\
				"else " 																							\
					"echo WARN: Cannot load the DT; " 																\
				"fi; " 																								\
			"fi; " 																									\
		"else " 																									\
			"bootz; " 																								\
		"fi;\0" 																									\
	"netargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} soc=$board_rev root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp fbcon=map:<02>\0" \
	"netboot=echo Booting from net ...; " 																			\
		"usb start;"																								\
		"run netargs; " 																							\
		"if test ${ip_dyn} = yes; then " 																			\
			"setenv get_cmd dhcp; " 																				\
		"else " 																									\
			"setenv get_cmd tftp; " 																				\
		"fi; " 																										\
		"${get_cmd} ${image}; " 																					\
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " 												\
			"if ${get_cmd} ${fdt_addr_r} ${fdtfile}; then " 														\
				"bootz ${loadaddr} - ${fdt_addr_r}; " 																\
			"else " 																								\
				"if test ${boot_fdt} = try; then " 																	\
					"bootz; " 																						\
				"else " 																							\
					"echo WARN: Cannot load the DT; " 																\
				"fi; " 																								\
			"fi; " 																									\
		"else " 																									\
			"bootz; " 																								\
		"fi;\0" 																									\


#define CONFIG_BOOTCOMMAND 																							\
	"mmc dev ${mmcdev}; " 																							\
	"if mmc rescan; then " 																							\
		"if run loadimage; then " 																					\
			"run mmcboot; " 																						\
		"else "																										\
			"setenv mmcdev " __stringify(SYS_MMC_ENV_DEV2) "; "												        \
			"setenv mmcroot " __stringify(MMCROOT2) "; "														    \
			"mmc dev ${mmcdev}; " 																					\
			"if mmc rescan; then " 																					\
					"if run loadimage; then "																		\
						"run mmcboot; " 																			\
					"else "																							\
						"run netboot; " 																			\
					"fi; " 																							\
			"else "																									\
				"run netboot; " 																					\
			"fi; "																									\
		"fi; "																										\
	"else "																											\
		"setenv mmcdev " __stringify(SYS_MMC_ENV_DEV2) "; "													        \
		"setenv mmcroot " __stringify(MMCROOT2) "; "															    \
		"mmc dev ${mmcdev}; " 																						\
		"if mmc rescan; then " 																						\
			"if run loadimage; then "																				\
				"run mmcboot; " 																					\
			"else "																									\
				"run netboot; " 																					\
			"fi; " 																									\
		"else "																										\
			"run netboot; " 																						\
		"fi; "																										\
	"fi;"																											\

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_LOAD_ADDR           	CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           	1
#define CONFIG_SYS_SDRAM_BASE          	MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_ADDR       	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE					(8 * 1024)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_FSL_ESDHC_ADDR		0
#define SYS_MMC_ENV_DEV1			    0	/* 0: SDHC1; 1:SDHC4 */
#define SYS_MMC_ENV_DEV2			    1	/* 0: SDHC1; 1:SDHC4 */
#define CONFIG_SYS_MMC_ENV_DEV			SYS_MMC_ENV_DEV1
#define CONFIG_ENV_OFFSET				(8 * 64 * 1024)

/* Framebuffer */
#define CONFIG_CMD_BMP
#define CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_VIDEO_SKIP


#endif                         /* __MX6TURING_CONFIG_H */
