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

#define CONFIG_MX6
#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_IMX6_THERMAL
#define CONFIG_SYS_GENERIC_BOARD

#define CONFIG_SYS_MALLOC_LEN			(10 * SZ_1M)
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO
#define CONFIG_MXC_UART
#define CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
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
#define CONFIG_MMCROOT1					"/dev/mmcblk0p2"
#define CONFIG_MMCROOT2					"/dev/mmcblk3p2"
#define CONFIG_MMCROOT					CONFIG_MMCROOT1

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT3
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_RBTREE
#define CONFIG_LZO
#define CONFIG_CMD_FS_GENERIC

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* USB */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC					(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS					0
#define CONFIG_USB_MAX_CONTROLLER_COUNT			2
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL

#define CONFIG_SYS_NO_FLASH

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR

#define CONFIG_BOOTDELAY			3

#define CONFIG_LOADADDR				0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

#define CONFIG_MXC_UART_BASE		UART5_BASE
#define CONFIG_CONSOLE_DEV			"ttymxc4"

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=zImage\0" 																								\
	"fdtfile=undefined\0" 																							\
	"fdt_addr_r=0x18000000\0" 																						\
	"boot_fdt=try\0" 																								\
	"ip_dyn=yes\0" 																									\
	"console=" CONFIG_CONSOLE_DEV "\0" 																				\
	"bootm_size=0x10000000\0" 																						\
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV1) "\0"																\
	"mmcpart=1\0" 																									\
	"mmcroot=" CONFIG_MMCROOT1 " rootwait rw\0" 																	\
	"mmcargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} root=${mmcroot}\0" 	\
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
	"netargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " 																			\
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
	"findfdt="																										\
		"if test $board_rev = MX6Q ; then " 																		\
			"setenv fdtfile imx6q-turing-eval.dtb; fi; " 															\
		"if test $board_rev = MX6DL ; then " 																		\
			"setenv fdtfile imx6dl-turing-eval.dtb; fi; " 															\
		"if test $fdtfile = undefined; then " 																		\
			"echo WARNING: Could not determine dtb to use; fi; \0" 													\

#define CONFIG_BOOTCOMMAND 																							\
	"run findfdt; " 																								\
	"mmc dev ${mmcdev}; " 																							\
	"if mmc rescan; then " 																							\
		"if run loadimage; then " 																					\
			"run mmcboot; " 																						\
		"else "																										\
			"setenv mmcdev " __stringify(CONFIG_SYS_MMC_ENV_DEV2) "; "												\
			"setenv mmcroot " __stringify(CONFIG_MMCROOT2) "; "														\
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
		"setenv mmcdev " __stringify(CONFIG_SYS_MMC_ENV_DEV2) "; "													\
		"setenv mmcroot " __stringify(CONFIG_MMCROOT2) "; "															\
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
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              	256
#define CONFIG_SYS_MAXARGS             	16
#define CONFIG_SYS_BARGSIZE 			CONFIG_SYS_CBSIZE

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
#define CONFIG_SYS_MMC_ENV_DEV1			0	/* 0: SDHC1; 1:SDHC4 */
#define CONFIG_SYS_MMC_ENV_DEV2			1	/* 0: SDHC1; 1:SDHC4 */
#define CONFIG_SYS_MMC_ENV_DEV			CONFIG_SYS_MMC_ENV_DEV1
#define CONFIG_ENV_OFFSET				(8 * 64 * 1024)

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_CACHE

#endif                         /* __MX6TURING_CONFIG_H */
