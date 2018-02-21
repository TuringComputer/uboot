/*
 * Copyright (C) 2017 Turing Computer
 *
 * Configuration settings for the Turing i.MX6UL boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6ULTURING_H
#define __MX6ULTURING_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

/* SPL options */
#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		    (16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		    UART4_BASE
#define CONSOLE_DEV                     "ttymxc3"

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_USDHC_NUM	    1
#define MMCROOT1                        "/dev/mmcblk0p2"
#define CONFIG_MMCROOT                  MMCROOT1
#define CONFIG_SYS_FSL_ESDHC_ADDR       0
#define SYS_MMC_ENV_DEV1                0   /* 0: SDHC1 */
#define CONFIG_SYS_MMC_ENV_DEV          SYS_MMC_ENV_DEV1

/* NAND Configs */
# define CONFIG_SYS_MAX_NAND_DEVICE     1
# define CONFIG_SYS_NAND_BASE           0x40000000
# define CONFIG_SYS_NAND_5_ADDR_CYCLE
# define CONFIG_SYS_NAND_ONFI_DETECTION
# define CONFIG_SYS_NAND_U_BOOT_START   CONFIG_SYS_TEXT_BASE
# define CONFIG_SYS_NAND_U_BOOT_OFFS    0xE00000

/* MTD device */
# define CONFIG_MTD_DEVICE
# define CONFIG_CMD_MTDPARTS
# define CONFIG_MTD_PARTITIONS
# define MTDIDS_DEFAULT                 "nand0=gpmi-nand"
# define MTDPARTS_DEFAULT               "mtdparts=gpmi-nand:14m(spl),2m(uboot)," \
                                        "2m(env),8m(kernel)," \
                                        "2m(dtb-ul-0),2m(dtb-ull-0)," \
                                        "2m(dtb-ul-1),2m(dtb-ull-1)," \
                                        "-(rootfs)"

/* UBI */
# define CONFIG_CMD_UBIFS
# define CONFIG_RBTREE
# define CONFIG_LZO

# define CONFIG_APBH_DMA
# define CONFIG_APBH_DMA_BURST
# define CONFIG_APBH_DMA_BURST8

/* I2C configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_MXC_I2C4     /* enable I2C bus 4 */
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS                                                                                   \
    "image=zImage\0"                                                                                                \
    "fdt_high=0xffffffff\0"                                                                                         \
    "initrd_high=0xffffffff\0"                                                                                      \
    "fdtfile=undefined\0"                                                                                           \
    "fdt_addr=0x83000000\0"                                                                                         \
    "boot_fdt=try\0"                                                                                                \
    "ip_dyn=yes\0"                                                                                                  \
    "console=" CONSOLE_DEV "\0"                                                                                     \
    "bootm_size=0x10000000\0"                                                                                       \
    "mmcdev=" __stringify(SYS_MMC_ENV_DEV1) "\0"                                                                    \
    "mmcpart=1\0"                                                                                                   \
    "mmcroot=" MMCROOT1 " rootwait rw\0"                                                                            \
    "nandroot=ubi0 rootfstype=ubifs rootwait rw\0"                                                                  \
    "mmcautodetect=yes\0"                                                                                           \
    "mmcargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} soc=$board_rev ${mtdparts} "         \
        "root=${mmcroot}\0"                                                                                         \
    "loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0"                                      \
    "bootscript=echo Running bootscript from mmc ...; source\0"                                                     \
    "loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0"                                             \
    "loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0"                                             \
    "mmcboot=echo Booting from mmc...; "                                                                            \
        "run mmcargs; "                                                                                             \
        "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then "                                                \
            "if run loadfdt; then "                                                                                 \
                "bootz ${loadaddr} - ${fdt_addr}; "                                                                 \
            "else "                                                                                                 \
                "if test ${boot_fdt} = try; then "                                                                  \
                    "bootz; "                                                                                       \
                "else "                                                                                             \
                    "echo WARN: Cannot load the DT; "                                                               \
                "fi; "                                                                                              \
            "fi; "                                                                                                  \
        "else "                                                                                                     \
            "bootz; "                                                                                               \
        "fi;\0"                                                                                                     \
    "netargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} soc=$board_rev ${mtdparts} "         \
        "root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0"                                             \
    "netboot=echo Booting from net...; "                                                                            \
        "usb start;"                                                                                                \
        "run netargs; "                                                                                             \
        "if test ${ip_dyn} = yes; then "                                                                            \
            "setenv get_cmd dhcp; "                                                                                 \
        "else "                                                                                                     \
            "setenv get_cmd tftp; "                                                                                 \
        "fi; "                                                                                                      \
        "${get_cmd} ${image}; "                                                                                     \
        "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then "                                                \
            "if ${get_cmd} ${fdt_addr} ${fdtfile}; then "                                                           \
                "bootz ${loadaddr} - ${fdt_addr}; "                                                                 \
            "else "                                                                                                 \
                "if test ${boot_fdt} = try; then "                                                                  \
                    "bootz; "                                                                                       \
                "else "                                                                                             \
                    "echo WARN: Cannot load the DT; "                                                               \
                "fi; "                                                                                              \
            "fi; "                                                                                                  \
        "else "                                                                                                     \
            "bootz; "                                                                                               \
        "fi;\0"                                                                                                     \
    "nandargs=setenv bootargs console=${console},${baudrate} no_console_suspend ${bootargs_mem} soc=$board_rev ${mtdparts} "        \
        "ubi.mtd=8,8192 root=${nandroot}\0"                                                                         \
    "nandboot=echo Booting from nand...; "                                                                          \
        "run nandargs; "                                                                                            \
        "nand read ${loadaddr} kernel 0x800000; "                                                                   \
        "nand read ${fdt_addr} ${fdtnand} 0x100000; "                                                               \
        "bootz ${loadaddr} - ${fdt_addr}\0"                                                                         \

#define CONFIG_BOOTCOMMAND                                                                                          \
       "if mtdparts; then "                                                                                         \
            "echo Found mtdparts...; "                                                                              \
        "else "                                                                                                     \
            "echo Using default mtdparts...; "                                                                      \
            "mtdparts default; "                                                                                    \
       "fi; "                                                                                                       \
       "if test ${boot_dev} = nand; then "                                                                          \
           "run nandboot; "                                                                                         \
       "fi; "                                                                                                       \
       "mmc dev ${mmcdev}; "                                                                                        \
       "if mmc rescan; then "                                                                                       \
            "if run loadimage; then "                                                                               \
               "run mmcboot; "                                                                                      \
            "else"                                                                                                  \
                "Failed to boot from uSD... trying Ethernet...; "                                                   \
                "run netboot; "                                                                                     \
            "fi; "                                                                                                  \
       "fi; "                                                                                                       \

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			    1000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE		    SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			        MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET   (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR     (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE               SZ_128K

#ifndef CONFIG_ENV_IS_NOWHERE
/* Environment in MMC */
# if defined(CONFIG_ENV_IS_IN_MMC)
  #define CONFIG_ENV_OFFSET           (8 * SZ_64K)
/* Environment in NAND */
# elif defined(CONFIG_ENV_IS_IN_NAND)
#  define CONFIG_ENV_OFFSET           0x1000000
#  define CONFIG_ENV_SECT_SIZE        0x200000
# endif
#endif

/* USB Configs */
/* USB */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC                   (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS                    0
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_USB_HOST_ETHER
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_SMSC95XX

#define CONFIG_IMX_THERMAL

#ifdef CONFIG_MX6ULL
#define CONFIG_IOMUX_LPSR
#endif

#endif                         /* __MX6ULTURING_CONFIG_H */
