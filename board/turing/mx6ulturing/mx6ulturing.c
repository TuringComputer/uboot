/*
 * Copyright (C) 2017 Turing Computer
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <usb.h>
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		                    \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		                        \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		                    \
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		                        \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |                        \
    PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |                               \
    PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart4_pads[] = {
    MX6_PAD_UART4_TX_DATA__UART4_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_UART4_RX_DATA__UART4_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#ifdef CONFIG_FSL_ESDHC
static iomux_v3_cfg_t const usdhc1_pads[] = {
    MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};
#endif

#ifdef CONFIG_USB_EHCI_MX6
static iomux_v3_cfg_t const usb_otg_pads[] = {
    MX6_PAD_UART3_TX_DATA__ANATOP_OTG1_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};
#endif

#ifdef CONFIG_NAND_MXS
static iomux_v3_cfg_t const nand_pads[] = {
    MX6_PAD_NAND_DATA00__RAWNAND_DATA00 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA01__RAWNAND_DATA01 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA02__RAWNAND_DATA02 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA03__RAWNAND_DATA03 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA04__RAWNAND_DATA04 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA05__RAWNAND_DATA05 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA06__RAWNAND_DATA06 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DATA07__RAWNAND_DATA07 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_CLE__RAWNAND_CLE | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_ALE__RAWNAND_ALE | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_CE0_B__RAWNAND_CE0_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_CE1_B__RAWNAND_CE1_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_RE_B__RAWNAND_RE_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_WE_B__RAWNAND_WE_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_WP_B__RAWNAND_WP_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_READY_B__RAWNAND_READY_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
    MX6_PAD_NAND_DQS__RAWNAND_DQS | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
};
#endif

#define USDHC1_CD_GPIO          IMX_GPIO_NR(1, 9)
#define USB_HUB_RSTn            IMX_GPIO_NR(1, 11)
#define USB_OTG_PWR_EN          IMX_GPIO_NR(4, 26)

static void setup_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_NAND_MXS
static void setup_gpmi_nand(void)
{
    struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

    /* config gpmi nand iomux */
    imx_iomux_v3_setup_multiple_pads(nand_pads, ARRAY_SIZE(nand_pads));

    setup_gpmi_io_clk((3 << MXC_CCM_CSCDR1_BCH_PODF_OFFSET) | (3 << MXC_CCM_CSCDR1_GPMI_PODF_OFFSET));

    /* enable apbh clock gating */
    setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET    0x800
#define UCTRL_PWR_POL           (1 << 9)

/* At default the 3v3 enables the MIC2026 for VBUS power */
static void setup_usb(void)
{
    imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
    if (port == 1)
        return USB_INIT_HOST;
    else
        return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
    u32 *usbnc_usb_ctrl;

    if (port > 1)
        return -EINVAL;

    usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET + port * 4);

    /* Set Power polarity */
    setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

    return 0;
}

int board_ehci_power(int port, int on)
{
    switch (port)
    {
        case 0:
            if (on)
                gpio_direction_output(USB_OTG_PWR_EN, 1);
            else
                gpio_direction_output(USB_OTG_PWR_EN, 0);
            break;
        case 1:
            if (on)
                gpio_direction_output(USB_HUB_RSTn, 1);
            else
                gpio_direction_output(USB_HUB_RSTn, 0);
            break;
        default:
            printf("MXC USB port %d not yet supported\n", port);
            return -EINVAL;
    }

    return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR, 0, 4},
};

int board_mmc_getcd(struct mmc *mmc)
{
	return !gpio_get_value(USDHC1_CD_GPIO);
}

int board_mmc_init(bd_t *bis)
{
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	gpio_direction_input(USDHC1_CD_GPIO);
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
    gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}
#endif

int board_early_init_f(void)
{
	setup_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_USB_EHCI_MX6
    setup_usb();
#endif

#ifdef CONFIG_NAND_MXS
    setup_gpmi_nand();
#endif

	return 0;
}

int board_late_init(void)
{
    // Read boot device from registers
    struct src *psrc = (struct src *)SRC_BASE_ADDR;
    unsigned int gpr10_boot = readl(&psrc->gpr10) & (1 << 28);
    unsigned reg = gpr10_boot ? readl(&psrc->gpr9) : readl(&psrc->sbmr1);
    unsigned int bmode = readl(&psrc->sbmr2);

    /*
     * Check for BMODE if serial downloader is enabled
     * BOOT_MODE - see IMX6DQRM Table 8-1
     */
    printf("Booting from ");
    if (((bmode >> 24) & 0x03) == 0x01) /* Serial Downloader */
    {
        printf("Serial Downloader (not supported!)");
        return -1;
    }

    /* BOOT_CFG1[7:4] - see IMX6DQRM Table 8-8 */
    switch ((reg & 0x000000FF) >> 4)
    {
        /* SD/eSD: 8.5.3, Table 8-15  */
        case 0x4:
        case 0x5:
        /* MMC/eMMC: 8.5.3 */
        case 0x6:
        case 0x7:
            printf("uSD/MMC\n");
            setenv("boot_dev", "mmc");
            break;
        /* NAND Flash: 8.5.2 */
        case 0x8 ... 0xf:
            printf("NAND\n");
            setenv("boot_dev", "nand");
            break;
        default:
            printf("Unknown boot source.\n");
            setenv("boot_dev", "undefined");
            break;
    }

    if (is_cpu_type(MXC_CPU_MX6UL))
    {
        setenv("board_rev", "MX6UL");
        setenv("fdtfile", "imx6ul-turing-eval.dtb");
        setenv("fdtnand", "dtb-ul-0");
    }
    else
    {
        setenv("board_rev", "MX6ULL");
        setenv("fdtfile", "imx6ull-turing-eval.dtb");
        setenv("fdtnand", "dtb-ull-0");
    }

	return 0;
}

int checkboard(void)
{
    puts("Board: MX6-Turing\n");
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
    return 1;
}

#ifdef CONFIG_SPL_BUILD
#include <libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>


static struct mx6ul_iomux_grp_regs mx6ul_grp_ioregs = {
    .grp_addds          = 0x00000030,
    .grp_ddrmode_ctl    = 0x00020000,
    .grp_b0ds           = 0x00000030,
    .grp_ctlds          = 0x00000030,
    .grp_b1ds           = 0x00000030,
    .grp_ddrpke         = 0x00000000,
    .grp_ddrmode        = 0x00020000,
	.grp_ddr_type       = 0x000c0000,
};

static struct mx6ul_iomux_ddr_regs mx6ul_ddr_ioregs = {
    .dram_dqm0          = 0x00000030,
    .dram_dqm1          = 0x00000030,
    .dram_ras           = 0x00000030,
    .dram_cas           = 0x00000030,
    .dram_odt0          = 0x00000030,
    .dram_odt1          = 0x00000030,
    .dram_sdba2         = 0x00000000,
    .dram_sdclk_0       = 0x00000030,
    .dram_sdqs0         = 0x00000030,
    .dram_sdqs1         = 0x00000030,
    .dram_reset         = 0x00000030,
};

static struct mx6_mmdc_calibration mx6ul_mmcd_calib = {
    .p0_mpwldectrl0     = 0x00000000,
    .p0_mpwldectrl1     = 0x00000000,
    .p0_mpdgctrl0       = 0x4164015C,
    .p0_mpdgctrl1       = 0x00000000,
    .p0_mprddlctl       = 0x40404042,
    .p0_mpwrdlctl       = 0x40405652,
};

static struct mx6_mmdc_calibration mx6ull_mmcd_calib = {
    .p0_mpwldectrl0     = 0x00010007,
    .p0_mpwldectrl1     = 0x00080008,
    .p0_mpdgctrl0       = 0x414C0148,
    .p0_mpdgctrl1       = 0x00000000,
    .p0_mprddlctl       = 0x40402E30,
    .p0_mpwrdlctl       = 0x4040342E,
};

struct mx6_ddr_sysinfo ddr_sysinfo = {
    .dsize = 0,
    .cs_density = 20,
    .ncs = 1,
    .cs1_mirror = 0,
    .rtt_wr = 2,
    .rtt_nom = 1,       /* RTT_Nom = RZQ/2 */
    .walat = 0,         /* Write additional latency */
    .ralat = 5,         /* Read additional latency */
    .mif3_mode = 3,     /* Command prediction working mode */
    .bi_on = 1,         /* Bank interleaving enabled */
    .sde_to_rst = 0x10, /* 14 cycles, 200us (JEDEC default) */
    .rst_to_cke = 0x23, /* 33 cycles, 500us (JEDEC default) */
    .ddr_type = DDR_TYPE_DDR3,
    .refr = 1,          /* 2 refresh commands per refresh cycle */
};

/* MT41K256M16HA-107:P */
static struct mx6_ddr3_cfg mem_ddr = {
    .mem_speed = 800,
    .density   = 4,
    .width     = 16,
    .banks     = 8,
    .rowaddr   = 15,
    .coladdr   = 10,
    .pagesz    = 2,
    .trcd      = 1300,
    .trcmin    = 4500,
    .trasmin   = 3200,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
    writel(0xFFFFFFFF, &ccm->CCGR1);
    writel(0xFFFFFFFF, &ccm->CCGR2);
    writel(0xFFFFFFFF, &ccm->CCGR3);
    writel(0xFFFFFFFF, &ccm->CCGR4);
    writel(0xFFFFFFFF, &ccm->CCGR5);
    writel(0xFFFFFFFF, &ccm->CCGR6);
    writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
    if (is_cpu_type(MXC_CPU_MX6UL))
    {
        mx6ul_dram_iocfg(mem_ddr.width, &mx6ul_ddr_ioregs, &mx6ul_grp_ioregs);
        mx6_dram_cfg(&ddr_sysinfo, &mx6ul_mmcd_calib, &mem_ddr);
    }
    else
    {
        mx6ul_dram_iocfg(mem_ddr.width, &mx6ul_ddr_ioregs, &mx6ul_grp_ioregs);
        mx6_dram_cfg(&ddr_sysinfo, &mx6ull_mmcd_calib, &mem_ddr);
    }
}

void board_init_f(ulong dummy)
{
	ccgr_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

    if (is_cpu_type(MXC_CPU_MX6UL))
    {
        printf("Configuring DDR3 for i.MX6 Ultra Lite\n");
    }
    else
    {
        printf("Configuring DDR3 for i.MX6 Ultra Lite L\n");
    }

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
