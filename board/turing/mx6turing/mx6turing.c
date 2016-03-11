/*
 * Copyright (C) 2015 Turing Computer, Ltda.
 *
 * Author: Mauricio Cirelli <mauricio@turingcomputer.com.br>
 *
 * Copyright (C) 2013 Jon Nettleton <jon.nettleton@gmail.com>
 *
 * Based on SPL code from Solidrun tree, which is:
 * Author: Tungyi Lin <tungyilin1127@gmail.com>
 *
 * Derived from EDM_CF_IMX6 code by TechNexion,Inc
 * Ported to SolidRun microSOM by Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <malloc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <spl.h>
#include <usb.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PMIC	1

#define USDHC1_CD_GPIO				IMX_GPIO_NR(2, 0)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart5_pads[] = {
	IOMUX_PADS(PAD_KEY_COL1__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW1__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart3_pads[] = {
	IOMUX_PADS(PAD_EIM_D24__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D25__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D23__UART3_CTS_B   | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__UART3_RTS_B   | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	IOMUX_PADS(PAD_SD1_CLK__SD1_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_CMD__SD1_CMD	   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00	| MUX_PAD_CTRL(UART_PAD_CTRL)), 	/* CD */
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK	   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD	   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static struct i2c_pads_info i2c_pad_info = {
	.scl = {
		.i2c_mode = IOMUX_PADS(PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL)),
		.gpio_mode = IOMUX_PADS(PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL)),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = IOMUX_PADS(PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL)),
		.gpio_mode = IOMUX_PADS(PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL)),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart5_pads);
	SETUP_IOMUX_PADS(uart3_pads);
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		if (!ret) {
			puts("USDHC1 Card not present!\n");
		}
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret = 0;
	int index = 0;

	for (index = 0; index < CONFIG_SYS_FSL_ESDHC_NUM; index++) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			usdhc_cfg[index].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			usdhc_cfg[index].esdhc_base = USDHC1_BASE_ADDR;
			gpio_direction_input(USDHC1_CD_GPIO);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[index].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[index].esdhc_base = USDHC4_BASE_ADDR;
			break;
		default:
			printf("Warning: you configured more USDHC controllers (%d) as supported by the board (2)\n", CONFIG_SYS_FSL_ESDHC_NUM);
			return -EINVAL;
		}
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}

	return 0;
}

#ifdef CONFIG_VIDEO_IPUV3

struct display_info_t const displays[] = {{

}};

size_t display_count = ARRAY_SIZE(displays);

static int setup_display(void)
{
	// TODO

	return 0;
}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_USB_EHCI_MX6

int board_ehci_hcd_init(int port)
{
	return 0;
}

#endif

int board_early_init_f(void)
{
	int ret = 0;
	setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return ret;
}

int power_init_board(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg;

	ret = power_pfuze100_init(I2C_PMIC);
	if (ret)
		return ret;

	p = pmic_get("PFUZE100");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE100_DEVICEID, &reg);
	printf("PMIC:  PFUZE100 ID=0x%02x\n", reg);

	/* Set VGEN3 to 2.5V */
	pmic_reg_read(p, PFUZE100_VGEN3VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_50V;
	pmic_reg_write(p, PFUZE100_VGEN3VOL, reg);

	/* Set VGEN4 to 2.8V */
	pmic_reg_read(p, PFUZE100_VGEN4VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_80V;
	pmic_reg_write(p, PFUZE100_VGEN4VOL, reg);

	/* Set VGEN5 to 2.8V */
	pmic_reg_read(p, PFUZE100_VGEN5VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_80V;
	pmic_reg_write(p, PFUZE100_VGEN5VOL, reg);

	/* Set SW2VOLT to 3.3V */
	pmic_reg_read(p, PFUZE100_SW2VOL, &reg);
	reg = 0x72; // 0x72 = 3.3V
	pmic_reg_write(p, PFUZE100_SW2VOL, reg);

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info);

	return 0;
}

int checkboard(void)
{
	return 0;
}

int board_late_init(void)
{
	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
		setenv("board_rev", "MX6Q");
	else
		setenv("board_rev", "MX6DL");

	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
static const struct mx6dq_iomux_ddr_regs mx6q_ddr_ioregs = {
	.dram_sdclk_0 =  0x00000030,		// OK
	.dram_sdclk_1 =  0x00000030,		// OK
	.dram_cas =  0x00000030,			// OK
	.dram_ras =  0x00000030,			// OK
	.dram_reset =  0x00000030,			// OK
	.dram_sdcke0 =  0x00003000,			// N/A
	.dram_sdcke1 =  0x00003000,			// N/A
	.dram_sdba2 =  0x00000000,			// OK
	.dram_sdodt0 =  0x00000030,			// OK	
	.dram_sdodt1 =  0x00000030,			// OK
	.dram_sdqs0 =  0x00000030,			// OK
	.dram_sdqs1 =  0x00000030,			// OK
	.dram_sdqs2 =  0x00000030,			// OK
	.dram_sdqs3 =  0x00000030,			// OK
	.dram_sdqs4 =  0x00000030,			// OK
	.dram_sdqs5 =  0x00000030,			// OK
	.dram_sdqs6 =  0x00000030,			// OK
	.dram_sdqs7 =  0x00000030,			// OK
	.dram_dqm0 =  0x00000030,			// OK
	.dram_dqm1 =  0x00000030,			// OK
	.dram_dqm2 =  0x00000030,			// OK
	.dram_dqm3 =  0x00000030,			// OK
	.dram_dqm4 =  0x00000030,			// OK	
	.dram_dqm5 =  0x00000030,			// OK
	.dram_dqm6 =  0x00000030,			// OK
	.dram_dqm7 =  0x00000030,			// OK
};

static const struct mx6sdl_iomux_ddr_regs mx6dl_ddr_ioregs = {
	.dram_sdclk_0 = 0x00000028,			// FIXME
	.dram_sdclk_1 = 0x00000028,			// FIXME
	.dram_cas =	0x00000028,				// FIXME
	.dram_ras =	0x00000028,				// FIXME
	.dram_reset =	0x000c0028,			// FIXME
	.dram_sdcke0 =	0x00003000,			// FIXME
	.dram_sdcke1 =	0x00003000,			// FIXME
	.dram_sdba2 =	0x00000000,			// FIXME
	.dram_sdodt0 =	0x00003030,			// FIXME
	.dram_sdodt1 =	0x00003030,			// FIXME
	.dram_sdqs0 =	0x00000028,			// FIXME
	.dram_sdqs1 =	0x00000028,			// FIXME
	.dram_sdqs2 =	0x00000028,			// FIXME
	.dram_sdqs3 =	0x00000028,			// FIXME
	.dram_sdqs4 =	0x00000028,			// FIXME
	.dram_sdqs5 =	0x00000028,			// FIXME
	.dram_sdqs6 =	0x00000028,			// FIXME
	.dram_sdqs7 =	0x00000028,			// FIXME
	.dram_dqm0 =	0x00000028,			// FIXME
	.dram_dqm1 =	0x00000028,			// FIXME
	.dram_dqm2 =	0x00000028,			// FIXME
	.dram_dqm3 =	0x00000028,			// FIXME
	.dram_dqm4 =	0x00000028,			// FIXME
	.dram_dqm5 =	0x00000028,			// FIXME
	.dram_dqm6 =	0x00000028,			// FIXME
	.dram_dqm7 =	0x00000028,			// FIXME
};

static const struct mx6dq_iomux_grp_regs mx6q_grp_ioregs = {
	.grp_ddr_type =  0x000C0000,		// OK
	.grp_ddrmode_ctl =  0x00020000,		// OK
	.grp_ddrpke = 0x00000000,			// OK
	.grp_addds =  0x00000030,			// OK
	.grp_ctlds =  0x00000030,			// OK
	.grp_ddrmode = 0x00020000,			// OK
	.grp_b0ds =  0x00000030,			// OK
	.grp_b1ds =  0x00000030,			// OK
	.grp_b2ds =  0x00000030,			// OK
	.grp_b3ds =  0x00000030,			// OK
	.grp_b4ds =  0x00000030,			// OK
	.grp_b5ds =  0x00000030,			// OK
	.grp_b6ds =  0x00000030,			// OK
	.grp_b7ds =  0x00000030,			// OK
};

static const struct mx6sdl_iomux_grp_regs mx6sdl_grp_ioregs = {
	.grp_ddr_type = 0x000c0000,			// FIXME
	.grp_ddrmode_ctl = 0x00020000,		// FIXME
	.grp_ddrpke = 0x00000000,			// FIXME
	.grp_addds = 0x00000028,			// FIXME
	.grp_ctlds = 0x00000028,			// FIXME
	.grp_ddrmode = 0x00020000,			// FIXME
	.grp_b0ds = 0x00000028,				// FIXME
	.grp_b1ds = 0x00000028,				// FIXME
	.grp_b2ds = 0x00000028,				// FIXME
	.grp_b3ds = 0x00000028,				// FIXME
	.grp_b4ds = 0x00000028,				// FIXME
	.grp_b5ds = 0x00000028,				// FIXME
	.grp_b6ds = 0x00000028,				// FIXME
	.grp_b7ds = 0x00000028,				// FIXME
};

/* SOM with Quad processor and 2GB memory */
static const struct mx6_mmdc_calibration mx6q_2g_mmcd_calib = {
	.p0_mpwldectrl0 =  0x001F001F,			// OK
	.p0_mpwldectrl1 =  0x001F001F,			// OK
	.p1_mpwldectrl0 =  0x001F001F,			// OK
	.p1_mpwldectrl1 =  0x001F001F,			// OK
	.p0_mpdgctrl0 =    0x4333033F,			// OK
	.p0_mpdgctrl1 =    0x032C031D,			// OK
	.p1_mpdgctrl0 =    0x43200332,			// OK
	.p1_mpdgctrl1 =    0x031A026A,			// OK
	.p0_mprddlctl =    0x4D464746,			// OK
	.p1_mprddlctl =    0x47453F4D,			// OK
	.p0_mpwrdlctl =    0x3E434440,			// OK
	.p1_mpwrdlctl =    0x47384839,			// OK
};

/* SOM with Dual processor and 1GB memory */
static const struct mx6_mmdc_calibration mx6q_1g_mmcd_calib = {
	.p0_mpwldectrl0 =  0x001F001F,			// OK
	.p0_mpwldectrl1 =  0x001F001F,			// OK
	.p1_mpwldectrl0 =  0x001F001F,			// OK
	.p1_mpwldectrl1 =  0x001F001F,			// OK
	.p0_mpdgctrl0 =    0x4333033F,			// OK
	.p0_mpdgctrl1 =    0x032C031D,			// OK
	.p1_mpdgctrl0 =    0x43200332,			// OK
	.p1_mpdgctrl1 =    0x031A026A,			// OK
	.p0_mprddlctl =    0x4D464746,			// OK
	.p1_mprddlctl =    0x47453F4D,			// OK
	.p0_mpwrdlctl =    0x3E434440,			// OK
	.p1_mpwrdlctl =    0x47384839,			// OK
};

/* SOM with Solo processor and 512MB memory */
static const struct mx6_mmdc_calibration mx6dl_512m_mmcd_calib = {
	.p0_mpwldectrl0 = 0x0045004D,			// FIXME
	.p0_mpwldectrl1 = 0x003A0047,			// FIXME
	.p0_mpdgctrl0 =   0x023C0224,			// FIXME
	.p0_mpdgctrl1 =   0x02000220,			// FIXME
	.p0_mprddlctl =   0x44444846,			// FIXME
	.p0_mpwrdlctl =   0x32343032,			// FIXME
};

/* SOM with Dual lite processor and 1GB memory */
static const struct mx6_mmdc_calibration mx6dl_1g_mmcd_calib = {
	.p0_mpwldectrl0 =  0x0045004D,			// FIXME
	.p0_mpwldectrl1 =  0x003A0047,			// FIXME
	.p1_mpwldectrl0 =  0x001F001F,			// FIXME
	.p1_mpwldectrl1 =  0x00210035,			// FIXME
	.p0_mpdgctrl0 =    0x023C0224,			// FIXME
	.p0_mpdgctrl1 =    0x02000220,			// FIXME
	.p1_mpdgctrl0 =    0x02200220,			// FIXME
	.p1_mpdgctrl1 =    0x02000220,			// FIXME
	.p0_mprddlctl =    0x44444846,			// FIXME
	.p1_mprddlctl =    0x4042463C,			// FIXME
	.p0_mpwrdlctl =    0x32343032,			// FIXME
	.p1_mpwrdlctl =    0x36363430,			// FIXME
};

/* 4x MT41K256M16HA-125:E */
static struct mx6_ddr3_cfg mem_ddr_4g = {
	.mem_speed = 1600,				// OK
	.density   = 4,					// OK
	.width     = 16,				// OK
	.banks     = 8,					// OK
	.rowaddr   = 15,				// OK
	.coladdr   = 10,				// OK
	.pagesz    = 2,					// OK
	.trcd      = 1375,				// OK
	.trcmin    = 4875,				// OK
	.trasmin   = 3500,				// OK
	.SRT       = 0,					// OK
};

/* 2x MT41K256M16HA-125:E */
static struct mx6_ddr3_cfg mem_ddr_2g = {
	.mem_speed = 1600,				// OK
	.density   = 2,					// OK
	.width     = 16,				// OK
	.banks     = 8,					// OK
	.rowaddr   = 14,				// OK
	.coladdr   = 10,				// OK
	.pagesz    = 2,					// OK
	.trcd      = 1375,				// OK
	.trcmin    = 4875,				// OK
	.trasmin   = 3500,				// OK
	.SRT       = 0,					// OK
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	writel(0x007F007F, &iomux->gpr[6]);
	writel(0x007F007F, &iomux->gpr[7]);
}

static void spl_dram_init(int width)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus: 0=16, 1=32, 2=64 */
		.dsize = width / 32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32,	/* 32Gb per CS */
		.ncs = 1,			/* single chip select */
		.cs1_mirror = 0,
		.rtt_wr = 1 		/*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
		.rtt_nom = 1 		/*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
		.walat = 1,			/* Write additional latency */
		.ralat = 5,			/* Read additional latency */
		.mif3_mode = 3,		/* Command prediction working mode */
		.bi_on = 1,			/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	};

	if (is_cpu_type(MXC_CPU_MX6D) || is_cpu_type(MXC_CPU_MX6Q))
		mx6dq_dram_iocfg(width, &mx6q_ddr_ioregs, &mx6q_grp_ioregs);
	else
		mx6sdl_dram_iocfg(width, &mx6dl_ddr_ioregs, &mx6sdl_grp_ioregs);

	if (is_cpu_type(MXC_CPU_MX6D))
		mx6_dram_cfg(&sysinfo, &mx6q_1g_mmcd_calib, &mem_ddr_2g);
	else if (is_cpu_type(MXC_CPU_MX6Q))
		mx6_dram_cfg(&sysinfo, &mx6q_2g_mmcd_calib, &mem_ddr_4g);
	else if (is_cpu_type(MXC_CPU_MX6DL))
		mx6_dram_cfg(&sysinfo, &mx6q_1g_mmcd_calib, &mem_ddr_2g);
	else if (is_cpu_type(MXC_CPU_MX6SOLO))
		mx6_dram_cfg(&sysinfo, &mx6dl_512m_mmcd_calib, &mem_ddr_2g);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	if (is_cpu_type(MXC_CPU_MX6SOLO))
		spl_dram_init(32);
	else
		spl_dram_init(64);

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
