/*
 * Copyright (C) 2015 Quanta Tecnologia Eletronica, Ltda.
 *
 * Author: Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>
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

//static iomux_v3_cfg_t const usb_pads[] = {
//	IOMUX_PADS(PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
//};

#if 0
static iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
};

static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02        | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03        | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN4__IPU1_DI0_PIN04        | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD1_DAT3__GPIO1_IO21            | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif

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
	// FIXME: Usar Card Detect para uSD Card
	return 1; /* USDHC4 is always present */
	
#if 0
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
#endif

}

int board_mmc_init(bd_t *bis)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x1:
		imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}

#ifdef CONFIG_VIDEO_IPUV3
static void enable_rgb(struct display_info_t const *dev)
{
	//imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
	// FIXME: Verificar qual o pino de enable do LCD
	//gpio_direction_output(DISP0_PWR_EN, 1);
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *) IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

struct display_info_t const displays[] = 
{
	{
		.bus	= -1,
		.addr	= 0,
		.pixfmt	= IPU_PIX_FMT_RGB666,
		.detect	= NULL,
		.enable	= enable_lvds,
		.mode	= {
			.name           = "Hannstar-XGA",
			.refresh        = 60,
			.xres           = 1024,
			.yres           = 768,
			.pixclock       = 15385,
			.left_margin    = 220,
			.right_margin   = 40,
			.upper_margin   = 21,
			.lower_margin   = 7,
			.hsync_len      = 60,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}, 
	{
		.bus	= 0,
		.addr	= 0,
		.pixfmt	= IPU_PIX_FMT_RGB24,
		.detect	= NULL,
		.enable	= enable_rgb,
		.mode	= {
			.name           = "SEIKO-WVGA",
			.refresh        = 60,
			.xres           = 800,
			.yres           = 480,
			.pixclock       = 29850,
			.left_margin    = 89,
			.right_margin   = 164,
			.upper_margin   = 23,
			.lower_margin   = 10,
			.hsync_len      = 10,
			.vsync_len      = 10,
			.sync           = 0,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}
};

size_t display_count = ARRAY_SIZE(displays);

static int setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	//imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

	enable_ipu_clock();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

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

#ifdef CONFIG_VIDEO_IPUV3
	//ret = setup_display();
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
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
static const struct mx6dq_iomux_ddr_regs mx6q_ddr_ioregs = {
	.dram_sdclk_0 =  0x00000030,			// OK
	.dram_sdclk_1 =  0x00000030,			// OK
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

static const struct mx6dq_iomux_grp_regs mx6q_grp_ioregs = {
	.grp_ddr_type =  0x000C0000,			// OK
	.grp_ddrmode_ctl =  0x00020000,			// OK
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

/* microSOM with Quad processor and 2GB memory */
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

/* MT41K256M16HA-125:E */
static struct mx6_ddr3_cfg mem_ddr_2g = {
	.mem_speed = 1600,				// OK
	.density   = 4,					// OK
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

/*
 * FIXME: This section requires the differentiation between Quanta's i.MX6 boards.
 */
static void spl_dram_init(int width)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus: 0=16, 1=32, 2=64 */
		.dsize = width / 32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32,	/* 32Gb per CS */
		.ncs = 1,		/* single chip select */
		.cs1_mirror = 0,
		.rtt_wr = 1 		/*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
		.rtt_nom = 1 		/*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
		.walat = 1,		/* Write additional latency */
		.ralat = 5,		/* Read additional latency */
		.mif3_mode = 3,		/* Command prediction working mode */
		.bi_on = 1,		/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	};

	mx6dq_dram_iocfg(width, &mx6q_ddr_ioregs, &mx6q_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6q_2g_mmcd_calib, &mem_ddr_2g);
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
	spl_dram_init(64);

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
