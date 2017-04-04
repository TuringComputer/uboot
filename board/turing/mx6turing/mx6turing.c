/*
 * Copyright (C) 2015 Turing Computer.
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
#include <linux/errno.h>
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

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |						\
						PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm |	\
						PAD_CTL_SRE_FAST    | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |						\
						PAD_CTL_SPEED_LOW  | PAD_CTL_DSE_80ohm |	\
						PAD_CTL_SRE_FAST   | PAD_CTL_HYS)

#define I2C_PAD_CTRL   (PAD_CTL_PUS_100K_UP |									\
					    PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
					    PAD_CTL_ODE 	    | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |				\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PMIC	1

#define DISP0_PWR_EN			IMX_GPIO_NR(1, 22)
#define USB_HUB_RSTn			IMX_GPIO_NR(3, 20)
#define USB_OTG_PWR_EN			IMX_GPIO_NR(3, 22)

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

static iomux_v3_cfg_t const i2c2_pads[] = {
	IOMUX_PADS(PAD_KEY_COL3__I2C2_SCL  | MUX_PAD_CTRL(I2C_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW3__I2C2_SDA  | MUX_PAD_CTRL(I2C_PAD_CTRL)),
};

static iomux_v3_cfg_t const gpio_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__GPIO1_IO22  | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static struct i2c_pads_info i2cq_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static struct i2c_pads_info i2cdl_pad_info = {
	.scl = {
		.i2c_mode = MX6DL_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

#ifdef CONFIG_USB_EHCI_MX6

#define USB_OTHERREGS_OFFSET			0x800
#define UCTRL_PWR_POL					(1 << 9)

/**
 * USB OTG Pins (ID & PWR EN)
 */
static iomux_v3_cfg_t const usb_otg_pads[] = {
	IOMUX_PADS(PAD_EIM_D22__USB_OTG_PWR   | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL)),
};

/**
 * USB Host Power Enable (Hub RSTn)
 */
static iomux_v3_cfg_t const usb_hc1_pads[] = {
	IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_usb(void)
{
	SETUP_IOMUX_PADS(usb_otg_pads);

	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);

	SETUP_IOMUX_PADS(usb_hc1_pads);

	/**
	 * Make sure the Hub is Enabled
	 */
	gpio_direction_output(USB_HUB_RSTn, 1);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET + port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(USB_OTG_PWR_EN, 1);
		else
			gpio_direction_output(USB_OTG_PWR_EN, 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart5_pads);
	SETUP_IOMUX_PADS(uart3_pads);
}

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC1_CD_GPIO		IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/USDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    uSD 	(USDHC1)
	 * mmc1                    eMMC (USDHC4)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			SETUP_IOMUX_PADS(usdhc1_pads);
			gpio_direction_input(USDHC1_CD_GPIO);
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			usdhc_cfg[i].max_bus_width = 4;
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[i].max_bus_width = 8;
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				   "(%d) then supported by the board (%d)\n",
				   i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			return ret;
		}
	}

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x0                  SD1
	 * 0x1                  SD2
	 * 0x2					SD3
	 * 0x3                  SD4
	 */

	int port = reg & 0x3;
	switch (port) {
	case 0x0:
		SETUP_IOMUX_PADS(usdhc1_pads);
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		printf("Initializing USDHC1 as boot device\n");
		break;
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		printf("Initializing USDHC4 as boot device\n");
		break;
	default:
		printf("Unknown USDHC boot device: 0x%x\n", port);
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK | IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *) IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= (IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT);
	writel(reg, &iomux->gpr[2]);
	gpio_direction_output(DISP0_PWR_EN, 1);
}

struct display_info_t const displays[] = {
	{
		.bus	= 0,
		.addr	= 0,
		.pixfmt	= IPU_PIX_FMT_LVDS666,
		.detect	= NULL,
		.enable	= enable_lvds,
		.mode	= {
			.name           = "atm0700l6bt",
			.refresh        = 60,
			.xres           = 800,
			.yres           = 480,
			.pixclock       = 30000,
			.left_margin    = 46,
			.right_margin   = 210,
			.upper_margin   = 23,
			.lower_margin   = 22,
			.hsync_len      = 10,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}
};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *) CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *) IOMUXC_BASE_ADDR;

	int reg;

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_IPU1_IPU_DI0_OFFSET | MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)	| (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK
		   | MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK
		   | MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |=(CHSCCDR_CLK_SEL_LDB_DI0 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET)
		| (CHSCCDR_PODF_DIVIDE_BY_3 << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
		| (CHSCCDR_IPU_PRE_CLK_540M_PFD
					<< MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
			| IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
			| IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
			| IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
			| IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
			| IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
			| IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
			| IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
			| IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK) | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{
	int ret = 0;
	SETUP_IOMUX_PADS(gpio_pads);
	setup_iomux_uart();
    if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
	{
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2cq_pad_info);
	}
	else
	{
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2cdl_pad_info);
	}
#ifdef CONFIG_VIDEO_IPUV3
	setup_display();
#endif
#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
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

	/* Set SW4VOLT to 3.3V */
	pmic_reg_read(p, PFUZE100_SW4VOL, &reg);
	reg = 0x72; // 0x72 = 3.3V
	pmic_reg_write(p, PFUZE100_SW4VOL, reg);

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6-Turing\n");
	return 0;
}

int board_late_init(void)
{
	// Defines the processor family, to load the correct device tree binaries
	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
	{
		setenv("board_rev", "MX6Q");
	}
	else
	{
		setenv("board_rev", "MX6DL");
	}

	/* Kernel Memory Allocation */
#ifndef TURING_SMART_VARIANT
	if (is_cpu_type(MXC_CPU_MX6Q)) {
		setenv("bootargs_mem", "cma=320M");
	}
	else if (is_cpu_type(MXC_CPU_MX6D)) {
		setenv("bootargs_mem", "cma=320M");
	}
	else if (is_cpu_type(MXC_CPU_MX6DL)) {
		setenv("bootargs_mem", "cma=320M");
	}
	else if (is_cpu_type(MXC_CPU_MX6SOLO)) {
		setenv("bootargs_mem", "cma=256M");
	}
#else
	if (is_cpu_type(MXC_CPU_MX6Q)) {
		setenv("bootargs_mem", "cma=320M");
	}
	else if (is_cpu_type(MXC_CPU_MX6D)) {
		setenv("bootargs_mem", "cma=320M");
	}
	else if (is_cpu_type(MXC_CPU_MX6DL)) {
		setenv("bootargs_mem", "cma=256M");
	}
	else if (is_cpu_type(MXC_CPU_MX6SOLO)) {
		setenv("bootargs_mem", "cma=256M");
	}
#endif

	return 0;
}

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
    return 1;
}
#endif

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
static const struct mx6dq_iomux_ddr_regs mx6q_ddr_ioregs = {
	.dram_sdclk_0 	=  0x00020030,
	.dram_sdclk_1 	=  0x00020030,
	.dram_cas 		=  0x00020030,
	.dram_ras 		=  0x00020030,
	.dram_reset 	=  0x00020030,
	.dram_sdcke0 	=  0x00003000,
	.dram_sdcke1 	=  0x00003000,
	.dram_sdba2 	=  0x00000000,
	.dram_sdodt0 	=  0x00003030,
	.dram_sdodt1 	=  0x00003030,
	.dram_sdqs0 	=  0x00000030,
	.dram_sdqs1 	=  0x00000030,
	.dram_sdqs2 	=  0x00000030,
	.dram_sdqs3 	=  0x00000030,
	.dram_sdqs4 	=  0x00000030,
	.dram_sdqs5 	=  0x00000030,
	.dram_sdqs6 	=  0x00000030,
	.dram_sdqs7 	=  0x00000030,
	.dram_dqm0 		=  0x00020030,
	.dram_dqm1 		=  0x00020030,
	.dram_dqm2 		=  0x00020030,
	.dram_dqm3 		=  0x00020030,
	.dram_dqm4 		=  0x00020030,
	.dram_dqm5 		=  0x00020030,
	.dram_dqm6 		=  0x00020030,
	.dram_dqm7 		=  0x00020030,
};

static const struct mx6dq_iomux_grp_regs mx6q_grp_ioregs = {
	.grp_ddr_type    =  0x000C0000,
	.grp_ddrmode_ctl =  0x00020000,
	.grp_ddrpke      =  0x00000000,
	.grp_addds       =  0x00000030,
	.grp_ctlds       =  0x00000030,
	.grp_ddrmode     =  0x00020000,
	.grp_b0ds        =  0x00000030,
	.grp_b1ds        =  0x00000030,
	.grp_b2ds        =  0x00000030,
	.grp_b3ds        =  0x00000030,
	.grp_b4ds        =  0x00000030,
	.grp_b5ds        =  0x00000030,
	.grp_b6ds        =  0x00000030,
	.grp_b7ds        =  0x00000030,
};

static const struct mx6sdl_iomux_ddr_regs mx6dl_ddr_ioregs = {
	.dram_sdclk_0 =  0x00000030,
	.dram_sdclk_1 =  0x00000030,
	.dram_cas     =  0x00000030,
	.dram_ras     =  0x00000030,
	.dram_reset   =  0x00000030,
	.dram_sdcke0  =  0x00003000,
	.dram_sdcke1  =  0x00003000,
	.dram_sdba2   =  0x00000000,
	.dram_sdodt0  =  0x00000030,
	.dram_sdodt1  =  0x00000030,
	.dram_sdqs0   =  0x00000030,
	.dram_sdqs1   =  0x00000030,
	.dram_sdqs2   =  0x00000030,
	.dram_sdqs3   =  0x00000030,
	.dram_sdqs4   =  0x00000030,
	.dram_sdqs5   =  0x00000030,
	.dram_sdqs6   =  0x00000030,
	.dram_sdqs7   =  0x00000030,
	.dram_dqm0    =  0x00000030,
	.dram_dqm1    =  0x00000030,
	.dram_dqm2    =  0x00000030,
	.dram_dqm3    =  0x00000030,
	.dram_dqm4    =  0x00000030,
	.dram_dqm5    =  0x00000030,
	.dram_dqm6    =  0x00000030,
	.dram_dqm7    =  0x00000030,
};

static const struct mx6sdl_iomux_grp_regs mx6dl_grp_ioregs = {
	.grp_ddr_type    =  0x000C0000,
	.grp_ddrmode_ctl =  0x00020000,
	.grp_ddrpke      =  0x00000000,
	.grp_addds       =  0x00000030,
	.grp_ctlds       =  0x00000030,
	.grp_ddrmode     =  0x00020000,
	.grp_b0ds        =  0x00000030,
	.grp_b1ds        =  0x00000030,
	.grp_b2ds        =  0x00000030,
	.grp_b3ds        =  0x00000030,
	.grp_b4ds        =  0x00000030,
	.grp_b5ds        =  0x00000030,
	.grp_b6ds        =  0x00000030,
	.grp_b7ds        =  0x00000030,
};

/* SOM with Dual/Quad processors */
static const struct mx6_mmdc_calibration mx6q_mmcd_calib = {
	.p0_mpwldectrl0 =  0x00000000,
	.p0_mpwldectrl1 =  0x00000000,
	.p1_mpwldectrl0 =  0x00000000,
	.p1_mpwldectrl1 =  0x00000000,
	.p0_mpdgctrl0 =    0x0314031c,
	.p0_mpdgctrl1 =    0x023e0304,
	.p1_mpdgctrl0 =    0x03240330,
	.p1_mpdgctrl1 =    0x03180260,
	.p0_mprddlctl =    0x3630323c,
	.p1_mprddlctl =    0x3436283a,
	.p0_mpwrdlctl =    0x36344038,
	.p1_mpwrdlctl =    0x422a423c,
};

/* SOM with Solo/Dual Lite processors */
static const struct mx6_mmdc_calibration mx6dl_mmcd_calib = {
	.p0_mpwldectrl0 =  0x0045004D,
	.p0_mpwldectrl1 =  0x003A0047,
	.p0_mpdgctrl0 =    0x023C0224,
	.p0_mpdgctrl1 =    0x02000220,
	.p0_mprddlctl =    0x44444846,
	.p0_mpwrdlctl =    0x32343032,
	.p1_mpwldectrl0 =  0x001F001F,
	.p1_mpwldectrl1 =  0x00210035,
	.p1_mpdgctrl0 =    0x02200220,
	.p1_mpdgctrl1 =    0x02000220,
	.p1_mprddlctl =    0x4042463C,
	.p1_mpwrdlctl =    0x36363430,
};


/* MT41K256M16HA-125:E */
static struct mx6_ddr3_cfg mem_ddr_4g = {
	.mem_speed = 1600,
	.density   = 4,
	.width     = 16,
	.banks     = 8,
	.rowaddr   = 15,
	.coladdr   = 10,
	.pagesz    = 2,
	.trcd      = 1375,
	.trcmin    = 4875,
	.trasmin   = 3500,
	.SRT       = 0,
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
	{
		mx6dq_dram_iocfg(width, &mx6q_ddr_ioregs, &mx6q_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6q_mmcd_calib,  &mem_ddr_4g);
	}
	else
	{
		mx6sdl_dram_iocfg(width, &mx6dl_ddr_ioregs, &mx6dl_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6dl_mmcd_calib, &mem_ddr_4g);
	}
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of uart */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
#ifndef TURING_SMART_VARIANT
	if (is_cpu_type(MXC_CPU_MX6Q)) {
		printf("Configuring DDR3 for i.MX6 Quad\n");
		spl_dram_init(64);
	}
	else if (is_cpu_type(MXC_CPU_MX6D)) {
		printf("Configuring DDR3 for i.MX6 Dual\n");
		spl_dram_init(32);
	}
	else if (is_cpu_type(MXC_CPU_MX6DL)) {
		printf("Configuring DDR3 for i.MX6 Dual Lite\n");
		spl_dram_init(32);
	}
	else if (is_cpu_type(MXC_CPU_MX6SOLO)) {
		printf("Configuring DDR3 for i.MX6 Solo\n");
		spl_dram_init(16);
	}
#else
	if (is_cpu_type(MXC_CPU_MX6Q)) {
		printf("Configuring DDR3 for i.MX6 Quad Smart\n");
		spl_dram_init(32);
	}
	else if (is_cpu_type(MXC_CPU_MX6D)) {
		printf("Configuring DDR3 for i.MX6 Dual\n");
		spl_dram_init(32);
	}
	else if (is_cpu_type(MXC_CPU_MX6DL)) {
		printf("Configuring DDR3 for i.MX6 Dual Lite Smart\n");
		spl_dram_init(16);
	}
	else if (is_cpu_type(MXC_CPU_MX6SOLO)) {
		printf("Configuring DDR3 for i.MX6 Solo\n");
		spl_dram_init(16);
	}
#endif

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
