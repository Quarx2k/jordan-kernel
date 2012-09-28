/*
 * drivers/mmc/host/omap_hsmmc_raw.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <plat/dma.h>
#include <mach/hardware.h>
#include <plat/board.h>
#include <plat/mmc.h>
#include <plat/cpu.h>
#include <plat/control.h>

#define OMAP_HSMMC1_BASE	0x4809C000
#define OMAP_HSMMC2_BASE	0x480B4000

/* OMAP HSMMC Host Controller Registers */
#define OMAP_HSMMC_SYSCONFIG	0x0010
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CSRE		0x0024
#define OMAP_HSMMC_SYSTEST	0x0028
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_PWCNT	0x0030
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_PSTATE	0x0124
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_AC12		0x013C
#define OMAP_HSMMC_CAPA		0x0140
#define OMAP_HSMMC_CUR_CAPA	0x0148

#define CM_FCLKEN1_CORE			0x48004A00
#define CM_ICLKEN1_CORE			0x48004A10
#define CM_IDLEST1_CORE			0x48004A20
#define CM_AUTOIDLE1_CORE		0x48004A30
#define OMAP3430_EN_MMC1		(1 << 24)
#define OMAP3430_EN_MMC1_SHIFT		24
#define OMAP3430_EN_MMC2		(1 << 25)
#define OMAP3430_EN_MMC2_SHIFT		25

#define VS18			(1 << 26)
#define VS30			(1 << 25)
#define SDVS18			(0x5 << 9)
#define SDVS30			(0x6 << 9)
#define SDVS33			(0x7 << 9)
#define SDVS_MASK		0x00000E00
#define SDVSCLR			0xFFFFF1FF
#define SDVSDET			0x00000400
#define AUTOIDLE		0x1
#define SDBP			(1 << 8)
#define DTO			0xe
#define ICE			0x1
#define ICS			0x2
#define CEN			(1 << 2)
#define CLKD_MASK		0x0000FFC0
#define CLKD_SHIFT		6
#define DTO_MASK		0x000F0000
#define DTO_SHIFT		16
#define ALL_AVAILABLE_INT	0x317F0337
#define INT_EN_MASK		0x307F0033
#define BWR_ENABLE		(1 << 4)
#define BRR_ENABLE		(1 << 5)
#define INIT_STREAM		(1 << 1)
#define DP_SELECT		(1 << 21)
#define DDIR			(1 << 4)
#define DMA_EN			0x1
#define MSBS			(1 << 5)
#define BCE			(1 << 1)
#define FOUR_BIT		(1 << 1)
#define DW8			(1 << 5)
#define CC			0x1
#define TC			0x02
#define OD			0x1
#define ERR			(1 << 15)
#define CMD_TIMEOUT		(1 << 16)
#define DATA_TIMEOUT		(1 << 20)
#define CMD_CRC			(1 << 17)
#define DATA_CRC		(1 << 21)
#define CARD_ERR		(1 << 28)
#define STAT_CLEAR		0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define DUAL_VOLT_OCR_BIT	7
#define SRC			(1 << 25)
#define SRD			(1 << 26)
#define SOFTRESET		(1 << 1)
#define RESETDONE		(1 << 0)
#define WTA			(1 << 8)
#define RTA			(1 << 9)
#define BWE			(1 << 10)
#define BRE			(1 << 11)
#define DLEV_0			(1 << 20)

/*
 * FIXME: Most likely all the data using these _DEVID defines should come
 * from the platform_data, or implemented in controller and slot specific
 * functions.
 */
#define OMAP_MMC1_DEVID		0
#define OMAP_MMC2_DEVID		1
#define OMAP_MMC3_DEVID		2

/* loops_per_jiffy = 0x3b6000 at highest opp */
#define MMC_TIMEOUT_MS		200
#define MMC_TIMEOUT_COUNT	(0x3b6000 * msecs_to_jiffies(MMC_TIMEOUT_MS))
#define OMAP_MMC_MASTER_CLOCK	96000000
#define DRIVER_NAME		"mmci-omap-hs"

/* Timeouts for entering power saving states on inactivity, msec */
#define OMAP_MMC_DISABLED_TIMEOUT	100
#define OMAP_MMC_SLEEP_TIMEOUT		1000
#define OMAP_MMC_OFF_TIMEOUT		8000

#define CURRENT_STATE_MASK	0x1E00
#define CURRENT_STATE_TRAN	0x800

#define UNSTUFF_BITS(resp, start, size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

#define ERASE_WR_BLK_START	32
#define ERASE_WR_BLK_END	33

/*
 * MMC Host controller read/write API's
 */
#define OMAP_HSMMC_READ(base, reg)	\
	omap_readl((base) + OMAP_HSMMC_##reg)

#define OMAP_HSMMC_WRITE(base, reg, val) \
	omap_writel((val), (base) + OMAP_HSMMC_##reg)

#undef DEBUG

#ifdef DEBUG
#define DPRINTK(fmt, args...) \
	printk(fmt, ## args)
#else
#define DPRINTK(fmt, args...)
#endif


struct raw_mmc_card {
	unsigned int		rca;		/* relative card address */
	unsigned int		type;		/* card type */
	unsigned int		state;		/* (our) card state */
#define MMC_STATE_PRESENT	(1<<0)		/* present in sysfs */
#define MMC_STATE_READONLY	(1<<1)		/* card is read-only */
#define MMC_STATE_HIGHSPEED	(1<<2)		/* card is in high speed mode */
#define MMC_STATE_BLOCKADDR	(1<<3)		/* card uses block-addressing */
	u32			raw_cid[4];	/* raw card CID */
	u32			raw_csd[4];	/* raw card CSD */
	u32			raw_scr[2];	/* raw card SCR */
	struct mmc_csd		csd;
	struct sd_scr		scr;
	struct sd_switch_caps	sw_caps;
};

struct raw_mmc_data {
	unsigned int		timeout_ns;
					/* data timeout (in ns, max 80ms) */
	unsigned int		timeout_clks;	/* data timeout (in clocks) */
	unsigned int		blksz;		/* data block size */
	unsigned int		blocks;		/* number of blocks */
	unsigned int		error;		/* data error */
	unsigned int		flags;
	char			*buf;
	unsigned int		len;
	unsigned int		bytes_xfered;
};

struct raw_mmc_request {
	struct mmc_command	*cmd;
	struct raw_mmc_data	*data;
	struct mmc_command	*stop;
};

struct raw_omap_hsmmc_host {
	struct	raw_mmc_host	*mmc;
	struct	mmc_ios		ios;		/* current io bus settings */
	struct	raw_mmc_card	card;
	struct	raw_mmc_request	*mrq;
	struct	mmc_command	*cmd;
	unsigned int		ocr_avail;
	unsigned int		ocr;
	unsigned int		caps;
	unsigned int		f_min;
	unsigned int		f_max;
	u32			base;
	unsigned long		flags;
	unsigned int		id;
	unsigned int		dma_len;
	unsigned int		dma_sg_idx;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	u32			*buffer;
	u32			bytesleft;
	int			suspended;
	int			slot_id;
	int			vdd;
	int			reqs_blocked;
};

static struct raw_omap_hsmmc_host emmc_host;
static struct raw_omap_hsmmc_host usd_host;
static struct raw_omap_hsmmc_host *kpanic_host;


static void omap_hsmmc_prepare_data(struct raw_omap_hsmmc_host *host,
					struct raw_mmc_request *req);
static int get_mmc_status(struct raw_omap_hsmmc_host *host,
			unsigned int *status);

#define DEV_KOBJ_NAME_LEN 20
static void raw_omap_hsmmc_enable_clk(struct raw_omap_hsmmc_host *host)
{
	u32 reg;
	char dev_kobj_name[DEV_KOBJ_NAME_LEN];
	struct clk *iclk, *fclk;
	struct platform_device dummy_pdev = {
		.dev = {
			.bus = &platform_bus_type,
		}
	};
	struct device *dev = &dummy_pdev.dev;

	dummy_pdev.id = host->id;
	snprintf(dev_kobj_name, DEV_KOBJ_NAME_LEN - 1, "mmci-omap-hs.%d",
		host->id);
	dev_kobj_name[DEV_KOBJ_NAME_LEN - 1] = '\0';
	dummy_pdev.dev.kobj.name = dev_kobj_name;

	reg = omap_readl(CM_AUTOIDLE1_CORE);
	switch (host->id) {
	case OMAP_MMC1_DEVID:
		reg &= ~OMAP3430_EN_MMC1;
		break;
	case OMAP_MMC2_DEVID:
		reg &= ~OMAP3430_EN_MMC2;
		break;
	default:
		break;
	}
	omap_writel(reg, CM_AUTOIDLE1_CORE);

	iclk = clk_get(dev, "ick");
	if (iclk && clk_enable(iclk))
		printk(KERN_WARNING "KPANIC-MMC: MMC iclk not enabled\n");
	fclk = clk_get(dev, "fck");
	if (fclk && clk_enable(fclk))
		printk(KERN_WARNING "KPANIC-MMC: MMC fclk not enabled\n");
}

static void raw_omap_hsmmc_stop_clk(struct raw_omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		printk(KERN_WARNING "KPANIC-MMC: MMC Clock is not stoped\n");
}

void raw_omap_hsmmc_dump_regs(int index)
{
	unsigned long base;
	unsigned long reg;

	if (index == OMAP_MMC1_DEVID)
		base = OMAP_HSMMC1_BASE;
	else if (index == OMAP_MMC2_DEVID)
		base = OMAP_HSMMC2_BASE;
	else
		base = 0;

	reg = omap_readl(CM_ICLKEN1_CORE);
	reg |= OMAP3430_EN_MMC1 | OMAP3430_EN_MMC2;
	omap_writel(reg, CM_ICLKEN1_CORE);

	reg = omap_readl(CM_FCLKEN1_CORE);
	reg |= OMAP3430_EN_MMC1 | OMAP3430_EN_MMC2;
	omap_writel(reg, CM_FCLKEN1_CORE);

	printk(KERN_ERR "%s: enter with index=%d\n", __func__, index);
	printk(KERN_ERR "SYSCONFIG=0x%x CSRE=0x%x SYSTEST=0x%x CON=0x%x\n",
		OMAP_HSMMC_READ(base, STAT), OMAP_HSMMC_READ(base, CSRE),
		OMAP_HSMMC_READ(base, SYSTEST), OMAP_HSMMC_READ(base, CON));
	printk(KERN_ERR "PWCNT=0x%x BLK=0x%x ARG=0x%x CMD=0x%x\n",
		OMAP_HSMMC_READ(base, PWCNT), OMAP_HSMMC_READ(base, BLK),
		OMAP_HSMMC_READ(base, ARG), OMAP_HSMMC_READ(base, CMD));
	printk(KERN_ERR "RSP10=0x%x RSP32=0x%x RSP54=0x%x RSP76=0x%x\n",
		OMAP_HSMMC_READ(base, RSP10), OMAP_HSMMC_READ(base, RSP32),
		OMAP_HSMMC_READ(base, RSP54), OMAP_HSMMC_READ(base, RSP76));
	printk(KERN_ERR "PSTATE=0x%x HCTL=0x%x SYSCTL=0x%x\n",
		OMAP_HSMMC_READ(base, PSTATE),
		OMAP_HSMMC_READ(base, HCTL), OMAP_HSMMC_READ(base, SYSCTL));
	printk(KERN_ERR "STAT=0x%x IE=0x%x ISE=0x%x AC12=0x%x\n",
		OMAP_HSMMC_READ(base, STAT), OMAP_HSMMC_READ(base, IE),
		OMAP_HSMMC_READ(base, ISE), OMAP_HSMMC_READ(base, AC12));
	printk(KERN_ERR "CAPA=0x%x CUR_CAPA=0x%x\n",
		OMAP_HSMMC_READ(base, CAPA), OMAP_HSMMC_READ(base, CUR_CAPA));
}

static void
raw_omap_hsmmc_reset_controller_fsm(struct raw_omap_hsmmc_host *host,
					unsigned long bit)
{
	unsigned long i = 0;

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			 OMAP_HSMMC_READ(host->base, SYSCTL) | bit);

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & bit) &&
			(i++ < MMC_TIMEOUT_COUNT))
		;

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & bit)
		printk(KERN_WARNING "KPANIC-MMC: Timeout waiting on "
			"controller reset in %s\n", __func__);
}

/*
 * Send init stream sequence to card
 * before sending IDLE command
 */
static void raw_send_init_stream(struct raw_omap_hsmmc_host *host)
{
	unsigned int reg = 0;
	unsigned int timeout = 0;

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	while ((reg != CC) && (timeout++ < MMC_TIMEOUT_COUNT))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;

	if (reg != CC)
		printk(KERN_ERR "KPANIC-MMC: %s timeout, will carry on\n",
			__func__);

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_READ(host->base, STAT);
}

/*
 * Configure the response type and send the cmd.
 * this function has no idea of mrq
 */
static int
raw_omap_hsmmc_start_command(struct raw_omap_hsmmc_host *host,
	struct mmc_command *cmd, struct raw_mmc_data *data)
{
	int cmdreg = 0, resptype = 0, cmdtype = 0;
	int status;
	unsigned long timeout;
	unsigned long reg = 0;

	host->cmd = cmd;
	/*
	 * Clear status bits
	 */
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			resptype = 1;
		else if (cmd->flags & MMC_RSP_BUSY)
			resptype = 3;
		else
			resptype = 2;
	}

	/*
	 * Unlike OMAP1 controller, the cmdtype does not seem to be based on
	 * ac, bc, adtc, bcr. Only commands ending an open ended transfer need
	 * a val of 0x3, rest 0x0.
	 */
	if (host->mrq && (cmd == host->mrq->stop))
		cmdtype = 0x3;

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	if (data) {
		cmdreg |= DP_SELECT | MSBS | BCE;
		if (data->flags & MMC_DATA_READ)
			cmdreg |= DDIR;
		else
			cmdreg &= ~(DDIR);
		/* switch to infinite mode if NBLK overflowed */
		if (data->blocks > 0xFFFF)
			cmdreg &= ~(BCE);
	}

	DPRINTK(KERN_ERR "KPANIC-MMC: CMD%d, argument 0x%08x, data=%#x\n",
		cmd->opcode, cmd->arg, (unsigned int)data);
	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);

	timeout = 0;
	while ((reg != CC) && (timeout++ < MMC_TIMEOUT_COUNT))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;
	if (reg != CC) {
		raw_omap_hsmmc_dump_regs(host->id);
		printk(KERN_ERR "KPANIC-MMC: %s timeout on CMD%d-CC\n",
			__func__, cmd->opcode);
		goto out;
	}

	/* response busy command */
	/* I believe DLEV_0 has some issue, which can't trust */
	if (cmd->flags & MMC_RSP_BUSY) {
		timeout = 0;
		while ((reg != TC) && (timeout++ < MMC_TIMEOUT_COUNT))
			reg = OMAP_HSMMC_READ(host->base, STAT) & TC;
		if (reg != TC) {
			raw_omap_hsmmc_dump_regs(host->id);
			printk(KERN_ERR "KPANIC-MMC: %s timeout on CMD%d-TC\n",
				__func__, cmd->opcode);
			goto out;
		}
	}

	status = OMAP_HSMMC_READ(host->base, STAT);

	if (status & ERR) {
		if (status & CMD_TIMEOUT)
			raw_omap_hsmmc_reset_controller_fsm(host, SRC);
		cmd->error = -status;
		raw_omap_hsmmc_dump_regs(host->id);
		printk(KERN_ERR "KPANIC-MMC: %s error with status=%#x\n",
			__func__, status);
		goto out;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}

	OMAP_HSMMC_WRITE(host->base, STAT, status);

	/* Flush posted write */
	OMAP_HSMMC_READ(host->base, STAT);

	DPRINTK(KERN_ERR "KPANIC-MMC: cmd done (CMD%d): %08x %08x %08x %08x\n",
		cmd->opcode, cmd->resp[0], cmd->resp[1],
		cmd->resp[2], cmd->resp[3]);
	/* remember to do clear stuff */
	host->cmd = NULL;
	return 0;
out:
	host->cmd = NULL;
	return -1;
}

static int raw_omap_hsmmc_write_data(struct raw_omap_hsmmc_host *host,
				struct raw_mmc_request *req)
{
	int status = 0;
	unsigned long timeout = 0;
	unsigned long reg = 0;
	int i, j;

	/*
	 * Clear status bits
	 */
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);

	while ((reg != WTA) && (timeout++ < MMC_TIMEOUT_COUNT))
		reg = OMAP_HSMMC_READ(host->base, PSTATE) & WTA;
	if (reg != WTA) {
		raw_omap_hsmmc_dump_regs(host->id);
		printk(KERN_ERR "KPANIC-MMC: %s timeout on WTA, reg=0x%lx\n",
			__func__, reg);
		goto out;
	}
	for (i = 0; i < req->data->blocks; i++) {
		reg = 0;
		timeout = 0;
		while ((reg != BWE) && (timeout++ < MMC_TIMEOUT_COUNT))
			reg = OMAP_HSMMC_READ(host->base, PSTATE) & BWE;
		if (reg != BWE) {
			raw_omap_hsmmc_dump_regs(host->id);
			printk(KERN_ERR "KPANIC-MMC: %s timeout on BWE, "
				"reg=0x%lx\n", __func__, reg);
			goto out;
		}
		for (j = 0; j < req->data->blksz / 4; j++) {
			/* if not aligned to blksz, pad it with 0 */
			if ((i * req->data->blksz + j * 4) >= req->data->len)
				OMAP_HSMMC_WRITE(host->base, DATA, 0);
			else
				OMAP_HSMMC_WRITE(host->base, DATA,
					*((int *)(req->data->buf +
					i * req->data->blksz + j * 4)));
		}
		req->data->bytes_xfered += req->data->blksz;
	}

	/* infinite mode if NBLK overflowed */
	if (req->data->blocks <= 0xFFFF) {
		timeout = 0;
		while ((status != TC) && (timeout++ < MMC_TIMEOUT_COUNT))
			status = OMAP_HSMMC_READ(host->base, STAT) & TC;
		if (status != TC) {
			raw_omap_hsmmc_dump_regs(host->id);
			printk(KERN_ERR "KPANIC-MMC: %s timeout on TC, status=%#x\n",
				__func__, status);
			goto out;
		}
	}

	if (status & ERR) {
		if ((status & DATA_TIMEOUT) || (status & DATA_CRC))
			raw_omap_hsmmc_reset_controller_fsm(host, SRD);
		req->data->error = -status;
		printk(KERN_ERR "KPANIC-MMC: error writing data to FIFO, "
			"status=%#x\n", status);
		goto out;
	}
	if (status & CARD_ERR)
		printk(KERN_ERR "KPANIC-MMC: Ignoring card err CMD%d\n",
			req->cmd->opcode);

	OMAP_HSMMC_WRITE(host->base, STAT, status);
	OMAP_HSMMC_READ(host->base, STAT);

	return 0;
out:
	return -1;
}

static int raw_omap_hsmmc_read_data(struct raw_omap_hsmmc_host *host,
				struct raw_mmc_request *req)
{
	int status = 0;
	unsigned long timeout = 0;
	unsigned long reg = 0;
	int i, j;

	/*
	 * Clear status bits
	 */
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);

	while ((reg != RTA) && (timeout++ < MMC_TIMEOUT_COUNT))
		reg = OMAP_HSMMC_READ(host->base, PSTATE) & RTA;
	if (reg != RTA) {
		raw_omap_hsmmc_dump_regs(host->id);
		printk(KERN_ERR "KPANIC-MMC: %s timeout on RTA, reg=0x%lx\n",
			__func__, reg);
		goto out;
	}
	for (i = 0; i < req->data->blocks; i++) {
		reg = 0;
		timeout = 0;
		while ((reg != BRE) && (timeout++ < MMC_TIMEOUT_COUNT))
			reg = OMAP_HSMMC_READ(host->base, PSTATE) & BRE;
		if (reg != BRE) {
			raw_omap_hsmmc_dump_regs(host->id);
			printk(KERN_ERR "KPANIC-MMC: %s timeout on BRE, "
				"reg=0x%lx\n", __func__, reg);
			goto out;
		}
		for (j = 0; j < req->data->blksz / 4; j++) {
			*((unsigned int *)(req->data->buf +
				i * req->data->blksz + j * 4)) =
				OMAP_HSMMC_READ(host->base, DATA);
		}
		req->data->bytes_xfered += req->data->blksz;
	}

	timeout = 0;
	while ((status != TC) && (timeout++ < MMC_TIMEOUT_COUNT))
		status = OMAP_HSMMC_READ(host->base, STAT) & TC;
	if (status != TC) {
		raw_omap_hsmmc_dump_regs(host->id);
		printk(KERN_ERR "KPANIC-MMC: %s timeout on TC, status=%#x\n",
			__func__, status);
		goto out;
	}

	if (status & ERR) {
		if ((status & DATA_TIMEOUT) || (status & DATA_CRC))
			raw_omap_hsmmc_reset_controller_fsm(host, SRD);
		req->data->error = -status;
		printk(KERN_ERR "KPANIC-MMC: error writing data to FIFO, "
			"status=%#x\n", status);
		goto out;
	}
	if (status & CARD_ERR)
		printk(KERN_ERR "KPANIC-MMC: Ignoring card err CMD%d\n",
			req->cmd->opcode);

	OMAP_HSMMC_WRITE(host->base, STAT, status);
	/* Flush posted write */
	OMAP_HSMMC_READ(host->base, STAT);

	return 0;
out:
	return -1;
}

/*
 * Request function. for writing operation only
 * Data maybe not rounded up to block size before calling this function
 */
static void raw_omap_hsmmc_request(struct raw_omap_hsmmc_host *host,
				struct raw_mmc_request *req)
{
	int err;

	host->mrq = req;
	omap_hsmmc_prepare_data(host, req);

	err = raw_omap_hsmmc_start_command(host, req->cmd, req->data);
	if (err < 0) {
		printk(KERN_ERR "KPANIC-MMC: request failed with CMD%d %d"
			"blks %s from %#x\n", req->cmd->opcode,
			req->data->blocks, (req->data->flags & MMC_DATA_WRITE)
			? "W" : "R", req->cmd->arg);
		goto out;
	}

	if (!req->data)
		goto out;

	if (req->data->flags & MMC_DATA_WRITE)
		err = raw_omap_hsmmc_write_data(host, req);
	else
		err = raw_omap_hsmmc_read_data(host, req);
	if (err < 0) {
		printk(KERN_ERR "KPANIC-MMC: request failed with data %d"
			"blocks %s from %#x\n",
			req->data->blocks, (req->data->flags & MMC_DATA_WRITE)
			? "W" : "R", req->cmd->arg);
		goto out;
	}

	/* start command again, stop command */
	if (req->stop) {
		err = raw_omap_hsmmc_start_command(host, req->stop, NULL);
		if (err < 0) {
			printk(KERN_ERR "KPANIC-MMC: request failed with "
				"CMD%d %d bytes %s from %#x\n",
				req->stop->opcode, req->data->blocks,
				(req->data->flags & MMC_DATA_WRITE) ?
				"W" : "R", req->cmd->arg);
			goto out;
		}
	}
out:
	return;
}

static noinline void
omap_hsmmc_reset_controller(struct raw_omap_hsmmc_host *host)
{
	unsigned long i = 0;

	OMAP_HSMMC_WRITE(host->base, SYSCONFIG,
		OMAP_HSMMC_READ(host->base, SYSCONFIG) | SOFTRESET);

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & SOFTRESET) &&
			(i++ < MMC_TIMEOUT_COUNT))
		;

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & SOFTRESET)
		printk(KERN_WARNING "KPANIC-MMC: Timeout waiting on "
			"controller reset in %s\n", __func__);
}

static void set_data_timeout(struct raw_omap_hsmmc_host *host,
			     unsigned int timeout_ns,
			     unsigned int timeout_clks)
{
	unsigned int timeout, cycle_ns;
	uint32_t reg, clkd, dto = 0;

	reg = OMAP_HSMMC_READ(host->base, SYSCTL);
	clkd = (reg & CLKD_MASK) >> CLKD_SHIFT;
	if (clkd == 0)
		clkd = 1;

	cycle_ns = 1000000000 / (host->ios.clock / clkd);
	timeout = timeout_ns / cycle_ns;
	timeout += timeout_clks;
	if (timeout) {
		while ((timeout & 0x80000000) == 0) {
			dto += 1;
			timeout <<= 1;
		}
		dto = 31 - dto;
		timeout <<= 1;
		if (timeout && dto)
			dto += 1;
		if (dto >= 13)
			dto -= 13;
		else
			dto = 0;
		if (dto > 14)
			dto = 14;
	}

	reg &= ~DTO_MASK;
	reg |= dto << DTO_SHIFT;
	DPRINTK(KERN_ERR "KPANIC-MMC: set SYSCTL=%#x\n", reg);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, reg);
}

/*
 * Configure block length for MMC/SD cards and initiate the transfer.
 */
static void
omap_hsmmc_prepare_data(struct raw_omap_hsmmc_host *host,
			struct raw_mmc_request *req)
{
	DPRINTK(KERN_ERR "KPANIC-MMC: set BLK[BLEN:%#x,NBLK:%#x]\n",
		req->data->blksz, req->data->blocks << 16);
	/* switch to infinite mode if NBLK overflowed */
	if (req->data->blocks > 0xFFFF)
		OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz));
	else
		OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz)
					| (req->data->blocks << 16));
	set_data_timeout(host, req->data->timeout_ns, req->data->timeout_clks);
}

static void raw_omap_hsmmc_set_ios(struct raw_omap_hsmmc_host *host)
{
	u16 dsor = 0;
	unsigned long regval;
	unsigned long i = 0;
	u32 con;
	int do_send_init_stream = 0;
	struct mmc_ios *ios = &host->ios;

	if (ios->power_mode != host->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			host->vdd = 0;
			break;
		case MMC_POWER_UP:
			host->vdd = ios->vdd;
			break;
		case MMC_POWER_ON:
			do_send_init_stream = 1;
			break;
		}
		host->power_mode = ios->power_mode;
	}

	con = OMAP_HSMMC_READ(host->base, CON);
	switch (host->ios.bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON, con | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}

	if (host->id == OMAP_MMC1_DEVID) {
		/* Only MMC1 can interface at 3V without some flavor
		 * of external transceiver; but they all handle 1.8V.
		 */
		if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
			(ios->vdd == DUAL_VOLT_OCR_BIT)) {
				/*
				 * The mmc_select_voltage fn of the core does
				 * not seem to set the power_mode to
				 * MMC_POWER_UP upon recalculating the voltage.
				 * vdd 1.8v.
				 */
		}
	}

	if (ios->clock) {
		dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
		if (dsor < 1)
			dsor = 1;

		if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}

	/* dsor: 5 (19.2MHz) and 6 (16MHz) is a boundary */
	/* freq higher than 5 will make card stay in busy state */
	raw_omap_hsmmc_stop_clk(host);

	regval = OMAP_HSMMC_READ(host->base, SYSCTL);
	regval = regval & ~(CLKD_MASK);
	regval = regval | (dsor << 6) | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regval);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* Wait till the ICS bit is set */
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS
			&& (i++ < MMC_TIMEOUT_COUNT))
		;

	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS)
		printk(KERN_ERR "KPANIC-MMC: clock takes longer to get stable");

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

	if (do_send_init_stream)
		raw_send_init_stream(host);

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);

	DPRINTK("KPANIC-MMC: %s (%d) power_mode=%d, bus_width=%d, dsor=%d,"
		" bus_mode=%d\n", __func__, host->id, ios->power_mode,
		ios->bus_width, dsor, ios->bus_mode);
}

static void raw_set_sd_bus_power(struct raw_omap_hsmmc_host *host)
{
	unsigned long i;

	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
	for (i = 0; i < loops_per_jiffy; i++) {
		if (OMAP_HSMMC_READ(host->base, HCTL) & SDBP)
			break;
	}
}

static void omap_hsmmc_conf_bus_power(struct raw_omap_hsmmc_host *host)
{
	u32 hctl, capa, value;

	/* Only MMC1 supports 3.0V */
	if (host->id == OMAP_MMC1_DEVID) {
		hctl = SDVS30;
		capa = VS30;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	value = OMAP_HSMMC_READ(host->base, HCTL) & ~SDVS_MASK;
	OMAP_HSMMC_WRITE(host->base, HCTL, value | hctl);

	value = OMAP_HSMMC_READ(host->base, CAPA);
	OMAP_HSMMC_WRITE(host->base, CAPA, value | capa);

	/* Clear AUTO IDLE mode */
	value = OMAP_HSMMC_READ(host->base, SYSCONFIG);
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG, value & ~AUTOIDLE);

	/* Set SD bus power bit */
	raw_set_sd_bus_power(host);
}

static int raw_mmc_go_idle(struct raw_omap_hsmmc_host *host)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);

	return err;
}

static int raw_mmc_send_op_cond(struct raw_omap_hsmmc_host *host,
				u32 ocr, u32 *rocr)
{
	struct mmc_command cmd;
	int i, err = 0;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SEND_OP_COND;
	cmd.arg = ocr;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (cmd.resp[0] & MMC_CARD_BUSY)
				break;

		err = -ETIMEDOUT;

		mdelay(10);
	}

	if (rocr)
		*rocr = cmd.resp[0];

	return err;
}

static int raw_mmc_app_cmd(struct raw_omap_hsmmc_host *host)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_APP_CMD;
	cmd.arg = host->card.rca << 16;	/* card->rca can be zero here */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err) {
		printk(KERN_ERR "KPANIC-MMC: sending app cmd failed\n");
		return err;
	}

	if (!(cmd.resp[0] & R1_APP_CMD)) {
		printk(KERN_ERR "KPANIC-MMC: app cmd not supported\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int raw_mmc_send_if_cond(struct raw_omap_hsmmc_host *host, u32 ocr)
{
	struct mmc_command cmd;
	static const u8 test_pattern = 0xAA;
	u8 result_pattern;
	int err = 0;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_SEND_IF_COND;
	cmd.arg = ((ocr & 0xFF8000) != 0) << 8 | test_pattern;
	cmd.flags = MMC_RSP_SPI_R7 | MMC_RSP_R7 | MMC_CMD_BCR;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	result_pattern = cmd.resp[0] & 0xFF;

	if (result_pattern != test_pattern)
		return -EIO;

	return 0;
}

static int raw_mmc_send_app_op_cond(struct raw_omap_hsmmc_host *host,
				u32 ocr, u32 *rocr)
{
	struct mmc_command cmd;
	int i, err = 0;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_APP_OP_COND;
	cmd.arg = ocr;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = raw_mmc_app_cmd(host);
		if (err)
			break;
		err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (cmd.resp[0] & MMC_CARD_BUSY) {
			mdelay(20);
			break;
		}

		err = -ETIMEDOUT;

		mdelay(10);
	}

	if (rocr)
		*rocr = cmd.resp[0];

	return err;
}

static int raw_mmc_all_send_cid(struct raw_omap_hsmmc_host *host, u32 *cid)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_ALL_SEND_CID;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	memcpy(cid, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static int raw_mmc_set_relative_addr(struct raw_omap_hsmmc_host *host,
					unsigned int rca)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SET_RELATIVE_ADDR;
	cmd.arg = rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	return 0;
}

static int raw_mmc_send_relative_addr(struct raw_omap_hsmmc_host *host,
					unsigned int *rca)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_SEND_RELATIVE_ADDR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R6 | MMC_CMD_BCR;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	*rca = cmd.resp[0] >> 16;

	return 0;
}

static int raw_mmc_app_send_scr(struct raw_omap_hsmmc_host *host)
{
	int err;
	unsigned int scr_struct;
	u32 resp[4];

	struct mmc_command	cmd;
	struct raw_mmc_data	data;
	struct raw_mmc_request	req;

	err = raw_mmc_app_cmd(host);
	if (err)
		return err;

	memset(&req, 0, sizeof(struct raw_mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct raw_mmc_data));

	cmd.opcode = SD_APP_SEND_SCR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 8;
	data.blocks = 1;
	data.buf = (char *)&host->card.raw_scr[0];
	data.len = 8;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;
	data.flags |= MMC_DATA_READ;

	req.cmd = &cmd;
	req.data = &data;

	raw_omap_hsmmc_request(host, &req);
	if (req.cmd->error || req.data->error) {
		printk(KERN_ERR "KPANIC-MMC: done req with cmd->error=%#x,"
			"data->error=%#x\n", req.cmd->error, req.data->error);
		err = -1;
	}

	host->card.raw_scr[0] = be32_to_cpu(host->card.raw_scr[0]);
	host->card.raw_scr[1] = be32_to_cpu(host->card.raw_scr[1]);
	resp[3] = host->card.raw_scr[1];
	resp[2] = host->card.raw_scr[0];

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0)
		printk(KERN_WARNING "KPANIC-MMC: unrecognised SCR structure "
			"version %d\n", scr_struct);
	host->card.scr.sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	host->card.scr.bus_widths = UNSTUFF_BITS(resp, 48, 4);

	return err;
}

static int raw_mmc_send_csd(struct raw_omap_hsmmc_host *host)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SEND_CSD;
	cmd.arg = host->card.rca << 16;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	memcpy(host->card.raw_csd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static int raw_sd_send_csd(struct raw_omap_hsmmc_host *host)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SEND_CSD;
	cmd.arg = host->card.rca << 16;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	memcpy(host->card.raw_csd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static void raw_sd_decode_csd(struct raw_omap_hsmmc_host *host)
{
	unsigned int e, m, csd_struct;
	u32 *resp;

	resp = host->card.raw_csd;
	csd_struct = UNSTUFF_BITS(resp, 126, 2);
	switch (csd_struct) {
	case 0:
		host->card.csd.cmdclass = UNSTUFF_BITS(resp, 84, 12);
		e = UNSTUFF_BITS(resp, 47, 3);
		m = UNSTUFF_BITS(resp, 62, 12);
		host->card.csd.capacity = (1 + m) << (e + 2);
		host->card.csd.read_blkbits = UNSTUFF_BITS(resp, 80, 4);
		break;
	case 1:
		host->card.state |= MMC_STATE_BLOCKADDR;
		host->card.csd.cmdclass = UNSTUFF_BITS(resp, 84, 12);
		m = UNSTUFF_BITS(resp, 48, 22);
		host->card.csd.capacity = (1 + m) << 10;
		host->card.csd.read_blkbits = 9;
		break;
	default:
		printk(KERN_ERR "KPANIC-MMC: unrecognised CSD structure"
			" version %d\n", csd_struct);
		break;
	}
}

static int raw_omap_hsmmc_select_card(struct raw_omap_hsmmc_host *host)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SELECT_CARD;

	cmd.arg = host->card.rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	return 0;
}

static int raw_mmc_read_ext_csd(struct raw_omap_hsmmc_host *host)
{
	int err = 0;
	char raw_ext_csd[512];
	struct mmc_command	cmd;
	struct raw_mmc_data	data;
	struct raw_mmc_request	req;

	memset(&req, 0, sizeof(struct raw_mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct raw_mmc_data));
	memset(raw_ext_csd, 0, sizeof(raw_ext_csd));

	cmd.opcode = MMC_SEND_EXT_CSD;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 512;
	data.blocks = 1;
	data.buf = raw_ext_csd;
	data.len = 512;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;
	data.flags |= MMC_DATA_READ;

	req.cmd = &cmd;
	req.data = &data;

	raw_omap_hsmmc_request(host, &req);
	if (req.cmd->error || req.data->error) {
		printk(KERN_ERR "KPANIC-MMC: done req with cmd->error=%#x,"
			"data->error=%#x\n", req.cmd->error, req.data->error);
		err = -1;
	}

	return err;
}

static int raw_mmc_sd_switch(struct raw_omap_hsmmc_host *host, int mode,
	int group, u8 value, u8 *resp)
{
	struct mmc_command	cmd;
	struct raw_mmc_data	data;
	struct raw_mmc_request	req;

	mode = !!mode;
	value &= 0xF;

	memset(&req, 0, sizeof(struct raw_mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct raw_mmc_data));

	cmd.opcode = SD_SWITCH;
	cmd.arg = mode << 31 | 0x00FFFFFF;
	cmd.arg &= ~(0xF << (group * 4));
	cmd.arg |= value << (group * 4);
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 64;
	data.blocks = 1;
	data.buf = resp;
	data.len = 64;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;
	data.flags |= MMC_DATA_READ;

	req.cmd = &cmd;
	req.data = &data;

	raw_omap_hsmmc_request(host, &req);
	if (req.cmd->error || req.data->error) {
		printk(KERN_ERR "KPANIC-MMC: done req with cmd->error=%#x,"
			"data->error=%#x\n", req.cmd->error, req.data->error);
		return -EIO;
	}

	return 0;
}

static int raw_mmc_read_switch(struct raw_omap_hsmmc_host *host)
{
	int err;
	char status[64];

	if (host->card.scr.sda_vsn < SCR_SPEC_VER_1) {
		printk(KERN_ERR "sda_vsn(%d) < %d\n", host->card.scr.sda_vsn,
			SCR_SPEC_VER_1);
		return 0;
	}

	if (!(host->card.csd.cmdclass & CCC_SWITCH)) {
		printk(KERN_WARNING "KPANIC-MMC: card lacks mandatory switch "
			"function, performance might suffer.\n");
		return 0;
	}

	memset(status, 0, sizeof(status));
	err = raw_mmc_sd_switch(host, 0, 0, 1, status);
	if (err) {
		printk(KERN_WARNING "KPANIC-MMC: problem reading switch "
			"capabilities, performance might suffer.\n");
		err = 0;
		goto out;
	}

	if (status[13] & 0x02)
		host->card.sw_caps.hs_max_dtr = 50000000;
out:
	return err;
}

static int raw_mmc_switch_hs(struct raw_omap_hsmmc_host *host)
{
	int err;
	char status[64];

	if (host->card.scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(host->card.csd.cmdclass & CCC_SWITCH))
		return 0;

	if (!(host->caps & MMC_CAP_SD_HIGHSPEED))
		return 0;

	if (host->card.sw_caps.hs_max_dtr == 0)
		return 0;

	if (host->card.csd.read_blkbits == 10)
		return 0;

	err = raw_mmc_sd_switch(host, 1, 0, 1, status);
	if (err)
		goto out;

	if ((status[16] & 0xF) != 1)
		printk(KERN_WARNING "KPANIC-MMC: Problem switching card "
			"into high-speed mode!\n");
out:
	return err;
}

static int raw_mmc_app_set_bus_width(struct raw_omap_hsmmc_host *host,
					int width)
{
	int err;
	struct mmc_command cmd;

	err = raw_mmc_app_cmd(host);
	if (err) {
		printk(KERN_ERR "KPANIC-MMC: app cmd failed when setting bus width\n");
		return err;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = SD_APP_SET_BUS_WIDTH;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	switch (width) {
	case MMC_BUS_WIDTH_1:
		cmd.arg = SD_BUS_WIDTH_1;
		break;
	case MMC_BUS_WIDTH_4:
		cmd.arg = SD_BUS_WIDTH_4;
		break;
	default:
		return -EINVAL;
	}

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	return 0;
}

static int raw_mmc_switch(struct raw_omap_hsmmc_host *host,
				u8 set, u8 index, u8 value)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		  (index << 16) |
		  (value << 8) |
		  set;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	return 0;
}

static int raw_sd_erase(struct raw_omap_hsmmc_host *host, sector_t start,
			sector_t end)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = ERASE_WR_BLK_START;
	cmd.arg = start;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err) {
		printk(KERN_ERR "KPANIC-MMC: sending CMD%d failed\n",
			ERASE_WR_BLK_START);
		return err;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = ERASE_WR_BLK_END;
	cmd.arg = end;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err) {
		printk(KERN_ERR "KPANIC-MMC: sending CMD%d failed\n",
			ERASE_WR_BLK_END);
		return err;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err) {
		printk(KERN_ERR "KPANIC-MMC: sending CMD%d failed\n",
			MMC_ERASE);
		return err;
	}

	return 0;
}

static int raw_set_blksize(struct raw_omap_hsmmc_host *host)
{
	struct mmc_command cmd;
	int err;

	/* Block-addressed cards ignore MMC_SET_BLOCKLEN. */
	if (host->card.state & MMC_STATE_BLOCKADDR)
		return 0;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = 512;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err) {
		printk(KERN_ERR "KPANIC-MMC: sending CMD%d failed\n",
			ERASE_WR_BLK_START);
		return err;
	}

	return 0;
}

static int raw_mmc_send_status(struct raw_omap_hsmmc_host *host, u32 *status)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = host->card.rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	err = raw_omap_hsmmc_start_command(host, &cmd, NULL);
	if (err)
		return err;

	/* NOTE: callers are required to understand the difference
	 * between "native" and SPI format status words!
	 */
	if (status)
		*status = cmd.resp[0];

	return 0;
}

static int get_mmc_status(struct raw_omap_hsmmc_host *host,
				unsigned int *status)
{
	int err;

	err = raw_mmc_send_status(host, status);
	return err;
}

static void raw_mmc_power_up(struct raw_omap_hsmmc_host *host)
{
	host->ios.vdd = MMC_VDD_165_195;
	host->ios.chip_select = MMC_CS_DONTCARE;
	host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;

	host->ios.power_mode = MMC_POWER_UP;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	raw_omap_hsmmc_set_ios(host);

	mdelay(10);

	host->ios.clock = host->f_min;
	host->ios.power_mode = MMC_POWER_ON;
	raw_omap_hsmmc_set_ios(host);

	mdelay(10);
}

static int raw_mmc_write_mmc(char *buf, sector_t start_sect, sector_t nr_sects,
			unsigned int offset, unsigned int len)
{
	struct mmc_command	cmd;
	struct raw_mmc_data	data;
	struct mmc_command	stop;
	struct raw_mmc_request	req;

	DPRINTK(KERN_ERR "KPANIC-MMC: %s : start_sect=%u, nr_sects=%u, "
		"offset=%u, len=%u\n", __func__, (unsigned int)start_sect,
		(unsigned int)nr_sects, offset, len);
	if (!len)
		return 0;
	if (offset + len > nr_sects * 512) {
		printk(KERN_ERR "KPANIC-MMC: writing buf too long for "
			"the partition\n");
		return 0;
	}
	if (offset % 512 != 0) {
		printk(KERN_ERR "KPANIC-MMC: writing offset not aligned to "
			"sector size\n");
		return 0;
	}
	/* truncate those bytes that are not aligned to word */
	/* buffer not aligned to sector size is taken care of */

	memset(&req, 0, sizeof(struct raw_mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&stop, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct raw_mmc_data));

	cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
	cmd.arg = start_sect + offset / 512;
	if (kpanic_host && !(kpanic_host->card.state & MMC_STATE_BLOCKADDR))
		cmd.arg <<= 9;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	stop.opcode = MMC_STOP_TRANSMISSION;
	stop.arg = 0;
	stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	data.blksz = 512;
	data.blocks = (len + 511) / 512;
	data.buf = buf;
	data.len = len;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;
	data.flags |= MMC_DATA_WRITE;

	req.cmd = &cmd;
	req.data = &data;
	req.stop = &stop;

	if (kpanic_host != NULL)
		raw_omap_hsmmc_request(kpanic_host, &req);
	if (req.cmd->error || req.data->error || req.stop->error) {
		printk(KERN_ERR "KPANIC-MMC: done req with cmd->error=%#x,"
			"data->error=%#x, stop->error=%#x\n", req.cmd->error,
			req.data->error, req.stop->error);
		return 0;
	} else
		return len;
}

/*
 * You never know current state of eMMC card when panic happens
 * So need a clear startup
 */
static int raw_mmc_probe_emmc(struct raw_hd_struct *rhd)
{
	/* need to check the start_sector and nr_sector value are valid */
	struct raw_omap_hsmmc_host *host;
	int ret = 0;
	unsigned int status;
	unsigned int rocr;

	kpanic_host = NULL;	/* detect eMMC, then detect SD case */
	DPRINTK(KERN_ERR "KPANIC-MMC: probe eMMC chip\n");
	host = &emmc_host;
	memset(host, 0, sizeof(emmc_host));

	host->ocr_avail		= MMC_VDD_165_195;
	host->ocr		= MMC_VDD_165_195;
	host->base		= OMAP_HSMMC2_BASE;
	host->id		= OMAP_MMC2_DEVID;
	host->f_min		= 400000;
	host->f_max		= 52000000;
	host->power_mode	= MMC_POWER_OFF;

	/* enable iclk, fclk for mmchs1 */
	raw_omap_hsmmc_enable_clk(host);

	/* no more might_sleep after this point */
	local_irq_disable();

	/* assume gpio setting will not changed by someone else */

	host->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_8_BIT_DATA;

	omap_hsmmc_reset_controller(host);
	omap_hsmmc_conf_bus_power(host);
	OMAP_HSMMC_WRITE(host->base, ISE, 0x3);
	OMAP_HSMMC_WRITE(host->base, IE, 0x3);

	raw_mmc_power_up(host);

	ret = raw_mmc_go_idle(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) command go idle failed"
			" (%d)\n", host->id, ret);
		goto err;
	}

	mdelay(20);
	ret = raw_mmc_send_op_cond(host,
		host->ocr | MMC_OCR_REG_ACCESS_MODE_SECTOR, &rocr);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) setting OCR failed "
			"(%d)\n", host->id, ret);
		goto err;
	}

	if ((rocr & MMC_OCR_REG_ACCESS_MODE_MASK)
			== MMC_OCR_REG_ACCESS_MODE_SECTOR) {
		host->card.state |= MMC_STATE_BLOCKADDR;
	}

	ret = raw_mmc_all_send_cid(host, host->card.raw_cid);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) get CID failed (%d)\n",
			host->id, ret);
		goto err;
	}

	host->card.type = MMC_TYPE_MMC;
	host->card.rca = 1;

	ret = raw_mmc_set_relative_addr(host, host->card.rca);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) set RCA failed (%d)\n",
			host->id, ret);
		goto err;
	}

	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	raw_omap_hsmmc_set_ios(host);

	ret = raw_mmc_send_csd(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) getting CSD failed "
			"(%d)\n", host->id, ret);
		goto err;
	}

	ret = raw_omap_hsmmc_select_card(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) selecting card failed "
			"(%d)\n", host->id, ret);
		goto err;
	}

	ret = raw_mmc_read_ext_csd(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) get ext csd failed "
			"(%d)\n", host->id, ret);
		goto err;
	}
	mdelay(20);
	ret = raw_mmc_switch(host, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_HS_TIMING, 1);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) switching to HIGH"
			" speed failed (%d)\n", host->id, ret);
	} else {
		host->ios.timing = MMC_TIMING_MMC_HS;
		raw_omap_hsmmc_set_ios(host);

		host->ios.clock = 48000000;
		raw_omap_hsmmc_set_ios(host);
	}

	ret = get_mmc_status(host, &status);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) get mmc status "
			"failed (%d)\n", host->id, ret);
		goto err;
	}
	if ((status & CURRENT_STATE_MASK) != CURRENT_STATE_TRAN)
		printk(KERN_WARNING "KPANIC-MMC: host(%d) status=%#x\n",
			host->id, status);
	mdelay(20);
	ret = raw_mmc_switch(host, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_BUS_WIDTH, EXT_CSD_BUS_WIDTH_8);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) switching to wide"
			" bus failed (%d)\n", host->id, ret);
	} else {
		host->ios.bus_width = MMC_BUS_WIDTH_8;
		raw_omap_hsmmc_set_ios(host);
	}
	mdelay(20);

	ret = get_mmc_status(host, &status);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) get mmc status "
			"failed (%d)\n", host->id, ret);
		goto err;
	}
	if ((status & CURRENT_STATE_MASK) != CURRENT_STATE_TRAN)
		printk(KERN_WARNING "KPANIC-MMC: host(%d) status=%#x\n",
			host->id, status);

	ret = raw_set_blksize(host);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) set blksize failed"
			" (%d)\n", host->id, ret);
		goto err;
	}

	kpanic_host = &emmc_host;
err:
	return ret;
}

/*
 * You never know current state of uSD card when panic happens
 * So need a clear startup
 */
int raw_mmc_probe_usd(struct raw_hd_struct *rhd)
{
	/* need to check the start_sector and nr_sector value are valid */
	struct raw_omap_hsmmc_host *host;
	int ret = 0;
	unsigned int status;
	int retries;

	kpanic_host = NULL;
	DPRINTK(KERN_ERR "KPANIC-MMC: probe uSD card\n");
	local_irq_disable();
	host = &usd_host;
	memset(host, 0, sizeof(usd_host));

	host->ocr_avail		= MMC_VDD_32_33 | MMC_VDD_33_34
					| MMC_VDD_165_195;
	host->ocr		= MMC_VDD_32_33;
	host->base		= OMAP_HSMMC1_BASE;
	host->id		= OMAP_MMC1_DEVID;
	host->f_min		= 400000;
	host->f_max		= 52000000;
	host->power_mode	= MMC_POWER_OFF;

	/* enable iclk, fclk for mmchs0 */
	raw_omap_hsmmc_enable_clk(host);

	/* assume gpio setting will not changed by someone else */

	host->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA;

	omap_hsmmc_reset_controller(host);
	omap_hsmmc_conf_bus_power(host);
	OMAP_HSMMC_WRITE(host->base, ISE, 0x3);
	OMAP_HSMMC_WRITE(host->base, IE, 0x3);

	raw_mmc_power_up(host);

	ret = raw_mmc_go_idle(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) command go idle failed"
			" (%d)\n", host->id, ret);
		goto err;
	}

	ret = raw_mmc_send_if_cond(host, host->ocr);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) getting interface "
			"condition failed (%d)\n", host->id, ret);
		goto err;
	}

	host->ocr |= 1 << 30;
	ret = raw_mmc_send_app_op_cond(host, host->ocr, NULL);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) getting OCR failed"
			" (%d)\n", host->id, ret);
		goto err;
	}

	ret = raw_mmc_all_send_cid(host, host->card.raw_cid);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) get CID failed (%d)\n",
			host->id, ret);
		goto err;
	}

	host->card.type = MMC_TYPE_SD;

	ret = raw_mmc_send_relative_addr(host, &host->card.rca);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) get RCA failed (%d)\n",
			host->id, ret);
		goto err;
	}

	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	raw_omap_hsmmc_set_ios(host);

	ret = raw_sd_send_csd(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) getting CSD failed "
			"(%d)\n", host->id, ret);
		goto err;
	}
	raw_sd_decode_csd(host);

	ret = raw_omap_hsmmc_select_card(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) selecting card failed "
			"(%d)\n", host->id, ret);
		goto err;
	}

	ret = raw_mmc_app_send_scr(host);
	if (ret) {
		printk(KERN_ERR "KPANIC-MMC: host(%d) getting SCR failed "
			"(%d)\n", host->id, ret);
		goto err;
	}

	for (retries = 1; retries <= 3; retries++) {
		ret = raw_mmc_read_switch(host);
		if (!ret) {
			if (retries > 1)
				printk(KERN_WARNING "KPANIC-MMC: host(%d)"
					" recovered\n", host->id);
			break;
		} else
			printk(KERN_WARNING "KPANIC-MMC: host(%d) read switch"
				" failed (attempt %d)\n", host->id, retries);
	}

	ret = raw_mmc_switch_hs(host);
	if (ret)
		printk(KERN_WARNING "KPANIC-MMC: host(%d) switching to HIGH"
			" speed failed (%d)\n", host->id, ret);
	else {
		host->ios.timing = MMC_TIMING_SD_HS;
		raw_omap_hsmmc_set_ios(host);

		host->ios.clock = 48000000;
		raw_omap_hsmmc_set_ios(host);

		host->card.state |= MMC_STATE_HIGHSPEED;
	}

	ret = get_mmc_status(host, &status);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) get mmc status "
			"failed (%d)\n", host->id, ret);
		goto err;
	}
	if ((status & CURRENT_STATE_MASK) != CURRENT_STATE_TRAN)
		printk(KERN_WARNING "KPANIC-MMC: host(%d) status=%#x\n",
			host->id, status);

	if ((host->caps & MMC_CAP_4_BIT_DATA) &&
		(host->card.scr.bus_widths & SD_SCR_BUS_WIDTH_4)) {
		ret = raw_mmc_app_set_bus_width(host, MMC_BUS_WIDTH_4);
		if (ret)
			printk(KERN_WARNING "KPANIC-MMC: host(%d) setting"
				" bus width failed (%d)\n", host->id, ret);
		else {
			host->ios.bus_width = MMC_BUS_WIDTH_4;
			raw_omap_hsmmc_set_ios(host);
		}
	}

	ret = get_mmc_status(host, &status);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) get mmc status "
			"failed (%d)\n", host->id, ret);
		goto err;
	}
	if ((status & CURRENT_STATE_MASK) != CURRENT_STATE_TRAN)
		printk(KERN_WARNING "KPANIC-MMC: host(%d) status=%#x\n",
			host->id, status);

	ret = raw_set_blksize(host);
	if (ret) {
		printk(KERN_WARNING "KPANIC-MMC: host(%d) set blksize failed"
			" (%d)\n", host->id, ret);
		goto err;
	}

	kpanic_host = &usd_host;
err:
	return ret;
}

int raw_mmc_panic_probe(struct raw_hd_struct *rhd, int type)
{
	if (type == MMC_TYPE_MMC)
		return raw_mmc_probe_emmc(rhd);
	else if (type == MMC_TYPE_SD)
		return raw_mmc_probe_usd(rhd);
	else
		return -1;
}

int raw_mmc_panic_write(struct raw_hd_struct *rhd, char *buf,
			unsigned int offset, unsigned int len)
{
	return raw_mmc_write_mmc(buf, rhd->start_sect, rhd->nr_sects,
			offset, len);
}

/*
 * offset and len should be aligned to 512
 */
int raw_mmc_panic_erase(struct raw_hd_struct *rhd, unsigned int offset,
			unsigned int len)
{
	int ret = -1;

	if (!kpanic_host) {
		printk(KERN_ERR "KPANIC_MMC: no card probed\n");
		return -1;
	}
	if ((offset % 512) || (len % 512)) {
		printk(KERN_ERR "KPANIC_MMC: non-aligned erase\n");
		return -1;
	}
	if ((offset + len) / 512 > rhd->nr_sects) {
		printk(KERN_ERR "KPANIC_MMC: out of range erase\n");
		return -1;
	}
	if (!len)
		return 0;

	if (kpanic_host->card.type == MMC_TYPE_MMC)
		ret = -1;
	else if (kpanic_host->card.type == MMC_TYPE_SD)
		ret = raw_sd_erase(kpanic_host, rhd->start_sect +
			offset / 512, len / 512);
	else
		printk(KERN_ERR "KPANIC_MMC: card.type not recognized %d\n",
			kpanic_host->card.type);

	if (ret)
		printk(KERN_ERR "KPANIC_MMC: erase mmc/SD failed\n");
	return ret;
}

