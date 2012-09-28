 /*
  * Copyright (C)2007 - 2010 Motorola, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
  * 02111-1307, USA
  *
  */
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/smp_lock.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/sound.h>
#include <linux/poll.h>
#include <linux/cpcap_audio_platform_data.h>

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <plat/mux.h>
#include <plat/control.h>
#include <plat/gpio.h>
#include <plat/hardware.h>
#include <plat/board-mapphone.h>
#include <linux/spi/cpcap.h>
#include <linux/sound_fm_mixer.h>
#include "omap34xx_audio_driver.h"
#include "cpcap_audio_driver.h"

#define AUDIO_DRIVER_NAME "cpcap_audio"
#define CONFIG_USE_MCBSP_FIFO

#define STDAC_SSI OMAP_MCBSP2
#define CODEC_SSI OMAP_MCBSP3

#define CODEC_FIFO_SIZE 256
#define STDAC_FIFO_SIZE 8192
#define AUDIO_CAPTURE_SIZE 800
#define GPIO_AUDIO_SELECT_CPCAP  143

#define OMAP2_CONTROL_DEVCONF0_BIT6 6

/* This is the number of total kernel buffers */
#define AUDIO_NBFRAGS_WRITE 2
#define AUDIO_NBFRAGS_READ 15

#define AUDIO_TIMEOUT HZ

#define NUMBER_OF_RATES_SUPPORTED (sizeof(valid_sample_rates)/\
				   sizeof(struct sample_rate_info_t))

/* Log level standard used here:
 * Log level 3 all messages
 * Log level 2 all entry-exit points
 * Log level 1 major messages
 * Log level 0 no messages
 */
#define AUDIO_LOG_LEVEL 1

#define AUDIO_DEBUG_LOG(args...)  printk(KERN_INFO "AUDIO_DRIVER:" args)

#if (AUDIO_LOG_LEVEL >= 1)
#define AUDIO_LEVEL1_LOG(args...)  AUDIO_DEBUG_LOG(args)
#else
#define AUDIO_LEVEL1_LOG(args...)
#endif

#if (AUDIO_LOG_LEVEL >= 2)
#define AUDIO_LEVEL2_LOG(args...)  AUDIO_DEBUG_LOG(args)
#else
#define AUDIO_LEVEL2_LOG(args...)
#endif

#if (AUDIO_LOG_LEVEL >= 3)
#define AUDIO_LEVEL3_LOG(args...)  AUDIO_DEBUG_LOG(args)
#else
#define AUDIO_LEVEL3_LOG(args...)
#endif

#define AUDIO_ERROR_LOG(args...)  printk(KERN_ERR "AUDIO_DRIVER: Error " args)

#define TRY(a)  if (unlikely(a)) goto out;
DEFINE_MUTEX(audio_lock);

struct audio_buf {
	int offset;		/* current offset */
	char *data;		/* points to actual buffer */
	dma_addr_t buf_addr;	/* physical buffer address */
	int buf_ref;		/* DMA refcount - we do not know
				 * how many buffers can s/w take. */
	int master;		/* owner for buffer allocation,
				 * contain size when true */
};

/* Structure describing the data stream related information */
struct audio_stream {
	char *id;		/* identification string */
	struct audio_buf *buffers;
	/* pointer to audio buffer structures */
	u32 usr_head;		/* user side fragment index i.e.
				 * where app is reading/writing to */
	u32 buf_head;		/* BUF fragment index to go */
	u32 buf_tail;		/* BUF fragment index to complete */
	u32 fragsize;		/* fragment i.e. buffer size */
	u32 nbfrags;		/* nbr of fragments i.e. buffers */
	u32 pending_frags;	/* Fragments sent to BUF */
	u8 in_use;		/* Is this is use? */
	int *lch;		/* Chain of channels this stream is
				 * linked to */
	int input_output;	/* Direction of this data stream */
	int bytecount;		/* nbr of processed bytes */
	int fragcount;		/* nbr of fragment transitions */
	struct semaphore sem;	/* account for fragment usage */
	wait_queue_head_t wq;	/* for poll */
	int mapped:1;		/* mmap()'ed buffers */
	int active:1;		/* actually in progress */
	int stopped:1;		/* might be active but stopped */
	struct inode *inode;
};

static int audio_stdac_open(struct inode *, struct file *);
static int audio_stdac_release(struct inode *, struct file *);
static int audio_ioctl(struct inode *, struct file *file, unsigned int cmd,
			unsigned long arg);
static ssize_t audio_write(struct file *fp, const char *buf, size_t bytes,
			loff_t *nouse);
static int audio_codec_open(struct inode *, struct file *);
static int audio_codec_open_helper(struct inode *, struct file *);
static int audio_codec_release(struct inode *, struct file *);
static int audio_codec_release_helper(struct inode *, struct file *);
static ssize_t audio_codec_read(struct file *fp, char *buf, size_t bytes,
				loff_t *nouse);
static int audio_mixer_open(struct inode *, struct file *);
static int audio_mixer_close(struct inode *, struct file *);
static int audio_probe(struct platform_device *dev);
static int audio_remove(struct platform_device *dev);
static void mcbsp_dma_tx_cb(u32 ch_status, void *arg);
static void mcbsp_dma_rx_cb(u32 ch_status, void *arg);
static int audio_stop_ssi(struct inode *inode, struct file *file);
static int audio_configure_ssi(struct inode *inode, struct file *file);
static void audio_discard_buf(struct audio_stream *str, struct inode *inode);
static void audio_buffer_reset(struct audio_stream *str, struct inode *inode);
static int audio_process_buf(struct audio_stream *str, struct inode *inode);
static int audio_setup_buf(struct audio_stream *str, struct inode *inode);

/* File Ops structure */
static const struct file_operations audio_stdac_fops = {
	.owner = THIS_MODULE,
	.open = audio_stdac_open,
	.release = audio_stdac_release,
	.ioctl = audio_ioctl,
	.write = audio_write,
};

static const struct file_operations codec_fops = {
	.owner = THIS_MODULE,
	.open = audio_codec_open,
	.release = audio_codec_release,
	.ioctl = audio_ioctl,
	.write = audio_write,
	.read = audio_codec_read,
};

static const struct file_operations mixer_fops = {
	.owner = THIS_MODULE,
	.open = audio_mixer_open,
	.release = audio_mixer_close,
	.ioctl = audio_ioctl,
};

/* Driver information structure*/
static struct platform_driver audio_driver = {
	.probe = audio_probe,
	.remove = audio_remove,
	.driver = {
		   .name = AUDIO_DRIVER_NAME,
		   .owner = THIS_MODULE,
		},
};

static struct omap_mcbsp_dma_transfer_params tx_params = {
	.skip_alt = OMAP_MCBSP_SKIP_NONE,
	.auto_reset = OMAP_MCBSP_AUTO_XRST,
	.callback = mcbsp_dma_tx_cb,
	.word_length1 = OMAP_MCBSP_WORD_16,
};

static struct omap_mcbsp_dma_transfer_params rx_params = {
	.skip_alt = OMAP_MCBSP_SKIP_NONE,
	.auto_reset = OMAP_MCBSP_AUTO_RRST,
	.callback = mcbsp_dma_rx_cb,
	.word_length1 = OMAP_MCBSP_WORD_16,
};

static struct omap_mcbsp_cfg_param tx_cfg_params = {
	.fsync_src = OMAP_MCBSP_TXFSYNC_EXTERNAL,
	.fs_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH,
	.clk_polarity = OMAP_MCBSP_CLKX_POLARITY_RISING,
	.clk_mode = OMAP_MCBSP_CLKTXSRC_EXTERNAL,
	.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
	.word_length1 = OMAP_MCBSP_WORD_16,
	.justification = OMAP_MCBSP_RJUST_ZEROMSB,
	.reverse_compand = OMAP_MCBSP_MSBFIRST,
	.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
	.data_delay = OMAP_MCBSP_DATADELAY1,
};

static struct omap_mcbsp_cfg_param rx_cfg_params = {
	.fsync_src = OMAP_MCBSP_RXFSYNC_EXTERNAL,
	.fs_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH,
	.clk_polarity = OMAP_MCBSP_CLKR_POLARITY_FALLING,
	.clk_mode = OMAP_MCBSP_CLKRXSRC_EXTERNAL,
	.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
	.word_length1 = OMAP_MCBSP_WORD_16,
	.justification = OMAP_MCBSP_RJUST_ZEROMSB,
	.reverse_compand = OMAP_MCBSP_MSBFIRST,
	.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
	.data_delay = OMAP_MCBSP_DATADELAY1,
};

static struct omap_mcbsp_srg_fsg_cfg srg_fsg_params = {
	.period = 0, /* Frame period */
	.pulse_width = 0, /* Frame width */
	.fsgm = 0,
	.sample_rate = 0,
	.bits_per_sample = 16,
	.srg_src = OMAP_MCBSP_SRGCLKSRC_CLKX,
	.sync_mode = OMAP_MCBSP_SRG_FREERUNNING, /* SRG free running mode */
	.polarity = OMAP_MCBSP_CLKX_POLARITY_RISING,
	.dlb = 0, /* digital loopback mode */
};

struct sample_rate_info_t {
	u16 rate;
	int cpcap_audio_rate;
};

static struct {
	int dev_dsp;
	int dev_dsp1;
	int dev_mixer;
	int dev_dsp_open_count;
	int dev_dsp1_open_count;
	int dev_mixer_open_count;
	int stdac_ssi_started;
	int codec_ssi_started;
	int fm_on;
	struct audio_stream *stdac_out_stream;
	struct audio_stream *stdac_in_stream;
	struct audio_stream *codec_out_stream;
	struct audio_stream *codec_in_stream;
} state;

struct cpcap_audio_state cpcap_audio_state = {
	NULL,
	CPCAP_AUDIO_CODEC_OFF,
	CPCAP_AUDIO_CODEC_RATE_8000_HZ,
	CPCAP_AUDIO_CODEC_MUTE,
	CPCAP_AUDIO_STDAC_OFF,
	CPCAP_AUDIO_STDAC_RATE_44100_HZ,
	CPCAP_AUDIO_STDAC_MUTE,
	CPCAP_AUDIO_ANALOG_SOURCE_OFF,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_LOUDSPEAKER,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_BALANCE_NEUTRAL,
	CPCAP_AUDIO_BALANCE_NEUTRAL,
	CPCAP_AUDIO_BALANCE_NEUTRAL,
	7,			/*default output gain */
	0,			/*default FM gain*/
	CPCAP_AUDIO_IN_NONE,
	31,			/*default input_gain */
	31,			/*default input_gain */
	CPCAP_AUDIO_DAI_CONFIG_NORMAL
};

static const struct sample_rate_info_t valid_sample_rates[] = {
	{.rate = 8000,  .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_8000_HZ},
	{.rate = 11025, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_11025_HZ},
	{.rate = 12000, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_12000_HZ},
	{.rate = 16000, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_16000_HZ},
	{.rate = 22050, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_22050_HZ},
	{.rate = 24000, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_24000_HZ},
	{.rate = 32000, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_32000_HZ},
	{.rate = 44100, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_44100_HZ},
	{.rate = 48000, .cpcap_audio_rate = CPCAP_AUDIO_STDAC_RATE_48000_HZ},
};

static int read_buf_full;
static int read_buf_outstanding;
static int primary_spkr_setting = CPCAP_AUDIO_OUT_NONE;
static int secondary_spkr_setting = CPCAP_AUDIO_OUT_NONE;
static int mic_setting = CPCAP_AUDIO_IN_NONE;
static unsigned int capture_channels = 1;
static u8 enable_tx;
static struct omap_mcbsp_wrapper *mcbsp_wrapper;
static DEFINE_SPINLOCK(audio_write_lock);
#ifdef CONFIG_WAKELOCK
static struct wake_lock mcbsp_wakelock;
#endif

#ifdef MCBSP_WRAPPER

static void omap2_mcbsp_rx_dma_callback(int lch, unsigned short ch_status,
					void *data)
{
	struct omap_mcbsp *mcbsp_dma_rx = data;
	void __iomem *io_base;
	int id;

	for (id = 0; id < omap_mcbsp_count; id++) {
		if (data == mcbsp_ptr[id])
			break;
	}
	if (id == omap_mcbsp_count) {
		printk(KERN_INFO "No matching McBSP id for rx\n");
		return;
	}
	io_base = mcbsp_dma_rx->io_base;

	if (omap_mcbsp_read(io_base, OMAP_MCBSP_REG_IRQST) & 0x0020) {
		AUDIO_ERROR_LOG("McBSP Capture Overflow! Data Lost!");
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_IRQST, 0x0020);
	}

	/* If we are at the last transfer, Shut down the reciever */
	if ((mcbsp_wrapper[id].auto_reset & OMAP_MCBSP_AUTO_RRST)
	    && (omap_dma_chain_status(mcbsp_dma_rx->dma_rx_lch) ==
		OMAP_DMA_CHAIN_INACTIVE))
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR1) &
				 (~RRST));

	if (mcbsp_wrapper[id].rx_callback != NULL)
		mcbsp_wrapper[id].rx_callback(ch_status,
					      mcbsp_wrapper[id].rx_cb_arg);
}

static void omap2_mcbsp_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_tx = data;
	void __iomem *io_base;
	int id;

	for (id = 0; id < omap_mcbsp_count; id++) {
		if (data == mcbsp_ptr[id])
			break;
	}
	if (id == omap_mcbsp_count) {
		printk(KERN_INFO "No matching McBSP id for rx\n");
		return;
	}
	io_base = mcbsp_dma_tx->io_base;

	if (mcbsp_wrapper[id].tx_callback != NULL)
		mcbsp_wrapper[id].tx_callback(ch_status,
					      mcbsp_wrapper[id].tx_cb_arg);
}

static void omap2_mcbsp_set_recv_param(unsigned int id,
				       struct omap_mcbsp_reg_cfg *mcbsp_cfg,
				       struct omap_mcbsp_cfg_param *rp)
{
	mcbsp_cfg->spcr1 = RJUST(rp->justification);
	mcbsp_cfg->rcr2 = RCOMPAND(rp->reverse_compand) |
	    RDATDLY(rp->data_delay);
	if (rp->phase == OMAP_MCBSP_FRAME_SINGLEPHASE)
		mcbsp_cfg->rcr2 = mcbsp_cfg->rcr2 & ~(RPHASE);
	else
		mcbsp_cfg->rcr2 = mcbsp_cfg->rcr2 | (RPHASE) |
		    RWDLEN2(rp->word_length2) | RFRLEN2(rp->frame_length2);
	mcbsp_cfg->rcr1 = RWDLEN1(rp->word_length1) |
	    RFRLEN1(rp->frame_length1);
	if (rp->fsync_src == OMAP_MCBSP_RXFSYNC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSRM;
	if (rp->clk_mode == OMAP_MCBSP_CLKRXSRC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKRM;
	if (rp->clk_polarity == OMAP_MCBSP_CLKR_POLARITY_RISING)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKRP;
	if (rp->fs_polarity == OMAP_MCBSP_FS_ACTIVE_LOW)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSRP;

#ifdef CONFIG_USE_MCBSP_FIFO
	mcbsp_cfg->wken = mcbsp_cfg->wken | RRDYEN;
#endif
	return;
}

static void omap2_mcbsp_set_trans_param(unsigned int id,
					struct omap_mcbsp_reg_cfg *mcbsp_cfg,
					struct omap_mcbsp_cfg_param *tp)
{
	mcbsp_cfg->xcr2 = XCOMPAND(tp->reverse_compand) |
	    XDATDLY(tp->data_delay);
	if (tp->phase == OMAP_MCBSP_FRAME_SINGLEPHASE)
		mcbsp_cfg->xcr2 = mcbsp_cfg->xcr2 & ~(XPHASE);
	else
		mcbsp_cfg->xcr2 = mcbsp_cfg->xcr2 | (XPHASE) |
		    RWDLEN2(tp->word_length2) | RFRLEN2(tp->frame_length2);
	mcbsp_cfg->xcr1 = XWDLEN1(tp->word_length1) |
	    XFRLEN1(tp->frame_length1);
	if (tp->fs_polarity == OMAP_MCBSP_FS_ACTIVE_LOW)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSXP;
	if (tp->fsync_src == OMAP_MCBSP_TXFSYNC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSXM;
	if (tp->clk_mode == OMAP_MCBSP_CLKTXSRC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKXM;
	if (tp->clk_polarity == OMAP_MCBSP_CLKX_POLARITY_FALLING)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKXP;
	return;
}

static void omap2_mcbsp_set_srg_cfg_param(unsigned int id, int interface_mode,
					  struct omap_mcbsp_reg_cfg *mcbsp_cfg,
					  struct omap_mcbsp_srg_fsg_cfg *param)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	u32 clk_rate, clkgdv;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	mcbsp_wrapper[id].interface_mode = interface_mode;
	mcbsp_cfg->srgr1 = FWID(param->pulse_width);

	if (interface_mode == OMAP_MCBSP_MASTER) {
		/* clk_rate = clk_get_rate(omap_mcbsp_clk[id].fck); */
		clk_rate = 96000000;
		clkgdv = clk_rate / (param->sample_rate *
				     (param->bits_per_sample - 1));
		mcbsp_cfg->srgr1 = mcbsp_cfg->srgr1 | CLKGDV(clkgdv);
	}
	if (param->dlb)
		mcbsp_cfg->spcr1 = mcbsp_cfg->spcr1 & ~(ALB);

	if (param->sync_mode == OMAP_MCBSP_SRG_FREERUNNING)
		mcbsp_cfg->spcr2 = mcbsp_cfg->spcr2 | FREE;
	mcbsp_cfg->srgr2 = FPER(param->period) | (param->fsgm ? FSGM : 0);

	switch (param->srg_src) {

	case OMAP_MCBSP_SRGCLKSRC_CLKS:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSM);
		/*
		 * McBSP master operation at low voltage is only possible if
		 * CLKSP=0 In Master mode, if client driver tries to configiure
		 * input clock polarity as falling edge, we force it to Rising
		 */

		if ((param->polarity == OMAP_MCBSP_CLKS_POLARITY_RISING) ||
		    (interface_mode == OMAP_MCBSP_MASTER))
			mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSP);
		else
			mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSP);
		break;

	case OMAP_MCBSP_SRGCLKSRC_FCLK:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSM);

		break;

	case OMAP_MCBSP_SRGCLKSRC_CLKR:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | (SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSM);
		if (param->polarity == OMAP_MCBSP_CLKR_POLARITY_FALLING)
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(CLKRP);
		else
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | (CLKRP);

		break;

	case OMAP_MCBSP_SRGCLKSRC_CLKX:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | (SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSM);

		if (param->polarity == OMAP_MCBSP_CLKX_POLARITY_RISING)
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(CLKXP);
		else
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | (CLKXP);
		break;

	}
	if (param->sync_mode == OMAP_MCBSP_SRG_FREERUNNING)
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(GSYNC);
	else if (param->sync_mode == OMAP_MCBSP_SRG_RUNNING)
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (GSYNC);

	mcbsp_cfg->xccr = omap_mcbsp_read(io_base, OMAP_MCBSP_REG_XCCR);
	if (param->dlb)
		mcbsp_cfg->xccr = mcbsp_cfg->xccr | (DILB);
	mcbsp_cfg->rccr = omap_mcbsp_read(io_base, OMAP_MCBSP_REG_RCCR);

	return;
}

static void mcbsp_power_settings(unsigned int id, int level)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	if (level == MCBSP2_SYSCONFIG_LVL1) {
		if (id == OMAP_MCBSP2) {
			omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
				CLOCKACTIVITY(MCBSP_SYSC_IOFF_FON) |
				SIDLEMODE(SMART_IDLE) | ENAWAKEUP);
		} else {
			omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
				CLOCKACTIVITY(MCBSP_SYSC_IOFF_FON) |
				SIDLEMODE(NO_IDLE));
		}
	} else if (level == MCBSP2_SYSCONFIG_LVL2)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
				 CLOCKACTIVITY(MCBSP_SYSC_IOFF_FOFF) |
				 SIDLEMODE(FORCE_IDLE));
}

void omap2_mcbsp_set_srg_fsg(unsigned int id, unsigned char state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_DISABLE_FSG_SRG) {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) &
				 (~GRST));
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) &
				 (~FRST));
	} else {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) | GRST);
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) | FRST);
	}
	return;
}

int omap2_mcbsp_stop_datatx(unsigned int id)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	if (mcbsp->dma_tx_lch != -1) {
		if (omap_stop_dma_chain_transfers(mcbsp->dma_tx_lch) != 0)
			return -EINVAL;
	}
	mcbsp_wrapper[id].tx_dma_chain_state = 0;
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			 omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SPCR2) & (~XRST));

	if (!mcbsp_wrapper[id].rx_dma_chain_state)
		omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);

	return 0;
}

int omap2_mcbsp_stop_datarx(u32 id)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	if (mcbsp->dma_rx_lch != -1) {
		if (omap_stop_dma_chain_transfers(mcbsp->dma_rx_lch) != 0)
			return -EINVAL;
	}
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
			 omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SPCR1) & (~RRST));

	mcbsp_wrapper[id].rx_dma_chain_state = 0;
	if (!mcbsp_wrapper[id].tx_dma_chain_state)
		omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);

	return 0;
}

int omap2_mcbsp_reset(unsigned int id)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int counter = 0;
	int wait_for_reset = 10000;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
			 omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SYSCON) | (SOFTRST));

	while (omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SYSCON) & SOFTRST) {
		if (!in_interrupt()) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(10);
		}
		if (counter++ > wait_for_reset) {
			printk(KERN_ERR "mcbsp[%d] Reset timeout\n", id);
			return -ETIMEDOUT;
		}
	}
	mcbsp_power_settings(id, MCBSP2_SYSCONFIG_LVL1);
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_WKUPEN, 0xFFFF);
	return 0;
}

int omap2_mcbsp_set_xrst(unsigned int id, unsigned char state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_XRST_DISABLE)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) &
				 (~XRST));
	else
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) | XRST);
	udelay(10);

	return 0;
}

int omap2_mcbsp_set_rrst(unsigned int id, unsigned char state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_RRST_DISABLE)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR1) &
				 (~RRST));
	else
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR1) | RRST);
	udelay(10);
	return 0;
}

int omap2_mcbsp_dma_recv_params(unsigned int id,
				struct omap_mcbsp_dma_transfer_params *rp)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int err, chain_id = -1;
	struct omap_dma_channel_params rx_params;
	u32 dt = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 mcbsp_fifo_size;
#endif

#ifdef CONFIG_USE_MCBSP_FIFO
	if (id == OMAP_MCBSP2)
		mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
	else
		mcbsp_fifo_size = MCBSP_FIFO_SIZE;
#endif

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;
	dt = rp->word_length1;

	if (dt == OMAP_MCBSP_WORD_8)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt == OMAP_MCBSP_WORD_16)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else if (dt == OMAP_MCBSP_WORD_32)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S32;
	else
		return -EINVAL;

	rx_params.read_prio = DMA_CH_PRIO_HIGH;
	rx_params.write_prio = DMA_CH_PRIO_HIGH;
/* If McBSP FIFO is used, do a packet sync DMA */
#ifdef CONFIG_USE_MCBSP_FIFO
	mcbsp_wrapper[id].rx_config_done = 0;
	rx_params.sync_mode = OMAP_DMA_SYNC_PACKET;
	rx_params.src_fi = mcbsp_fifo_size;
#else
	rx_params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	rx_params.src_fi = 0;
#endif
	rx_params.trigger = mcbsp->dma_rx_sync;
	rx_params.src_or_dst_synch = 0x01;
	rx_params.src_amode = OMAP_DMA_AMODE_CONSTANT;
	rx_params.src_ei = 0x0;
	/* Indexing is always in bytes - so multiply with dt */

	dt = (rx_params.data_type == OMAP_DMA_DATA_TYPE_S8) ? 1 :
	    (rx_params.data_type == OMAP_DMA_DATA_TYPE_S16) ? 2 : 4;

	/* SKIP_FIRST and sKIP_SECOND- 24 bit data in stereo mode */
	if (rp->skip_alt == OMAP_MCBSP_SKIP_SECOND) {
		rx_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		rx_params.dst_ei = (1);
		rx_params.dst_fi = (1) + ((-1) * dt);
	} else if (rp->skip_alt == OMAP_MCBSP_SKIP_FIRST) {
		rx_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		rx_params.dst_ei = 1 + (-2) * dt;
		rx_params.dst_fi = 1 + (2) * dt;
	} else {
		rx_params.dst_amode = OMAP_DMA_AMODE_POST_INC;
		rx_params.dst_ei = 0;
		rx_params.dst_fi = 0;
	}

	mcbsp_wrapper[id].rxskip_alt = rp->skip_alt;
	mcbsp_wrapper[id].auto_reset &= ~OMAP_MCBSP_AUTO_RRST;
	mcbsp_wrapper[id].auto_reset |= (rp->auto_reset & OMAP_MCBSP_AUTO_RRST);

	mcbsp->rx_word_length = rx_params.data_type << 0x1;
	if (rx_params.data_type == 0)
		mcbsp->rx_word_length = 1;

	mcbsp_wrapper[id].rx_callback = rp->callback;
	mcbsp_wrapper[id].rx_params = rx_params;
	/* request for a chain of dma channels for data reception */
	if (mcbsp->dma_rx_lch == -1) {
		err = omap_request_dma_chain(id, "McBSP RX",
					     omap2_mcbsp_rx_dma_callback,
					     &chain_id, 2,
					     OMAP_DMA_DYNAMIC_CHAIN, rx_params);
		if (err < 0) {
			printk(KERN_ERR "Receive path configuration failed \n");
			return -EINVAL;
		}
		mcbsp->dma_rx_lch = chain_id;
		mcbsp_wrapper[id].rx_dma_chain_state = 0;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp->dma_rx_lch,
						   rx_params);
		if (err < 0)
			return -EINVAL;
	}

	return 0;
}

int omap2_mcbsp_dma_trans_params(unsigned int id,
				 struct omap_mcbsp_dma_transfer_params *tp)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	struct omap_dma_channel_params tx_params;
	int err = 0, chain_id = -1;
	u32 dt = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 mcbsp_fifo_size;
#endif

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;
#ifdef CONFIG_USE_MCBSP_FIFO
	if (id == OMAP_MCBSP2)
		mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
	else
		mcbsp_fifo_size = MCBSP_FIFO_SIZE;
#endif

	dt = tp->word_length1;
	if ((dt != OMAP_MCBSP_WORD_8) && (dt != OMAP_MCBSP_WORD_16)
	    && (dt != OMAP_MCBSP_WORD_32))
		return -EINVAL;
	if (dt == OMAP_MCBSP_WORD_8)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt == OMAP_MCBSP_WORD_16)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else if (dt == OMAP_MCBSP_WORD_32)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S32;
	else
		return -EINVAL;

	tx_params.read_prio = DMA_CH_PRIO_HIGH;
	tx_params.write_prio = DMA_CH_PRIO_HIGH;
/* IF McBSP FIFO is used, use packet sync DMA*/
#ifdef CONFIG_USE_MCBSP_FIFO
	tx_params.sync_mode = OMAP_DMA_SYNC_PACKET;
	tx_params.dst_fi = mcbsp_fifo_size;
#else
	tx_params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	tx_params.dst_fi = 0;
#endif
	tx_params.trigger = mcbsp->dma_tx_sync;
	tx_params.src_or_dst_synch = 0;
	/* Indexing is always in bytes - so multiply with dt */
	mcbsp->tx_word_length = tx_params.data_type << 0x1;

	if (tx_params.data_type == 0)
		mcbsp->tx_word_length = 1;
	dt = mcbsp->tx_word_length;

	/* SKIP_FIRST and sKIP_SECOND- 24 bit data in stereo mode */
	if (tp->skip_alt == OMAP_MCBSP_SKIP_SECOND) {
		tx_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		tx_params.src_ei = (1);
		tx_params.src_fi = (1) + ((-1) * dt);
	} else if (tp->skip_alt == OMAP_MCBSP_SKIP_FIRST) {
		tx_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		tx_params.src_ei = 1 + (-2) * dt;
		tx_params.src_fi = 1 + (2) * dt;
	} else {
		tx_params.src_amode = OMAP_DMA_AMODE_POST_INC;
		tx_params.src_ei = 0;
		tx_params.src_fi = 0;
	}

	tx_params.dst_amode = OMAP_DMA_AMODE_CONSTANT;
	tx_params.dst_ei = 0;
	mcbsp_wrapper[id].txskip_alt = tp->skip_alt;
	mcbsp_wrapper[id].auto_reset &= ~OMAP_MCBSP_AUTO_XRST;
	mcbsp_wrapper[id].auto_reset |= (tp->auto_reset & OMAP_MCBSP_AUTO_XRST);
	mcbsp_wrapper[id].tx_callback = tp->callback;

	/* Based on Rjust we can do double indexing DMA params configuration */
	if (mcbsp->dma_tx_lch == -1) {
		err = omap_request_dma_chain(id, "McBSP TX",
					     omap2_mcbsp_tx_dma_callback,
					     &chain_id, 2,
					     OMAP_DMA_DYNAMIC_CHAIN, tx_params);
		if (err < 0) {
			printk(KERN_ERR
			       "Transmit path configuration failed \n");
			return -EINVAL;
		}
		mcbsp_wrapper[id].tx_dma_chain_state = 0;
		mcbsp->dma_tx_lch = chain_id;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp->dma_tx_lch,
						   tx_params);
		if (err < 0)
			return -EINVAL;
	}
#ifdef CONFIG_USE_MCBSP_FIFO
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_THRSH2, (mcbsp_fifo_size - 1));
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_WKUPEN, MCBSP_WKUP_XRDYEN);
#endif

	return 0;
}

int omap2_mcbsp_receive_data(unsigned int id, void *cbdata,
			     dma_addr_t buf_start_addr, u32 buf_size)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int enable_rx = 0;
	int e_count = 0;
	int f_count = 0;
	int ret = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 thrsh1 = 256;	/* lowest value for McBSP threshold */
	u32 mcbsp_fifo_size;
	int err;
#endif

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;
	mcbsp_wrapper[id].rx_cb_arg = cbdata;

	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((mcbsp_wrapper[id].auto_reset & OMAP_MCBSP_AUTO_RRST) &&
	    (omap_dma_chain_status(mcbsp->dma_rx_lch)
	     == OMAP_DMA_CHAIN_INACTIVE)) {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR1) &
				 (~RRST));
		enable_rx = 1;
	}

	/*
	 * for skip_first and second, we need to set e_count =2,
	 * and f_count = number of frames = number of elements/e_count
	 */
	e_count = (buf_size / mcbsp->rx_word_length);

	/* IF McBSP FIFO is used, change receive side configuration */
#ifdef CONFIG_USE_MCBSP_FIFO
	if (mcbsp_wrapper[id].rx_config_done == 0) {
		mcbsp_wrapper[id].rx_config_done = 1;
		if (id == OMAP_MCBSP2)
			mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
		else
			mcbsp_fifo_size = MCBSP_FIFO_SIZE;

		if (e_count < mcbsp_fifo_size) {
			thrsh1 = e_count;
		} else {
			/* Find the optimum threshold value for MCBSP
			   to transfer complete data */
			if ((e_count % mcbsp_fifo_size) == 0)
				thrsh1 = mcbsp_fifo_size;
			else if ((e_count % ((mcbsp_fifo_size * 3) / 4)) == 0)
				thrsh1 = (mcbsp_fifo_size * 3) / 4;
			else if ((e_count % ((mcbsp_fifo_size * 1) / 2)) == 0)
				thrsh1 = (mcbsp_fifo_size * 1) / 2;
			else if ((e_count % ((mcbsp_fifo_size * 1) / 4)) == 0)
				thrsh1 = (mcbsp_fifo_size * 1) / 4;
			else
				thrsh1 = 1;
		}

		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_THRSH1, (thrsh1 - 1));

		if (thrsh1 != mcbsp_fifo_size) {
			mcbsp_wrapper[id].rx_params.src_fi = thrsh1;
			/* if threshold =1, use element sync DMA */
			if (thrsh1 == 1) {
				mcbsp_wrapper[id].rx_params.sync_mode =
				    OMAP_DMA_SYNC_ELEMENT;
				mcbsp_wrapper[id].rx_params.src_fi = 0;
			}
			err = omap_modify_dma_chain_params(mcbsp->dma_rx_lch,
						mcbsp_wrapper[id].rx_params);
			if (err < 0) {
				printk(KERN_ERR "DMA reconfiguration failed\n");
				return -EINVAL;
			}
		}
	}
#endif

	if (mcbsp_wrapper[id].rxskip_alt != OMAP_MCBSP_SKIP_NONE) {
		/*
		 * since the number of frames = total number of elements/element
		 * count, However, with double indexing for data transfers,
		 * double the number of elements need to be transmitted
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}
	/*
	 * If the DMA is to be configured to skip the first byte, we need
	 * to jump backwards, so we need to move one chunk forward and
	 * ask dma if we dont want the client driver knowing abt this.
	 */
	if (mcbsp_wrapper[id].rxskip_alt == OMAP_MCBSP_SKIP_FIRST)
		buf_start_addr += mcbsp->rx_word_length;

	ret = omap_dma_chain_a_transfer(mcbsp->dma_rx_lch,
					mcbsp->phys_base + OMAP_MCBSP_REG_DRR,
					buf_start_addr, e_count, f_count,
					mcbsp);
	if (ret < 0)
		return ret;

	if (mcbsp_wrapper[id].rx_dma_chain_state == 0) {
		if (mcbsp_wrapper[id].interface_mode == OMAP_MCBSP_MASTER)
			omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);

		ret = omap_start_dma_chain_transfers(mcbsp->dma_rx_lch);
		if (ret < 0)
			return ret;

		mcbsp_wrapper[id].rx_dma_chain_state = 1;
	}
	/* Auto RRST handling logic - Enable the Reciever after 1st dma */
	if (enable_rx && (omap_dma_chain_status(mcbsp->dma_rx_lch)
			  == OMAP_DMA_CHAIN_ACTIVE))
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR1) | RRST);

	return 0;
}

int omap2_mcbsp_send_data(unsigned int id, void *cbdata,
			  dma_addr_t buf_start_addr, u32 buf_size)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int e_count = 0;
	int f_count = 0;
	int ret = 0;

	mcbsp = mcbsp_ptr[id];
	io_base = mcbsp->io_base;
	mcbsp_wrapper[id].tx_cb_arg = cbdata;

	/*
	 * for skip_first and second, we need to set e_count =2, and
	 * f_count = number of frames = number of elements/e_count
	 */
	e_count = (buf_size / mcbsp->tx_word_length);
	if (mcbsp_wrapper[id].txskip_alt != OMAP_MCBSP_SKIP_NONE) {
		/*
		 * number of frames = total number of elements/element count,
		 * However, with double indexing for data transfers, double I
		 * the number of elements need to be transmitted
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}

	/*
	 * If the DMA is to be configured to skip the first byte, we need
	 * to jump backwards, so we need to move one chunk forward and ask
	 * dma if we dont want the client driver knowing abt this.
	 */
	if (mcbsp_wrapper[id].txskip_alt == OMAP_MCBSP_SKIP_FIRST)
		buf_start_addr += mcbsp->tx_word_length;

	ret = omap_dma_chain_a_transfer(mcbsp->dma_tx_lch,
					buf_start_addr,
					mcbsp->phys_base + OMAP_MCBSP_REG_DXR,
					e_count, f_count, mcbsp);
	if (ret < 0)
		return ret;

	if (mcbsp_wrapper[id].tx_dma_chain_state == 0) {
		if (mcbsp_wrapper[id].interface_mode == OMAP_MCBSP_MASTER)
			omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);
		ret = omap_start_dma_chain_transfers(mcbsp->dma_tx_lch);
		if (ret < 0)
			return ret;
		mcbsp_wrapper[id].tx_dma_chain_state = 1;
	}

	/* Auto XRST handling logic - Enable the Reciever after 1st dma */
	if ((enable_tx == 0) && (omap_dma_chain_status(mcbsp->dma_tx_lch)
			  == OMAP_DMA_CHAIN_ACTIVE)) {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
				 omap_mcbsp_read(io_base,
						 OMAP_MCBSP_REG_SPCR2) | XRST);
		enable_tx = 1;
	}

	return 0;
}

int omap2_mcbsp_params_cfg(unsigned int id, int interface_mode,
			   struct omap_mcbsp_cfg_param *rp,
			   struct omap_mcbsp_cfg_param *tp,
			   struct omap_mcbsp_srg_fsg_cfg *param)
{
	struct omap_mcbsp_reg_cfg mcbsp_cfg = {0};

	if (rp)
		omap2_mcbsp_set_recv_param(id, &mcbsp_cfg, rp);
	if (tp)
		omap2_mcbsp_set_trans_param(id, &mcbsp_cfg, tp);
	if (param)
		omap2_mcbsp_set_srg_cfg_param(id,
					      interface_mode, &mcbsp_cfg,
					      param);
	omap_mcbsp_config(id, &mcbsp_cfg);

	return 0;
}
#endif /* MCBSP_WRAPPER */

static char *getstring(int flag)
{
	if (flag)
		return "true";
	return "false";
}

static void dump_platform_config(void)
{
	printk(KERN_DEBUG "--------Audio Platform Config-------\n"
			  "Analog Downlink: %s\n"
			  "Independent BT bus: %s\n"
			  "I2S BP: %s\n"
			  "19Mhz BP: %s\n"
			  "Mic3: %s\n"
			  "Stereo Loudspeaker: %s\n",
			  getstring(cpcap_audio_has_analog_downlink()),
			  getstring(cpcap_audio_has_independent_bt()),
			  getstring(cpcap_audio_has_i2s_bp()),
			  getstring(cpcap_audio_has_19mhz_bp()),
			  getstring(cpcap_audio_has_mic3()),
			  getstring(cpcap_audio_has_stereo_loudspeaker()));

}

static void set_codec_mode(void)
{
	if ((cpcap_audio_state.dai_config ==
			CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) &&
			  cpcap_audio_has_independent_bt() &&
		(primary_spkr_setting == CPCAP_AUDIO_OUT_BT_MONO)
		&& (secondary_spkr_setting == CPCAP_AUDIO_OUT_NONE)) {
		cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_OFF;
		gpio_direction_output(GPIO_AUDIO_SELECT_CPCAP, 0);
	} else if ((primary_spkr_setting == CPCAP_AUDIO_OUT_BT_MONO)
		&& (secondary_spkr_setting == CPCAP_AUDIO_OUT_NONE)) {
		cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_CLOCK_ONLY;
		cpcap_audio_state.codec_mute = CPCAP_AUDIO_CODEC_MUTE;;
		if (cpcap_audio_has_independent_bt())
			gpio_direction_output(GPIO_AUDIO_SELECT_CPCAP, 1);
	} else {
		cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_ON;
		if (cpcap_audio_has_independent_bt())
			gpio_direction_output(GPIO_AUDIO_SELECT_CPCAP, 1);
	}
}

static void map_audioic_speakers(void)
{
	if (state.stdac_out_stream != NULL ||
			cpcap_audio_state.dai_config ==
					CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0) {
		cpcap_audio_state.stdac_primary_speaker = primary_spkr_setting;
		cpcap_audio_state.stdac_secondary_speaker =
						secondary_spkr_setting;
	} else {
		cpcap_audio_state.stdac_primary_speaker =
				CPCAP_AUDIO_OUT_NONE;
		cpcap_audio_state.stdac_secondary_speaker =
				CPCAP_AUDIO_OUT_NONE;
	}

	if (cpcap_audio_state.dai_config == CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) {
		if (cpcap_audio_has_analog_downlink()) {
			cpcap_audio_state.ext_primary_speaker =
							primary_spkr_setting;
			cpcap_audio_state.ext_secondary_speaker =
							secondary_spkr_setting;
			cpcap_audio_state.analog_source =
						CPCAP_AUDIO_ANALOG_SOURCE_L;
		} else {
			cpcap_audio_state.codec_primary_speaker =
							primary_spkr_setting;
			cpcap_audio_state.codec_secondary_speaker =
							secondary_spkr_setting;
		}
	} else if (cpcap_audio_state.dai_config ==
				CPCAP_AUDIO_DAI_CONFIG_NORMAL &&
				(state.codec_out_stream != NULL &&
				state.codec_out_stream->active == 1)) {
		cpcap_audio_state.codec_primary_speaker = primary_spkr_setting;
		cpcap_audio_state.codec_secondary_speaker =
							secondary_spkr_setting;
	} else {
		cpcap_audio_state.codec_primary_speaker =
				CPCAP_AUDIO_OUT_NONE;
		cpcap_audio_state.codec_secondary_speaker =
				CPCAP_AUDIO_OUT_NONE;
	}

	if (state.dev_dsp1_open_count > 0)
		set_codec_mode();
}

static int audio_select_speakers(int spkr)
{
	int local_spkr = -1;
	int spkr1 = CPCAP_AUDIO_OUT_NONE;
	int spkr2 = CPCAP_AUDIO_OUT_NONE;

	AUDIO_LEVEL3_LOG("[%s] enter with spkr = %d\n", __func__, spkr);

	while (spkr) {
		if ((spkr & CPCAP_AUDIO_OUT_STEREO_HEADSET) ==
						 CPCAP_AUDIO_OUT_STEREO_HEADSET)
			local_spkr = CPCAP_AUDIO_OUT_STEREO_HEADSET;

		else if ((spkr & CPCAP_AUDIO_OUT_MONO_HEADSET) ==
						 CPCAP_AUDIO_OUT_MONO_HEADSET)
			local_spkr = CPCAP_AUDIO_OUT_MONO_HEADSET;

		else if ((spkr & CPCAP_AUDIO_OUT_BT_MONO) ==
						 CPCAP_AUDIO_OUT_BT_MONO)
			local_spkr = CPCAP_AUDIO_OUT_BT_MONO;

		else if ((spkr & CPCAP_AUDIO_OUT_HANDSET) ==
							CPCAP_AUDIO_OUT_HANDSET)
			local_spkr = CPCAP_AUDIO_OUT_HANDSET;

		else if ((spkr & CPCAP_AUDIO_OUT_LOUDSPEAKER) ==
						CPCAP_AUDIO_OUT_LOUDSPEAKER)
			local_spkr = CPCAP_AUDIO_OUT_LOUDSPEAKER;

		else if ((spkr & CPCAP_AUDIO_OUT_LINEOUT) ==
							CPCAP_AUDIO_OUT_LINEOUT)
			local_spkr = CPCAP_AUDIO_OUT_LINEOUT;

		else if ((spkr & CPCAP_AUDIO_OUT_AUX_I2S) ==
						CPCAP_AUDIO_OUT_AUX_I2S)
			break;

		else if (local_spkr == -1 && spkr1 == CPCAP_AUDIO_OUT_NONE)
			return -EINVAL;

		if (spkr1 == CPCAP_AUDIO_OUT_NONE)
			spkr1 = local_spkr;
		else
			if (local_spkr != -1)
				spkr2 = local_spkr;

		spkr &= ~local_spkr;
		local_spkr = -1;
	}

	AUDIO_LEVEL1_LOG("spkr1 = %#x, spkr2 = %#x\n", spkr1, spkr2);

	if (spkr1 != primary_spkr_setting || spkr2 != secondary_spkr_setting) {
		primary_spkr_setting = spkr1;
		secondary_spkr_setting = spkr2;
		map_audioic_speakers();
		cpcap_audio_state.output_gain = 0;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
	}

	return 0;
}

static int audio_hw_transfer(struct audio_stream *str,
			     void *buffer_phy, u32 size, struct inode *inode)
{
	int ret = 0;
	int minor = MINOR(inode->i_rdev);
	int ssi = (minor == state.dev_dsp) ? STDAC_SSI : CODEC_SSI;

	AUDIO_LEVEL3_LOG("[%s] enter\n", __func__);

	if (unlikely(str == NULL || buffer_phy == NULL)) {
		AUDIO_ERROR_LOG("Stream/phy_buf NULL!!\n");
		return -EPERM;
	}

	if (str->input_output == FMODE_READ) {
		AUDIO_LEVEL3_LOG("RX-%d", size);
		TRY(ret = omap2_mcbsp_receive_data(ssi, str,
				(dma_addr_t) buffer_phy, size))
	} else {
		AUDIO_LEVEL3_LOG("TX-%d\n", size);
		ret = omap2_mcbsp_send_data(ssi, str, (dma_addr_t) buffer_phy,
					size);
	}

out:
	return ret;
}

static int audio_setup_buf(struct audio_stream *str, struct inode *inode)
{
	int frag;
	int bufsize = 0;
	char *bufbuf = NULL;
	dma_addr_t bufphys = 0;

	if (str == NULL) {
		AUDIO_ERROR_LOG("Stream not allocated\n");
		return -EPERM;
	}

	str->buffers = kmalloc(sizeof(struct audio_buf) * str->nbfrags,
			       GFP_KERNEL);

	if (!str->buffers) {
		AUDIO_ERROR_LOG("Error allocating buffers\n");
		goto out;
	}

	memset(str->buffers, 0, sizeof(struct audio_buf) * str->nbfrags);

	for (frag = 0; frag < str->nbfrags; frag++) {
		struct audio_buf *b = &str->buffers[frag];

		/*
		 * Let's allocate non-cached memory for DMA buffers.
		 * We try to allocate all memory at once.
		 * If this fails (a common reason is memory fragmentation),
		 * then we allocate more smaller buffers.
		 */
		if (!bufsize) {
			bufsize = (str->nbfrags - frag) * str->fragsize;

			do {
				bufbuf =
				    dma_alloc_coherent(NULL, bufsize, &bufphys,
							GFP_KERNEL | GFP_DMA);

				if (!bufbuf)
					bufsize -= str->fragsize;
			} while (!bufbuf && bufsize);

			if (!bufbuf)
				goto out;

			b->master = bufsize;
			memset(bufbuf, 0, bufsize);
		}

		b->data = bufbuf;
		b->buf_addr = bufphys;
		bufbuf += str->fragsize;
		bufphys += str->fragsize;
		bufsize -= str->fragsize;
	}

	str->bytecount = 0;
	str->fragcount = 0;
	sema_init(&str->sem, str->nbfrags);
	return 0;

out:
	audio_discard_buf(str, inode);
	return -ENOMEM;
}

static void audio_discard_buf(struct audio_stream *str, struct inode *inode)
{
	/* ensure DMA isn't using those buffers */
	audio_buffer_reset(str, inode);

	if (str->buffers) {
		int frag;
		for (frag = 0; frag < str->nbfrags; frag++) {
			if (!str->buffers[frag].master)
				continue;

			dma_free_coherent(NULL, str->buffers[frag].master,
					  str->buffers[frag].data,
					  str->buffers[frag].buf_addr);
		}

		kfree(str->buffers);
		str->buffers = NULL;
	}
}

static int audio_process_buf(struct audio_stream *str, struct inode *inode)
{
	int ret = 0;
	unsigned long flags;

	if (str == NULL) {
		AUDIO_ERROR_LOG("Invalid stream parameter\n");
		ret = -EPERM;
		goto out;
	}

	if (str->input_output == FMODE_READ) {
		struct audio_buf *b = &str->buffers[str->buf_head];

		if (str->fragsize) {
			ret = audio_hw_transfer(str, (void *)(b->buf_addr),
					      str->fragsize, str->inode);

			if (!ret) {
				b->buf_ref++;
				AUDIO_LEVEL2_LOG
				    ("b->buf_ref = %d, buf->head = %d\n",
				     b->buf_ref, str->buf_head);
			}
		}

		if (++str->buf_head >= str->nbfrags)
			str->buf_head = 0;
	} else {
		spin_lock_irqsave(&audio_write_lock, flags);
		if (str->in_use) {
			spin_unlock_irqrestore(&audio_write_lock, flags);
			return ret;
		}
		str->in_use = 1;
		spin_unlock_irqrestore(&audio_write_lock, flags);

		while (str->pending_frags) {
			struct audio_buf *b = &str->buffers[str->buf_head];
			u32 buf_size = str->fragsize - b->offset;

			AUDIO_LEVEL3_LOG
			    ("buf_size=%d, fragsize=%d, offset=%d\n",
			     buf_size, str->fragsize, b->offset);

			if (buf_size) {
				ret = audio_hw_transfer(str,
					(void *)(b->buf_addr + b->offset),
						buf_size, str->inode);
			}

			if (ret)
				goto out;

			b->buf_ref++;
			b->offset += buf_size;

			if (b->offset >= str->fragsize) {
				str->pending_frags--;
				if (++str->buf_head >= str->nbfrags)
					str->buf_head = 0;
			}
		}
out:
		spin_lock_irqsave(&audio_write_lock, flags);
		str->in_use = 0;
		spin_unlock_irqrestore(&audio_write_lock, flags);
	}

	return ret;
}

static void audio_buffer_reset(struct audio_stream *str, struct inode *inode)
{
	int frag;

	if (str->buffers) {
		for (frag = 0; frag < str->nbfrags; frag++) {
			struct audio_buf *b = &str->buffers[frag];
			b->offset = 0;
			b->buf_ref = 0;
		}
	}

	str->active = 0;
	str->stopped = 0;
	str->fragcount = 0;
	str->bytecount = 0;
	str->buf_tail = 0;
	str->buf_head = 0;
	str->usr_head = 0;
	str->fragsize = 0;
	str->in_use   = 0;
}

static void mcbsp_dma_tx_cb(u32 ch_status, void *arg)
{
	struct audio_stream *str;
	struct audio_buf *b;

	if (unlikely(!arg)) {
		AUDIO_ERROR_LOG("No Stream information!!\n");
		return;
	}

	str = (struct audio_stream *)arg;
	b = &str->buffers[str->buf_tail];

	if (ch_status) {
		AUDIO_ERROR_LOG("Error happend[%d 0x%x]!!\n", ch_status,
				ch_status);
	}

	/* Try to fill again */
	if (!str->buffers) {
		AUDIO_ERROR_LOG("received DMA IRQ for non "
				"existent buffers!\n");
		return;
	} else if (b->buf_ref && --b->buf_ref == 0
		   && b->offset >= str->fragsize) {
		/* This fragment is done */
		b->offset = 0;
		str->bytecount += str->fragsize;
		str->fragcount++;

		if (++str->buf_tail >= str->nbfrags)
			str->buf_tail = 0;

		up(&str->sem);

		audio_process_buf(str, str->inode);
	}
}

static void mcbsp_dma_rx_cb(u32 ch_status, void *arg)
{
	struct audio_stream *str;
	struct audio_buf *b;

	if (unlikely(!arg)) {
		AUDIO_ERROR_LOG("No Stream information!!\n");
		return;
	}

	str = (struct audio_stream *)arg;

	b = &str->buffers[str->buf_tail];

	if (ch_status) {
		AUDIO_ERROR_LOG("Error happend[%d 0x%x]!!\n", ch_status,
				ch_status);
	}

	AUDIO_LEVEL2_LOG("b->buf_ref = %d, str->buf_tail = %d full!!\n",
			 b->buf_ref, str->buf_tail);

	/* Try to fill again */
	if (!str->buffers) {
		AUDIO_ERROR_LOG("received DMA IRQ for "
				"non existent buffers!\n");
		return;
	} else if (b->buf_ref && --b->buf_ref == 0) {
		if (++str->buf_tail >= str->nbfrags)
			str->buf_tail = 0;

		if (++read_buf_full >= str->nbfrags) {
			read_buf_full = 0;
			read_buf_outstanding += str->nbfrags;
			AUDIO_LEVEL2_LOG("%d %s read buf "
				"overwrite read_buf_outstanding=%d\n",
				__LINE__, __func__, read_buf_outstanding);
		}

		audio_process_buf(str, str->inode);

		wake_up_interruptible(&str->wq);
	} else {
		AUDIO_LEVEL2_LOG("b->buf_ref = %d\n", b->buf_ref);
	}
}

static int audio_configure_ssi(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	unsigned int ssi;
	int mode =  file->f_flags & O_ACCMODE;

	if (minor == state.dev_dsp) {	/* STDAC setting */
		tx_cfg_params.word_length1 = OMAP_MCBSP_WORD_32;
		tx_params.word_length1 = OMAP_MCBSP_WORD_32;
		ssi = STDAC_SSI;
		/*fs_polarity made low to fix L/R swap issue*/
		tx_cfg_params.fs_polarity  = OMAP_MCBSP_FS_ACTIVE_HIGH;
		omap_ctrl_writel(omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0) |
					(1 << OMAP2_CONTROL_DEVCONF0_BIT6),
						OMAP2_CONTROL_DEVCONF0);
	} else {		/* CODEC setting */
		ssi = CODEC_SSI;
		tx_cfg_params.fs_polarity  = OMAP_MCBSP_FS_ACTIVE_HIGH;

		/*TX (playback) config*/
		if (mode == O_RDWR &&  cpcap_audio_state.dai_config ==
					CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0) {
			tx_cfg_params.word_length1 = OMAP_MCBSP_WORD_32;
			tx_params.word_length1 = OMAP_MCBSP_WORD_32;
		} else {
			tx_cfg_params.word_length1 = OMAP_MCBSP_WORD_16;
			tx_params.word_length1 = OMAP_MCBSP_WORD_16;
		}

		/*RX (capture) config*/
			if (capture_channels <= 1) { /* mono capture */
				rx_cfg_params.word_length1 = OMAP_MCBSP_WORD_16;
				rx_params.word_length1 = OMAP_MCBSP_WORD_16;
				rx_cfg_params.phase =
						OMAP_MCBSP_FRAME_SINGLEPHASE;
			} else if (capture_channels == 2) {
				rx_cfg_params.word_length1 = OMAP_MCBSP_WORD_32;
				rx_params.word_length1 = OMAP_MCBSP_WORD_32;
				rx_cfg_params.phase =
						OMAP_MCBSP_FRAME_SINGLEPHASE;
			} else if (capture_channels == 3) {
				rx_cfg_params.word_length1 = OMAP_MCBSP_WORD_32;
				rx_params.word_length1 = OMAP_MCBSP_WORD_32;
				rx_cfg_params.word_length2 = OMAP_MCBSP_WORD_16;
				rx_params.word_length2 = OMAP_MCBSP_WORD_16;
				rx_cfg_params.phase =
						OMAP_MCBSP_FRAME_DUALPHASE;
			} else if (capture_channels == 4) {
				rx_cfg_params.word_length1 = OMAP_MCBSP_WORD_32;
				rx_params.word_length1 = OMAP_MCBSP_WORD_32;
				rx_cfg_params.word_length2 = OMAP_MCBSP_WORD_32;
				rx_params.word_length2 = OMAP_MCBSP_WORD_32;
				rx_cfg_params.phase =
						OMAP_MCBSP_FRAME_DUALPHASE;
			} else {
				AUDIO_ERROR_LOG("[%d] Unsupported config!\n",
						capture_channels);
			}
	}

	if (ssi == STDAC_SSI ||
	   (ssi == CODEC_SSI && state.codec_ssi_started == 0)) {
		TRY(omap_mcbsp_set_io_type(ssi, 0))
#ifdef CONFIG_WAKELOCK
		wake_lock(&mcbsp_wakelock);
#endif
		TRY(omap_mcbsp_request(ssi))
		TRY(omap2_mcbsp_reset(ssi))
		if (ssi == STDAC_SSI)
			state.stdac_ssi_started = 1;
		else
			state.codec_ssi_started = 1;

		TRY(omap2_mcbsp_params_cfg(ssi, OMAP_MCBSP_SLAVE,
			&rx_cfg_params, &tx_cfg_params, &srg_fsg_params))


		TRY(omap2_mcbsp_dma_trans_params(ssi, &tx_params))

		omap2_mcbsp_dma_recv_params(ssi, &rx_params);
	}


	return 0 ;

out:
	omap_mcbsp_free(ssi);
#ifdef CONFIG_WAKELOCK
	wake_unlock(&mcbsp_wakelock);
#endif
	return -EPERM ;
}

int audio_stop_ssi(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	int ssi;
	int mode = file->f_flags & O_ACCMODE;

	ssi = (minor == state.dev_dsp) ? STDAC_SSI : CODEC_SSI;

		TRY(omap2_mcbsp_set_xrst(ssi, OMAP_MCBSP_XRST_DISABLE))
		TRY(omap2_mcbsp_stop_datatx(ssi))
		enable_tx = 0;
		omap_ctrl_writel(omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0) &
					~(1 << OMAP2_CONTROL_DEVCONF0_BIT6),
						OMAP2_CONTROL_DEVCONF0);

		TRY(omap2_mcbsp_set_rrst(ssi, OMAP_MCBSP_RRST_DISABLE))
		TRY(omap2_mcbsp_stop_datarx(ssi))

	if (ssi == STDAC_SSI ||
		(ssi == CODEC_SSI && state.codec_ssi_started == 1)) {
		if (ssi == STDAC_SSI)
			state.stdac_ssi_started = 0;
		else
			state.codec_ssi_started = 0;
		(void)omap2_mcbsp_reset(ssi);
		(void)omap_mcbsp_free(ssi);
#ifdef CONFIG_WAKELOCK
		wake_unlock(&mcbsp_wakelock);
#endif
	}

	return 0;

out:
	return -EPERM;
}

static int audio_stdac_open(struct inode *inode, struct file *file)
{
	int error = 0;
	int mode = file->f_flags & O_ACCMODE;
	mutex_lock(&audio_lock);

	if (state.dev_dsp_open_count == 1) {
		error = -EBUSY;
		goto out;
	}

	state.dev_dsp_open_count = 1;
	file->private_data = inode;

	if (mode == O_WRONLY || mode == O_RDWR) {
		state.stdac_out_stream =
		    kmalloc(sizeof(struct audio_stream), GFP_KERNEL);
		memset(state.stdac_out_stream, 0,
		       sizeof(struct audio_stream));
		state.stdac_out_stream->inode = inode;
		audio_buffer_reset(state.stdac_out_stream, inode);
		TRY(error = audio_configure_ssi(inode, file))
		cpcap_audio_state.stdac_mode = CPCAP_AUDIO_STDAC_ON;
		map_audioic_speakers();
		cpcap_audio_set_audio_state(&cpcap_audio_state);
	}
out:
	mutex_unlock(&audio_lock);
	return error;
}

static int audio_stdac_release(struct inode *inode, struct file *file)
{
	int mode = file->f_flags & O_ACCMODE;
	mutex_lock(&audio_lock);
	state.dev_dsp_open_count = 0;

	if (mode == O_WRONLY || mode == O_RDWR) {
		audio_stop_ssi(inode, file);
		audio_discard_buf(state.stdac_out_stream, inode);
		kfree(state.stdac_out_stream);
		state.stdac_out_stream = NULL;

		cpcap_audio_state.stdac_mode = CPCAP_AUDIO_STDAC_OFF;
		cpcap_audio_state.stdac_mute = CPCAP_AUDIO_STDAC_MUTE;
		cpcap_audio_state.stdac_primary_speaker = CPCAP_AUDIO_OUT_NONE;
		cpcap_audio_state.stdac_secondary_speaker =
							CPCAP_AUDIO_OUT_NONE;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
	}
	/*clear stdac settings here -- clear DUAL & DUPLEX_0 in codec_release*/
	if (cpcap_audio_state.dai_config ==
			CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_1)
		cpcap_audio_state.dai_config = CPCAP_AUDIO_DAI_CONFIG_NORMAL;

	mutex_unlock(&audio_lock);

	return 0;
}

static int audio_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int minor = MINOR(inode->i_rdev);
	int ret = 0;
	int mode = file->f_flags & O_ACCMODE;

	mutex_lock(&audio_lock);

	switch (cmd) {
	case OSS_GETVERSION:
		ret = put_user(SOUND_VERSION, (int *)arg);
		break;

	case SNDCTL_DSP_SPEED:
	{
		unsigned int samp_rate;
		int count = 0;
		TRY(copy_from_user(&samp_rate, (unsigned int *)arg,
			     sizeof(unsigned int)))

		/* validate if rate is proper */
		for (; count < NUMBER_OF_RATES_SUPPORTED; count++) {
			if (valid_sample_rates[count].rate == samp_rate)
				break;
		}

		if (count >= NUMBER_OF_RATES_SUPPORTED) {
			AUDIO_ERROR_LOG("[%d] Unsupported sample rate!!\n",
				     (u32) samp_rate);
			ret = -EINVAL;
			goto out;
		}

		if (cpcap_audio_state.dai_config !=
			CPCAP_AUDIO_DAI_CONFIG_NORMAL &&
			cpcap_audio_state.dai_config !=
				CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) {
			/* in linked DAI modes, codec and stdac are sync'ed */
			cpcap_audio_state.stdac_rate =
				valid_sample_rates[count].cpcap_audio_rate;
			cpcap_audio_state.codec_rate =
				valid_sample_rates[count].cpcap_audio_rate;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
			break; /*goto out?*/
		}

		if (minor == state.dev_dsp) {
			if (samp_rate != cpcap_audio_state.stdac_rate) {
				cpcap_audio_state.stdac_rate =
				valid_sample_rates[count].cpcap_audio_rate;
				cpcap_audio_set_audio_state(&cpcap_audio_state);
			}
		} else if (samp_rate != cpcap_audio_state.codec_rate) {
			if ((mode == O_WRONLY || mode == O_RDWR) &&
				(samp_rate != 8000 && samp_rate != 16000)) {
				AUDIO_ERROR_LOG("[%d] Unsupported"
						" Codec sample rate!!\n",
						(u32) samp_rate);
				ret = -EINVAL;
				goto out;
			}
			cpcap_audio_state.codec_rate =
				valid_sample_rates[count].cpcap_audio_rate;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
		}
		break;
	}

	case SNDCTL_DSP_POST:
		break;

	case SNDCTL_DSP_CHANNELS:
	{
		int val;

		TRY(copy_from_user(&val, (int *)arg, sizeof(int)))

		if (minor == state.dev_dsp) {
			if (val != 2) {
				/*only stereo is supported for McBSP2:
				  I2S mode implies two channels */
				ret = -EINVAL;
				goto out;
			}
		} else {	/* Codec case */
			if (val == capture_channels)
				goto out;
			if (val == 2 && mode == O_RDWR 	&&
					state.dev_dsp_open_count == 0) {
				/*SPECIAL CASE codec->stdac loopback*/
				state.dev_dsp_open_count = 1;
				capture_channels = val;
				cpcap_audio_state.dai_config =
					CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0;
				cpcap_audio_state.stdac_mode =
						CPCAP_AUDIO_STDAC_ON;
				if (state.codec_ssi_started == 1)
					TRY(audio_stop_ssi(inode, file))
				if (state.codec_ssi_started == 0)
					TRY(audio_configure_ssi(inode, file))
				map_audioic_speakers();
				cpcap_audio_set_audio_state(&cpcap_audio_state);
			} else if (mode == O_WRONLY ||  mode == O_RDWR) {
				/*only mono output can be used on McBSP3*/
				if (val > 1) {
					ret = -EINVAL;
					goto out;
				}
			} else { /*up to 4-channel capture on McBSP3*/
				if (val <= 4) {
					capture_channels = val;
					if (state.codec_ssi_started == 1)
						TRY(audio_stop_ssi(inode,
								file))
					if (state.codec_ssi_started == 0)
						TRY(audio_configure_ssi(inode,
								file))
				} else {
					ret = -EINVAL;
					goto out;
				}
			}
		}
		break;
	}

	case SNDCTL_DSP_SYNC:
		break;

	case SNDCTL_DSP_GETBLKSIZE:
	{
		int val = 0;
		if (mode == O_WRONLY) {
			val = (minor == state.dev_dsp) ? STDAC_FIFO_SIZE :
				CODEC_FIFO_SIZE;
		} else {
			/* McBSP/DMA driver returns blank data
			 * if any other size other than
			 * 800 is used for capture */
			val = AUDIO_CAPTURE_SIZE;
		}
		put_user(val, (int *)arg);
		break;
	}

	case SNDCTL_DSP_GETOSPACE:
	{
		audio_buf_info inf = { 0 };
		struct audio_stream *str;
		if (minor == state.dev_dsp)
			str = state.stdac_out_stream;
		else
			str = state.codec_out_stream;

		if  (str != NULL) {
			inf.bytes = str->fragsize;
			inf.fragments = 1;
			inf.fragsize = str->fragsize;
			inf.fragstotal = str->nbfrags;
		}
		ret = copy_to_user((void *)arg, &inf, sizeof(inf));
		break;
	}

	case SNDCTL_DSP_GETISPACE:
	{
		audio_buf_info inf = { 0 };
		struct audio_stream *str;
		if (minor == state.dev_dsp) {
			str = state.stdac_in_stream;
		} else {
			str = state.codec_in_stream;
		}

		if (str != NULL) {
			inf.bytes = str->fragsize;
			inf.fragments = 1;
			inf.fragsize = str->fragsize;
			inf.fragstotal = str->nbfrags;
		}
		ret = copy_to_user((void *)arg, &inf, sizeof(inf));
		break;
	}

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		break;

	case SNDCTL_DSP_RESET:
	{
		struct audio_stream *str1 = NULL, *str2 = NULL;
		int ssi;
		if (minor == state.dev_dsp) {
			if (mode == O_WRONLY) {
				str1 = state.stdac_out_stream;
			} else if (mode == O_RDONLY) {
				str2 = state.stdac_in_stream;
			} else {
				str1 = state.stdac_in_stream;
				str2 = state.stdac_out_stream;
			}
			ssi = STDAC_SSI;
		} else {
			if (mode == O_WRONLY) {
				str1 = state.codec_out_stream;
			} else if (mode == O_RDONLY) {
				str2 = state.codec_in_stream;
			} else {
				str1 = state.codec_in_stream;
				str2 = state.codec_out_stream;
			}
			ssi = CODEC_SSI;
		}
		if (mode == O_WRONLY || mode == O_RDWR) {
			TRY(omap2_mcbsp_set_xrst(ssi, OMAP_MCBSP_XRST_DISABLE))
			TRY(omap2_mcbsp_set_xrst(ssi, OMAP_MCBSP_XRST_ENABLE))
		}
		if (mode == O_RDONLY || mode == O_RDWR) {
			TRY(omap2_mcbsp_set_rrst(ssi, OMAP_MCBSP_RRST_DISABLE))
			TRY(omap2_mcbsp_set_rrst(ssi, OMAP_MCBSP_RRST_ENABLE))
		}
		if (str1 != NULL)
			audio_buffer_reset(str1, inode);
		if (str2 != NULL)
			audio_buffer_reset(str2, inode);
		break;
	}

	case SNDCTL_DSP_GETOPTR:
	{
		int bytes_left_in_kernel = 0;

		struct audio_stream *str = (minor == state.dev_dsp) ?
			state.stdac_out_stream : state.codec_out_stream;

		if ((str != NULL) && (str->nbfrags != 0)) {
			bytes_left_in_kernel = ((str->nbfrags - str->buf_tail +
			str->usr_head - 1) % str->nbfrags) * str->fragsize;
		}

		TRY(put_user(bytes_left_in_kernel, (int *)arg))
		break;
	}

		/* MIXER ioctls */
	case SOUND_MIXER_OUTSRC:
	{
		int spkr;
		TRY(copy_from_user(&spkr, (int *)arg, sizeof(int)))
		AUDIO_LEVEL2_LOG("SOUND_MIXER_OUTSRC with spkr = %#x\n", spkr);
		ret = audio_select_speakers(spkr);
		break;
	}

	case SOUND_MIXER_RECSRC:
	{
		int mic;
		TRY(copy_from_user(&mic, (int *)arg, sizeof(int)))
		AUDIO_LEVEL2_LOG("SOUND_MIXER_RECSRC with mic = %#x\n", mic);
		if (mic != mic_setting) {
			if (state.codec_in_stream->active == 1 ||
					cpcap_audio_state.dai_config !=
						CPCAP_AUDIO_DAI_CONFIG_NORMAL) {
				cpcap_audio_state.microphone = mic;
				cpcap_audio_state.input_gain_l = 0;
				cpcap_audio_state.input_gain_r = 0;
				cpcap_audio_set_audio_state(&cpcap_audio_state);
			}
			mic_setting = mic;
		}
		break;
	}

	case SOUND_MIXER_VOLUME:
	{
		unsigned int gain;
		TRY(copy_from_user(&gain, (unsigned int *)arg,
					sizeof(unsigned int)))

		if (gain == 0) {
			cpcap_audio_state.stdac_mute = CPCAP_AUDIO_STDAC_MUTE;
			cpcap_audio_state.codec_mute = CPCAP_AUDIO_CODEC_MUTE;
		} else {
			/* unmute codec or stereo DAC */
			if (cpcap_audio_state.stdac_mode == CPCAP_AUDIO_STDAC_ON)
				cpcap_audio_state.stdac_mute =
						CPCAP_AUDIO_STDAC_UNMUTE;

			if (cpcap_audio_state.codec_mode ==
							CPCAP_AUDIO_CODEC_ON &&
				(cpcap_audio_state.dai_config !=
					CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL ||
					!cpcap_audio_has_analog_downlink()))
				cpcap_audio_state.codec_mute =
						CPCAP_AUDIO_CODEC_UNMUTE;
		}

		cpcap_audio_state.output_gain = gain;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		AUDIO_LEVEL2_LOG("SOUND_MIXER_VOLUME, output_gain = %d\n",
				cpcap_audio_state.output_gain);
		break;
	}

	case SOUND_MIXER_RECLEV:
	{
		unsigned int gain;
		TRY(copy_from_user(&gain, (unsigned int *)arg,
						sizeof(unsigned int)))
		cpcap_audio_state.input_gain_r = gain & 0xFF;
		cpcap_audio_state.input_gain_l = (gain & 0xFF00) >> 8;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		AUDIO_LEVEL2_LOG("SOUND_MIXER_RECLEV, input_gain_l = %u, "
			"input_gain_r = %u\n",
			cpcap_audio_state.input_gain_l,
			cpcap_audio_state.input_gain_r);
		break;
	}

	case SOUND_MIXER_FMPATH:
	{
		unsigned int spkr;
		TRY(copy_from_user(&spkr, (int *)arg, sizeof(spkr)))
		AUDIO_LEVEL2_LOG("SOUND_MIXER_FMPATH with spkr = %#x\n", spkr);
		cpcap_audio_state.ext_primary_speaker = (spkr & 0xFFFF);
		if (cpcap_audio_state.ext_primary_speaker !=
				CPCAP_AUDIO_OUT_LOUDSPEAKER)
			cpcap_audio_state.ext_primary_speaker =
					CPCAP_AUDIO_OUT_STEREO_HEADSET;

		cpcap_audio_state.fm_output_gain = (spkr >> 16) & 0xFF;
		if (cpcap_audio_state.fm_output_gain == 0) {
			cpcap_audio_state.analog_source =
						CPCAP_AUDIO_ANALOG_SOURCE_OFF;
		} else if (state.fm_on == 1) {
			cpcap_audio_state.analog_source =
					CPCAP_AUDIO_ANALOG_SOURCE_STEREO;
		}
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		break;
	}

	case SOUND_MIXER_FMON:
	{
		AUDIO_LEVEL2_LOG("SOUND_MIXER_FMON\n");
		if (cpcap_audio_has_analog_downlink())
			cpcap_regacc_write(cpcap_audio_state.cpcap,
				CPCAP_REG_GPIO1,
				CPCAP_BIT_GPIO1DRV, CPCAP_BIT_GPIO1DRV);

		cpcap_audio_state.analog_source =
					CPCAP_AUDIO_ANALOG_SOURCE_STEREO;
		state.fm_on = 1;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		break;
	}

	case SOUND_MIXER_FMOFF:
	{
		AUDIO_LEVEL2_LOG("SOUND_MIXER_FMOFF\n");
		if (cpcap_audio_has_analog_downlink())
			cpcap_regacc_write(cpcap_audio_state.cpcap,
				CPCAP_REG_GPIO1,
				0, CPCAP_BIT_GPIO1DRV);
		cpcap_audio_state.ext_primary_speaker = CPCAP_AUDIO_OUT_NONE;
		cpcap_audio_state.analog_source = CPCAP_AUDIO_ANALOG_SOURCE_OFF;
		cpcap_audio_state.fm_output_gain = 0;
		state.fm_on = 0;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		break;
	}

	case SOUND_MIXER_PRIVATE1:  /* Codec loopback mode */
	{
		AUDIO_LEVEL2_LOG("Audio IC loopback ioctl called\n");
		cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_LOOPBACK;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		break;
	}

	case SOUND_MIXER_PRIVATE2: /* Balance control */
	{
		unsigned int balance;
		TRY(copy_from_user(&balance, (unsigned int *)arg,
						sizeof(unsigned int)))
		cpcap_audio_state.stdac_primary_balance = balance;
		cpcap_audio_state.codec_primary_balance = balance;
		cpcap_audio_state.ext_primary_balance =  balance;
		cpcap_audio_set_audio_state(&cpcap_audio_state);
		break;
	}

	case SOUND_MIXER_PRIVATE3:
	{
		/* This operation returns the number of outstanding read
		   buffers.
		   It clears the number by assuming that the user monitors
		   how many buffers have lost since the last check.
		   If the user doesn't grab the current input buffer before
		   the driver fills all the buffers,
		   read_buf_outstanding is incremented by AUDIO_NBFRAGS_READ
		   to indicate the number of buffers lost.
		   a) before overwrite (read_buf_outstanding = 0)
			   v user_head
		   |1+++++|2-----|3-----|...|15-----|
		    ^ tail ^ head
		   b) after overwrite (buf3-15 and 1 are lost, user can read
		      only 2, read_buf_outstanding = 15)
			   v user_head
		   |1+++++|2+++++|3-----|...|15-----|
			   ^ tail ^ head
		*/
		unsigned int val = 0;
		if (file->f_mode & FMODE_WRITE || minor == state.dev_dsp) {
			val = 0;
		} else {
			val = read_buf_outstanding;
			/* outstanding number read. */
			AUDIO_LEVEL2_LOG("%d %s resetting "
				"read_buf_outstanding\n", __LINE__, __func__);
			read_buf_outstanding = 0;
		}
		AUDIO_LEVEL2_LOG("%d %s read_buf_outstanding=%d\n",
			 __LINE__, __func__, read_buf_outstanding);
		put_user(val, (int *)arg);
		break;
	}

	default:
		break;

	}
out:
	mutex_unlock(&audio_lock);
	return ret;
}

static ssize_t audio_write(struct file *file, const char *buffer, size_t count,
								loff_t *nouse)
{
	int chunksize, ret = 0;
	const char *buffer0 = buffer;
	struct inode *inode = (struct inode *)file->private_data;
	int minor = MINOR(inode->i_rdev);
	int mode =  file->f_flags & O_ACCMODE;
	struct audio_stream *str = (minor == state.dev_dsp) ?
			state.stdac_out_stream : state.codec_out_stream;

	mutex_lock(&audio_lock);

	if (str == NULL) {
		ret = -EPERM;
		goto out;
	}

	if (minor == state.dev_dsp) {
		if (!str->active) {
			int temp_size = count % STDAC_FIFO_SIZE;
			if (temp_size != 0)
				str->fragsize = (count - temp_size) + STDAC_FIFO_SIZE;
			else
				str->fragsize = count;
			str->nbfrags = AUDIO_NBFRAGS_WRITE;
			if (audio_setup_buf(str, file->private_data)) {
				AUDIO_ERROR_LOG("Unable to allocate memory\n");
				ret = -ENOMEM;
				goto out;
			}
			str->active = 1;
		}
	} else {
		if (!str->active) {
			int temp_size = count % CODEC_FIFO_SIZE;
			cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_ON;
			if (mode == O_WRONLY || mode == O_RDWR)
				audio_buffer_reset(state.codec_out_stream,
				inode);
			if (state.codec_ssi_started == 0)
				TRY(audio_configure_ssi(inode, file))
			if (temp_size != 0)
				str->fragsize = (count - temp_size) + CODEC_FIFO_SIZE;
			else
				str->fragsize = count;
			str->nbfrags = AUDIO_NBFRAGS_WRITE;
			if (audio_setup_buf(str, file->private_data)) {
				AUDIO_ERROR_LOG("Unable to allocate memory\n");
				ret = -ENOMEM;
				goto out;
			}
			str->active = 1;
			map_audioic_speakers();
			cpcap_audio_set_audio_state(&cpcap_audio_state);
		}
	}

	while (count > 0) {
		struct audio_buf *buf = &str->buffers[str->usr_head];

		/* Wait for a buffer to become free */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&str->sem))
				break;
		} else {
			mutex_unlock(&audio_lock);
			ret = down_timeout(&str->sem, AUDIO_TIMEOUT);
			mutex_lock(&audio_lock);
			if (ret) {
				AUDIO_ERROR_LOG("audio_write: timedout\n");
				break;
			}
		}

		chunksize = str->fragsize - buf->offset;

		if (chunksize > count)
			chunksize = count;

		if (copy_from_user(buf->data + buf->offset,
				buffer, chunksize)) {
			AUDIO_ERROR_LOG("Audio: CopyFrom User failed \n");
			up(&str->sem);
			ret = -EFAULT;
			goto out;
		}

		/* Workaround for CPCAP channel inversion issue */
		if (cpcap_audio_state.cpcap->revision == CPCAP_REVISION_2_1 &&
			cpcap_audio_state.cpcap->vendor == CPCAP_VENDOR_TI) {
			if (minor == state.dev_dsp &&
				(cpcap_audio_state.stdac_primary_speaker ==
					CPCAP_AUDIO_OUT_STEREO_HEADSET ||
				cpcap_audio_state.stdac_secondary_speaker ==
					CPCAP_AUDIO_OUT_STEREO_HEADSET)) {
				int lc;
				short *ptr = (short *)(buf->data + buf->offset);
				for (lc = 0; lc < chunksize / 2; lc += 2) {
					ptr[lc] = -ptr[lc];
					if (ptr[lc] == (short)0x8000)
						ptr[lc] = (short)0x7FFF;
				}
			}
		}

		buffer += chunksize;
		count -= chunksize;
		buf->offset += chunksize;

		if (buf->offset < str->fragsize) {
			up(&str->sem);
			break;
		}

		buf->offset = 0;

		if (++str->usr_head >= str->nbfrags)
			str->usr_head = 0;

		str->pending_frags++;

		ret = audio_process_buf(str, inode);
	}

	if (buffer - buffer0)
		ret = buffer - buffer0;

out:
	mutex_unlock(&audio_lock);
	return ret;
}

static int audio_codec_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	int mode =  file->f_flags & O_ACCMODE;
	mutex_lock(&audio_lock);

	if (state.dev_dsp1_open_count == 2) {
		ret = -EBUSY;
		goto out;
	}

	if (state.dev_dsp1_open_count == 1) {
		if (((file->f_flags & O_TRUNC) != 0) || (mode == O_RDWR) ||
			((mode == O_RDONLY) &&
				(cpcap_audio_state.microphone !=
					CPCAP_AUDIO_IN_NONE)) ||
			((mode == O_WRONLY) &&
				(cpcap_audio_state.codec_primary_speaker !=
					CPCAP_AUDIO_OUT_NONE))) {
			AUDIO_ERROR_LOG("codec_open- INVALID mode requested\n");
			ret = -EBUSY;
			goto out;
		}
	}

	/*turn the codec on for read and write in respective functions */
	if ((file->f_flags & O_TRUNC) != 0)
		cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_ON;

	file->private_data = inode;
	if (state.dev_dsp1_open_count == 0)
		ret = audio_codec_open_helper(inode, file);

	if (ret < 0)
		goto out;

	state.dev_dsp1_open_count++;

out:
	mutex_unlock(&audio_lock);
	return ret;
}

static int audio_codec_open_helper(struct inode *inode, struct file *file)
{
	file->private_data = inode;

	if (file->f_flags & O_TRUNC) {
		AUDIO_LEVEL1_LOG("CODEC in phone mode called \n");
		cpcap_audio_state.dai_config =
				CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL;
		cpcap_audio_state.output_gain = 0;
		cpcap_audio_state.codec_rate = CPCAP_AUDIO_CODEC_RATE_8000_HZ;

		if (cpcap_audio_has_analog_downlink()) {
			cpcap_audio_state.codec_mute = CPCAP_AUDIO_CODEC_MUTE;
			cpcap_audio_state.analog_source =
						CPCAP_AUDIO_ANALOG_SOURCE_L;
			cpcap_regacc_write(cpcap_audio_state.cpcap,
				CPCAP_REG_GPIO1, 0, CPCAP_BIT_GPIO1DRV);
		}

		if (primary_spkr_setting == CPCAP_AUDIO_OUT_LOUDSPEAKER) {
			if (cpcap_audio_has_analog_downlink())
				cpcap_audio_state.ext_primary_speaker =
							CPCAP_AUDIO_OUT_HANDSET;
			else
				cpcap_audio_state.codec_primary_speaker =
							CPCAP_AUDIO_OUT_HANDSET;

			primary_spkr_setting = CPCAP_AUDIO_OUT_HANDSET;

			if (cpcap_audio_has_mic3()) {
				cpcap_audio_state.microphone =
					CPCAP_AUDIO_IN_HANDSET |
					CPCAP_AUDIO_IN_TERTIARY_INTERNAL;
			} else {
				cpcap_audio_state.microphone =
					CPCAP_AUDIO_IN_HANDSET |
					CPCAP_AUDIO_IN_SECONDARY_INTERNAL;
			}
			mic_setting = cpcap_audio_state.microphone;
		} else {
			if (cpcap_audio_has_analog_downlink())
				cpcap_audio_state.ext_primary_speaker =
							primary_spkr_setting;
			else
				cpcap_audio_state.codec_primary_speaker =
							primary_spkr_setting;
			cpcap_audio_state.microphone = mic_setting;
		}
		cpcap_audio_set_audio_state(&cpcap_audio_state);
	} else {
		state.codec_out_stream =
			    kmalloc(sizeof(struct audio_stream), GFP_KERNEL);
		memset(state.codec_out_stream, 0, sizeof(struct audio_stream));
		state.codec_out_stream->inode = inode;

		state.codec_in_stream =
			kmalloc(sizeof(struct audio_stream), GFP_KERNEL);
		memset(state.codec_in_stream, 0, sizeof(struct audio_stream));
		state.codec_in_stream->inode = inode;
	}

	if (file->f_flags & O_TRUNC)
		set_codec_mode();
	return 0;
}

static int audio_codec_release(struct inode *inode, struct file *file)
{
	int mode =  file->f_flags & O_ACCMODE;
	mutex_lock(&audio_lock);

	state.dev_dsp1_open_count--;


	if (mode == O_WRONLY || mode == O_RDWR ||
		cpcap_audio_state.dai_config ==
				CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) {
		cpcap_audio_state.codec_primary_speaker = CPCAP_AUDIO_OUT_NONE;
		cpcap_audio_state.codec_secondary_speaker
							= CPCAP_AUDIO_OUT_NONE;
	}
	if (mode == O_RDONLY || mode == O_RDWR ||
		cpcap_audio_state.dai_config ==
				CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) {
		capture_channels = 1;
		read_buf_full = 0;
		read_buf_outstanding = 0;
		cpcap_audio_state.microphone = CPCAP_AUDIO_IN_NONE;
	}

	if (cpcap_audio_state.dai_config == CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) {
		if (cpcap_audio_has_analog_downlink()) {
			if (state.fm_on == 0) {
				cpcap_audio_state.ext_primary_speaker =
						CPCAP_AUDIO_OUT_NONE;
				cpcap_audio_state.ext_secondary_speaker =
						CPCAP_AUDIO_OUT_NONE;
				cpcap_audio_state.analog_source =
						CPCAP_AUDIO_ANALOG_SOURCE_OFF;
			} else {
				cpcap_regacc_write(cpcap_audio_state.cpcap,
					CPCAP_REG_GPIO1, CPCAP_BIT_GPIO1DRV,
					CPCAP_BIT_GPIO1DRV);

				cpcap_audio_state.analog_source =
					CPCAP_AUDIO_ANALOG_SOURCE_STEREO;
			}
		}
		if (cpcap_audio_has_independent_bt())
			gpio_direction_output(GPIO_AUDIO_SELECT_CPCAP, 1);
	}

	/* If writes switch to stdac while codec is open set enable_tx to 0 */
	if (mode == O_WRONLY &&
			(cpcap_audio_state.dai_config !=
					CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL))
		enable_tx = 0;

	/* stop ssi only if turning the codec off  and we started the ssi */
	if (state.dev_dsp1_open_count == 0) {
		if ((cpcap_audio_state.dai_config !=
				CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL)
			&& state.codec_ssi_started == 1)
			audio_stop_ssi(inode, file);

		audio_codec_release_helper(inode, file);
	}

	cpcap_audio_set_audio_state(&cpcap_audio_state);

	mutex_unlock(&audio_lock);
	return 0;
}
static int audio_codec_release_helper(struct inode *inode, struct file *file)
{
	cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_OFF;
	cpcap_audio_state.codec_mute = CPCAP_AUDIO_CODEC_MUTE;

	if (cpcap_audio_state.dai_config == CPCAP_AUDIO_DAI_CONFIG_VOICE_CALL) {
		AUDIO_LEVEL1_LOG("CODEC in phone mode released \n");
		cpcap_audio_state.dai_config = CPCAP_AUDIO_DAI_CONFIG_NORMAL;
	} else {
		if (state.codec_out_stream != NULL) {
			audio_discard_buf(state.codec_out_stream, inode);
			kfree(state.codec_out_stream);
			state.codec_out_stream = NULL;
		}

		if (state.codec_in_stream != NULL) {
			audio_discard_buf(state.codec_in_stream, inode);
			kfree(state.codec_in_stream);
			state.codec_in_stream = NULL;
		}
	}

	/*if we were using stdac on McBSP3, un-busy McBSP2*/
	if (cpcap_audio_state.dai_config ==
			CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0)
		state.dev_dsp_open_count = 0;

	/*clear codec changes here -- clear DUPLEX_1 in stdac_release*/
	if (cpcap_audio_state.dai_config == CPCAP_AUDIO_DAI_CONFIG_HIFI_DUAL ||
		cpcap_audio_state.dai_config ==
					CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0)
		cpcap_audio_state.dai_config = CPCAP_AUDIO_DAI_CONFIG_NORMAL;

	return 0;
}

static ssize_t audio_codec_read(struct file *file, char *buffer, size_t size,
								loff_t *nouse)
{
	struct audio_stream *str = state.codec_in_stream;
	int local_size = size, ret = 0;
	struct inode *inode = (struct inode *)file->private_data;
	int mode =  file->f_flags & O_ACCMODE;

	mutex_lock(&audio_lock);

	if (size <= 0) {
		ret = size;
		goto out;
	}

	if (str == NULL) {
		ret = -EPERM;
		goto out;
	}
	if (str->active == 0) {
		cpcap_audio_state.codec_mode = CPCAP_AUDIO_CODEC_ON;
		set_codec_mode();
		if (mode == O_RDONLY || mode == O_RDWR) {
			cpcap_audio_state.microphone = mic_setting;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
			if (mode == O_RDONLY) {
				msleep(8);
				if (state.codec_ssi_started == 0)
					TRY(audio_configure_ssi(inode, file))
			}
		}
	}

	if (str->fragsize != size) {
		str->fragsize = size;
		str->nbfrags = AUDIO_NBFRAGS_READ;
		str->input_output = FMODE_READ;
		init_waitqueue_head(&str->wq);
		if (audio_setup_buf(str, file->private_data)) {
			AUDIO_ERROR_LOG("Unable to allocate memory\n");
			ret = -ENOMEM;
			goto out;
		}
	}

	while (size > 0) {
		struct audio_buf *buf = &str->buffers[str->usr_head];

		/* Start the stream if has not already been started. The first
		 * time around we call process_buf back to back to start both
		 * receive DMA channels */
		if (str->active == 0) {
			ret = audio_process_buf(str,
				((struct inode *)file->private_data));
			if (ret == -EBUSY) {
				AUDIO_ERROR_LOG(
					"buffer processing failed to start\n");
				goto out;
			} else {
				ret = audio_process_buf(str,
					((struct inode *)file->private_data));
				if (ret == -EBUSY) {
					AUDIO_ERROR_LOG("buffer processing "
					"failed to start for second buf\n");
					goto out;
				} else {
					str->active = 1;
				}
			}
		}

		mutex_unlock(&audio_lock);

		wait_event_interruptible_timeout(str->wq, read_buf_full > 0,
						 AUDIO_TIMEOUT);

		mutex_lock(&audio_lock);

		read_buf_full--;

		if (read_buf_full < 0)
			read_buf_full = 0;

		if (copy_to_user(buffer, buf->data, str->fragsize)) {
			AUDIO_ERROR_LOG("Audio: CopyTo User failed \n");
			ret = -EFAULT;
			goto out;
		}

		if (++str->usr_head >= str->nbfrags)
			str->usr_head = 0;

		size -= str->fragsize;
	}

	ret = local_size;

out:
	mutex_unlock(&audio_lock);
	return ret;
}

static int audio_mixer_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	dump_platform_config();
	/*cpcap_audio_state_dump(&cpcap_audio_state);*/
	cpcap_audio_register_dump(&cpcap_audio_state);
	printk(KERN_DEBUG "-------End Audio Driver Dump--------\n");

	mutex_lock(&audio_lock);
	if (state.dev_mixer_open_count == 1) {
		ret = -EBUSY;
		goto err;
	}

	state.dev_mixer_open_count = 1;

err:
	mutex_unlock(&audio_lock);
	return ret;
}

static int audio_mixer_close(struct inode *inode, struct file *file)
{
	mutex_lock(&audio_lock);
	/* Reset mixer options so cpcap audio can enter low power state */
	cpcap_audio_state.microphone = CPCAP_AUDIO_IN_NONE;
	cpcap_audio_set_audio_state(&cpcap_audio_state);

	state.dev_mixer_open_count = 0;
	mutex_unlock(&audio_lock);
	return 0;
}

static int __init audio_init(void)
{
	int err = cpcap_driver_register(&audio_driver);

	if (err)
		return err;

#ifdef CONFIG_WAKELOCK
	wake_lock_init(&mcbsp_wakelock, WAKE_LOCK_SUSPEND, "mcbsp");
#endif

	return 0;
}

static void __exit audio_exit(void)
{
	platform_driver_unregister(&audio_driver);
	wake_lock_destroy(&mcbsp_wakelock);
}

static void audio_callback(int status)
{
	mutex_lock(&audio_lock);
	if (status == 1 || status == 2) {
		if (cpcap_audio_state.stdac_primary_speaker ==
					CPCAP_AUDIO_OUT_STEREO_HEADSET) {
			cpcap_audio_state.stdac_primary_speaker =
							CPCAP_AUDIO_OUT_NONE;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
			cpcap_audio_state.stdac_primary_speaker =
						CPCAP_AUDIO_OUT_STEREO_HEADSET;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
		}
		if (cpcap_audio_state.codec_primary_speaker ==
					CPCAP_AUDIO_OUT_STEREO_HEADSET) {
			cpcap_audio_state.codec_primary_speaker =
							CPCAP_AUDIO_OUT_NONE;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
			cpcap_audio_state.codec_primary_speaker =
					CPCAP_AUDIO_OUT_STEREO_HEADSET;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
		}
		if (cpcap_audio_state.ext_primary_speaker ==
					CPCAP_AUDIO_OUT_STEREO_HEADSET) {
			cpcap_audio_state.ext_primary_speaker =
							CPCAP_AUDIO_OUT_NONE;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
			cpcap_audio_state.ext_primary_speaker =
					CPCAP_AUDIO_OUT_STEREO_HEADSET;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
		}
		if (cpcap_audio_state.microphone == CPCAP_AUDIO_IN_HEADSET) {
			cpcap_audio_state.microphone = CPCAP_AUDIO_IN_NONE;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
			cpcap_audio_state.microphone = CPCAP_AUDIO_IN_HEADSET;
			cpcap_audio_set_audio_state(&cpcap_audio_state);
		}
	}
	mutex_unlock(&audio_lock);
}

static int audio_probe(struct platform_device *dev)
{
	struct cpcap_audio_pdata *pdata =
			(struct cpcap_audio_pdata *)(dev->dev.platform_data);

	mcbsp_wrapper =
		kzalloc(omap_mcbsp_count * sizeof(struct omap_mcbsp_wrapper),
		    GFP_KERNEL);

	/* /dev/dsp - stdac */
	state.dev_dsp = register_sound_dsp(&audio_stdac_fops, -1);
	/* /dev/dsp1 - codec */
	state.dev_dsp1 = register_sound_dsp(&codec_fops, -1);

	state.dev_mixer = register_sound_mixer(&mixer_fops, -1);

	state.dev_dsp_open_count = 0;
	state.dev_dsp1_open_count = 0;
	state.dev_mixer_open_count = 0;
	state.stdac_ssi_started = 0;
	state.codec_ssi_started = 0;
	state.fm_on  = 0;
	state.stdac_out_stream = NULL;
	state.stdac_in_stream = NULL;
	state.codec_out_stream = NULL;
	state.codec_in_stream = NULL;

	enable_tx = 0;
	cpcap_audio_state.cpcap = platform_get_drvdata(dev);
	cpcap_audio_set_platform_config(pdata);
	cpcap_audio_init(&cpcap_audio_state);

	cpcap_audio_state.cpcap->h2w_new_state = &audio_callback;
	return 0;
}

static int audio_remove(struct platform_device *dev)
{
	unregister_sound_dsp(state.dev_dsp);
	unregister_sound_dsp(state.dev_dsp1);
	unregister_sound_mixer(state.dev_mixer);

	state.dev_dsp_open_count = 0;
	state.dev_dsp1_open_count = 0;
	state.stdac_ssi_started = 0;
	state.codec_ssi_started = 0;
	state.fm_on = 0;
	state.dev_mixer_open_count = 0;
	state.stdac_out_stream = NULL;
	state.stdac_in_stream = NULL;
	state.codec_out_stream = NULL;
	state.codec_in_stream = NULL;

	return 0;
}

module_init(audio_init);
module_exit(audio_exit);

MODULE_DESCRIPTION("OMAP audio driver");
MODULE_AUTHOR("Motorola, Inc.");
MODULE_LICENSE("GPL");
