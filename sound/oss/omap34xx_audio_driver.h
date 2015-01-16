
#ifndef OMAP34XX_AUDIO_DRIVER_H
#define OMAP34XX_AUDIO_DRIVER_H

#define MCBSP_WRAPPER

#ifdef MCBSP_WRAPPER
#include <plat/dma.h>
#include <plat/mcbsp.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#endif /* MCBSP_WRAPPER */

#ifdef MCBSP_WRAPPER

#if defined(CONFIG_ARCH_OMAP24XX) || defined(CONFIG_ARCH_OMAP34XX)
#define OMAP_MCBSP_REG_WKUPEN 0xA8
#endif

#define MCBSP_WKUP_XRDYEN		0x400

#define OMAP_MCBSP_BIT(ARG) ((0x01)<<(ARG))

#define MCBSP2_SYSCONFIG_LVL1 1
#define MCBSP2_SYSCONFIG_LVL2 2

#define MCBSP_FIFO_SIZE 64
#define MCBSP2_FIFO_SIZE 1024
#define OMAP_MCBSP_REG_THRSH1 0x94
#define OMAP_MCBSP_REG_THRSH2 0x90

/********************** McBSP SYSCONFIG bit definitions ********************/
#define FORCE_IDLE 0x0
#define NO_IDLE 0x1
#define SMART_IDLE 0x2
#define MCBSP_SYSC_IOFF_FOFF 0x0
#define MCBSP_SYSC_IOFF_FON 0x2	/* Err in TRM ES2.0 ?? */

/* McBSP interface operating mode */
#define OMAP_MCBSP_MASTER 1
#define OMAP_MCBSP_SLAVE 0

#define OMAP_MCBSP_AUTO_RRST (0x1<<1)
#define OMAP_MCBSP_AUTO_XRST (0x1<<2)

/* SRG ENABLE/DISABLE state */
#define OMAP_MCBSP_ENABLE_FSG_SRG               1
#define OMAP_MCBSP_DISABLE_FSG_SRG              2
/* mono to mono mode*/
#define OMAP_MCBSP_SKIP_NONE (0x0)
/* mono to stereo mode */
#define OMAP_MCBSP_SKIP_FIRST (0x1<<1)
#define OMAP_MCBSP_SKIP_SECOND (0x1<<2)
/* RRST STATE */
#define OMAP_MCBSP_RRST_DISABLE 0
#define OMAP_MCBSP_RRST_ENABLE  1
/*XRST STATE */
#define OMAP_MCBSP_XRST_DISABLE 0
#define OMAP_MCBSP_XRST_ENABLE  1

#define OMAP_MCBSP_FRAME_SINGLEPHASE 1
#define OMAP_MCBSP_FRAME_DUALPHASE   2

/* Sample Rate Generator Clock source */
#define OMAP_MCBSP_SRGCLKSRC_CLKS 1
#define OMAP_MCBSP_SRGCLKSRC_FCLK 2
#define OMAP_MCBSP_SRGCLKSRC_CLKR 3
#define OMAP_MCBSP_SRGCLKSRC_CLKX 4

/* SRG input clock polarity */
#define OMAP_MCBSP_CLKS_POLARITY_RISING 1
#define OMAP_MCBSP_CLKX_POLARITY_RISING 1
#define OMAP_MCBSP_CLKX_POLARITY_FALLING 2
#define OMAP_MCBSP_CLKR_POLARITY_RISING 1
#define OMAP_MCBSP_CLKR_POLARITY_FALLING 2

/* SRG Clock synchronization mode */
#define OMAP_MCBSP_SRG_FREERUNNING 1
#define OMAP_MCBSP_SRG_RUNNING 2

/* Frame Sync Source */
#define OMAP_MCBSP_TXFSYNC_EXTERNAL 0
#define OMAP_MCBSP_TXFSYNC_INTERNAL 1
#define OMAP_MCBSP_RXFSYNC_EXTERNAL 0
#define OMAP_MCBSP_RXFSYNC_INTERNAL 1
#define OMAP_MCBSP_CLKRXSRC_EXTERNAL 1
#define OMAP_MCBSP_CLKRXSRC_INTERNAL 2
#define OMAP_MCBSP_CLKTXSRC_EXTERNAL 1
#define OMAP_MCBSP_CLKTXSRC_INTERNAL 2

/* Justification */
#define OMAP_MCBSP_RJUST_ZEROMSB 0

#define OMAP_MCBSP_DATADELAY1 1

/* Reverse mode for 243X and 34XX */
#define OMAP_MCBSP_MSBFIRST 0

#define OMAP_MCBSP_FRAMELEN_N(NUM_WORDS) ((NUM_WORDS - 1) & 0x7F)

struct omap_mcbsp_cfg_param {
	u8 fsync_src;
	u8 fs_polarity;
	u8 clk_polarity;
	u8 clk_mode;
	u8 frame_length1;
	u8 frame_length2;
	u8 word_length1;
	u8 word_length2;
	u8 justification;
	u8 reverse_compand;
	u8 phase;
	u8 data_delay;
};

struct omap_mcbsp_srg_fsg_cfg {
	u32 period;		/* Frame period */
	u32 pulse_width;	/* Frame width */
	u8 fsgm;
	u32 sample_rate;
	u32 bits_per_sample;
	u32 srg_src;
	u8 sync_mode;		/* SRG free running mode */
	u8 polarity;
	u8 dlb;			/* digital loopback mode */
};

struct omap_mcbsp_dma_transfer_params {
	/* Skip the alternate element use fro stereo mode */
	u8 skip_alt;
	/* Automagically handle Transfer [XR]RST? */
	u8 auto_reset;
	/* callback function executed for every tx/rx completion */
	void (*callback) (u32 ch_status, void *arg);
	/* word length of data */
	u32 word_length1;
	u32 word_length2;
};

struct omap_mcbsp_wrapper {
	u8 auto_reset;		/* Auto Reset */
	u8 txskip_alt;		/* Tx skip flags */
	u8 rxskip_alt;		/* Rx skip flags */
	void *rx_cb_arg;
	void *tx_cb_arg;
	void (*rx_callback) (u32 ch_status, void *arg);
	void (*tx_callback) (u32 ch_status, void *arg);
	int rx_dma_chain_state;
	int tx_dma_chain_state;
	int interface_mode;	/* Master / Slave */
	struct omap_dma_channel_params rx_params;	/* Used For Rx FIFO */
	int rx_config_done;
};
#endif /* MCBSP_WRAPPER */
#ifdef MCBSP_WRAPPER

void omap_mcbsp_write(void __iomem *io_base, u16 reg, u32 val);
int omap_mcbsp_read(void __iomem *io_base, u16 reg);

int omap2_mcbsp_stop_datatx(u32 id);
int omap2_mcbsp_stop_datarx(u32 id);
int omap2_mcbsp_reset(unsigned int id);
int omap2_mcbsp_set_xrst(unsigned int id, u8 state);
int omap2_mcbsp_set_rrst(unsigned int id, u8 state);
int omap2_mcbsp_dma_recv_params(unsigned int id,
				struct omap_mcbsp_dma_transfer_params *rp);
int omap2_mcbsp_dma_trans_params(unsigned int id,
				 struct omap_mcbsp_dma_transfer_params *tp);
int omap2_mcbsp_receive_data(unsigned int id, void *cbdata,
			     dma_addr_t buf_start_addr, u32 buf_size);
int omap2_mcbsp_send_data(unsigned int id, void *cbdata,
			  dma_addr_t buf_start_addr, u32 buf_size);
int omap2_mcbsp_params_cfg(unsigned int id, int interface_mode,
			   struct omap_mcbsp_cfg_param *rp,
			   struct omap_mcbsp_cfg_param *tp,
			   struct omap_mcbsp_srg_fsg_cfg *param);
#endif /* MCBSP_WRAPPER */

#endif /* OMAP34XX_AUDIO_DRIVER_H */
