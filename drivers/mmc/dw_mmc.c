/*
 * (C) Copyright 2012 SAMSUNG Electronics
 * Jaehoon Chung <jh80.chung@samsung.com>
 * Rajeshawari Shinde <rajeshwari.s@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,  MA 02111-1307 USA
 *
 */

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <dwmmc.h>
#include <asm/arch/clk.h>
#include <asm-generic/errno.h>

#define PAGE_SIZE 4096

static int dwmci_wait_reset(struct dwmci_host *host, u32 value)
{
	unsigned long timeout = 1000;
	u32 ctrl;

	dwmci_writel(host, DWMCI_CTRL, value);

	while (timeout--) {
		ctrl = dwmci_readl(host, DWMCI_CTRL);
		if (!(ctrl & DWMCI_RESET_ALL))
			return 1;
		if (timeout == 0)
			break;
	}
	return 0;
}

static void dwmci_set_idma_desc(u8 *idmac, unsigned int des0,
		unsigned int des1, unsigned int des2)
{
	struct dwmci_idmac *desc = (struct dwmci_idmac *)idmac;

	desc->des0 = des0;
	desc->des1 = des1;
	desc->des2 = des2;
	desc->des3 = (unsigned int)desc + sizeof(struct dwmci_idmac);
}

static void dwmci_prepare_data(struct dwmci_host *host,
		struct mmc_data *data)
{
	unsigned long ctrl;
	unsigned int i = 0, flag, cnt, blk_cnt;
	struct dwmci_idmac *p;
	ulong data_start, data_end, start_addr;
	ALLOC_CACHE_ALIGN_BUFFER(struct dwmci_idmac, idmac, 65565);


	p = idmac;
	blk_cnt = data->blocks;

	dwmci_wait_reset(host, DWMCI_CTRL_FIFO_RESET);

	if (data->flags == MMC_DATA_READ)
		start_addr = (unsigned int)data->dest;
	else
		start_addr = (unsigned int)data->src;

	do {
		flag = DWMCI_IDMAC_OWN | DWMCI_IDMAC_CH ;
		flag |= (i == 0) ? DWMCI_IDMAC_FS : 0;
		if (blk_cnt <= 8) {
			flag |= DWMCI_IDMAC_LD;
			cnt = data->blocksize * blk_cnt;
		} else {
			flag &= ~DWMCI_IDMAC_LD;
			cnt = data->blocksize * 8;
		}

		dwmci_set_idma_desc((u8 *)p, flag, cnt,
				start_addr + (i * PAGE_SIZE));

		if (blk_cnt <= 8)
			break;
		blk_cnt -= 8;
		p++;
		i++;
	} while(1);

	data_start = (ulong)idmac;
	data_end = (ulong)p;
	flush_dcache_range(data_start, data_end + ARCH_DMA_MINALIGN);

	dwmci_writel(host, DWMCI_DBADDR, (unsigned int)(idmac));

	ctrl = dwmci_readl(host, DWMCI_CTRL);
	ctrl |= DWMCI_IDMAC_EN | DWMCI_DMA_EN;
	dwmci_writel(host, DWMCI_CTRL, ctrl);

	ctrl = dwmci_readl(host, DWMCI_BMOD);
	ctrl |= DWMCI_BMOD_IDMAC_FB | DWMCI_BMOD_IDMAC_EN;
	dwmci_writel(host, DWMCI_BMOD, ctrl);

	dwmci_writel(host, DWMCI_BLKSIZ, data->blocksize);
	dwmci_writel(host, DWMCI_BYTCNT, data->blocksize * data->blocks);
}

static int dwmci_set_transfer_mode(struct dwmci_host *host,
		struct mmc_data *data)
{
	unsigned long mode;

	mode = DWMCI_CMD_DATA_EXP;
	if (data->flags & MMC_DATA_WRITE)
		mode |= DWMCI_CMD_RW;

	return mode;
}

static int dwmci_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
		struct mmc_data *data)
{
	struct dwmci_host *host = (struct dwmci_host *)mmc->priv;
	int flags = 0, i;
	unsigned int timeout = 100000;
	u32 retry = 10000;
	u32 mask, ctrl;

	while (dwmci_readl(host, DWMCI_STATUS) & DWMCI_BUSY) {
		if (timeout == 0) {
			printf("Timeout on data busy\n");
			return TIMEOUT;
		}
		timeout--;
	}

	dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_ALL);

	if (data)
		dwmci_prepare_data(host, data);


	dwmci_writel(host, DWMCI_CMDARG, cmd->cmdarg);

	if (data) {
		flags = dwmci_set_transfer_mode(host, data);
	}

	if ((cmd->resp_type & MMC_RSP_136) && (cmd->resp_type & MMC_RSP_BUSY))
		return -1;

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		flags |= DWMCI_CMD_ABORT_STOP;
	else
		flags |= DWMCI_CMD_PRV_DAT_WAIT;

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		flags |= DWMCI_CMD_RESP_EXP;
		if (cmd->resp_type & MMC_RSP_136)
			flags |= DWMCI_CMD_RESP_LENGTH;
	}

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= DWMCI_CMD_CHECK_CRC;

	flags |= (cmd->cmdidx | DWMCI_CMD_START | DWMCI_CMD_USE_HOLD_REG);

	debug("Sending CMD%d\n",cmd->cmdidx);

	dwmci_writel(host, DWMCI_CMD, flags);

	for (i = 0; i < retry; i++) {
		mask = dwmci_readl(host, DWMCI_RINTSTS);
		if (mask & DWMCI_INTMSK_CDONE) {
			if (!data)
				dwmci_writel(host, DWMCI_RINTSTS, mask);
			break;
		}
	}

	if (i == retry)
		return TIMEOUT;

	if (mask & DWMCI_INTMSK_RTO) {
		debug("Response Timeout..\n");
		return TIMEOUT;
	} else if (mask & DWMCI_INTMSK_RE) {
		debug("Response Error..\n");
		return -1;
	}


	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136) {
			cmd->response[0] = dwmci_readl(host, DWMCI_RESP3);
			cmd->response[1] = dwmci_readl(host, DWMCI_RESP2);
			cmd->response[2] = dwmci_readl(host, DWMCI_RESP1);
			cmd->response[3] = dwmci_readl(host, DWMCI_RESP0);
		} else {
			cmd->response[0] = dwmci_readl(host, DWMCI_RESP0);
		}
	}

	if (data) {
		while (1) {
			mask = dwmci_readl(host, DWMCI_RINTSTS);
			if (mask & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) {
				debug("DATA ERROR!\n");
				return -1;
			} else if (mask & DWMCI_INTMSK_DTO)
				break;
		}

		dwmci_writel(host, DWMCI_RINTSTS, mask);

		ctrl = dwmci_readl(host, DWMCI_CTRL);
		ctrl &= ~(DWMCI_DMA_EN);
		dwmci_writel(host, DWMCI_CTRL, ctrl);
	}

	udelay(100);

	return 0;
}

static int dwmci_setup_bus(struct dwmci_host *host, u32 freq)
{
	u32 div, status;
	int timeout = 10000;
	unsigned long sclk;

	if (freq == host->clock)
		return 0;

	/*
	 * If host->mmc_clk didn't define,
	 * then assume that host->bus_hz is source clock value.
	 * host->bus_hz should be set from user.
	 */
	if (host->mmc_clk)
		sclk = host->mmc_clk(host->dev_index);
	else if (host->bus_hz)
		sclk = host->bus_hz;
	else {
		printf("Didn't get source clock value..\n");
		return -EINVAL;
	}

	for (div = 1; div < 255; div++) {
		if ((sclk / (2 * div)) <= freq) {
			break;
		}
	}

	dwmci_writel(host, DWMCI_CLKENA, 0);
	dwmci_writel(host, DWMCI_CLKSRC, 0);

	dwmci_writel(host, DWMCI_CLKDIV, div);
	dwmci_writel(host, DWMCI_CMD, DWMCI_CMD_PRV_DAT_WAIT |
			DWMCI_CMD_UPD_CLK | DWMCI_CMD_START);

	do {
		status = dwmci_readl(host, DWMCI_CMD);
		if (timeout == 0) {
			printf("TIMEOUT error!!\n");
			return -ETIMEDOUT;
		}
	} while ((status & DWMCI_CMD_START) && timeout--);

	dwmci_writel(host, DWMCI_CLKENA, DWMCI_CLKEN_ENABLE |
			DWMCI_CLKEN_LOW_PWR);

	dwmci_writel(host, DWMCI_CMD, DWMCI_CMD_PRV_DAT_WAIT |
			DWMCI_CMD_UPD_CLK | DWMCI_CMD_START);

	timeout = 10000;
	do {
		status = dwmci_readl(host, DWMCI_CMD);
		if (timeout == 0) {
			printf("TIMEOUT error!!\n");
			return -ETIMEDOUT;
		}
	} while ((status & DWMCI_CMD_START) && timeout--);

	host->clock = freq;

	return 0;
}

static void dwmci_set_ios(struct mmc *mmc)
{
	struct dwmci_host *host = (struct dwmci_host *)mmc->priv;
	u32 ctype;

	debug("Buswidth = %d, clock: %d\n",mmc->bus_width, mmc->clock);

	dwmci_setup_bus(host, mmc->clock);
	switch (mmc->bus_width) {
	case 8:
		ctype = DWMCI_CTYPE_8BIT;
		break;
	case 4:
		ctype = DWMCI_CTYPE_4BIT;
		break;
	default:
		ctype = DWMCI_CTYPE_1BIT;
		break;
	}

	dwmci_writel(host, DWMCI_CTYPE, ctype);

	if (host->clksel)
		host->clksel(host);
}

static int dwmci_init(struct mmc *mmc)
{
	struct dwmci_host *host = (struct dwmci_host *)mmc->priv;
	u32 fifo_size, fifoth_val;

	dwmci_writel(host, DWMCI_PWREN, 1);

	if (!dwmci_wait_reset(host, DWMCI_RESET_ALL)) {
		debug("%s[%d] Fail-reset!!\n",__func__,__LINE__);
		return -1;
	}

	dwmci_writel(host, DWMCI_RINTSTS, 0xFFFFFFFF);
	dwmci_writel(host, DWMCI_INTMASK, 0);

	dwmci_writel(host, DWMCI_TMOUT, 0xFFFFFFFF);

	dwmci_writel(host, DWMCI_IDINTEN, 0);
	dwmci_writel(host, DWMCI_BMOD, 1);

	fifo_size = dwmci_readl(host, DWMCI_FIFOTH);
	if (host->fifoth_val)
		fifoth_val = host->fifoth_val;
	else
		fifoth_val = (0x2 << 28) | ((fifo_size/2 -1) << 16) |
			((fifo_size/2) << 0);
	dwmci_writel(host, DWMCI_FIFOTH, fifoth_val);

	dwmci_writel(host, DWMCI_CLKENA, 0);
	dwmci_writel(host, DWMCI_CLKSRC, 0);

	return 0;
}

int add_dwmci(struct dwmci_host *host, u32 max_clk, u32 min_clk, int index)
{
	struct mmc *mmc;
	int err = 0;

	mmc = malloc(sizeof(struct mmc));
	if (!mmc) {
		printf("mmc malloc fail!\n");
		return -1;
	}

	mmc->priv = host;
	host->mmc = mmc;

	sprintf(mmc->name, "%s", host->name);
	mmc->send_cmd = dwmci_send_cmd;
	mmc->set_ios = dwmci_set_ios;
	mmc->init = dwmci_init;
	mmc->f_min = min_clk;
	mmc->f_max = max_clk;

	mmc->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;

	if (host->caps)
		mmc->host_caps = host->caps;
	else
		mmc->host_caps = 0;

	if (host->buswidth == 8) {
		mmc->host_caps |= MMC_MODE_8BIT;
		mmc->host_caps &= ~MMC_MODE_4BIT;
	} else {
		mmc->host_caps |= MMC_MODE_4BIT;
		mmc->host_caps &= ~MMC_MODE_8BIT;
	}
	mmc->host_caps |= MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_HC;

	err = mmc_register(mmc);

	return err;
}
