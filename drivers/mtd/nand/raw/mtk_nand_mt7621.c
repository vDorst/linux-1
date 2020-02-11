/*
 * SPDX-License-Identifier: GPL-2.0
 * Driver for MediaTek SLC NAND Flash interface controller
 *
 * Copyright (C) 2018 MediaTek Inc.
 * Authors:	Xiangsheng Hou	<xiangsheng.hou@mediatek.com>
 *		Weijie Gao	<weijie.gao@mediatek.com>
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/bbm.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/mtd/partitions.h>
#include <linux/sizes.h>
#include "mtk_nand_mt7621.h"

#define MTK_NAME		"mtk7621-nand"

static int mtk_nfc_do_read_page_hwecc(struct nand_chip *chip, uint8_t *buf,
	    int page);

static int mtk_nfc_page_erase_write(struct nand_chip *chip, const uint8_t *buf,
	    const uint8_t *oob, int page);

static inline struct mtk_nfc_nand_chip *to_mtk_nand(struct nand_chip *nand)
{
	return container_of(nand, struct mtk_nfc_nand_chip, nand);
}

static inline u8 *data_ptr(struct nand_chip *chip, const u8 *p, int i)
{
	return (u8 *)p + i * chip->ecc.size;
}

static inline u8 *oob_buf_ptr(struct nand_chip *chip, u8 *p, int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u8 *poi;

	poi = p + i * nfc->caps->fdm_size;

	return poi;
}

static inline u8 *oob_ptr(struct nand_chip *chip, int i)
{
	return oob_buf_ptr(chip, chip->oob_poi, i);
}

static inline u8 *ecc_ptr(struct nand_chip *chip, int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(chip);
	u8 *poi;

	poi = chip->oob_poi + chip->ecc.steps * nfc->caps->fdm_size
		+ i * (mtk_nand->spare_per_sector - nfc->caps->fdm_size);

	return poi;
}

static inline int mtk_data_len(struct nand_chip *chip)
{
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(chip);

	return chip->ecc.size + mtk_nand->spare_per_sector;
}

static inline u8 *mtk_data_ptr(struct nand_chip *chip,  int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	return nfc->buffer + i * mtk_data_len(chip);
}

static inline u8 *mtk_oob_ptr(struct nand_chip *chip, int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	return nfc->buffer + i * mtk_data_len(chip) + chip->ecc.size;
}

static inline u8 *mtk_ecc_ptr(struct nand_chip *chip, int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	return mtk_oob_ptr(chip, i) + nfc->caps->fdm_size;
}

static inline void nfi_clear_reg16(struct mtk_nfc *nfc, u32 val, u32 reg)
{
	u16 temp_val = 0;

	temp_val = readw_relaxed(nfc->regs + reg);
	temp_val &= ~val;
	writew(temp_val, nfc->regs + reg);
}

static inline void nfi_set_reg16(struct mtk_nfc *nfc, u32 val, u32 reg)
{
	u16 temp_val = 0;

	temp_val = readw_relaxed(nfc->regs + reg);
	temp_val |= val;
	writew(temp_val, nfc->regs + reg);
}

static inline void nfi_writel(struct mtk_nfc *nfc, u32 val, u32 reg)
{
	writel(val, nfc->regs + reg);
}

static inline void nfi_writew(struct mtk_nfc *nfc, u16 val, u32 reg)
{
	writew(val, nfc->regs + reg);
}

static inline u32 nfi_readl(struct mtk_nfc *nfc, u32 reg)
{
	return readl_relaxed(nfc->regs + reg);
}

static inline u16 nfi_readw(struct mtk_nfc *nfc, u32 reg)
{
	return readw_relaxed(nfc->regs + reg);
}

static void mtk_nfc_hw_reset(struct mtk_nfc *nfc)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	/* reset all registers and force the NFI master to terminate */
	nfi_writel(nfc, CON_FIFO_FLUSH | CON_NFI_RST, NFI_CON);

	/* wait for the master to finish the last transaction */
	ret = readl_poll_timeout(nfc->regs + NFI_MASTER_STA, val,
				 !(val & MASTER_STA_MASK), 50,
				 MTK_RESET_TIMEOUT);
	if (ret)
		dev_warn(dev, "master active in reset [0x%x] = 0x%x\n",
			 NFI_MASTER_STA, val);

	/* ensure any status register affected by the NFI master is reset */
	nfi_writel(nfc, CON_FIFO_FLUSH | CON_NFI_RST, NFI_CON);
	nfi_writew(nfc, STAR_DE, NFI_STRDATA);
}

static int mtk_nfc_send_command(struct mtk_nfc *nfc, u8 command)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	nfi_writel(nfc, command, NFI_CMD);

	ret = readl_poll_timeout_atomic(nfc->regs + NFI_STA, val,
					!(val & STA_CMD), 10,  MTK_TIMEOUT);
	if (ret) {
		dev_warn(dev, "nfi core timed out entering command mode\n");
		return -EIO;
	}

	return 0;
}

static int mtk_nfc_send_address(struct mtk_nfc *nfc, int addr)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	nfi_writel(nfc, addr, NFI_COLADDR);
	nfi_writel(nfc, 0, NFI_ROWADDR);
	nfi_writew(nfc, 1, NFI_ADDRNOB);

	ret = readl_poll_timeout_atomic(nfc->regs + NFI_STA, val,
					!(val & STA_ADDR), 10, MTK_TIMEOUT);
	if (ret) {
		dev_warn(dev, "nfi core timed out entering address mode\n");
		return -EIO;
	}

	return 0;
}

static int mtk_nfc_hw_runtime_config(struct nand_chip *chip)
{
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(chip);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct device *dev = nfc->dev;
	u32 fmt, spare_bit;

	if (!mtd->writesize)
		return 0;

	chip->ecc.size = nfc->caps->sector_size;
	mtk_nand->spare_per_sector = mtd->oobsize
				/ (mtd->writesize / chip->ecc.size);

	if (mtk_nand->spare_per_sector >= 28) {
		spare_bit = PAGEFMT_SPARE_28;
		chip->ecc.strength = 12;
		mtk_nand->spare_per_sector = 28;
	} else if (mtk_nand->spare_per_sector >= 27) {
		spare_bit = PAGEFMT_SPARE_27;
		chip->ecc.strength = 8;
		mtk_nand->spare_per_sector = 27;
	} else if (mtk_nand->spare_per_sector >= 26) {
		spare_bit = PAGEFMT_SPARE_26;
		chip->ecc.strength = 8;
		mtk_nand->spare_per_sector = 26;
	} else if (mtk_nand->spare_per_sector >= 16) {
		spare_bit = PAGEFMT_SPARE_16;
		chip->ecc.strength = 4;
		mtk_nand->spare_per_sector = 16;
	} else {
		dev_err(dev, "MTK NFI not support oobsize: %x\n",
			mtk_nand->spare_per_sector);
		return -EINVAL;
	}

	switch (mtd->writesize) {
	case 512:
		fmt = PAGEFMT_512;
		break;
	case SZ_2K:
		fmt = PAGEFMT_2K;
		break;
	case SZ_4K:
		fmt = PAGEFMT_4K;
		break;
	default:
		dev_err(nfc->dev, "invalid page len: %d\n", mtd->writesize);
		return -EINVAL;
	}

	fmt |= spare_bit << nfc->caps->pageformat_spare_shift;
	fmt |= nfc->caps->fdm_size << PAGEFMT_FDM_SHIFT;
	fmt |= nfc->caps->fdm_ecc_size << PAGEFMT_FDM_ECC_SHIFT;
	nfi_writel(nfc, fmt, NFI_PAGEFMT);

	nfc->ecc_cfg.strength = chip->ecc.strength;
	nfc->ecc_cfg.len = chip->ecc.size + nfc->caps->fdm_ecc_size;

	return 0;
}

static void mtk_nfc_select_chip(struct nand_chip *nand, int chip)
{
	struct mtk_nfc *nfc = nand_get_controller_data(nand);
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(nand);

	if (chip < 0)
		return;

	mtk_nfc_hw_runtime_config(nand);

	nfi_writel(nfc, mtk_nand->sels[chip], NFI_CSEL);
}

static int mtk_nfc_dev_ready(struct nand_chip *chip)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	if (nfi_readl(nfc, NFI_STA) & STA_BUSY)
		return 0;

	return 1;
}

static void mtk_nfc_cmd_ctrl(struct nand_chip *chip, int dat, unsigned int ctrl)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	if (ctrl & NAND_ALE) {
		mtk_nfc_send_address(nfc, dat);
	} else if (ctrl & NAND_CLE) {
		mtk_nfc_hw_reset(nfc);

		nfi_writew(nfc, CNFG_OP_CUST, NFI_CNFG);
		mtk_nfc_send_command(nfc, dat);
	}
}

static inline void mtk_nfc_wait_ioready(struct mtk_nfc *nfc)
{
	int rc;
	u8 val;

	rc = readb_poll_timeout_atomic(nfc->regs + NFI_PIO_DIRDY, val,
				       val & PIO_DI_RDY, 10, MTK_TIMEOUT);
	if (rc < 0)
		dev_err(nfc->dev, "data not ready\n");
}

static u32 mtk_nfc_pio_read(struct nand_chip *chip, int byterw)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 reg;

	/* after each byte read, the NFI_STA reg is reset by the hardware */
	reg = nfi_readl(nfc, NFI_STA) & NFI_FSM_MASK;
	if (reg != NFI_FSM_CUSTDATA) {
		if (byterw)
			nfi_set_reg16(nfc, CNFG_BYTE_RW, NFI_CNFG);
		else
			nfi_clear_reg16(nfc, CNFG_BYTE_RW, NFI_CNFG);

		reg = nfi_readw(nfc, NFI_CNFG);
		reg |= CNFG_READ_EN;
		nfi_writew(nfc, reg, NFI_CNFG);

		/*
		 * set to max sector to allow the HW to continue reading over
		 * unaligned accesses
		 */
		reg = (nfc->caps->max_sector << CON_SEC_SHIFT) | CON_BRD;
		nfi_writel(nfc, reg, NFI_CON);

		/* trigger to fetch data */
		nfi_writew(nfc, STAR_EN, NFI_STRDATA);
	}

	mtk_nfc_wait_ioready(nfc);

	return nfi_readl(nfc, NFI_DATAR);
}

static inline u8 mtk_nfc_read_byte(struct nand_chip *chip)
{
	return mtk_nfc_pio_read(chip, 1) & 0xff;
}

static void mtk_nfc_read_buf(struct nand_chip *chip, u8 *buf, int len)
{
	int i;
	u32 *p = (u32 *) buf;

	if ((u32) buf % sizeof(u32) || len % sizeof(u32)) {
		for (i = 0; i < len; i++)
			buf[i] = mtk_nfc_pio_read(chip, 1);
	} else {
		for (i = 0; i < (len / sizeof(u32)); i++)
			p[i] = mtk_nfc_pio_read(chip, 0);
	}
}

static void mtk_nfc_pio_write(struct nand_chip *chip, u32 val, int byterw)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 reg;

	reg = nfi_readl(nfc, NFI_STA) & NFI_FSM_MASK;

	if (reg != NFI_FSM_CUSTDATA) {
		if (byterw)
			nfi_set_reg16(nfc, CNFG_BYTE_RW, NFI_CNFG);
		else
			nfi_clear_reg16(nfc, CNFG_BYTE_RW, NFI_CNFG);

		reg = nfc->caps->max_sector << CON_SEC_SHIFT | CON_BWR;
		nfi_writew(nfc, reg, NFI_CON);

		nfi_writew(nfc, STAR_EN, NFI_STRDATA);
	}

	mtk_nfc_wait_ioready(nfc);
	nfi_writel(nfc, val, NFI_DATAW);
}


static void mtk_nfc_write_byte(struct nand_chip *chip, u8 byte)
{
	mtk_nfc_pio_write(chip, byte, 1);
}

static void mtk_nfc_write_buf(struct nand_chip *chip, const u8 *buf, int len)
{
	int i;
	const u32 *p = (const u32 *) buf;

	if ((u32) buf % sizeof(u32) || len % sizeof(u32)) {
		for (i = 0; i < len; i++)
			mtk_nfc_pio_write(chip, buf[i], 1);
	} else {
		for (i = 0; i < (len / sizeof(u32)); i++)
			mtk_nfc_pio_write(chip, p[i], 0);
	}
}

static inline void mtk_nfc_read_fdm(struct nand_chip *chip, u32 start,
				    u32 sectors)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 vall, valm;
	u8 *oobptr;
	int i, j;

	for (i = 0; i < sectors; i++) {
		oobptr = oob_ptr(chip, start + i);
		vall = nfi_readl(nfc, NFI_FDML(start + i));
		valm = nfi_readl(nfc, NFI_FDMM(start + i));

		for (j = 0; j < nfc->caps->fdm_size; j++)
			oobptr[j] = (j >= 4 ? valm : vall) >> ((j % 4) * 8);
	}
}

static inline void mtk_nfc_write_fdm(struct nand_chip *chip)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 vall, valm;
	u8 *oobptr;
	int i, j;

	for (i = 0; i < chip->ecc.steps; i++) {
		oobptr = oob_ptr(chip, i);
		vall = 0;
		valm = 0;
		for (j = 0; j < 8; j++) {
			if (j < 4)
				vall |= (j < nfc->caps->fdm_size ? oobptr[j]
					: 0xff) << (j * 8);
			else
				valm |= (j < nfc->caps->fdm_size ? oobptr[j]
					: 0xff) << ((j - 4) * 8);
		}
		nfi_writel(nfc, vall, NFI_FDML(i));
		nfi_writel(nfc, valm, NFI_FDMM(i));
	}
}

static int mtk_nfc_check_empty_page(struct nand_chip *chip, const u8 *buf)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 i, j;
	u8 *oob_poi;

	for (i = 0; i < mtd->writesize; i++)
		if (buf[i] != 0xff)
			return 0;

	for (i = 0; i < chip->ecc.steps; i++) {
		oob_poi = oob_ptr(chip, i);
		for (j = 0; j < nfc->caps->fdm_ecc_size; j++)
			if (oob_poi[j] != 0xff)
				return 0;
	}

	return 1;
}

static int mtk_nfc_write_page_raw(struct nand_chip *chip,
				  const u8 *buf, int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct device *dev = nfc->dev;
	u32 i, ret, reg;

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);

	for (i = 0; i < chip->ecc.steps; i++)
		memcpy(mtk_oob_ptr(chip, i), oob_ptr(chip, i),
		       nfc->caps->fdm_size);

	if (buf) {
		for (i = 0; i < chip->ecc.steps; i++)
			memcpy(mtk_data_ptr(chip, i), data_ptr(chip, buf, i),
			       chip->ecc.size);
	}

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	nfi_clear_reg16(nfc, CNFG_READ_EN | CNFG_AUTO_FMT_EN
		| CNFG_HW_ECC_EN, NFI_CNFG);
	mtk_nfc_write_buf(chip, nfc->buffer, mtd->writesize + mtd->oobsize);

	ret = readl_poll_timeout_atomic(nfc->regs + NFI_ADDRCNTR, reg,
					ADDRCNTR_SEC(reg) >= chip->ecc.steps,
					10, MTK_TIMEOUT);
	if (ret)
		dev_err(dev, "raw write timeout\n");

	nfi_writel(nfc, 0, NFI_CON);

	return nand_prog_page_end_op(chip);
}

static int mtk_nfc_write_page_hwecc(struct nand_chip *chip, const u8 *buf,
				    int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct device *dev = nfc->dev;
	int ret;
	u32 reg;

	if (mtk_nfc_check_empty_page(chip, buf)) {
		/*
		 * When the entire page is 0xff including oob data,
		 * do not use ecc engine which will write ecc parity code
		 * back to oob region.
		 *
		 * For 4-bit ecc strength, the ecc parity code of a full
		 * 0xff subpage is 26 20 98 1b 87 6e fc
		 *
		 * Use raw mode instead.
		 */
		return mtk_nfc_write_page_raw(chip, NULL, oob_on, page);
	}

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	nfi_clear_reg16(nfc, CNFG_READ_EN, NFI_CNFG);
	nfi_set_reg16(nfc, CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN, NFI_CNFG);

	nfc->ecc_cfg.op = ECC_ENCODE;
	mtk_ecc_init(nfc, nfc->ecc, &nfc->ecc_cfg);
	mtk_ecc_enable(nfc->ecc, &nfc->ecc_cfg);

	mtk_nfc_write_fdm(chip);
	reg = chip->ecc.steps << CON_SEC_SHIFT | CON_BWR;
	nfi_writew(nfc, reg, NFI_CON);
	mtk_nfc_write_buf(chip, buf, mtd->writesize);

	ret = readl_poll_timeout_atomic(nfc->regs + NFI_ADDRCNTR, reg,
					ADDRCNTR_SEC(reg) >= chip->ecc.steps,
					10, MTK_TIMEOUT);
	if (ret)
		dev_err(dev, "hwecc write timeout\n");

	mtk_ecc_disable(nfc->ecc);
	nfi_writel(nfc, 0, NFI_CON);
	return nand_prog_page_end_op(chip);
}

static int mtk_nfc_write_oob_raw(struct nand_chip *chip, int page)
{
	return mtk_nfc_write_page_raw(chip, NULL, 1, page);
}

static int mtk_nfc_write_oob_compat_check(struct nand_chip *chip, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int i, j, oobsame = 1;
	u8 *oob_poi, *oob_flash;

	/* backup pending oob data */
	memcpy(nfc->pending_oob[0], chip->oob_poi, mtd->oobsize);

	mtk_nfc_do_read_page_hwecc(chip, nfc->pending_page, page);

	if (mtk_nfc_check_empty_page(chip, nfc->pending_page)) {
		/* page is empty, writing directly */
		memcpy(chip->oob_poi, nfc->pending_oob[0], mtd->oobsize);
		return 0;
	}

	for (i = 0; i < chip->ecc.steps; i++) {
		oob_poi = oob_ptr(chip, i);
		oob_flash = oob_buf_ptr(chip, nfc->pending_oob[0], i);
		for (j = 0; j < nfc->caps->fdm_ecc_size; j++) {
			if (oob_poi[j] != oob_flash[i]) {
				oobsame = 0;
				break;
			}
		}
	}

	if (oobsame) {
		/* both oob are the same, doing nothing */
		memcpy(chip->oob_poi, nfc->pending_oob[0], mtd->oobsize);
		return 1;
	}

	/* backup nand oob data */
	memcpy(nfc->pending_oob[1], chip->oob_poi, mtd->oobsize);

	/* merge oob data */
	for (i = 0; i < mtd->oobsize; i++)
		nfc->pending_oob[1][i] &= nfc->pending_oob[0][i];

	mtk_nfc_page_erase_write(chip, nfc->pending_page,
		nfc->pending_oob[1], page);

	/* restore original oob data */
	memcpy(chip->oob_poi, nfc->pending_oob[0], mtd->oobsize);

	return 1;
}

static int mtk_nfc_write_oob_std(struct nand_chip *chip, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);

	if (mtk_nfc_write_oob_compat_check(chip, page))
		return 0;

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);

	return mtk_nfc_write_page_hwecc(chip, nfc->buffer, 1, page);
}

static int mtk_nfc_read_page_hwecc(struct nand_chip *chip,
	u8 *buf, int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int bitflips = 0;
	int rc, i;
	u32 reg;

	nand_read_page_op(chip, page, 0, NULL, 0);

	nfi_set_reg16(nfc, CNFG_READ_EN | CNFG_AUTO_FMT_EN
			| CNFG_HW_ECC_EN, NFI_CNFG);

	nfc->ecc_cfg.op = ECC_DECODE;
	mtk_ecc_init(nfc, nfc->ecc, &nfc->ecc_cfg);
	mtk_ecc_enable(nfc->ecc, &nfc->ecc_cfg);

	reg =  chip->ecc.steps << CON_SEC_SHIFT | CON_BRD;
	nfi_writew(nfc, reg, NFI_CON);

	for (i = 0; i < chip->ecc.steps; i++) {
		mtk_nfc_read_buf(chip, data_ptr(chip, buf, i), chip->ecc.size);
		rc = mtk_ecc_wait_decode_done(nfc->ecc, i);

		mtk_nfc_read_fdm(chip, i, 1);

		if (rc < 0) {
			bitflips = -EIO;
		} else {
			rc = mtk_ecc_correct_check(mtd, nfc->ecc,
				data_ptr(chip, buf, i), oob_ptr(chip, i), i);

			if (rc < 0)
				bitflips = -ETIMEDOUT;
			else if (bitflips >= 0)
				bitflips += rc;
		}
	}

	mtk_ecc_disable(nfc->ecc);
	nfi_writew(nfc, 0, NFI_CON);

	return bitflips;
}

static int mtk_nfc_read_page_raw(struct nand_chip *chip,
				 u8 *buf, int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int i;
	u32 reg;

	nand_read_page_op(chip, page, 0, NULL, 0);

	nfi_set_reg16(nfc, CNFG_READ_EN, NFI_CNFG);
	nfi_clear_reg16(nfc, CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN, NFI_CNFG);
	reg =  chip->ecc.steps << CON_SEC_SHIFT | CON_BRD;
	nfi_writew(nfc, reg, NFI_CON);

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);
	mtk_nfc_read_buf(chip, nfc->buffer, mtd->writesize + mtd->oobsize);
	nfi_writew(nfc, 0, NFI_CON);

	for (i = 0; i < chip->ecc.steps; i++) {
		memcpy(oob_ptr(chip, i), mtk_oob_ptr(chip, i),
		       nfc->caps->fdm_size);
		memcpy(ecc_ptr(chip, i), mtk_ecc_ptr(chip, i),
		       mtk_nand->spare_per_sector - nfc->caps->fdm_size);

		if (buf)
			memcpy(data_ptr(chip, buf, i), mtk_data_ptr(chip, i),
			       chip->ecc.size);
	}

	return 0;
}

static int mtk_nfc_read_oob_raw(struct nand_chip *chip, int page)
{
	return mtk_nfc_read_page_raw(chip, NULL, 1, page);
}

static int mtk_nfc_read_oob_std(struct nand_chip *chip, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	nand_read_page_op(chip, page, 0, NULL, 0);

	return mtk_nfc_read_page_hwecc(chip, nfc->buffer, 1, page);
}

static inline void mtk_nfc_hw_init(struct mtk_nfc *nfc)
{
	/*
	 * CNRNB: nand ready/busy register
	 * -------------------------------
	 * 7:4: timeout register for polling the NAND busy/ready signal
	 * 0  : poll the status of the busy/ready signal after [7:4]*16 cycles.
	 */
	nfi_writew(nfc, 0xf1, NFI_CNRNB);
	nfi_writel(nfc, PAGEFMT_4K, NFI_PAGEFMT);

	mtk_nfc_hw_reset(nfc);

	nfi_readl(nfc, NFI_INTR_STA);
	nfi_writel(nfc, 0, NFI_INTR_EN);
}

static int mtk_nfc_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oob_region)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 eccsteps;

	eccsteps = mtd->writesize / chip->ecc.size;

	if (section >= eccsteps)
		return -ERANGE;

	oob_region->length = nfc->caps->fdm_size - 1;
	oob_region->offset = section * nfc->caps->fdm_size + 1;

	return 0;
}

static int mtk_nfc_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oob_region)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 eccsteps;

	if (section)
		return -ERANGE;

	eccsteps = mtd->writesize / chip->ecc.size;
	oob_region->offset = nfc->caps->fdm_size * eccsteps;
	oob_region->length = mtd->oobsize - oob_region->offset;

	return 0;
}

static const struct mtd_ooblayout_ops mtk_nfc_ooblayout_ops = {
	.free = mtk_nfc_ooblayout_free,
	.ecc = mtk_nfc_ooblayout_ecc,
};

static int mtk_nfc_block_bad(struct nand_chip *chip, loff_t ofs)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int page, res = 0, i = 0;
	u16 bad;

	if (chip->options & NAND_BBM_LASTPAGE)
		ofs += mtd->erasesize - mtd->writesize;

	page = (int) (ofs >> chip->page_shift) & chip->pagemask;

	do {
		nand_read_page_op(chip, page, chip->ecc.size + chip->badblockpos,
			NULL, 0);

		bad = chip->legacy.read_byte(chip);
		res = bad != 0xFF;

		ofs += mtd->writesize;
		page = (int) (ofs >> chip->page_shift) & chip->pagemask;
		i++;
	} while (!res && i < 2 && (chip->options & NAND_BBM_SECONDPAGE));

	return res;
}

static int mtk_nfc_block_markbad(struct nand_chip *chip, loff_t ofs)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	loff_t lofs;
	int page, ret = 0, res, i = 0;

	/* Create bad block mark OOB bata */
	memset(chip->oob_poi, 0xff, mtd->oobsize);
	chip->oob_poi[chip->badblockpos] = 0;

	/* For BootROM compatibility, always write to offset 0 */
	chip->oob_poi[0] = 0;

	/* Write to last page(s) if necessary */
	if (chip->options & NAND_BBM_LASTPAGE) {
		lofs = ofs + mtd->erasesize - mtd->writesize;
		if (chip->options & NAND_BBM_SECONDPAGE)
			lofs -= mtd->writesize;

		do {
			page = lofs >> mtd->writesize_shift;
			res = mtk_nfc_write_oob_raw(chip, page);
			if (!ret)
				ret = res;

			i++;
			lofs += mtd->writesize;
		} while ((chip->options & NAND_BBM_SECONDPAGE) && i < 2);
	}

	/* For BootROM compatibility, always write to first page(s) */
	i = 0;
	do {
		page = ofs >> mtd->writesize_shift;
		res = mtk_nfc_write_oob_raw(chip, page);
		if (!ret)
			ret = res;

		i++;
		ofs += mtd->writesize;
	} while ((chip->options & NAND_BBM_SECONDPAGE) && i < 2);

	return ret;
}

static int mtk_nfc_do_write_page_hwecc(struct nand_chip *chip,
		    const uint8_t *buf, int page)
{
	return mtk_nfc_write_page_hwecc(chip, buf, 1, page);
}

static int mtk_nfc_do_read_page_hwecc(struct nand_chip *chip,
		    uint8_t *buf, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	unsigned int ecc_failures = mtd->ecc_stats.failed;
	int status;

	nand_read_page_op(chip, page, 0, NULL, 0);

	status = mtk_nfc_read_page_hwecc(chip, buf, 1, page);

	if (status < 0)
		return status;

	if (mtd->ecc_stats.failed - ecc_failures)
		return -EBADMSG;

	return 0;
}

/* single_earase() ??? */
static int mtk_nfc_do_erase(struct nand_chip *chip, int page)
{
	unsigned int eraseblock;

	eraseblock = page >> (chip->phys_erase_shift - chip->page_shift);
	return nand_erase_op(chip, eraseblock);
}

static int mtk_nfc_page_erase_write(struct nand_chip *chip, const uint8_t *buf,
	const uint8_t *oob, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int pages_per_block, page_start;
	int i;

	pages_per_block = mtd->erasesize / mtd->writesize;
	page_start = page - page % pages_per_block;

	/* read all pages within this block except the one to be rewritten */
	for (i = 0; i < pages_per_block; i++) {
		if (page_start + i != page) {
			mtk_nfc_do_read_page_hwecc(chip,
				nfc->block_buffer[i], page_start + i);
			memcpy(nfc->block_buffer[i] + mtd->writesize,
				chip->oob_poi,
				nfc->caps->fdm_size * chip->ecc.steps);
		}
	}

	/* erase this block */
	mtk_nfc_do_erase(chip, page_start);

	/* write back pages except the one to be rewritten */
	for (i = 0; i < pages_per_block; i++) {
		if (page_start + i != page) {
			memcpy(chip->oob_poi,
				nfc->block_buffer[i] + mtd->writesize,
				nfc->caps->fdm_size * chip->ecc.steps);
			mtk_nfc_do_write_page_hwecc(chip,
				nfc->block_buffer[i], page_start + i);
		}
	}

	/* write page */
	memcpy(chip->oob_poi, oob, nfc->caps->fdm_size * chip->ecc.steps);

	mtk_nfc_do_write_page_hwecc(chip, nfc->pending_page, page);

	return 0;
}

static int mtk_nfc_write_page_compat_check(struct nand_chip *chip,
	const uint8_t *buf, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int i;

	/* backup pending oob data */
	memcpy(nfc->pending_oob[0], chip->oob_poi, mtd->oobsize);

	mtk_nfc_do_read_page_hwecc(chip, nfc->pending_page, page);

	if (mtk_nfc_check_empty_page(chip, nfc->pending_page)) {
		/* page is empty, writing directly */
		memcpy(chip->oob_poi, nfc->pending_oob[0], mtd->oobsize);
		return 0;
	}

	/* backup in-flash oob data */
	memcpy(nfc->pending_oob[1], chip->oob_poi, mtd->oobsize);

	/* merge page data */
	for (i = 0; i < mtd->writesize; i++)
		nfc->pending_page[i] &= buf[i];

	for (i = 0; i < mtd->oobsize; i++)
		nfc->pending_oob[1][i] &= nfc->pending_oob[0][i];

	mtk_nfc_page_erase_write(chip, nfc->pending_page,
		nfc->pending_oob[1], page);

	/* restore original oob */
	memcpy(chip->oob_poi, nfc->pending_oob[0], mtd->oobsize);

	return 1;
}

static int mtk_nfc_attach_chip(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct device *dev = mtd->dev.parent;
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	int len, i, npgs;

	if (chip->options & NAND_BUSWIDTH_16) {
		dev_err(dev, "16bits buswidth not supported");
		return -EINVAL;
	}

	dev_info(dev,"Using programmed access timings: %08x\n", nfi_readl(nfc, NFI_ACCCON));

	/* store bbt magic in page, cause OOB is not protected */
	if (chip->bbt_options & NAND_BBT_USE_FLASH)
		chip->bbt_options |= NAND_BBT_NO_OOB;

	len = mtd->writesize + mtd->oobsize;

	nfc->buffer = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!nfc->buffer)
		return  -ENOMEM;

	npgs = mtd->erasesize / mtd->writesize;
	nfc->block_buffer = devm_kzalloc(dev, npgs * sizeof(*nfc->block_buffer), GFP_KERNEL);
	if (!nfc->block_buffer)
		return -ENOMEM;

	for (i = 0; i < npgs; i++) {
		nfc->block_buffer[i] = devm_kzalloc(dev, len, GFP_KERNEL);
		if (!nfc->block_buffer[i])
			return -ENOMEM;
	}

	nfc->pending_page = devm_kzalloc(dev, mtd->writesize, GFP_KERNEL);
	if (!nfc->pending_page)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(nfc->pending_oob); i++) {
		nfc->pending_oob[i] = devm_kzalloc(dev, mtd->oobsize, GFP_KERNEL);
		if (!nfc->pending_oob[i])
			return -ENOMEM;
	}

	return 0;
}

static const struct nand_controller_ops mtk_nfc_controller_ops = {
	.attach_chip = mtk_nfc_attach_chip,
};

static int mtk_nfc_nand_chip_init(struct device *dev, struct mtk_nfc *nfc,
				  struct device_node *np)
{
	struct mtk_nfc_nand_chip *chip;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	int nsels, i, ret;
	u32 tmp;

	if (!of_get_property(np, "reg", &nsels))
		return -ENODEV;

	nsels /= sizeof(u32);
	if (!nsels || nsels > MTK_NAND_MAX_NSELS) {
		dev_err(dev, "invalid reg property size %d\n", nsels);
		return -EINVAL;
	}

	chip = devm_kzalloc(dev, sizeof(*chip) + nsels * sizeof(u8),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->nsels = nsels;
	for (i = 0; i < nsels; i++) {
		ret = of_property_read_u32_index(np, "reg", i, &tmp);
		if (ret) {
			dev_err(dev, "reg property failure : %d\n", ret);
			return ret;
		}
		chip->sels[i] = tmp;
	}

	nand = &chip->nand;
	nand->controller = &nfc->controller;

	nand_set_flash_node(nand, np);
	nand_set_controller_data(nand, nfc);

	nand->options |= NAND_USE_BOUNCE_BUFFER | NAND_NO_SUBPAGE_WRITE;
	nand->legacy.dev_ready = mtk_nfc_dev_ready;
	nand->legacy.select_chip = mtk_nfc_select_chip;
	nand->legacy.write_byte = mtk_nfc_write_byte;
	nand->legacy.write_buf = mtk_nfc_write_buf;
	nand->legacy.read_byte = mtk_nfc_read_byte;
	nand->legacy.read_buf = mtk_nfc_read_buf;
	nand->legacy.cmd_ctrl = mtk_nfc_cmd_ctrl;
	nand->legacy.block_bad = mtk_nfc_block_bad;
	nand->legacy.block_markbad = mtk_nfc_block_markbad;

	/* set default mode in case dt entry is missing */
	nand->ecc.mode = NAND_ECC_HW;

	nand->ecc.write_page_raw = mtk_nfc_write_page_raw;
	nand->ecc.write_page = mtk_nfc_write_page_hwecc;
	nand->ecc.write_oob_raw = mtk_nfc_write_oob_raw;
	nand->ecc.write_oob = mtk_nfc_write_oob_std;

	nand->ecc.read_page_raw = mtk_nfc_read_page_raw;
	nand->ecc.read_page = mtk_nfc_read_page_hwecc;
	nand->ecc.read_oob_raw = mtk_nfc_read_oob_raw;
	nand->ecc.read_oob = mtk_nfc_read_oob_std;

	mtd = nand_to_mtd(nand);
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = dev;
	mtd->name = MTK_NAME;
	mtd_set_ooblayout(mtd, &mtk_nfc_ooblayout_ops);

	mtk_nfc_hw_init(nfc);

	ret = nand_scan(nand, nsels);
	if (ret)
		return ret;

	mtd->dev.of_node = of_get_next_available_child(dev->of_node, NULL);
	if (!mtd->dev.of_node) {
		dev_err(dev, "no nand device to configure\n");
		return -ENODEV;
	}

	ret = mtd_device_register(mtd, NULL, 0);

	if (ret) {
		dev_err(dev, "mtd parse partition error\n");
		nand_release(nand);
		return ret;
	}

	list_add_tail(&chip->node, &nfc->chips);

	return 0;
}

static int mtk_nfc_nand_chips_init(struct device *dev, struct mtk_nfc *nfc)
{
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int ret;

	for_each_child_of_node(np, nand_np) {
		ret = mtk_nfc_nand_chip_init(dev, nfc, nand_np);
		if (ret) {
			of_node_put(nand_np);
			return ret;
		}
	}

	return 0;
}

static const struct mtk_nfc_caps mtk_nfc_caps_mt7621 = {
	.pageformat_spare_shift = 4,
	.max_sector = 8,
	.sector_size = 512,
	.fdm_size = 8,
	.fdm_ecc_size = 8,
};

static const struct of_device_id mtk_nfc_id_table[] = {
	{
		.compatible = "mediatek,mt7621-nfc",
		.data = &mtk_nfc_caps_mt7621,
	},
	{}
};
MODULE_DEVICE_TABLE(of, mtk_nfc_id_table);

static int mtk_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtk_nfc *nfc;
	struct resource *res;
	int ret;

	nfc = devm_kzalloc(dev, sizeof(*nfc), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nand_controller_init(&nfc->controller);
	INIT_LIST_HEAD(&nfc->chips);
	nfc->controller.ops = &mtk_nfc_controller_ops;

	/* probe defer if not ready */
	nfc->ecc = of_mtk_ecc_get(np);
	if (IS_ERR(nfc->ecc))
		return PTR_ERR(nfc->ecc);
	else if (!nfc->ecc)
		return -ENODEV;

	nfc->caps = of_device_get_match_data(dev);
	nfc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(nfc->regs)) {
		ret = PTR_ERR(nfc->regs);
		goto release_ecc;
	}

	platform_set_drvdata(pdev, nfc);

	ret = mtk_nfc_nand_chips_init(dev, nfc);
	if (ret) {
		dev_err(dev, "failed to init nand chips\n");
		goto release_ecc;
	}

	return 0;

release_ecc:
	mtk_ecc_release(nfc->ecc);

	return ret;
}

static int mtk_nfc_remove(struct platform_device *pdev)
{
	struct mtk_nfc *nfc = platform_get_drvdata(pdev);
	struct mtk_nfc_nand_chip *chip;

	while (!list_empty(&nfc->chips)) {
		chip = list_first_entry(&nfc->chips, struct mtk_nfc_nand_chip,
					node);
		nand_release(&chip->nand);
		list_del(&chip->node);
	}

	mtk_ecc_release(nfc->ecc);

	return 0;
}

static struct platform_driver mtk_nfc_driver = {
	.probe  = mtk_nfc_probe,
	.remove = mtk_nfc_remove,
	.driver = {
		.name  = MTK_NAME,
		.of_match_table = mtk_nfc_id_table,
	},
};

module_platform_driver(mtk_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xiangsheng Hou <xiangsheng.hou@mediatek.com>");
MODULE_AUTHOR("Weijie Gao <weijie.gao@mediatek.com>");
MODULE_DESCRIPTION("MTK Nand Flash Controller Driver");
