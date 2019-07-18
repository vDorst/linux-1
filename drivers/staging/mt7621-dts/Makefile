# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_DTB_GNUBEE1)      += gbpc1.dtb
dtb-$(CONFIG_DTB_GNUBEE2)      += gbpc2.dtb
dtb-$(CONFIG_DTB_UBNT_ERX_SFP) += UBNT-ERX-SFP.dtb
dtb-$(CONFIG_DTB_MTC_WR1201)   += mt7621_mtc_wr1201.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))
