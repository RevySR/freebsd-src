/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Han Gao <gaohan@iscas.ac.cn>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Synopsys DesignWare Mobile Storage Host Controller with the PHY used by
 * the T-Head TH1520 and the Zhihe A210. HS200 and HS400 need tuning, which
 * is not implemented; eMMC with enhanced strobe runs in HS400ES.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/phy/phy.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>

#include <dev/sdhci/sdhci.h>
#include <dev/sdhci/sdhci_fdt.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"

#include "opt_mmccam.h"

#define	DWCMSHC_P_VENDOR_AREA1		0xe8
#define	 DWCMSHC_AREA1_MASK		0xfff
#define	DWCMSHC_EMMC_CONTROL		0x2c	/* In vendor area 1 */
#define	 DWCMSHC_CARD_IS_EMMC		(1 << 0)
#define	 DWCMSHC_ENHANCED_STROBE	(1 << 8)
#define	DWCMSHC_CTRL_HS400		0x7	/* Host control 2 UHS mode */

#define	PHY_CNFG			0x300
#define	 PHY_CNFG_RSTN_DEASSERT		(1 << 0)
#define	 PHY_CNFG_PAD_SP(x)		((x) << 16)	/* PMOS drive */
#define	 PHY_CNFG_PAD_SN(x)		((x) << 20)	/* NMOS drive */
#define	PHY_CMDPAD_CNFG			0x304
#define	PHY_DATAPAD_CNFG		0x306
#define	PHY_CLKPAD_CNFG			0x308
#define	PHY_STBPAD_CNFG			0x30a
#define	PHY_RSTNPAD_CNFG		0x30c
#define	 PHY_PAD_RXSEL_1V8		0x1
#define	 PHY_PAD_RXSEL_3V3		0x2
#define	 PHY_PAD_WEAKPULL_PULLUP	(0x1 << 3)
#define	 PHY_PAD_WEAKPULL_PULLDOWN	(0x2 << 3)
#define	 PHY_PAD_TXSLEW_CTRL_P(x)	((x) << 5)
#define	 PHY_PAD_TXSLEW_CTRL_N(x)	((x) << 9)
#define	PHY_SDCLKDL_CNFG		0x31d
#define	 PHY_SDCLKDL_CNFG_UPDATE	(1 << 4)
#define	PHY_SDCLKDL_DC			0x31e
#define	 PHY_SDCLKDL_DC_DEFAULT		0x32
#define	 PHY_SDCLKDL_DC_HS400		0x18
#define	PHY_DLL_CTRL			0x324
#define	 PHY_DLL_CTRL_ENABLE		0x1
#define	PHY_DLL_CNFG1			0x325
#define	 PHY_DLL_CNFG1_SLVDLY(x)	((x) << 4)
#define	 PHY_DLL_CNFG1_WAITCYCLE	0x5
#define	PHY_DLL_CNFG2			0x326
#define	 PHY_DLL_CNFG2_JUMPSTEP		0xa
#define	PHY_DLLDL_CNFG			0x328
#define	 PHY_DLLDL_CNFG_SLV_INPSEL(x)	((x) << 5)

#define	PHY_PAD_TXSLEW		(PHY_PAD_TXSLEW_CTRL_P(0x3) | \
				 PHY_PAD_TXSLEW_CTRL_N(0x3))

struct sdhci_zhihe_softc {
	struct sdhci_fdt_softc	base;
	clk_t			clk_bus;
	uint32_t		base_mhz;	/* Core clock, in MHz */
	bus_size_t		area1;		/* Vendor specific area 1 */
	bool			fixed_1v8;	/* I/O is wired to 1.8 V */
	bool			emmc;
	bool			hs400;		/* HS400 delay line selected */
};

static struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-dwcmshc",	1 },
	{ NULL,			0 }
};

#define	RD1(sc, o)	bus_read_1((sc)->base.mem_res[0], (o))
#define	RD2(sc, o)	bus_read_2((sc)->base.mem_res[0], (o))
#define	RD4(sc, o)	bus_read_4((sc)->base.mem_res[0], (o))
#define	WR1(sc, o, v)	bus_write_1((sc)->base.mem_res[0], (o), (v))
#define	WR2(sc, o, v)	bus_write_2((sc)->base.mem_res[0], (o), (v))
#define	WR4(sc, o, v)	bus_write_4((sc)->base.mem_res[0], (o), (v))

static void
sdhci_zhihe_phy_init(struct sdhci_zhihe_softc *sc)
{
	uint16_t pad;
	uint8_t rxsel;

	rxsel = sc->fixed_1v8 ? PHY_PAD_RXSEL_1V8 : PHY_PAD_RXSEL_3V3;

	/* Take the PHY out of reset and set the TX drive strength. */
	WR4(sc, PHY_CNFG, PHY_CNFG_RSTN_DEASSERT | PHY_CNFG_PAD_SP(0xc) |
	    PHY_CNFG_PAD_SN(0xc));

	/* Program the SD clock delay line while it is held for update. */
	WR1(sc, PHY_SDCLKDL_CNFG, PHY_SDCLKDL_CNFG_UPDATE);
	WR1(sc, PHY_SDCLKDL_DC, sc->hs400 ? PHY_SDCLKDL_DC_HS400 :
	    PHY_SDCLKDL_DC_DEFAULT);
	WR1(sc, PHY_DLL_CNFG2, PHY_DLL_CNFG2_JUMPSTEP);
	WR1(sc, PHY_SDCLKDL_CNFG,
	    RD1(sc, PHY_SDCLKDL_CNFG) & ~PHY_SDCLKDL_CNFG_UPDATE);

	pad = rxsel | PHY_PAD_WEAKPULL_PULLUP | PHY_PAD_TXSLEW;
	WR2(sc, PHY_CMDPAD_CNFG, pad);
	WR2(sc, PHY_DATAPAD_CNFG, pad);
	WR2(sc, PHY_RSTNPAD_CNFG, pad);
	WR2(sc, PHY_CLKPAD_CNFG, PHY_PAD_TXSLEW);
	WR2(sc, PHY_STBPAD_CNFG,
	    rxsel | PHY_PAD_WEAKPULL_PULLDOWN | PHY_PAD_TXSLEW);

	/* Sample through the slave delay line at 1.8 V. */
	WR1(sc, PHY_DLLDL_CNFG,
	    sc->fixed_1v8 ? PHY_DLLDL_CNFG_SLV_INPSEL(0x3) : 0);
	WR1(sc, PHY_DLL_CTRL, PHY_DLL_CTRL_ENABLE);

	if (sc->area1 == 0)
		sc->area1 = RD4(sc, DWCMSHC_P_VENDOR_AREA1) &
		    DWCMSHC_AREA1_MASK;
	if (sc->emmc)
		WR2(sc, sc->area1 + DWCMSHC_EMMC_CONTROL,
		    RD2(sc, sc->area1 + DWCMSHC_EMMC_CONTROL) |
		    DWCMSHC_CARD_IS_EMMC);
	WR1(sc, PHY_DLL_CNFG1, PHY_DLL_CNFG1_SLVDLY(0x2) |
	    PHY_DLL_CNFG1_WAITCYCLE);
}

static uint32_t
sdhci_zhihe_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_zhihe_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	val = RD4(sc, off);
	/* The base clock field does not describe the core clock. */
	if (off == SDHCI_CAPABILITIES) {
		val &= ~SDHCI_CLOCK_V3_BASE_MASK;
		val |= (sc->base_mhz << SDHCI_CLOCK_BASE_SHIFT) &
		    SDHCI_CLOCK_V3_BASE_MASK;
	}
	return (val);
}

static void
sdhci_zhihe_reset(device_t dev, struct sdhci_slot *slot, uint8_t mask)
{
	struct sdhci_zhihe_softc *sc;

	sc = device_get_softc(dev);
	sdhci_generic_reset(dev, slot, mask);

	/* A command line reset does not clear Command Complete. */
	if (mask & SDHCI_RESET_CMD)
		WR4(sc, SDHCI_INT_STATUS, SDHCI_INT_RESPONSE);
	if (sc->fixed_1v8)
		WR2(sc, SDHCI_HOST_CONTROL2,
		    RD2(sc, SDHCI_HOST_CONTROL2) | SDHCI_CTRL2_S18_ENABLE);
}

static void
sdhci_zhihe_set_uhs_timing(device_t dev, struct sdhci_slot *slot)
{
	struct sdhci_zhihe_softc *sc;
	const struct mmc_ios *ios;
	uint16_t ctrl, hostctrl2;
	bool hs400;

	sc = device_get_softc(dev);
	ios = &slot->host.ios;
	hs400 = ios->timing == bus_timing_mmc_hs400 ||
	    ios->timing == bus_timing_mmc_hs400es;

	sdhci_generic_set_uhs_timing(dev, slot);

	if (sc->area1 == 0)
		sc->area1 = RD4(sc, DWCMSHC_P_VENDOR_AREA1) &
		    DWCMSHC_AREA1_MASK;
	ctrl = RD2(sc, sc->area1 + DWCMSHC_EMMC_CONTROL);
	if (ios->timing == bus_timing_mmc_hs400es)
		ctrl |= DWCMSHC_ENHANCED_STROBE;
	else
		ctrl &= ~DWCMSHC_ENHANCED_STROBE;
	WR2(sc, sc->area1 + DWCMSHC_EMMC_CONTROL, ctrl);

	/* The controller uses its own encoding for HS400. */
	if (hs400 && ios->clock > SD_SDR50_MAX) {
		hostctrl2 = RD2(sc, SDHCI_HOST_CONTROL2);
		hostctrl2 &= ~SDHCI_CTRL2_UHS_MASK;
		hostctrl2 |= DWCMSHC_CTRL_HS400;
		WR2(sc, SDHCI_HOST_CONTROL2, hostctrl2);
	}

	sc->hs400 = hs400;
	sdhci_zhihe_phy_init(sc);
}

static int
sdhci_zhihe_switch_vccq(device_t brdev, device_t reqdev)
{
	struct sdhci_zhihe_softc *sc;
	struct sdhci_slot *slot;

	sc = device_get_softc(brdev);
	slot = device_get_ivars(reqdev);
	/* The I/O rail is wired to 1.8 V: only 1.8 V signaling works. */
	if (sc->fixed_1v8)
		return (slot->host.ios.vccq == vccq_180 ? 0 : EINVAL);
	return (sdhci_generic_switch_vccq(brdev, reqdev));
}

static int
sdhci_zhihe_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	/* XXX: register accesses on the SDIO-only controller hang the bus. */
	if (ofw_bus_has_prop(dev, "no-mmc") && ofw_bus_has_prop(dev, "no-sd")) {
		device_printf(dev, "SDIO is not supported yet\n");
		return (ENXIO);
	}

	device_set_desc(dev, "Zhihe A210 DesignWare SDHCI controller");
	return (BUS_PROBE_DEFAULT);
}

static int
sdhci_zhihe_attach(device_t dev)
{
	struct sdhci_zhihe_softc *sc;
	phandle_t node;
	uint64_t freq;
	int error;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	sc->base.quirks = SDHCI_QUIRK_PRESET_VALUE_BROKEN |
	    SDHCI_QUIRK_BROKEN_MMC_HS200;
	sc->fixed_1v8 = OF_hasprop(node, "mmc-ddr-1_8v") ||
	    OF_hasprop(node, "mmc-hs200-1_8v") ||
	    OF_hasprop(node, "mmc-hs400-1_8v");
	sc->emmc = OF_hasprop(node, "no-sd") && OF_hasprop(node, "no-sdio");

	error = clk_set_assigned(dev, node);
	if (error != 0 && error != ENOENT)
		device_printf(dev, "cannot set assigned clocks\n");

	if (clk_get_by_ofw_name(dev, 0, "core", &sc->base.clk_core) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "bus", &sc->clk_bus) != 0) {
		device_printf(dev, "cannot get clocks\n");
		return (ENXIO);
	}
	if (clk_enable(sc->clk_bus) != 0 ||
	    clk_enable(sc->base.clk_core) != 0) {
		device_printf(dev, "cannot enable clocks\n");
		return (ENXIO);
	}
	if (clk_get_freq(sc->base.clk_core, &freq) != 0 || freq == 0) {
		device_printf(dev, "cannot get core clock frequency\n");
		return (ENXIO);
	}
	sc->base_mhz = MIN(freq / 1000000, 255);

	error = sdhci_fdt_attach(dev);
	if (error != 0)
		return (error);

	/*
	 * HS400 with enhanced strobe samples data with the strobe from the
	 * card and needs no tuning.  HS200 and plain HS400 need tuning,
	 * which is not implemented.
	 */
	if (sc->fixed_1v8 && OF_hasprop(node, "mmc-hs400-1_8v") &&
	    OF_hasprop(node, "mmc-hs400-enhanced-strobe") &&
	    OF_hasprop(node, "non-removable")) {
		for (int i = 0; i < sc->base.num_slots; i++)
			sc->base.slots[i].host.caps |= MMC_CAP_MMC_HS400_180 |
			    MMC_CAP_MMC_ENH_STROBE | MMC_CAP_SIGNALING_180;
	}

	return (0);
}

static device_method_t sdhci_zhihe_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		sdhci_zhihe_probe),
	DEVMETHOD(device_attach,	sdhci_zhihe_attach),

	/* SDHCI accessors */
	DEVMETHOD(sdhci_read_4,		sdhci_zhihe_read_4),
	DEVMETHOD(sdhci_reset,		sdhci_zhihe_reset),
	DEVMETHOD(sdhci_set_uhs_timing,	sdhci_zhihe_set_uhs_timing),

	/* mmcbr_if */
	DEVMETHOD(mmcbr_switch_vccq,	sdhci_zhihe_switch_vccq),

	DEVMETHOD_END
};
extern driver_t sdhci_fdt_driver;

DEFINE_CLASS_1(sdhci_zhihe, sdhci_fdt_zhihe_driver, sdhci_zhihe_methods,
    sizeof(struct sdhci_zhihe_softc), sdhci_fdt_driver);
DRIVER_MODULE(sdhci_zhihe, simplebus, sdhci_fdt_zhihe_driver, NULL, NULL);

#ifndef MMCCAM
MMC_DECLARE_BRIDGE(sdhci_fdt_zhihe);
#endif
