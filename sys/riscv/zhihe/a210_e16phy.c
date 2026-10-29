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
 * Synopsys E16 PHY shared by the Zhihe A210 PCIe and SATA controllers.
 * The "data-lanes" property selects how its four lanes are routed; every
 * consumer shares that fixed routing and the single initialization.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/phy/phy.h>

#include "phynode_if.h"

#define	E16_GLOBAL_CTRL		0x000
#define	 E16_GLOBAL_OVERRIDE	((1 << 16) | (1 << 20))
#define	 E16_SPLIT_PCIE		(1 << 8)
#define	 E16_SATA_ENABLE	(1 << 0)
#define	E16_PORT_SOURCE		0x004
#define	 E16_SATA_SOURCE	((1 << 8) | (1 << 12))
#define	E16_PROTOCOL		0x008
#define	 E16_SATA_PROTOCOL	((1 << 9) | (1 << 13))
#define	E16_SRAM_STATUS		0x00c
#define	 E16_SRAM_INIT_DONE	((1 << 8) | (1 << 24))
#define	E16_RES_TUNE		0x048
#define	 E16_RES_ACK_IN		(1 << 16)
#define	 E16_RTUNE_ACK		(1 << 4)
#define	 E16_RTUNE_REQUEST	(1 << 0)
#define	E16_PCIE_OVERRIDE	0x108
#define	 E16_PCIE_OVERRIDE_EN	((1 << 0) | (1 << 4))

/* Analog settings from the vendor's 100 MHz reference-clock profile. */
#define	E16_MPLL_CTRL		0x0048
#define	 E16_MPLL_100MHZ	0x0350
#define	E16_RX_EQ_CTRL(lane)	(0x4008 + (lane) * 0x400)
#define	 E16_RX_EQ_DEFAULT	0x01f8
#define	E16_RX_PLL_CTRL(lane)	(0xc020 + (lane) * 0x400)
#define	 E16_RX_PLL_DEFAULT	0x000f

#define	E16_BANKS		2
#define	E16_LANES_PER_BANK	2
#define	E16_REF_FREQ		100000000

static struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-e16-phy",	1 },
	{ NULL,			0 }
};

struct a210_e16phy_softc {
	device_t		dev;
	struct resource		*sys;
	struct resource		*analog[E16_BANKS];
	clk_t			clk_apb;
	clk_t			clk_ref;
	hwreset_t		rst_apb;
	hwreset_t		rst_phy;
	uint32_t		global_ctrl;
	bool			sata;
};

struct a210_e16phy_node {
	device_t		dev;
};

static int
a210_e16phy_wait(struct resource *res, bus_size_t reg, uint32_t mask,
    uint32_t *val)
{
	int i;

	for (i = 0; i < 10000; i++) {
		*val = bus_read_4(res, reg);
		if ((*val & mask) == mask)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
a210_e16phy_cr_write(struct a210_e16phy_softc *sc, int bank, bus_size_t reg,
    uint32_t val)
{
	uint32_t readback;

	bus_write_4(sc->analog[bank], reg, val);
	readback = bus_read_4(sc->analog[bank], reg);
	if (readback != val) {
		device_printf(sc->dev, "PHY%d CR %#jx: wrote %#x, read %#x\n",
		    bank, (uintmax_t)reg, val, readback);
		return (EIO);
	}
	return (0);
}

static int
a210_e16phy_init(struct a210_e16phy_softc *sc)
{
	uint64_t freq;
	uint32_t val;
	int bank, error, lane;

	error = clk_get_freq(sc->clk_ref, &freq);
	if (error != 0 || freq != E16_REF_FREQ) {
		device_printf(sc->dev, "unsupported reference clock %ju Hz\n",
		    (uintmax_t)freq);
		return (EINVAL);
	}
	error = hwreset_deassert(sc->rst_apb);
	if (error == 0)
		error = hwreset_assert(sc->rst_phy);
	if (error != 0)
		return (error);
	DELAY(200);

	bus_write_4(sc->sys, E16_GLOBAL_CTRL, sc->global_ctrl);
	bus_write_4(sc->sys, E16_PORT_SOURCE, sc->sata ? E16_SATA_SOURCE : 0);
	bus_write_4(sc->sys, E16_PROTOCOL, sc->sata ? E16_SATA_PROTOCOL : 0);
	bus_write_4(sc->sys, E16_PCIE_OVERRIDE, E16_PCIE_OVERRIDE_EN);
	/* Request calibration only after the CR interface has been set up. */
	bus_write_4(sc->sys, E16_RES_TUNE, E16_RES_ACK_IN);

	error = hwreset_deassert(sc->rst_phy);
	if (error != 0)
		return (error);
	if (a210_e16phy_wait(sc->sys, E16_SRAM_STATUS, E16_SRAM_INIT_DONE,
	    &val) != 0) {
		device_printf(sc->dev, "initialization timed out: %#x\n", val);
		return (ETIMEDOUT);
	}

	/* Analog CR writes made under reset do not survive its release. */
	for (bank = 0; bank < E16_BANKS; bank++) {
		error = a210_e16phy_cr_write(sc, bank, E16_MPLL_CTRL,
		    E16_MPLL_100MHZ);
		for (lane = 0; error == 0 && lane < E16_LANES_PER_BANK;
		    lane++) {
			error = a210_e16phy_cr_write(sc, bank,
			    E16_RX_PLL_CTRL(lane), E16_RX_PLL_DEFAULT);
			if (error == 0)
				error = a210_e16phy_cr_write(sc, bank,
				    E16_RX_EQ_CTRL(lane), E16_RX_EQ_DEFAULT);
		}
		if (error != 0)
			return (error);
	}

	bus_write_4(sc->sys, E16_RES_TUNE, E16_RES_ACK_IN | E16_RTUNE_REQUEST);
	if (a210_e16phy_wait(sc->sys, E16_RES_TUNE, E16_RTUNE_ACK,
	    &val) != 0) {
		device_printf(sc->dev, "resistance calibration timed out\n");
		return (ETIMEDOUT);
	}

	/* Allow the analog PLLs to settle before a host starts LTSSM. */
	pause("e16pll", MAX(1, hz / 5));
	return (0);
}

static void
a210_e16phy_shutdown(struct a210_e16phy_softc *sc)
{

	hwreset_assert(sc->rst_phy);
	hwreset_assert(sc->rst_apb);
	clk_disable(sc->clk_ref);
	clk_disable(sc->clk_apb);
}

static int
a210_e16phy_enable(struct phynode *phynode, bool enable)
{
	struct a210_e16phy_node *node;
	struct a210_e16phy_softc *sc;
	int error;

	node = phynode_get_softc(phynode);
	sc = device_get_softc(node->dev);

	if (!enable) {
		a210_e16phy_shutdown(sc);
		return (0);
	}

	/* Clocks and resets come from providers attached later. */
	if (sc->clk_apb == NULL &&
	    (clk_get_by_ofw_name(sc->dev, 0, "apb", &sc->clk_apb) != 0 ||
	    clk_get_by_ofw_name(sc->dev, 0, "ref", &sc->clk_ref) != 0 ||
	    hwreset_get_by_ofw_name(sc->dev, 0, "apb", &sc->rst_apb) != 0 ||
	    hwreset_get_by_ofw_name(sc->dev, 0, "phy", &sc->rst_phy) != 0)) {
		sc->clk_apb = NULL;
		device_printf(sc->dev, "cannot get clocks or resets\n");
		return (ENXIO);
	}

	error = clk_enable(sc->clk_apb);
	if (error != 0)
		return (error);
	error = clk_enable(sc->clk_ref);
	if (error != 0) {
		clk_disable(sc->clk_apb);
		return (error);
	}
	error = a210_e16phy_init(sc);
	if (error != 0)
		a210_e16phy_shutdown(sc);
	return (error);
}

static int
a210_e16phy_set_mode(struct phynode *phynode, phy_mode_t mode,
    phy_submode_t submode)
{
	struct a210_e16phy_node *node;
	struct a210_e16phy_softc *sc;

	node = phynode_get_softc(phynode);
	sc = device_get_softc(node->dev);

	/* All consumers share the board's fixed lane routing. */
	if (mode == PHY_MODE_PCIE || (mode == PHY_MODE_SATA && sc->sata))
		return (0);
	return (EOPNOTSUPP);
}

static phynode_method_t a210_e16phy_phynode_methods[] = {
	PHYNODEMETHOD(phynode_enable,	a210_e16phy_enable),
	PHYNODEMETHOD(phynode_set_mode,	a210_e16phy_set_mode),
	PHYNODEMETHOD_END
};
DEFINE_CLASS_1(a210_e16phy_phynode, a210_e16phy_phynode_class,
    a210_e16phy_phynode_methods, sizeof(struct a210_e16phy_node),
    phynode_class);

static struct resource *
a210_e16phy_alloc(device_t dev, const char *name)
{
	int rid;

	if (ofw_bus_find_string_index(ofw_bus_get_node(dev), "reg-names",
	    name, &rid) != 0)
		return (NULL);
	return (bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE));
}

static int
a210_e16phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 Synopsys E16 PCIe/SATA PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
a210_e16phy_release(device_t dev)
{
	struct a210_e16phy_softc *sc;
	int i;

	sc = device_get_softc(dev);
	for (i = 0; i < E16_BANKS; i++)
		if (sc->analog[i] != NULL)
			bus_release_resource(dev, sc->analog[i]);
	if (sc->sys != NULL)
		bus_release_resource(dev, sc->sys);
	return (ENXIO);
}

static int
a210_e16phy_attach(device_t dev)
{
	struct a210_e16phy_softc *sc;
	struct phynode_init_def def;
	struct a210_e16phy_node *node;
	struct phynode *phynode;
	uint32_t lanes[4];
	phandle_t onode;

	sc = device_get_softc(dev);
	sc->dev = dev;
	onode = ofw_bus_get_node(dev);

	if (OF_getencprop(onode, "data-lanes", lanes, sizeof(lanes)) !=
	    sizeof(lanes)) {
		device_printf(dev, "missing lane routing\n");
		return (ENXIO);
	}
	sc->global_ctrl = E16_GLOBAL_OVERRIDE;
	if (lanes[0] != 1) {
		device_printf(dev, "invalid lane routing\n");
		return (ENXIO);
	} else if (lanes[1] == 1 && lanes[2] == 1 && lanes[3] == 1) {
		/* One four-lane PCIe port. */
	} else if (lanes[1] == 1 && lanes[2] == 2 && lanes[3] == 0) {
		sc->global_ctrl |= E16_SPLIT_PCIE;
	} else if ((lanes[1] == 1 || lanes[1] == 2) &&
	    lanes[2] == 3 && lanes[3] == 4) {
		sc->sata = true;
		sc->global_ctrl |= E16_SATA_ENABLE;
		if (lanes[1] == 2)
			sc->global_ctrl |= E16_SPLIT_PCIE;
	} else {
		device_printf(dev, "invalid lane routing\n");
		return (ENXIO);
	}

	sc->sys = a210_e16phy_alloc(dev, "sysreg");
	sc->analog[0] = a210_e16phy_alloc(dev, "phy0");
	sc->analog[1] = a210_e16phy_alloc(dev, "phy1");
	if (sc->sys == NULL || sc->analog[0] == NULL ||
	    sc->analog[1] == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (a210_e16phy_release(dev));
	}

	bzero(&def, sizeof(def));
	def.id = 0;
	def.ofw_node = onode;
	phynode = phynode_create(dev, &a210_e16phy_phynode_class, &def);
	if (phynode == NULL)
		goto fail;
	node = phynode_get_softc(phynode);
	node->dev = dev;
	if (phynode_register(phynode) == NULL)
		goto fail;
	return (0);

fail:
	device_printf(dev, "cannot register phy\n");
	return (a210_e16phy_release(dev));
}

static device_method_t a210_e16phy_methods[] = {
	DEVMETHOD(device_probe,		a210_e16phy_probe),
	DEVMETHOD(device_attach,	a210_e16phy_attach),

	DEVMETHOD_END
};

static driver_t a210_e16phy_driver = {
	"a210_e16phy",
	a210_e16phy_methods,
	sizeof(struct a210_e16phy_softc),
};

EARLY_DRIVER_MODULE(a210_e16phy, simplebus, a210_e16phy_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(a210_e16phy, 1);
