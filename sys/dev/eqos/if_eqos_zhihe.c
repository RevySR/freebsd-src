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

/* Zhihe A210 glue for the DesignWare EQOS Ethernet controller. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>

#include <machine/bus.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_media.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>
#include <dev/syscon/syscon.h>

#include <dev/eqos/if_eqos_var.h>

#include "if_eqos_if.h"
#include "syscon_if.h"

/* Registers in the GMAC system controller ("zhihe,gmacsys"). */
#define	A210_GMAC_CLKCTRL		0x00
#define	 A210_GMAC_CLKCTRL_EN_ALL	0x1f
#define	 A210_GMAC_CLKCTRL_SPEED_MASK	(0x3 << 8)
#define	 A210_GMAC_CLKCTRL_SPEED_1G	(0x0 << 8)
#define	 A210_GMAC_CLKCTRL_SPEED_10M	(0x2 << 8)
#define	 A210_GMAC_CLKCTRL_SPEED_100M	(0x3 << 8)
#define	A210_GMAC_CTRL			0x04
#define	 A210_GMAC_CTRL_INTF_MASK	0xf
#define	 A210_GMAC_CTRL_INTF_RGMII	0x1
#define	 A210_GMAC_CTRL_INTF_RMII	0x4

#define	RD4(sc, o)	bus_read_4((sc)->base.res[EQOS_RES_MEM], (o))
#define	WR4(sc, o, v)	bus_write_4((sc)->base.res[EQOS_RES_MEM], (o), (v))

static const struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-dwmac",	1 },
	{ NULL,			0 }
};

struct eqos_zhihe_softc {
	struct eqos_softc	base;
	struct syscon		*gmacsys;
	bus_size_t		gmacsys_off;	/* In MAC window if !gmacsys */
	clk_t			stmmaceth;
	clk_t			pclk;
};

static void
eqos_zhihe_gmacsys_modify(struct eqos_zhihe_softc *sc, bus_size_t reg,
    uint32_t clr, uint32_t set)
{

	if (sc->gmacsys != NULL) {
		SYSCON_MODIFY_4(sc->gmacsys, reg, clr, set);
		return;
	}
	reg += sc->gmacsys_off;
	WR4(sc, reg, (RD4(sc, reg) & ~clr) | set);
}

/*
 * The GMAC system controller sits at the end of the register window the
 * device tree gives the MAC, and memory resources cannot overlap, so it is
 * usually reached through the MAC mapping rather than a syscon device.
 */
static int
eqos_zhihe_get_gmacsys(device_t dev, struct eqos_zhihe_softc *sc)
{
	struct resource *res;
	phandle_t node, xref;
	bus_addr_t addr;
	bus_size_t size;

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "zhihe,gmacsys", &xref, sizeof(xref)) <= 0)
		return (ENXIO);
	if (ofw_reg_to_paddr(OF_node_from_xref(xref), 0, &addr, &size,
	    NULL) != 0)
		return (ENXIO);

	res = sc->base.res[EQOS_RES_MEM];
	if (addr >= rman_get_start(res) &&
	    addr + A210_GMAC_CTRL + 4 <= rman_get_end(res) + 1) {
		sc->gmacsys_off = addr - rman_get_start(res);
		return (0);
	}
	return (syscon_get_by_ofw_property(dev, node, "zhihe,gmacsys",
	    &sc->gmacsys));
}

static int
eqos_zhihe_set_speed(device_t dev, int speed)
{
	struct eqos_zhihe_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	switch (speed) {
	case IFM_1000_T:
	case IFM_1000_SX:
		val = A210_GMAC_CLKCTRL_SPEED_1G;
		break;
	case IFM_100_TX:
		val = A210_GMAC_CLKCTRL_SPEED_100M;
		break;
	case IFM_10_T:
		val = A210_GMAC_CLKCTRL_SPEED_10M;
		break;
	default:
		device_printf(dev, "unsupported media %u\n", speed);
		return (EINVAL);
	}

	eqos_zhihe_gmacsys_modify(sc, A210_GMAC_CLKCTRL,
	    A210_GMAC_CLKCTRL_SPEED_MASK, val);
	return (0);
}

static int
eqos_zhihe_enable_clock(device_t dev, const char *name, clk_t *clk)
{
	int error;

	error = clk_get_by_ofw_name(dev, 0, name, clk);
	if (error != 0) {
		device_printf(dev, "cannot get %s clock\n", name);
		return (error);
	}
	error = clk_enable(*clk);
	if (error != 0)
		device_printf(dev, "cannot enable %s clock\n", name);
	return (error);
}

static int
eqos_zhihe_init(device_t dev)
{
	struct eqos_zhihe_softc *sc;
	hwreset_t rst_ahb, rst_stmmaceth;
	phandle_t node;
	uint8_t eaddr[ETHER_ADDR_LEN];
	uint64_t freq;
	char *mode;
	uint32_t intf, pbl;
	int error;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	if (eqos_zhihe_get_gmacsys(dev, sc) != 0) {
		device_printf(dev, "cannot get GMAC system controller\n");
		return (ENXIO);
	}

	intf = A210_GMAC_CTRL_INTF_RGMII;
	if (OF_getprop_alloc(node, "phy-mode", (void **)&mode) > 0) {
		if (strcmp(mode, "rmii") == 0)
			intf = A210_GMAC_CTRL_INTF_RMII;
		else if (strncmp(mode, "rgmii", 5) != 0) {
			device_printf(dev, "unsupported phy-mode %s\n", mode);
			OF_prop_free(mode);
			return (ENXIO);
		}
		OF_prop_free(mode);
	}

	if (OF_getencprop(node, "snps,pbl", &pbl, sizeof(pbl)) > 0) {
		sc->base.txpbl = pbl;
		sc->base.rxpbl = pbl;
	}

	if (hwreset_get_by_ofw_name(dev, 0, "ahb", &rst_ahb) != 0 ||
	    hwreset_get_by_ofw_name(dev, 0, "stmmaceth", &rst_stmmaceth) != 0) {
		device_printf(dev, "cannot get resets\n");
		return (ENXIO);
	}
	if (hwreset_assert(rst_stmmaceth) != 0 ||
	    hwreset_assert(rst_ahb) != 0) {
		device_printf(dev, "cannot assert resets\n");
		return (ENXIO);
	}

	error = eqos_zhihe_enable_clock(dev, "pclk", &sc->pclk);
	if (error == 0)
		error = eqos_zhihe_enable_clock(dev, "stmmaceth",
		    &sc->stmmaceth);
	if (error != 0)
		return (ENXIO);

	/* The MDIO clock divider is derived from the CSR (APB) clock. */
	error = clk_get_freq(sc->pclk, &freq);
	if (error != 0 || freq == 0) {
		device_printf(dev, "cannot get pclk frequency\n");
		return (ENXIO);
	}
	sc->base.csr_clock = freq;
	if (freq < 35000000)
		sc->base.csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_20_35;
	else if (freq < 60000000)
		sc->base.csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_35_60;
	else if (freq < 100000000)
		sc->base.csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_60_100;
	else if (freq < 150000000)
		sc->base.csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_100_150;
	else if (freq < 250000000)
		sc->base.csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_150_250;
	else
		sc->base.csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_250_300;

	if (hwreset_deassert(rst_ahb) != 0 ||
	    hwreset_deassert(rst_stmmaceth) != 0) {
		device_printf(dev, "cannot deassert resets\n");
		return (ENXIO);
	}

	/* Select the PHY interface and enable the internal MAC clocks. */
	eqos_zhihe_gmacsys_modify(sc, A210_GMAC_CTRL,
	    A210_GMAC_CTRL_INTF_MASK, intf);
	eqos_zhihe_gmacsys_modify(sc, A210_GMAC_CLKCTRL,
	    A210_GMAC_CLKCTRL_EN_ALL, A210_GMAC_CLKCTRL_EN_ALL);

	if (OF_getprop(node, "local-mac-address", eaddr, sizeof(eaddr)) ==
	    sizeof(eaddr)) {
		WR4(sc, GMAC_MAC_ADDRESS0_HIGH, eaddr[5] | (eaddr[4] << 8));
		WR4(sc, GMAC_MAC_ADDRESS0_LOW, eaddr[3] | (eaddr[2] << 8) |
		    (eaddr[1] << 16) | (eaddr[0] << 24));
	}

	return (0);
}

static int
eqos_zhihe_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 DesignWare EQOS Gigabit Ethernet");
	return (BUS_PROBE_DEFAULT);
}

static device_method_t eqos_zhihe_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		eqos_zhihe_probe),

	/* EQOS interface */
	DEVMETHOD(if_eqos_init,		eqos_zhihe_init),
	DEVMETHOD(if_eqos_set_speed,	eqos_zhihe_set_speed),

	DEVMETHOD_END
};

DEFINE_CLASS_1(eqos, eqos_zhihe_driver, eqos_zhihe_methods,
    sizeof(struct eqos_zhihe_softc), eqos_driver);
DRIVER_MODULE(eqos_zhihe, simplebus, eqos_zhihe_driver, 0, 0);
