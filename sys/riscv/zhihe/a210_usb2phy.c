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

/* USB 2.0 femtoPHYs of the Zhihe A210 USB20 block. */

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
#include <dev/phy/phy_usb.h>

#include "phynode_if.h"
#include "phynode_usb_if.h"

#define	A210_USB2_ANALOG		0x00
#define	 A210_USB2_TX_VREF_SHIFT	17
#define	 A210_USB2_TX_VREF_MASK		(0xf << A210_USB2_TX_VREF_SHIFT)
#define	 A210_USB2_TX_LEVEL		11
#define	A210_USB2_CONFIG		0x04
#define	A210_USB31_OTG_CONTROL		0x20
#define	 A210_USB31_VBUS_COMPARE	(1 << 12)
#define	 A210_USB31_OTG_DISABLE		(1 << 4)

/*
 * USB20 PHYs: per-port D+/D- pulldown enables, set for host mode.
 * The USB31 controller's USB 2.0 PHY has no pulldown control.
 */
#define	A210_USB31_PHY			(1u << 31)
static struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-usb2-phy",	0x3 << 0 },
	{ "zhihe,a210-usb2-phy1",	0x3 << 2 },
	{ "zhihe,a210-usb31-usb2-phy",	A210_USB31_PHY },
	{ NULL,				0 }
};

struct a210_usb2phy_softc {
	device_t		dev;
	struct resource		*res;
	clk_t			clk;
	hwreset_t		rst;
	uint32_t		pulldown;
};

struct a210_usb2phy_node {
	device_t		dev;
	int			mode;
};

static int
a210_usb2phy_enable(struct phynode *phynode, bool enable)
{
	struct a210_usb2phy_node *node;
	struct a210_usb2phy_softc *sc;
	uint32_t val;
	int error;

	node = phynode_get_softc(phynode);
	sc = device_get_softc(node->dev);

	/*
	 * The clock and reset come from firmware services that attach after
	 * this driver, so look them up when the controller enables the PHY.
	 */
	if (sc->clk == NULL &&
	    clk_get_by_ofw_name(sc->dev, 0, "apb", &sc->clk) != 0) {
		device_printf(sc->dev, "cannot get clock\n");
		return (ENXIO);
	}
	if (sc->rst == NULL &&
	    hwreset_get_by_ofw_name(sc->dev, 0, "phy", &sc->rst) != 0) {
		device_printf(sc->dev, "cannot get reset\n");
		return (ENXIO);
	}

	if (!enable) {
		hwreset_assert(sc->rst);
		return (clk_disable(sc->clk));
	}

	error = clk_enable(sc->clk);
	if (error != 0)
		return (error);
	error = hwreset_assert(sc->rst);
	if (error != 0)
		goto fail;
	DELAY(200);

	if (sc->pulldown == A210_USB31_PHY) {
		/* VBUS itself is switched by the Type-C port controller. */
		val = bus_read_4(sc->res, A210_USB31_OTG_CONTROL);
		val &= ~A210_USB31_OTG_DISABLE;
		val |= A210_USB31_VBUS_COMPARE;
		bus_write_4(sc->res, A210_USB31_OTG_CONTROL, val);
	} else {
		val = bus_read_4(sc->res, A210_USB2_ANALOG);
		val &= ~A210_USB2_TX_VREF_MASK;
		val |= A210_USB2_TX_LEVEL << A210_USB2_TX_VREF_SHIFT;
		bus_write_4(sc->res, A210_USB2_ANALOG, val);

		val = bus_read_4(sc->res, A210_USB2_CONFIG) & ~sc->pulldown;
		if (node->mode == PHY_USB_MODE_HOST)
			val |= sc->pulldown;
		bus_write_4(sc->res, A210_USB2_CONFIG, val);
	}

	error = hwreset_deassert(sc->rst);
	if (error != 0)
		goto fail;
	DELAY(1500);
	return (0);

fail:
	clk_disable(sc->clk);
	return (error);
}

static int
a210_usb2phy_get_mode(struct phynode *phynode, int *mode)
{
	struct a210_usb2phy_node *node;

	node = phynode_get_softc(phynode);
	*mode = node->mode;
	return (0);
}

static int
a210_usb2phy_set_mode(struct phynode *phynode, int mode)
{
	struct a210_usb2phy_node *node;

	if (mode != PHY_USB_MODE_HOST && mode != PHY_USB_MODE_DEVICE)
		return (EINVAL);
	node = phynode_get_softc(phynode);
	node->mode = mode;
	return (0);
}

static phynode_method_t a210_usb2phy_phynode_methods[] = {
	PHYNODEUSBMETHOD(phynode_enable,	a210_usb2phy_enable),
	PHYNODEMETHOD(phynode_usb_get_mode,	a210_usb2phy_get_mode),
	PHYNODEMETHOD(phynode_usb_set_mode,	a210_usb2phy_set_mode),
	PHYNODEUSBMETHOD_END
};
DEFINE_CLASS_1(a210_usb2phy_phynode, a210_usb2phy_phynode_class,
    a210_usb2phy_phynode_methods, sizeof(struct a210_usb2phy_node),
    phynode_usb_class);

static int
a210_usb2phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 USB 2.0 PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
a210_usb2phy_attach(device_t dev)
{
	struct a210_usb2phy_softc *sc;
	struct phynode_init_def def;
	struct a210_usb2phy_node *node;
	struct phynode *phynode;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->pulldown = ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}

	bzero(&def, sizeof(def));
	def.id = 0;
	def.ofw_node = ofw_bus_get_node(dev);
	phynode = phynode_create(dev, &a210_usb2phy_phynode_class, &def);
	if (phynode == NULL) {
		device_printf(dev, "cannot create phy\n");
		goto fail;
	}
	node = phynode_get_softc(phynode);
	node->dev = dev;
	node->mode = PHY_USB_MODE_HOST;
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "cannot register phy\n");
		goto fail;
	}
	return (0);

fail:
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
	return (ENXIO);
}

static device_method_t a210_usb2phy_methods[] = {
	DEVMETHOD(device_probe,		a210_usb2phy_probe),
	DEVMETHOD(device_attach,	a210_usb2phy_attach),

	DEVMETHOD_END
};

static driver_t a210_usb2phy_driver = {
	"a210_usb2phy",
	a210_usb2phy_methods,
	sizeof(struct a210_usb2phy_softc),
};

EARLY_DRIVER_MODULE(a210_usb2phy, simplebus, a210_usb2phy_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
