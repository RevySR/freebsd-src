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
 * Synopsys USB 3 C10 PHY of the Zhihe A210 USB31 controller.  Its Type-C
 * assist block (TCA) routes the SuperSpeed lanes for the plug orientation
 * reported by the Type-C port controller.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/phy/phy_usb.h>

#include <riscv/zhihe/a210_c10phy.h>

#include "phynode_if.h"

#define	TCA_CLOCK_RESET		0x00
#define	 TCA_RESET_RELEASE	(1 << 9)
#define	TCA_INTERRUPT_STATUS	0x08
#define	 TCA_INTERRUPT_CLEAR	0xffff
#define	TCA_GLOBAL_CONFIG	0x10
#define	 TCA_OPERATION_MODE	0x3
#define	 TCA_SYSTEM_MODE	0
#define	 TCA_SYNC_MODE		1
#define	TCA_TCPC		0x14
#define	 TCA_TCPC_VALID		(1 << 4)
#define	 TCA_TCPC_LOW_POWER	(1 << 3)
#define	 TCA_TCPC_FLIP		(1 << 2)
#define	 TCA_TCPC_USB		(1 << 0)
#define	TCA_SYSTEM_CONFIG	0x18
#define	 TCA_SYSTEM_DISABLE	(1 << 3)
#define	 TCA_SYSTEM_FLIP	(1 << 2)
#define	 TCA_SYSTEM_CONNECTION	0x3
#define	 TCA_SYSTEM_USB_DP	3

#define	RD4(sc, o)	bus_read_4((sc)->res, (o))
#define	WR4(sc, o, v)	bus_write_4((sc)->res, (o), (v))

static struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-c10-phy",	1 },
	{ NULL,			0 }
};

struct a210_c10phy_softc {
	device_t		dev;
	struct resource		*res;
	struct mtx		mtx;
	clk_t			clk_apb;
	clk_t			clk_ref;
	clk_t			clk_suspend;
	hwreset_t		rst_apb;
	hwreset_t		rst_phy;
	int			orientation;
	bool			initialized;
};

struct a210_c10phy_node {
	device_t		dev;
	int			mode;
};

/* Route the lanes for the current orientation; called with mtx held. */
static void
a210_c10phy_configure(struct a210_c10phy_softc *sc)
{
	uint32_t tcpc, val;

	tcpc = TCA_TCPC_VALID;

	val = RD4(sc, TCA_GLOBAL_CONFIG) & ~TCA_OPERATION_MODE;
	WR4(sc, TCA_GLOBAL_CONFIG, val | TCA_SYSTEM_MODE);

	val = RD4(sc, TCA_SYSTEM_CONFIG) | TCA_SYSTEM_DISABLE;
	WR4(sc, TCA_SYSTEM_CONFIG, val);
	/* Allow PHY to quiesce before switching lanes. */
	DELAY(10);
	val &= ~(TCA_SYSTEM_FLIP | TCA_SYSTEM_CONNECTION);
	val |= TCA_SYSTEM_USB_DP;
	if (sc->orientation == A210_C10_REVERSE) {
		val |= TCA_SYSTEM_FLIP;
		tcpc |= TCA_TCPC_FLIP;
	}
	WR4(sc, TCA_SYSTEM_CONFIG, val);
	DELAY(10);
	WR4(sc, TCA_SYSTEM_CONFIG, val & ~TCA_SYSTEM_DISABLE);

	val = RD4(sc, TCA_GLOBAL_CONFIG) & ~TCA_OPERATION_MODE;
	WR4(sc, TCA_GLOBAL_CONFIG, val | TCA_SYNC_MODE);
	/* Clear any pending acks before triggering the handshake. */
	WR4(sc, TCA_INTERRUPT_STATUS, TCA_INTERRUPT_CLEAR);

	if (sc->orientation == A210_C10_DETACHED)
		tcpc |= TCA_TCPC_LOW_POWER;
	else
		tcpc |= TCA_TCPC_USB;
	WR4(sc, TCA_TCPC, tcpc);
	/* The controller completes the handshake after leaving reset. */
}

int
a210_c10phy_set_orientation(device_t dev, int orientation)
{
	struct a210_c10phy_softc *sc;

	if (orientation != A210_C10_DETACHED &&
	    orientation != A210_C10_NORMAL && orientation != A210_C10_REVERSE)
		return (EINVAL);

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	if (sc->orientation != orientation) {
		sc->orientation = orientation;
		if (sc->initialized)
			a210_c10phy_configure(sc);
	}
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
a210_c10phy_enable(struct phynode *phynode, bool enable)
{
	struct a210_c10phy_node *node;
	struct a210_c10phy_softc *sc;
	int error;

	node = phynode_get_softc(phynode);
	sc = device_get_softc(node->dev);

	/* Clocks and resets come from firmware services attached later. */
	if (sc->clk_apb == NULL &&
	    (clk_get_by_ofw_name(sc->dev, 0, "apb", &sc->clk_apb) != 0 ||
	    clk_get_by_ofw_name(sc->dev, 0, "ref", &sc->clk_ref) != 0 ||
	    clk_get_by_ofw_name(sc->dev, 0, "suspend", &sc->clk_suspend) != 0 ||
	    hwreset_get_by_ofw_name(sc->dev, 0, "apb", &sc->rst_apb) != 0 ||
	    hwreset_get_by_ofw_name(sc->dev, 0, "phy", &sc->rst_phy) != 0)) {
		sc->clk_apb = NULL;
		device_printf(sc->dev, "cannot get clocks or resets\n");
		return (ENXIO);
	}

	mtx_lock(&sc->mtx);
	if (!enable) {
		sc->initialized = false;
		mtx_unlock(&sc->mtx);
		hwreset_assert(sc->rst_phy);
		hwreset_assert(sc->rst_apb);
		clk_disable(sc->clk_suspend);
		clk_disable(sc->clk_ref);
		clk_disable(sc->clk_apb);
		return (0);
	}
	mtx_unlock(&sc->mtx);

	error = clk_enable(sc->clk_apb);
	if (error == 0)
		error = clk_enable(sc->clk_ref);
	if (error == 0)
		error = clk_enable(sc->clk_suspend);
	if (error == 0)
		error = hwreset_deassert(sc->rst_apb);
	if (error == 0)
		error = hwreset_assert(sc->rst_phy);
	if (error != 0) {
		device_printf(sc->dev, "cannot enable PHY resources\n");
		return (error);
	}

	mtx_lock(&sc->mtx);
	WR4(sc, TCA_CLOCK_RESET, RD4(sc, TCA_CLOCK_RESET) & ~TCA_RESET_RELEASE);
	DELAY(100);
	WR4(sc, TCA_CLOCK_RESET, RD4(sc, TCA_CLOCK_RESET) | TCA_RESET_RELEASE);
	a210_c10phy_configure(sc);
	mtx_unlock(&sc->mtx);

	error = hwreset_deassert(sc->rst_phy);
	if (error != 0)
		return (error);
	DELAY(1500);

	mtx_lock(&sc->mtx);
	sc->initialized = true;
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
a210_c10phy_get_mode(struct phynode *phynode, int *mode)
{
	struct a210_c10phy_node *node;

	node = phynode_get_softc(phynode);
	*mode = node->mode;
	return (0);
}

static int
a210_c10phy_set_mode(struct phynode *phynode, int mode)
{
	struct a210_c10phy_node *node;

	node = phynode_get_softc(phynode);
	node->mode = mode;
	return (0);
}

static phynode_method_t a210_c10phy_phynode_methods[] = {
	PHYNODEUSBMETHOD(phynode_enable,	a210_c10phy_enable),
	PHYNODEMETHOD(phynode_usb_get_mode,	a210_c10phy_get_mode),
	PHYNODEMETHOD(phynode_usb_set_mode,	a210_c10phy_set_mode),
	PHYNODEUSBMETHOD_END
};
DEFINE_CLASS_1(a210_c10phy_phynode, a210_c10phy_phynode_class,
    a210_c10phy_phynode_methods, sizeof(struct a210_c10phy_node),
    phynode_usb_class);

static int
a210_c10phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 Synopsys C10 USB 3 PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
a210_c10phy_attach(device_t dev)
{
	struct a210_c10phy_softc *sc;
	struct phynode_init_def def;
	struct a210_c10phy_node *node;
	struct phynode *phynode;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/*
	 * A switched port starts in low power until the Type-C port
	 * controller reports an orientation.  A fixed host always uses the
	 * normal lane routing.
	 */
	if (OF_hasprop(ofw_bus_get_node(dev), "orientation-switch"))
		sc->orientation = A210_C10_DETACHED;
	else
		sc->orientation = A210_C10_NORMAL;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot map registers\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	bzero(&def, sizeof(def));
	def.id = 0;
	def.ofw_node = ofw_bus_get_node(dev);
	phynode = phynode_create(dev, &a210_c10phy_phynode_class, &def);
	if (phynode == NULL)
		goto fail;
	node = phynode_get_softc(phynode);
	node->dev = dev;
	node->mode = PHY_USB_MODE_HOST;
	if (phynode_register(phynode) == NULL)
		goto fail;
	return (0);

fail:
	device_printf(dev, "cannot register phy\n");
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
	mtx_destroy(&sc->mtx);
	return (ENXIO);
}

static device_method_t a210_c10phy_methods[] = {
	DEVMETHOD(device_probe,		a210_c10phy_probe),
	DEVMETHOD(device_attach,	a210_c10phy_attach),

	DEVMETHOD_END
};

static driver_t a210_c10phy_driver = {
	"a210_c10phy",
	a210_c10phy_methods,
	sizeof(struct a210_c10phy_softc),
};

EARLY_DRIVER_MODULE(a210_c10phy, simplebus, a210_c10phy_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(a210_c10phy, 1);
