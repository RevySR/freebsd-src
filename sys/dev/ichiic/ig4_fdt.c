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

/* Device tree attachment for the Synopsys DesignWare I2C controller. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sx.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/iicbus/iiconf.h>

#include <dev/ichiic/ig4_reg.h>
#include <dev/ichiic/ig4_var.h>

#include "ofw_bus_if.h"

static struct ofw_compat_data compat_data[] = {
	{ "snps,designware-i2c",	1 },
	{ NULL,				0 }
};

static int	ig4iic_fdt_detach(device_t dev);

static int
ig4iic_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Synopsys DesignWare I2C Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
ig4iic_fdt_clocks(device_t dev, ig4iic_softc_t *sc)
{
	clk_t clk;
	uint64_t freq;

	/* The interface clock is optional, the reference clock is not. */
	if (clk_get_by_ofw_name(dev, 0, "pclk", &clk) == 0 &&
	    clk_enable(clk) != 0) {
		device_printf(dev, "cannot enable pclk\n");
		return (ENXIO);
	}
	if (clk_get_by_ofw_name(dev, 0, "ref", &clk) != 0 &&
	    clk_get_by_ofw_index(dev, 0, 0, &clk) != 0) {
		device_printf(dev, "cannot get reference clock\n");
		return (ENXIO);
	}
	if (clk_enable(clk) != 0 || clk_get_freq(clk, &freq) != 0 ||
	    freq < 1000000) {
		device_printf(dev, "cannot use reference clock\n");
		return (ENXIO);
	}
	sc->ic_clock_rate = freq / 1000000;
	return (0);
}

static int
ig4iic_fdt_attach(device_t dev)
{
	ig4iic_softc_t *sc;
	hwreset_array_t rsts;
	pcell_t speed;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->version = IG4_FDT;

	error = ig4iic_fdt_clocks(dev, sc);
	if (error != 0)
		return (error);
	if (hwreset_array_get_ofw(dev, 0, &rsts) == 0 &&
	    hwreset_array_deassert(rsts) != 0) {
		device_printf(dev, "cannot deassert resets\n");
		return (ENXIO);
	}
	if (OF_getencprop(ofw_bus_get_node(dev), "clock-frequency", &speed,
	    sizeof(speed)) > 0 && speed <= 100000)
		sc->std_speed = true;

	sc->regs_rid = 0;
	sc->regs_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->regs_rid, RF_ACTIVE);
	if (sc->regs_res == NULL) {
		device_printf(dev, "unable to map registers\n");
		ig4iic_fdt_detach(dev);
		return (ENXIO);
	}
	sc->intr_rid = 0;
	sc->intr_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->intr_rid, RF_SHAREABLE | RF_ACTIVE);
	if (sc->intr_res == NULL) {
		device_printf(dev, "unable to map interrupt\n");
		ig4iic_fdt_detach(dev);
		return (ENXIO);
	}
	sc->platform_attached = true;

	error = ig4iic_attach(sc);
	if (error != 0)
		ig4iic_fdt_detach(dev);

	return (error);
}

static int
ig4iic_fdt_detach(device_t dev)
{
	ig4iic_softc_t *sc;
	int error;

	sc = device_get_softc(dev);
	if (sc->platform_attached) {
		error = ig4iic_detach(sc);
		if (error != 0)
			return (error);
		sc->platform_attached = false;
	}
	if (sc->intr_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->intr_rid,
		    sc->intr_res);
		sc->intr_res = NULL;
	}
	if (sc->regs_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->regs_rid,
		    sc->regs_res);
		sc->regs_res = NULL;
	}
	return (0);
}

static phandle_t
ig4iic_fdt_get_node(device_t bus, device_t dev)
{

	/* Share the controller node with the iicbus child. */
	return (ofw_bus_get_node(bus));
}

static device_method_t ig4iic_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ig4iic_fdt_probe),
	DEVMETHOD(device_attach,	ig4iic_fdt_attach),
	DEVMETHOD(device_detach,	ig4iic_fdt_detach),

	/* Bus interface */
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_alloc_resource,	bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,	bus_generic_adjust_resource),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_node,	ig4iic_fdt_get_node),

	/* iicbus interface */
	DEVMETHOD(iicbus_transfer,	ig4iic_transfer),
	DEVMETHOD(iicbus_reset,		ig4iic_reset),
	DEVMETHOD(iicbus_callback,	ig4iic_callback),

	DEVMETHOD_END
};

static driver_t ig4iic_fdt_driver = {
	"ig4iic",
	ig4iic_fdt_methods,
	sizeof(struct ig4iic_softc),
};

DRIVER_MODULE(ig4iic, simplebus, ig4iic_fdt_driver, 0, 0);
