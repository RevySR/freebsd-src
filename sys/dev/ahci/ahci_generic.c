/*-
 * Copyright (c) 2009-2012 Alexander Motin <mav@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ahci/ahci.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#endif

#ifdef FDT
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/phy/phy.h>
#include <dev/regulator/regulator.h>

#define	AHCI_FDT_MAX_CLKS	5

struct ahci_fdt_softc {
	struct ahci_controller	ctlr;		/* Must be first. */
	struct intr_config_hook	init_hook;
	bool			attached;
	clk_t			clks[AHCI_FDT_MAX_CLKS];
	int			nclks;
	hwreset_array_t		rsts;
	regulator_t		supply;
	regulator_t		target_supplies[AHCI_MAX_PORTS];
	phy_t			phys[AHCI_MAX_PORTS];
	uint32_t		port_mask;
};

static struct ofw_compat_data compat_data[] = {
	{"generic-ahci", 		1},
	{"snps,dwc-ahci",		1},
	{"marvell,armada-3700-ahci",	1},
	{NULL,				0}
};

static int
ahci_fdt_probe(device_t dev)
{
	struct ahci_controller *ctlr = device_get_softc(dev);
	phandle_t node;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "AHCI SATA controller");
	node = ofw_bus_get_node(dev);
	ctlr->dma_coherent = OF_hasprop(node, "dma-coherent");
	return (BUS_PROBE_DEFAULT);
}

static int
ahci_fdt_enable_supply(device_t dev, phandle_t node, char *name,
    regulator_t *supply)
{
	regulator_t reg;
	int error;

	error = regulator_get_by_ofw_property(dev, node, name, &reg);
	if (error != 0) {
		if (error != ENOENT)
			device_printf(dev, "Cannot get %s: %d\n", name, error);
		return (0);
	}
	error = regulator_enable(reg);
	if (error != 0) {
		device_printf(dev, "Cannot enable %s: %d\n", name, error);
		regulator_release(reg);
		return (error);
	}
	*supply = reg;
	return (0);
}

/*
 * Enable the resources described by the generic AHCI platform binding:
 * supplies, clocks, resets and the per-port PHYs.  A resource whose
 * provider has no driver is left as configured by the firmware.
 */
static int
ahci_fdt_enable_resources(device_t dev)
{
	struct ahci_fdt_softc *sc;
	phandle_t child, node, ports[AHCI_MAX_PORTS];
	pcell_t port;
	clk_t clk;
	phy_t phy;
	bool port_nodes;
	int error, i;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	memset(ports, 0, sizeof(ports));
	port_nodes = false;
	sc->port_mask = 0;
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		port_nodes = true;
		if (!ofw_bus_node_status_okay(child))
			continue;
		if (OF_getencprop(child, "reg", &port, sizeof(port)) <= 0 ||
		    port >= AHCI_MAX_PORTS) {
			device_printf(dev, "Invalid port node\n");
			continue;
		}
		ports[port] = child;
		sc->port_mask |= 1u << port;
	}
	/* Without port nodes the controller node describes the port. */
	if (!port_nodes) {
		ports[0] = node;
		sc->port_mask = 0xffffffff;
	}

	error = ahci_fdt_enable_supply(dev, node, "ahci-supply", &sc->supply);
	if (error != 0)
		return (error);
	for (i = 0; i < AHCI_MAX_PORTS; i++) {
		if (ports[i] == 0)
			continue;
		error = ahci_fdt_enable_supply(dev, ports[i], "target-supply",
		    &sc->target_supplies[i]);
		if (error != 0)
			return (error);
	}

	for (i = 0; i < AHCI_FDT_MAX_CLKS; i++) {
		error = clk_get_by_ofw_index(dev, node, i, &clk);
		if (error != 0) {
			if (error != ENOENT)
				device_printf(dev, "Cannot get clock %d: %d\n",
				    i, error);
			break;
		}
		error = clk_enable(clk);
		if (error != 0) {
			device_printf(dev, "Cannot enable clock %s: %d\n",
			    clk_get_name(clk), error);
			clk_release(clk);
			return (error);
		}
		sc->clks[sc->nclks++] = clk;
	}

	error = hwreset_array_get_ofw(dev, node, &sc->rsts);
	if (error == 0) {
		error = hwreset_array_deassert(sc->rsts);
		if (error != 0) {
			device_printf(dev, "Cannot deassert resets: %d\n",
			    error);
			return (error);
		}
	} else if (error != ENOENT)
		device_printf(dev, "Cannot get resets: %d\n", error);

	for (i = 0; i < AHCI_MAX_PORTS; i++) {
		if (ports[i] == 0)
			continue;
		error = phy_get_by_ofw_idx(dev, ports[i], 0, &phy);
		if (error != 0) {
			if (error != ENOENT)
				device_printf(dev,
				    "Cannot get port %d PHY: %d\n", i, error);
			continue;
		}
		error = phy_enable(phy);
		if (error != 0) {
			device_printf(dev, "Cannot enable port %d PHY: %d\n",
			    i, error);
			phy_release(phy);
			return (error);
		}
		sc->phys[i] = phy;
	}
	return (0);
}

static void
ahci_fdt_disable_resources(device_t dev)
{
	struct ahci_fdt_softc *sc;
	int i;

	sc = device_get_softc(dev);
	for (i = 0; i < AHCI_MAX_PORTS; i++) {
		if (sc->phys[i] == NULL)
			continue;
		phy_disable(sc->phys[i]);
		phy_release(sc->phys[i]);
		sc->phys[i] = NULL;
	}
	if (sc->rsts != NULL) {
		hwreset_array_assert(sc->rsts);
		hwreset_array_release(sc->rsts);
		sc->rsts = NULL;
	}
	while (sc->nclks > 0) {
		sc->nclks--;
		clk_disable(sc->clks[sc->nclks]);
		clk_release(sc->clks[sc->nclks]);
	}
	for (i = 0; i < AHCI_MAX_PORTS; i++) {
		if (sc->target_supplies[i] == NULL)
			continue;
		regulator_disable(sc->target_supplies[i]);
		regulator_release(sc->target_supplies[i]);
		sc->target_supplies[i] = NULL;
	}
	if (sc->supply != NULL) {
		regulator_disable(sc->supply);
		regulator_release(sc->supply);
		sc->supply = NULL;
	}
}

/*
 * PI is normally set up by the firmware.  The "ports-implemented" property
 * overrides it, which controllers whose PI reads as zero after reset need,
 * and ports with a disabled port node are masked out.
 */
static void
ahci_fdt_setup_ports(device_t dev)
{
	struct ahci_fdt_softc *sc;
	pcell_t ports;
	uint32_t pi;

	sc = device_get_softc(dev);
	if (OF_getencprop(ofw_bus_get_node(dev), "ports-implemented", &ports,
	    sizeof(ports)) > 0)
		pi = ports;
	else
		pi = ATA_INL(sc->ctlr.r_mem, AHCI_PI);
	pi &= sc->port_mask;
	if (pi != ATA_INL(sc->ctlr.r_mem, AHCI_PI))
		ATA_OUTL(sc->ctlr.r_mem, AHCI_PI, pi);
}

static void
ahci_fdt_init(void *arg)
{
	struct ahci_fdt_softc *sc;
	struct ahci_controller *ctlr;
	device_t dev;
	int error;

	dev = arg;
	sc = device_get_softc(dev);
	ctlr = &sc->ctlr;

	error = ahci_fdt_enable_resources(dev);
	if (error != 0)
		goto fail;

	ctlr->r_rid = 0;
	ctlr->r_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &ctlr->r_rid,
	    RF_ACTIVE);
	if (ctlr->r_mem == NULL) {
		error = ENXIO;
		goto fail;
	}

	/* Setup controller defaults. */
	ctlr->numirqs = 1;

	/* Reset controller */
	error = ahci_ctlr_reset(dev);
	if (error == 0) {
		ahci_fdt_setup_ports(dev);
		error = ahci_attach(dev);
	}
	if (error != 0) {
		if (ctlr->r_mem != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY, ctlr->r_rid,
			    ctlr->r_mem);
		ctlr->r_mem = NULL;
		goto fail;
	}
	sc->attached = true;
	config_intrhook_disestablish(&sc->init_hook);
	return;

fail:
	device_printf(dev, "Initialization failed: %d\n", error);
	ahci_fdt_disable_resources(dev);
	config_intrhook_disestablish(&sc->init_hook);
}

static int
ahci_fdt_attach(device_t dev)
{
	struct ahci_fdt_softc *sc;

	sc = device_get_softc(dev);

	/* Supplies and PHYs may be provided by devices attached later. */
	sc->init_hook.ich_func = ahci_fdt_init;
	sc->init_hook.ich_arg = dev;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);
	return (0);
}

static int
ahci_fdt_detach(device_t dev)
{
	struct ahci_fdt_softc *sc;
	int error;

	sc = device_get_softc(dev);
	config_intrhook_drain(&sc->init_hook);
	if (sc->attached) {
		error = ahci_detach(dev);
		if (error != 0)
			return (error);
		sc->attached = false;
	}
	ahci_fdt_disable_resources(dev);
	return (0);
}
#endif

#ifdef DEV_ACPI

static const struct ahci_acpi_quirk {
	const char*	hid;
	u_int		quirks;
} ahci_acpi_quirks[] = {
	{ "NXP0004", AHCI_Q_NOPMP },
	{ NULL, 0 }
};


static int
ahci_acpi_probe(device_t dev)
{
	struct ahci_controller *ctlr = device_get_softc(dev);
	ACPI_HANDLE h;
	int i;

	if ((h = acpi_get_handle(dev)) == NULL)
		return (ENXIO);

	if (pci_get_class(dev) == PCIC_STORAGE &&
	    pci_get_subclass(dev) == PCIS_STORAGE_SATA &&
	    pci_get_progif(dev) == PCIP_STORAGE_SATA_AHCI_1_0) {
		device_set_desc(dev, "AHCI SATA controller");
		if (ACPI_FAILURE(acpi_GetInteger(h, "_CCA",
		      &ctlr->dma_coherent)))
			ctlr->dma_coherent = 0;
		if (bootverbose)
			device_printf(dev, "Bus is%s cache-coherent\n",
			  ctlr->dma_coherent ? "" : " not");
		for (i = 0; ahci_acpi_quirks[i].hid != NULL; i++) {
			if (acpi_MatchHid(h, ahci_acpi_quirks[i].hid) != ACPI_MATCHHID_NOMATCH) {
				ctlr->quirks = ahci_acpi_quirks[i].quirks;
				break;
			}
		}
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
ahci_gen_ctlr_reset(device_t dev)
{

	return ahci_ctlr_reset(dev);
}

static int
ahci_gen_attach(device_t dev)
{
	struct ahci_controller *ctlr = device_get_softc(dev);
	int	error;

	ctlr->r_rid = 0;
	ctlr->r_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &ctlr->r_rid,
	    RF_ACTIVE);
	if (ctlr->r_mem == NULL)
		return (ENXIO);

	/* Setup controller defaults. */
	ctlr->numirqs = 1;

	/* Reset controller */
	if ((error = ahci_gen_ctlr_reset(dev)) == 0)
		error = ahci_attach(dev);

	if (error != 0) {
		if (ctlr->r_mem != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY, ctlr->r_rid,
			    ctlr->r_mem);
	}
	return error;
}

static int
ahci_gen_detach(device_t dev)
{

	ahci_detach(dev);
	return (0);
}
#endif

#ifdef FDT
static device_method_t ahci_fdt_methods[] = {
	DEVMETHOD(device_probe,     ahci_fdt_probe),
	DEVMETHOD(device_attach,    ahci_fdt_attach),
	DEVMETHOD(device_detach,    ahci_fdt_detach),
	DEVMETHOD(bus_print_child,  ahci_print_child),
	DEVMETHOD(bus_alloc_resource,       ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,     ahci_release_resource),
	DEVMETHOD(bus_setup_intr,   ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,ahci_teardown_intr),
	DEVMETHOD(bus_child_location, ahci_child_location),
	DEVMETHOD(bus_get_dma_tag,  ahci_get_dma_tag),
	DEVMETHOD_END
};
static driver_t ahci_fdt_driver = {
	"ahci",
	ahci_fdt_methods,
	sizeof(struct ahci_fdt_softc)
};
DRIVER_MODULE(ahci_fdt, simplebus, ahci_fdt_driver, NULL, NULL);
#endif

#ifdef DEV_ACPI
static device_method_t ahci_acpi_methods[] = {
	DEVMETHOD(device_probe,     ahci_acpi_probe),
	DEVMETHOD(device_attach,    ahci_gen_attach),
	DEVMETHOD(device_detach,    ahci_gen_detach),
	DEVMETHOD(bus_print_child,  ahci_print_child),
	DEVMETHOD(bus_alloc_resource,       ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,     ahci_release_resource),
	DEVMETHOD(bus_setup_intr,   ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,ahci_teardown_intr),
	DEVMETHOD(bus_child_location, ahci_child_location),
	DEVMETHOD(bus_get_dma_tag,  ahci_get_dma_tag),
	DEVMETHOD_END
};
static driver_t ahci_acpi_driver = {
	"ahci",
	ahci_acpi_methods,
	sizeof(struct ahci_controller)
};
DRIVER_MODULE(ahci_acpi, acpi, ahci_acpi_driver, NULL, NULL);
#endif
