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
 * Zhihe A210 DesignWare PCIe root complex.
 *
 * The endpoint reset line and reference-clock gate are behind I2C GPIO
 * expanders, so the controller is brought up from a configuration hook
 * once those providers have attached.  The slot supplies stay enabled even
 * without a link: a mini-PCIe card can also expose functions on USB.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcib_private.h>
#include <dev/pci/pci_dw.h>
#include <dev/phy/phy.h>
#include <dev/regulator/regulator.h>

#include "pcib_if.h"
#include "pci_dw_if.h"

#define	A210_PCIE_DBI2			0x100000

#define	A210_PCIE_CONTROL		0x000
#define	 A210_PCIE_TYPE_MASK		0xf
#define	 A210_PCIE_ROOT_PORT		4
#define	 A210_PCIE_TYPE_OVERRIDE	(1 << 4)
#define	 A210_PCIE_LTSSM_ENABLE		(1 << 8)
#define	A210_PCIE_LINK_STATUS		0x034
#define	 A210_PCIE_RDLH_UP		(1 << 0)
#define	 A210_PCIE_SMLH_UP		(1 << 4)

#define	DW_PORT_LINK_FAST_LINK		(1 << 7)
#define	DW_PORT_LINK_DLL_LINK_EN	(1 << 5)

#define	PCIEM_LINK_CTL2_TLS		0x000f

static const char *a210_pcie_supply_names[] = {
	"vpcie3v3-supply", "vpcie1v5-supply", "vpcie12v-supply",
};

static const char *a210_pcie_clock_names[] = {
	"aux", "slv", "mstr", "dbi",
};

#define	A210_PCIE_NSUPPLIES	nitems(a210_pcie_supply_names)
#define	A210_PCIE_NCLOCKS	nitems(a210_pcie_clock_names)

struct a210_pcie_softc {
	struct pci_dw_softc	dw_sc;	/* Must be first */
	device_t		dev;
	struct resource		*app_res;
	struct intr_config_hook	init_hook;
	regulator_t		supplies[A210_PCIE_NSUPPLIES];
	clk_t			clks[A210_PCIE_NCLOCKS];
	hwreset_t		rst_dbi;
	hwreset_t		rst_pwr;
	gpio_pin_t		perst;
	phy_t			phy;
	uint32_t		max_speed;
};

static struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-pcie",	1 },
	{ NULL,			0 }
};

static void
a210_pcie_app_update(struct a210_pcie_softc *sc, uint32_t mask, uint32_t val)
{
	uint32_t reg;

	reg = bus_read_4(sc->app_res, A210_PCIE_CONTROL);
	bus_write_4(sc->app_res, A210_PCIE_CONTROL, (reg & ~mask) | val);
}

static bool
a210_pcie_link_up(struct a210_pcie_softc *sc)
{
	uint32_t mask;

	mask = A210_PCIE_RDLH_UP | A210_PCIE_SMLH_UP;
	return ((bus_read_4(sc->app_res, A210_PCIE_LINK_STATUS) & mask) ==
	    mask);
}

static int
a210_pcie_get_link(device_t dev, bool *status)
{

	*status = a210_pcie_link_up(device_get_softc(dev));
	return (0);
}

static u_int
a210_pcie_find_express(device_t dev)
{
	u_int cap, i;

	cap = pci_dw_dbi_rd1(dev, PCIR_CAP_PTR);
	for (i = 0; cap != 0 && i < 48; i++) {
		if (pci_dw_dbi_rd1(dev, cap + PCICAP_ID) == PCIY_EXPRESS)
			return (cap);
		cap = pci_dw_dbi_rd1(dev, cap + PCICAP_NEXTPTR);
	}
	return (0);
}

static int
a210_pcie_get_resources(struct a210_pcie_softc *sc)
{
	device_t dev;
	phandle_t node;
	int error, i;

	dev = sc->dev;
	node = ofw_bus_get_node(dev);

	if (OF_hasprop(node, "reset-gpios")) {
		error = gpio_pin_get_by_ofw_property(dev, node, "reset-gpios",
		    &sc->perst);
		if (error != 0) {
			device_printf(dev, "cannot get PERST# GPIO: %d\n",
			    error);
			return (error);
		}
	}
	for (i = 0; i < A210_PCIE_NSUPPLIES; i++) {
		if (!OF_hasprop(node, a210_pcie_supply_names[i]))
			continue;
		error = regulator_get_by_ofw_property(dev, node,
		    __DECONST(char *, a210_pcie_supply_names[i]),
		    &sc->supplies[i]);
		if (error != 0) {
			device_printf(dev, "cannot get %s: %d\n",
			    a210_pcie_supply_names[i], error);
			return (error);
		}
	}
	for (i = 0; i < A210_PCIE_NCLOCKS; i++) {
		error = clk_get_by_ofw_name(dev, node,
		    a210_pcie_clock_names[i], &sc->clks[i]);
		if (error != 0) {
			device_printf(dev, "cannot get %s clock: %d\n",
			    a210_pcie_clock_names[i], error);
			return (error);
		}
	}
	error = hwreset_get_by_ofw_name(dev, node, "dbi", &sc->rst_dbi);
	if (error == 0)
		error = hwreset_get_by_ofw_name(dev, node, "pwr", &sc->rst_pwr);
	if (error != 0) {
		device_printf(dev, "cannot get resets: %d\n", error);
		return (error);
	}
	error = phy_get_by_ofw_name(dev, node, "pcie", &sc->phy);
	if (error != 0) {
		device_printf(dev, "cannot get PHY: %d\n", error);
		return (error);
	}
	return (0);
}

static int
a210_pcie_power_up(struct a210_pcie_softc *sc)
{
	device_t dev;
	int error, i;

	dev = sc->dev;

	if (sc->perst != NULL) {
		error = gpio_pin_setflags(sc->perst, GPIO_PIN_OUTPUT);
		if (error == 0)
			error = gpio_pin_set_active(sc->perst, true);
		if (error != 0) {
			device_printf(dev, "cannot assert PERST#: %d\n", error);
			return (error);
		}
	}
	for (i = 0; i < A210_PCIE_NSUPPLIES; i++) {
		if (sc->supplies[i] == NULL)
			continue;
		error = regulator_enable(sc->supplies[i]);
		if (error != 0) {
			device_printf(dev, "cannot enable %s: %d\n",
			    a210_pcie_supply_names[i], error);
			return (error);
		}
	}
	for (i = 0; i < A210_PCIE_NCLOCKS; i++) {
		error = clk_enable(sc->clks[i]);
		if (error != 0) {
			device_printf(dev, "cannot enable %s clock: %d\n",
			    a210_pcie_clock_names[i], error);
			return (error);
		}
	}

	/* Restart the core from reset with the application registers set. */
	error = hwreset_assert(sc->rst_pwr);
	if (error == 0)
		error = hwreset_deassert(sc->rst_dbi);
	if (error != 0)
		return (error);
	a210_pcie_app_update(sc, A210_PCIE_LTSSM_ENABLE, 0);
	a210_pcie_app_update(sc, A210_PCIE_TYPE_MASK | A210_PCIE_TYPE_OVERRIDE,
	    A210_PCIE_ROOT_PORT | A210_PCIE_TYPE_OVERRIDE);

	error = phy_set_mode(sc->phy, PHY_MODE_PCIE, PHY_SUBMODE_NA);
	if (error == 0)
		error = phy_enable(sc->phy);
	if (error != 0) {
		device_printf(dev, "cannot enable PHY: %d\n", error);
		return (error);
	}
	error = hwreset_deassert(sc->rst_pwr);
	if (error != 0)
		return (error);
	DELAY(100);
	return (0);
}

static int
a210_pcie_setup_dbi(struct a210_pcie_softc *sc)
{
	device_t dev;
	uint32_t reg;
	u_int cap;

	dev = sc->dev;
	cap = a210_pcie_find_express(dev);
	if (cap == 0) {
		device_printf(dev, "no PCI Express capability\n");
		return (ENXIO);
	}

	/* BAR masks are accessed through the DBI CS2 window. */
	pci_dw_dbi_wr4(dev, A210_PCIE_DBI2 + PCIR_BAR(0), 0);
	pci_dw_dbi_wr4(dev, A210_PCIE_DBI2 + PCIR_BAR(1), 0);

	reg = pci_dw_dbi_rd4(dev, DW_MISC_CONTROL_1);
	pci_dw_dbi_wr4(dev, DW_MISC_CONTROL_1, reg | DBI_RO_WR_EN);

	/* The PHY wakeup path is not configured for ASPM. */
	reg = pci_dw_dbi_rd4(dev, cap + PCIER_LINK_CAP);
	reg &= ~PCIEM_LINK_CAP_ASPM;
	if (sc->max_speed != 0) {
		reg &= ~PCIEM_LINK_CAP_MAX_SPEED;
		reg |= sc->max_speed;
	}
	pci_dw_dbi_wr4(dev, cap + PCIER_LINK_CAP, reg);
	reg = pci_dw_dbi_rd2(dev, cap + PCIER_LINK_CTL);
	pci_dw_dbi_wr2(dev, cap + PCIER_LINK_CTL, reg & ~PCIEM_LINK_CTL_ASPMC);
	if (sc->max_speed != 0) {
		reg = pci_dw_dbi_rd2(dev, cap + PCIER_LINK_CTL2);
		reg &= ~PCIEM_LINK_CTL2_TLS;
		pci_dw_dbi_wr2(dev, cap + PCIER_LINK_CTL2, reg | sc->max_speed);
	}

	reg = pci_dw_dbi_rd4(dev, DW_PORT_LINK_CTRL);
	reg &= ~DW_PORT_LINK_FAST_LINK;
	reg |= DW_PORT_LINK_DLL_LINK_EN;
	pci_dw_dbi_wr4(dev, DW_PORT_LINK_CTRL, reg);

	reg = pci_dw_dbi_rd4(dev, DW_MISC_CONTROL_1);
	pci_dw_dbi_wr4(dev, DW_MISC_CONTROL_1, reg & ~DBI_RO_WR_EN);
	return (0);
}

static void
a210_pcie_start_link(struct a210_pcie_softc *sc)
{
	device_t dev;
	uint32_t sta;
	u_int cap;
	int i;

	dev = sc->dev;

	/* Endpoint power and reference clock must precede PERST# release. */
	pause("a210pwr", MAX(1, hz / 10));
	if (sc->perst != NULL)
		gpio_pin_set_active(sc->perst, false);
	DELAY(2000);
	a210_pcie_app_update(sc, A210_PCIE_LTSSM_ENABLE,
	    A210_PCIE_LTSSM_ENABLE);

	for (i = 0; i < 100 && !a210_pcie_link_up(sc); i++)
		pause("a210lnk", MAX(1, hz / 100));
	if (!a210_pcie_link_up(sc)) {
		device_printf(dev, "link down\n");
		return;
	}

	cap = a210_pcie_find_express(dev);
	sta = pci_dw_dbi_rd2(dev, cap + PCIER_LINK_STA);
	if (sc->max_speed > 1 &&
	    (sta & PCIEM_LINK_STA_SPEED) < sc->max_speed) {
		/* A210 needs the speed-change request after reaching L0. */
		pci_dw_dbi_wr4(dev, DW_GEN2_CTRL,
		    pci_dw_dbi_rd4(dev, DW_GEN2_CTRL) | DIRECT_SPEED_CHANGE);
	}

	/* Devices may not answer configuration requests before 100 ms. */
	pause("a210cfg", MAX(1, hz / 10));
	sta = pci_dw_dbi_rd2(dev, cap + PCIER_LINK_STA);
	device_printf(dev, "link up, Gen%u x%u\n", sta & PCIEM_LINK_STA_SPEED,
	    (sta & PCIEM_LINK_STA_WIDTH) >> 4);
}

static void
a210_pcie_init(void *arg)
{
	struct a210_pcie_softc *sc;
	int error;

	sc = arg;
	error = a210_pcie_get_resources(sc);
	if (error == 0)
		error = a210_pcie_power_up(sc);
	if (error == 0)
		error = a210_pcie_setup_dbi(sc);
	if (error == 0)
		error = pci_dw_init(sc->dev);
	if (error == 0) {
		a210_pcie_start_link(sc);
		bus_attach_children(sc->dev);
	} else
		device_printf(sc->dev, "initialization failed: %d\n", error);
	config_intrhook_disestablish(&sc->init_hook);
}

static int
a210_pcie_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 PCIe controller");
	return (BUS_PROBE_DEFAULT);
}

static int
a210_pcie_attach(device_t dev)
{
	struct a210_pcie_softc *sc;
	phandle_t node;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	if (OF_getencprop(node, "max-link-speed", &sc->max_speed,
	    sizeof(sc->max_speed)) <= 0 || sc->max_speed > 4)
		sc->max_speed = 0;

	if (ofw_bus_find_string_index(node, "reg-names", "dbi", &rid) != 0) {
		device_printf(dev, "cannot find DBI registers\n");
		return (ENXIO);
	}
	sc->dw_sc.dbi_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->dw_sc.dbi_res == NULL) {
		device_printf(dev, "cannot map DBI registers\n");
		return (ENXIO);
	}
	if (rman_get_size(sc->dw_sc.dbi_res) < A210_PCIE_DBI2 + 0x1000) {
		device_printf(dev, "DBI window too small\n");
		goto fail;
	}
	if (ofw_bus_find_string_index(node, "reg-names", "app", &rid) != 0) {
		device_printf(dev, "cannot find application registers\n");
		goto fail;
	}
	sc->app_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->app_res == NULL) {
		device_printf(dev, "cannot map application registers\n");
		goto fail;
	}

	sc->init_hook.ich_func = a210_pcie_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		goto fail;
	return (0);

fail:
	if (sc->app_res != NULL)
		bus_release_resource(dev, sc->app_res);
	bus_release_resource(dev, sc->dw_sc.dbi_res);
	return (ENXIO);
}

static device_method_t a210_pcie_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		a210_pcie_probe),
	DEVMETHOD(device_attach,	a210_pcie_attach),

	/* PCI DW interface */
	DEVMETHOD(pci_dw_get_link,	a210_pcie_get_link),

	DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, a210_pcie_driver, a210_pcie_methods,
    sizeof(struct a210_pcie_softc), pci_dw_driver);
DRIVER_MODULE(a210_pcie, simplebus, a210_pcie_driver, NULL, NULL);
