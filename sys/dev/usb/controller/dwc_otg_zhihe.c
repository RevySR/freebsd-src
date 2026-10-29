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

/* Zhihe A210 integration of the DesignWare USB 2.0 OTG controller. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/phy/phy_usb.h>
#include <dev/regulator/regulator.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>

#include <dev/usb/controller/dwc_otg.h>
#include <dev/usb/controller/dwc_otgreg.h>
#include <dev/usb/controller/dwc_otg_fdt.h>

/* Hardware configuration registers. */
#define	A210_GHWCFG2_HSPHYTYPE(x)	(((x) >> 6) & 0x3)
#define	 A210_HSPHYTYPE_UTMI		1
#define	 A210_HSPHYTYPE_ULPI		2
#define	A210_GHWCFG4_UTMI_WIDTH(x)	(((x) >> 14) & 0x3)
#define	 A210_UTMI_WIDTH_8		0	/* 1: 16 bits, 2: either */

struct zhihe_dwc_otg_softc {
	struct dwc_otg_fdt_softc	base;	/* must be first */
	struct intr_config_hook		vbus_hook;
};

static int
zhihe_dwc_otg_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "zhihe,a210-usb"))
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 DWC OTG 2.0 USB controller");
	return (BUS_PROBE_VENDOR);
}

/* Select the PHY interface the controller was built for, as dwc2 does. */
static int
zhihe_dwc_otg_phy_config(device_t dev, struct dwc_otg_softc *sc)
{
	struct resource *res;
	uint32_t hwcfg2, hwcfg4;
	int rid;

	rid = 0;
	res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (res == NULL)
		return (ENXIO);
	hwcfg2 = bus_read_4(res, DOTG_GHWCFG2);
	hwcfg4 = bus_read_4(res, DOTG_GHWCFG4);
	bus_release_resource(dev, SYS_RES_MEMORY, rid, res);

	/* Use buffer DMA when the controller has its internal DMA engine. */
	if ((hwcfg2 & GHWCFG2_OTGARCH_MASK) >> GHWCFG2_OTGARCH_SHIFT ==
	    GHWCFG2_OTGARCH_INTERNAL_DMA)
		sc->sc_host_dma = 1;

	switch (A210_GHWCFG2_HSPHYTYPE(hwcfg2)) {
	case A210_HSPHYTYPE_ULPI:
		sc->sc_phy_type = DWC_OTG_PHY_ULPI;
		break;
	default:
		sc->sc_phy_type = DWC_OTG_PHY_UTMI;
		/* Prefer 16 bits unless the PHY interface is 8 bits only. */
		sc->sc_phy_bits =
		    A210_GHWCFG4_UTMI_WIDTH(hwcfg4) == A210_UTMI_WIDTH_8 ?
		    8 : 16;
		break;
	}
	return (0);
}

/*
 * The VBUS switch is usually behind an I2C GPIO expander that attaches after
 * this controller, so turn VBUS on once all devices have attached.
 */
static void
zhihe_dwc_otg_vbus(void *arg)
{
	struct zhihe_dwc_otg_softc *sc;
	regulator_t vbus;
	device_t dev;

	dev = arg;
	sc = device_get_softc(dev);
	if (regulator_get_by_ofw_property(dev, 0, "vbus-supply", &vbus) != 0)
		device_printf(dev, "cannot get VBUS supply\n");
	else if (regulator_enable(vbus) != 0)
		device_printf(dev, "cannot enable VBUS supply\n");
	config_intrhook_disestablish(&sc->vbus_hook);
}

static int
zhihe_dwc_otg_attach(device_t dev)
{
	struct zhihe_dwc_otg_softc *sc;
	hwreset_t rst;
	phandle_t node;
	clk_t clk;
	phy_t phy;
	char mode[16];
	int error, phy_mode;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	phy_mode = PHY_USB_MODE_OTG;
	if (OF_getprop(node, "dr_mode", mode, sizeof(mode)) > 0) {
		mode[sizeof(mode) - 1] = '\0';
		if (strcmp(mode, "host") == 0)
			phy_mode = PHY_USB_MODE_HOST;
		else if (strcmp(mode, "peripheral") == 0)
			phy_mode = PHY_USB_MODE_DEVICE;
	}
	/*
	 * Without a configured gadget function the stack keeps failing to
	 * enumerate the attached host, so leave device-only ports alone.
	 */
	if (phy_mode == PHY_USB_MODE_DEVICE) {
		device_printf(dev, "device mode is not supported yet\n");
		return (ENXIO);
	}

	error = clk_get_by_ofw_name(dev, 0, "otg", &clk);
	if (error == 0)
		error = clk_enable(clk);
	if (error != 0) {
		device_printf(dev, "cannot enable clock\n");
		return (ENXIO);
	}

	if (hwreset_get_by_ofw_name(dev, 0, "dwc2", &rst) != 0 ||
	    hwreset_assert(rst) != 0) {
		device_printf(dev, "cannot assert reset\n");
		return (ENXIO);
	}
	DELAY(20);
	if (hwreset_deassert(rst) != 0) {
		device_printf(dev, "cannot deassert reset\n");
		return (ENXIO);
	}

	if (phy_get_by_ofw_name(dev, 0, "usb2-phy", &phy) != 0) {
		device_printf(dev, "cannot get PHY\n");
		return (ENXIO);
	}
	if (phy_mode != PHY_USB_MODE_OTG)
		phy_usb_set_mode(phy, phy_mode);
	if (phy_enable(phy) != 0) {
		device_printf(dev, "cannot enable PHY\n");
		return (ENXIO);
	}

	if (zhihe_dwc_otg_phy_config(dev, &sc->base.sc_otg) != 0) {
		device_printf(dev, "cannot read hardware configuration\n");
		return (ENXIO);
	}

	error = dwc_otg_attach(dev);
	if (error == 0 && OF_hasprop(node, "vbus-supply")) {
		sc->vbus_hook.ich_func = zhihe_dwc_otg_vbus;
		sc->vbus_hook.ich_arg = dev;
		if (config_intrhook_establish(&sc->vbus_hook) != 0)
			device_printf(dev, "cannot schedule VBUS enable\n");
	}
	return (error);
}

static device_method_t zhihe_dwc_otg_methods[] = {
	DEVMETHOD(device_probe,		zhihe_dwc_otg_probe),
	DEVMETHOD(device_attach,	zhihe_dwc_otg_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(zhihe_dwcotg, zhihe_dwc_otg_driver, zhihe_dwc_otg_methods,
    sizeof(struct zhihe_dwc_otg_softc), dwc_otg_driver);
DRIVER_MODULE(zhihe_dwcotg, simplebus, zhihe_dwc_otg_driver, 0, 0);
MODULE_DEPEND(zhihe_dwcotg, usb, 1, 1, 1);
