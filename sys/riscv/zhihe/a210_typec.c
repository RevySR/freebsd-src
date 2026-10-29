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
 * Host-only FUSB302 Type-C port handling for the Zhihe A210 USB31 port.
 *
 * The port presents Rp at default USB current and polls both CC pins. Once
 * a sink (Rd) is seen on exactly one pin for the debounce time, the plug
 * orientation is passed to the C10 PHY and VBUS is switched on, unless
 * VBUS is already present. There is no USB PD, VCONN or role swapping.
 * The CC measurement follows the Linux fusb302 driver.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/taskqueue.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/regulator/regulator.h>

#include <riscv/zhihe/a210_c10phy.h>

#define	FUSB_DEVICE_ID		0x01
#define	FUSB_SWITCHES0		0x02
#define	 FUSB_PULLUP_CC2	(1 << 7)
#define	 FUSB_PULLUP_CC1	(1 << 6)
#define	 FUSB_MEASURE_CC2	(1 << 3)
#define	 FUSB_MEASURE_CC1	(1 << 2)
#define	FUSB_MEASURE		0x04
#define	FUSB_CONTROL0		0x06
#define	 FUSB_MASK_INTERRUPTS	(1 << 5)
#define	 FUSB_HOST_CUR_DEFAULT	(1 << 2)
#define	FUSB_CONTROL2		0x08
#define	FUSB_POWER		0x0b
#define	 FUSB_POWER_ALL		0x0f
#define	FUSB_RESET		0x0c
#define	 FUSB_SW_RESET		(1 << 0)
#define	FUSB_STATUS0		0x40
#define	 FUSB_VBUSOK		(1 << 7)
#define	 FUSB_COMP		(1 << 5)

/* MDAC thresholds for default-current Rp, in 42 mV steps. */
#define	FUSB_RD_THRESHOLD	38	/* 1.638 V: Open above, Rd below */
#define	FUSB_RA_THRESHOLD	4	/* 0.210 V: Rd above, Ra below */

#define	A210_TYPEC_POLL_MS	50
#define	A210_TYPEC_DEBOUNCE_MS	120

enum cc_state { CC_OPEN, CC_RA, CC_RD };

struct a210_typec_softc {
	device_t		dev;
	device_t		phy;
	regulator_t		vbus;
	bool			vbus_on;
	struct timeout_task	poll_task;
	bool			detaching;
	int			candidate;
	sbintime_t		candidate_since;
	int			orientation;
};

static struct ofw_compat_data compat_data[] = {
	{ "fcs,fusb302",	1 },
	{ NULL,			0 }
};

static int
a210_typec_read(struct a210_typec_softc *sc, uint8_t reg, uint8_t *val)
{

	return (iicdev_readfrom(sc->dev, reg, val, 1, IIC_WAIT));
}

static int
a210_typec_write(struct a210_typec_softc *sc, uint8_t reg, uint8_t val)
{

	return (iicdev_writeto(sc->dev, reg, &val, 1, IIC_WAIT));
}

/* Classify one CC pin with the comparator against the two thresholds. */
static int
a210_typec_measure(struct a210_typec_softc *sc, int pin, enum cc_state *cc,
    bool *vbus)
{
	uint8_t status;
	int error;

	error = a210_typec_write(sc, FUSB_SWITCHES0, FUSB_PULLUP_CC1 |
	    FUSB_PULLUP_CC2 | (pin ? FUSB_MEASURE_CC2 : FUSB_MEASURE_CC1));
	if (error == 0)
		error = a210_typec_write(sc, FUSB_MEASURE, FUSB_RD_THRESHOLD);
	if (error != 0)
		return (error);
	DELAY(300);
	error = a210_typec_read(sc, FUSB_STATUS0, &status);
	if (error != 0)
		return (error);
	*vbus = (status & FUSB_VBUSOK) != 0;
	if (status & FUSB_COMP) {
		*cc = CC_OPEN;
		return (0);
	}
	error = a210_typec_write(sc, FUSB_MEASURE, FUSB_RA_THRESHOLD);
	if (error != 0)
		return (error);
	DELAY(300);
	error = a210_typec_read(sc, FUSB_STATUS0, &status);
	if (error != 0)
		return (error);
	*cc = (status & FUSB_COMP) ? CC_RD : CC_RA;
	return (0);
}

static void
a210_typec_set(struct a210_typec_softc *sc, int orientation)
{

	if (orientation == sc->orientation)
		return;
	if (sc->orientation != A210_C10_DETACHED) {
		if (sc->vbus_on && regulator_disable(sc->vbus) == 0)
			sc->vbus_on = false;
		a210_c10phy_set_orientation(sc->phy, A210_C10_DETACHED);
		sc->orientation = A210_C10_DETACHED;
		device_printf(sc->dev, "detached\n");
	}
	if (orientation != A210_C10_DETACHED) {
		a210_c10phy_set_orientation(sc->phy, orientation);
		/* The supply registers after this device attaches. */
		if (sc->vbus == NULL &&
		    regulator_get_by_ofw_property(sc->dev, 0, "vbus-supply",
		    &sc->vbus) != 0) {
			device_printf(sc->dev, "cannot get VBUS supply\n");
			sc->vbus = NULL;
		}
		if (sc->vbus != NULL && !sc->vbus_on &&
		    regulator_enable(sc->vbus) == 0)
			sc->vbus_on = true;
		sc->orientation = orientation;
		device_printf(sc->dev, "sink attached on CC%d, VBUS %s\n",
		    orientation == A210_C10_NORMAL ? 1 : 2,
		    sc->vbus_on ? "on" : "off");
	}
}

static void
a210_typec_poll(void *arg, int pending)
{
	struct a210_typec_softc *sc;
	enum cc_state cc1, cc2;
	bool vbus1, vbus2;
	int candidate;

	sc = arg;
	if (a210_typec_measure(sc, 0, &cc1, &vbus1) != 0 ||
	    a210_typec_measure(sc, 1, &cc2, &vbus2) != 0) {
		candidate = A210_C10_DETACHED;
		vbus1 = vbus2 = false;
	} else if (cc1 == CC_RD && cc2 != CC_RD) {
		candidate = A210_C10_NORMAL;
	} else if (cc2 == CC_RD && cc1 != CC_RD) {
		candidate = A210_C10_REVERSE;
	} else {
		candidate = A210_C10_DETACHED;
	}

	if (candidate != sc->candidate) {
		sc->candidate = candidate;
		sc->candidate_since = getsbinuptime();
	}
	if (candidate != sc->orientation) {
		sbintime_t stable = getsbinuptime() - sc->candidate_since;
		/*
		 * Debounce both attach and detach (USB Type-C §4.5.2.1).
		 * A single glitch poll caused by VBUS rise must not immediately
		 * tear down the port.
		 */
		if (stable < A210_TYPEC_DEBOUNCE_MS * SBT_1MS) {
			/* still settling, do nothing */
		} else if (candidate == A210_C10_DETACHED) {
			a210_typec_set(sc, A210_C10_DETACHED);
		/* Never drive VBUS against a source that provides it. */
		} else if (sc->orientation == A210_C10_DETACHED &&
		    !(vbus1 || vbus2 || sc->vbus_on)) {
			a210_typec_set(sc, candidate);
		} else if (sc->orientation == A210_C10_DETACHED) {
			/* VBUS present and not ours: do not attach */
		} else {
			/* orientation change while attached (rare) */
			a210_typec_set(sc, candidate);
		}
	}

	if (!sc->detaching)
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->poll_task,
		    A210_TYPEC_POLL_MS * hz / 1000);
}

/* Follow the connector's graph endpoints to the C10 PHY. */
static device_t
a210_typec_find_phy(phandle_t tcpc)
{
	phandle_t connector, node, remote, ep;
	pcell_t xref;
	device_t phy;

	connector = ofw_bus_find_child(tcpc, "connector");
	if (connector <= 0 ||
	    !ofw_bus_node_is_compatible(connector, "usb-c-connector"))
		return (NULL);
	phy = NULL;
	for (node = OF_child(ofw_bus_find_child(connector, "ports"));
	    node > 0 && phy == NULL; node = OF_peer(node)) {
		for (ep = OF_child(node); ep > 0 && phy == NULL;
		    ep = OF_peer(ep)) {
			if (OF_getencprop(ep, "remote-endpoint", &xref,
			    sizeof(xref)) <= 0)
				continue;
			for (remote = OF_node_from_xref(xref); remote > 0;
			    remote = OF_parent(remote)) {
				if (ofw_bus_node_is_compatible(remote,
				    "zhihe,a210-c10-phy")) {
					phy = OF_device_from_xref(
					    OF_xref_from_node(remote));
					break;
				}
			}
		}
	}
	return (phy);
}

static int
a210_typec_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "FUSB302 Type-C port (A210 USB3 host)");
	return (BUS_PROBE_DEFAULT);
}

static int
a210_typec_attach(device_t dev)
{
	struct a210_typec_softc *sc;
	phandle_t node;
	uint8_t id;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	sc->phy = a210_typec_find_phy(node);
	if (sc->phy == NULL) {
		device_printf(dev, "no C10 PHY on the connector graph\n");
		return (ENXIO);
	}
	if (!OF_hasprop(node, "vbus-supply")) {
		device_printf(dev, "no VBUS supply\n");
		return (ENXIO);
	}

	error = a210_typec_read(sc, FUSB_DEVICE_ID, &id);
	if (error != 0 || id == 0 || id == 0xff) {
		device_printf(dev, "no FUSB302 found\n");
		return (ENXIO);
	}
	error = a210_typec_write(sc, FUSB_RESET, FUSB_SW_RESET);
	if (error == 0) {
		DELAY(1000);
		error = a210_typec_write(sc, FUSB_CONTROL2, 0);
	}
	if (error == 0)
		error = a210_typec_write(sc, FUSB_CONTROL0,
		    FUSB_MASK_INTERRUPTS | FUSB_HOST_CUR_DEFAULT);
	if (error == 0)
		error = a210_typec_write(sc, FUSB_POWER, FUSB_POWER_ALL);
	if (error == 0)
		error = a210_typec_write(sc, FUSB_SWITCHES0,
		    FUSB_PULLUP_CC1 | FUSB_PULLUP_CC2);
	if (error != 0) {
		device_printf(dev, "cannot configure FUSB302\n");
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(dev, "device ID 0x%02x\n", id);

	sc->orientation = sc->candidate = A210_C10_DETACHED;
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->poll_task, 0,
	    a210_typec_poll, sc);
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->poll_task, 0);
	return (0);
}

static int
a210_typec_detach(device_t dev)
{
	struct a210_typec_softc *sc;

	sc = device_get_softc(dev);
	sc->detaching = true;
	while (taskqueue_cancel_timeout(taskqueue_thread, &sc->poll_task,
	    NULL) != 0)
		taskqueue_drain_timeout(taskqueue_thread, &sc->poll_task);
	a210_typec_set(sc, A210_C10_DETACHED);
	if (sc->vbus != NULL)
		regulator_release(sc->vbus);
	return (0);
}

static device_method_t a210_typec_methods[] = {
	DEVMETHOD(device_probe,		a210_typec_probe),
	DEVMETHOD(device_attach,	a210_typec_attach),
	DEVMETHOD(device_detach,	a210_typec_detach),

	DEVMETHOD_END
};

static driver_t a210_typec_driver = {
	"a210_typec",
	a210_typec_methods,
	sizeof(struct a210_typec_softc),
};

DRIVER_MODULE(a210_typec, iicbus, a210_typec_driver, 0, 0);
MODULE_DEPEND(a210_typec, iicbus, IICBUS_MINVER, IICBUS_PREFVER,
    IICBUS_MAXVER);
