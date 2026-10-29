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
 * Doorbell from the application processors to the E902 management core on
 * the Zhihe A210. Channel 0 carries requests; the payload lives in shared
 * memory owned by the client, which polls it for completion.
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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "mbox_if.h"

#define	A210_MBOX_STATUS	0x00
#define	A210_MBOX_CLEAR		0x04
#define	A210_MBOX_MASK		0x0c
#define	A210_MBOX_GENERATE	0x10
#define	A210_MBOX_INFO(n)	(0x14 + 4 * (n))
#define	 A210_MBOX_E902_PENDING	(1u << 0)
#define	 A210_MBOX_NOTIFY	(1u << 6)

struct a210_mbox_softc {
	device_t		dev;
	struct resource		*local;
	struct resource		*remote;
	struct resource		*rx;
	struct mtx		mtx;
};

static struct ofw_compat_data compat_data[] = {
	{ "zhihe,a210-ap-mailbox",	1 },
	{ NULL,				0 }
};

static struct resource *
a210_mbox_alloc(device_t dev, const char *name, bool optional)
{
	struct resource *res;
	int rid;

	if (ofw_bus_find_string_index(ofw_bus_get_node(dev), "reg-names",
	    name, &rid) != 0) {
		if (!optional)
			device_printf(dev, "missing '%s' registers\n", name);
		return (NULL);
	}
	res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (res == NULL)
		device_printf(dev, "cannot map '%s' registers\n", name);
	return (res);
}

static void
a210_mbox_release(struct a210_mbox_softc *sc)
{

	if (sc->rx != NULL)
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->rx), sc->rx);
	if (sc->remote != NULL)
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->remote), sc->remote);
	if (sc->local != NULL)
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->local), sc->local);
}

static int
a210_mbox_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zhihe A210 AP to E902 mailbox");
	return (BUS_PROBE_DEFAULT);
}

static int
a210_mbox_attach(device_t dev)
{
	struct a210_mbox_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->local = a210_mbox_alloc(dev, "interrupt", false);
	sc->remote = a210_mbox_alloc(dev, "tx", false);
	sc->rx = a210_mbox_alloc(dev, "rx", true);
	if (sc->local == NULL || sc->remote == NULL) {
		a210_mbox_release(sc);
		return (ENXIO);
	}
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_SPIN);

	/* Completions are polled; keep the E902 doorbell interrupt masked. */
	mtx_lock_spin(&sc->mtx);
	bus_write_4(sc->local, A210_MBOX_MASK,
	    bus_read_4(sc->local, A210_MBOX_MASK) & ~A210_MBOX_E902_PENDING);
	mtx_unlock_spin(&sc->mtx);

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);
	return (0);
}

static int
a210_mbox_detach(device_t dev)
{

	/* The SCMI client keeps using the doorbell for system services. */
	return (EBUSY);
}

static int
a210_mbox_read(device_t dev, int chan, uint32_t *data)
{

	return (EOPNOTSUPP);
}

static int
a210_mbox_write(device_t dev, int chan, uint32_t data)
{
	struct a210_mbox_softc *sc;

	/* Channel 1 only carries notifications from the E902. */
	if (chan != 0)
		return (EINVAL);

	sc = device_get_softc(dev);
	mtx_lock_spin(&sc->mtx);
	/* Drop the doorbell left by the previous, polled, completion. */
	bus_write_4(sc->local, A210_MBOX_CLEAR, A210_MBOX_E902_PENDING);
	if (sc->rx != NULL)
		bus_write_4(sc->rx, A210_MBOX_INFO(0), 0);
	bus_write_4(sc->remote, A210_MBOX_GENERATE, A210_MBOX_NOTIFY);
	mtx_unlock_spin(&sc->mtx);
	return (0);
}

static device_method_t a210_mbox_methods[] = {
	DEVMETHOD(device_probe,		a210_mbox_probe),
	DEVMETHOD(device_attach,	a210_mbox_attach),
	DEVMETHOD(device_detach,	a210_mbox_detach),

	DEVMETHOD(mbox_read,		a210_mbox_read),
	DEVMETHOD(mbox_write,		a210_mbox_write),

	DEVMETHOD_END
};

static driver_t a210_mbox_driver = {
	"a210_mbox",
	a210_mbox_methods,
	sizeof(struct a210_mbox_softc),
};

/* SCMI attaches in the default pass and needs the doorbell by then. */
EARLY_DRIVER_MODULE(a210_mbox, simplebus, a210_mbox_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(a210_mbox, 1);
