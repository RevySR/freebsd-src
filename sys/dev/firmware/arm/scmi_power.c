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
 * SCMI Power Domain Management Protocol.
 *
 * There is no power domain framework for drivers to request their domains,
 * so the domains referenced by enabled devices are powered on when the
 * protocol attaches, before those devices probe, and are left on.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include "scmi.h"
#include "scmi_protocols.h"

#define	SCMI_POWER_STATE_SET		0x4
#define	SCMI_POWER_STATE_GET		0x5

struct scmi_power_protocol_attrs_out {
	uint32_t attributes;
#define	SCMI_POWER_NDOMAINS_M	0xffff
};

struct scmi_power_state_set_in {
	uint32_t flags;
	uint32_t domain_id;
	uint32_t power_state;
#define	SCMI_POWER_STATE_ON	0
#define	SCMI_POWER_STATE_OFF	(1u << 30)
};

struct scmi_power_state_get_in {
	uint32_t domain_id;
};

struct scmi_power_state_get_out {
	uint32_t power_state;
};

struct scmi_power_softc {
	device_t	dev;
	device_t	scmi;
	u_int		ndomains;
	bool		*used;
};

static int
scmi_power_discover(struct scmi_power_softc *sc)
{
	struct scmi_power_protocol_attrs_out *out;
	void *in;
	int error;

	in = scmi_buf_get(sc->scmi, SCMI_PROTOCOL_ID_POWER_DOMAIN,
	    SCMI_PROTOCOL_ATTRIBUTES, 0, sizeof(*out));
	if (in == NULL)
		return (ENXIO);

	error = scmi_request(sc->scmi, in, (void **)&out);
	if (error == 0)
		sc->ndomains = out->attributes & SCMI_POWER_NDOMAINS_M;

	scmi_buf_put(sc->scmi, in);

	return (error);
}

static int
scmi_power_set(struct scmi_power_softc *sc, u_int id, uint32_t state)
{
	struct scmi_power_state_set_in *in;
	void *out;
	int error;

	in = scmi_buf_get(sc->scmi, SCMI_PROTOCOL_ID_POWER_DOMAIN,
	    SCMI_POWER_STATE_SET, sizeof(*in), 0);
	if (in == NULL)
		return (ENXIO);

	in->flags = 0;
	in->domain_id = id;
	in->power_state = state;
	error = scmi_request(sc->scmi, in, &out);

	scmi_buf_put(sc->scmi, in);

	return (error);
}

static int
scmi_power_get(struct scmi_power_softc *sc, u_int id, uint32_t *state)
{
	struct scmi_power_state_get_out *out;
	struct scmi_power_state_get_in *in;
	int error;

	in = scmi_buf_get(sc->scmi, SCMI_PROTOCOL_ID_POWER_DOMAIN,
	    SCMI_POWER_STATE_GET, sizeof(*in), sizeof(*out));
	if (in == NULL)
		return (ENXIO);

	in->domain_id = id;
	error = scmi_request(sc->scmi, in, (void **)&out);
	if (error == 0)
		*state = out->power_state;

	scmi_buf_put(sc->scmi, in);

	return (error);
}

/* Record the domains of this provider used by enabled nodes below node. */
static void
scmi_power_scan(struct scmi_power_softc *sc, phandle_t xref, phandle_t node)
{
	phandle_t child, provider;
	pcell_t *cells;
	int i, n, ncells;

	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (!ofw_bus_node_status_okay(child))
			continue;
		if (ofw_bus_parse_xref_list_get_length(child, "power-domains",
		    "#power-domain-cells", &n) == 0) {
			for (i = 0; i < n; i++) {
				if (ofw_bus_parse_xref_list_alloc(child,
				    "power-domains", "#power-domain-cells", i,
				    &provider, &ncells, &cells) != 0)
					continue;
				if (provider == xref && ncells == 1 &&
				    cells[0] < sc->ndomains)
					sc->used[cells[0]] = true;
				OF_prop_free(cells);
			}
		}
		scmi_power_scan(sc, xref, child);
	}
}

static int
scmi_power_probe(device_t dev)
{
	uint32_t reg;

	if (OF_getencprop(ofw_bus_get_node(dev), "reg", &reg,
	    sizeof(reg)) < 0)
		return (ENXIO);
	if (reg != SCMI_PROTOCOL_ID_POWER_DOMAIN)
		return (ENXIO);

	device_set_desc(dev, "SCMI Power Domain Management");

	return (BUS_PROBE_DEFAULT);
}

static int
scmi_power_attach(device_t dev)
{
	struct scmi_power_softc *sc;
	phandle_t xref;
	uint32_t state;
	u_int id;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->scmi = device_get_parent(dev);

	error = scmi_power_discover(sc);
	if (error != 0 || sc->ndomains == 0) {
		device_printf(dev, "Could not discover power domains.\n");
		return (ENXIO);
	}
	sc->used = malloc(sc->ndomains * sizeof(*sc->used), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	xref = OF_xref_from_node(ofw_bus_get_node(dev));
	scmi_power_scan(sc, xref, OF_peer(0));

	for (id = 0; id < sc->ndomains; id++) {
		if (!sc->used[id])
			continue;
		error = scmi_power_set(sc, id, SCMI_POWER_STATE_ON);
		if (error == 0)
			error = scmi_power_get(sc, id, &state);
		if (error == 0 && state != SCMI_POWER_STATE_ON)
			error = EIO;
		if (error != 0)
			device_printf(dev,
			    "Cannot power on domain %u: %d\n", id, error);
		else if (bootverbose)
			device_printf(dev, "Domain %u on\n", id);
	}

	return (0);
}

static int
scmi_power_detach(device_t dev)
{

	/* Consumers do not track their domains. */
	return (EBUSY);
}

static device_method_t scmi_power_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		scmi_power_probe),
	DEVMETHOD(device_attach,	scmi_power_attach),
	DEVMETHOD(device_detach,	scmi_power_detach),

	DEVMETHOD_END
};

static driver_t scmi_power_driver = {
	"scmi_power",
	scmi_power_methods,
	sizeof(struct scmi_power_softc),
};

EARLY_DRIVER_MODULE(scmi_power, scmi, scmi_power_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(scmi_power, 1);
