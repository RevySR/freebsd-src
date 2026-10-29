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

/* SCMI Reset Domain Management Protocol as an hwreset provider. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "scmi.h"
#include "scmi_protocols.h"

#include "hwreset_if.h"

#define	SCMI_RESET_RESET		0x4

struct scmi_reset_protocol_attrs_out {
	uint32_t attributes;
#define	SCMI_RESET_NDOMAINS_M	0xffff
};

struct scmi_reset_in {
	uint32_t domain_id;
	uint32_t flags;
#define	SCMI_RESET_AUTONOMOUS		(1 << 0)
#define	SCMI_RESET_EXPLICIT_ASSERT	(1 << 1)
#define	SCMI_RESET_ASYNC		(1 << 2)
	uint32_t reset_state;
#define	SCMI_RESET_ARCH_COLD_RESET	0
};

struct scmi_reset_softc {
	device_t	dev;
	device_t	scmi;
	u_int		ndomains;
};

static int
scmi_reset_discover(struct scmi_reset_softc *sc)
{
	struct scmi_reset_protocol_attrs_out *out;
	void *in;
	int error;

	in = scmi_buf_get(sc->scmi, SCMI_PROTOCOL_ID_RESET_DOMAIN,
	    SCMI_PROTOCOL_ATTRIBUTES, 0, sizeof(*out));
	if (in == NULL)
		return (ENXIO);

	error = scmi_request(sc->scmi, in, (void **)&out);
	if (error == 0)
		sc->ndomains = out->attributes & SCMI_RESET_NDOMAINS_M;

	scmi_buf_put(sc->scmi, in);

	return (error);
}

static int
scmi_reset_assert(device_t dev, intptr_t id, bool reset)
{
	struct scmi_reset_softc *sc;
	struct scmi_reset_in *in;
	void *out;
	int error;

	sc = device_get_softc(dev);
	if (id < 0 || id >= sc->ndomains)
		return (EINVAL);

	in = scmi_buf_get(sc->scmi, SCMI_PROTOCOL_ID_RESET_DOMAIN,
	    SCMI_RESET_RESET, sizeof(*in), 0);
	if (in == NULL)
		return (ENXIO);

	in->domain_id = id;
	in->flags = reset ? SCMI_RESET_EXPLICIT_ASSERT : 0;
	in->reset_state = SCMI_RESET_ARCH_COLD_RESET;
	error = scmi_request(sc->scmi, in, &out);

	scmi_buf_put(sc->scmi, in);

	return (error);
}

static int
scmi_reset_probe(device_t dev)
{
	uint32_t reg;

	if (OF_getencprop(ofw_bus_get_node(dev), "reg", &reg,
	    sizeof(reg)) < 0)
		return (ENXIO);
	if (reg != SCMI_PROTOCOL_ID_RESET_DOMAIN)
		return (ENXIO);

	device_set_desc(dev, "SCMI Reset Domain Management");

	return (BUS_PROBE_DEFAULT);
}

static int
scmi_reset_attach(device_t dev)
{
	struct scmi_reset_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->scmi = device_get_parent(dev);

	error = scmi_reset_discover(sc);
	if (error != 0) {
		device_printf(dev, "Could not discover reset domains.\n");
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(dev, "Found %u reset domains.\n", sc->ndomains);

	hwreset_register_ofw_provider(dev);

	return (0);
}

static int
scmi_reset_detach(device_t dev)
{

	return (EBUSY);
}

static device_method_t scmi_reset_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		scmi_reset_probe),
	DEVMETHOD(device_attach,	scmi_reset_attach),
	DEVMETHOD(device_detach,	scmi_reset_detach),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	scmi_reset_assert),

	DEVMETHOD_END
};

static driver_t scmi_reset_driver = {
	"scmi_reset",
	scmi_reset_methods,
	sizeof(struct scmi_reset_softc),
};

EARLY_DRIVER_MODULE(scmi_reset, scmi, scmi_reset_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(scmi_reset, 1);
