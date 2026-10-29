/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Ruslan Bukin <br@bsdpad.com>
 * Copyright (c) 2023 Arm Ltd
 *
 * This work was supported by Innovate UK project 105694, "Digital Security
 * by Design (DSbD) Technology Platform Prototype".
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

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/fdt/simplebus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "dev/mailbox/arm/arm_doorbell.h"

#include "mbox_if.h"
#include "scmi.h"
#include "scmi_protocols.h"
#include "scmi_shmem.h"

#define SCMI_MBOX_POLL_INTERVAL_MS	3

struct scmi_mailbox_softc {
	struct scmi_softc	base;
	device_t		a2p_dev;
	struct arm_doorbell	*db;		/* ARM MHU doorbell */
	device_t		mbox_dev;	/* Other mailboxes */
	int			mbox_chan;
};

static int	scmi_mailbox_transport_init(device_t);
static void	scmi_mailbox_transport_cleanup(device_t);
static int	scmi_mailbox_xfer_msg(device_t, struct scmi_msg *);
static int	scmi_mailbox_poll_msg(device_t, struct scmi_msg *,
    unsigned int);
static int	scmi_mailbox_collect_reply(device_t, struct scmi_msg *);
static void	scmi_mailbox_tx_complete(device_t, void *);

static int	scmi_mailbox_probe(device_t);

static void
scmi_mailbox_a2p_callback(void *arg)
{
	struct scmi_mailbox_softc *sc;
	uint32_t msg_header, rx_len;
	int ret;

	sc = arg;

	ret = scmi_shmem_read_msg_header(sc->a2p_dev, &msg_header, &rx_len);
	if (ret == 0)
		scmi_rx_irq_callback(sc->base.dev, sc->a2p_dev, msg_header, rx_len);
}

/*
 * Find the "tx" mailbox. ARM MHU doorbells are handled by arm_doorbell;
 * other controllers implement the mbox interface, where a write to the
 * channel rings the doorbell.
 */
static int
scmi_mailbox_ofw_get(device_t dev, phandle_t node, device_t *mbox,
    int *chan)
{
	phandle_t provider;
	pcell_t *cells;
	int error, idx, ncells;

	if (ofw_bus_find_string_index(node, "mbox-names", "tx", &idx) != 0)
		idx = 0;
	error = ofw_bus_parse_xref_list_alloc(node, "mboxes", "#mbox-cells",
	    idx, &provider, &ncells, &cells);
	if (error != 0)
		return (error);

	error = ENOENT;
	if (!ofw_bus_node_is_compatible(OF_node_from_xref(provider),
	    "arm,mhu-doorbell")) {
		*mbox = OF_device_from_xref(provider);
		*chan = ncells > 0 ? cells[0] : 0;
		error = *mbox == NULL ? ENXIO : 0;
	}
	OF_prop_free(cells);

	return (error);
}

static int
scmi_mailbox_transport_init(device_t dev)
{
	struct scmi_mailbox_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);

	node = ofw_bus_get_node(dev);
	if (node == -1)
		return (ENXIO);
	/*
	 * TODO
	 * - Support P2A shmem + IRQ/doorbell
	 * - Support other mailbox devices
	 */
	sc->a2p_dev = scmi_shmem_get(dev, node, SCMI_CHAN_A2P);
	if (sc->a2p_dev == NULL) {
		device_printf(dev, "A2P shmem dev not found.\n");
		return (ENXIO);
	}

	sc->base.trs_desc.reply_timo_ms = 30;

	error = scmi_mailbox_ofw_get(dev, node, &sc->mbox_dev,
	    &sc->mbox_chan);
	if (error == 0) {
		/* Replies are polled for in shared memory. */
		sc->base.trs_desc.no_completion_irq = true;
		return (0);
	}
	if (error != ENOENT) {
		device_printf(dev, "Mailbox device not found.\n");
		return (ENXIO);
	}

	/* TODO: Fix ofw_get...mbox doorbell names NOT required in Linux DT */
	sc->db = arm_doorbell_ofw_get(dev, "tx");
	if (sc->db == NULL) {
		device_printf(dev, "Doorbell device not found.\n");
		return (ENXIO);
	}

	arm_doorbell_set_handler(sc->db, scmi_mailbox_a2p_callback, sc);

	return (0);
}

static void
scmi_mailbox_transport_cleanup(device_t dev)
{
	struct scmi_mailbox_softc *sc;

	sc = device_get_softc(dev);

	if (sc->db != NULL)
		arm_doorbell_set_handler(sc->db, NULL, NULL);
}

static int
scmi_mailbox_xfer_msg(device_t dev, struct scmi_msg *msg)
{
	struct scmi_mailbox_softc *sc;
	int ret;

	sc = device_get_softc(dev);

	ret = scmi_shmem_prepare_msg(sc->a2p_dev, (uint8_t *)&msg->hdr,
	    msg->tx_len, msg->polling);
	if (ret != 0)
		return (ret);

	/* Interrupt SCP firmware. */
	if (sc->db != NULL)
		arm_doorbell_set(sc->db);
	else
		ret = MBOX_WRITE(sc->mbox_dev, sc->mbox_chan, 0);

	return (ret);
}

static int
scmi_mailbox_poll_msg(device_t dev, struct scmi_msg *msg, unsigned int tmo_ms)
{
	struct scmi_mailbox_softc *sc;
	unsigned int tmo_loops = tmo_ms / SCMI_MBOX_POLL_INTERVAL_MS;

	sc = device_get_softc(dev);

	do {
		if (scmi_shmem_poll_msg(sc->a2p_dev, &msg->hdr, &msg->rx_len))
			break;
		DELAY(SCMI_MBOX_POLL_INTERVAL_MS * 1000);
	} while (tmo_loops--);

	return (tmo_loops > 0 ? 0 : ETIMEDOUT);
}

static int
scmi_mailbox_collect_reply(device_t dev, struct scmi_msg *msg)
{
	struct scmi_mailbox_softc *sc;
	int ret;

	sc = device_get_softc(dev);

	ret = scmi_shmem_read_msg_payload(sc->a2p_dev,
	    msg->payld, msg->rx_len - SCMI_MSG_HDR_SIZE, msg->rx_len);

	return (ret);
}

static void
scmi_mailbox_tx_complete(device_t dev, void *chan)
{
	struct scmi_mailbox_softc *sc;

	sc = device_get_softc(dev);
	scmi_shmem_tx_complete(sc->a2p_dev);
}

static void
scmi_mailbox_clear_channel(device_t dev, void *chan)
{
	/* Only P2A channel can be cleared forcibly by agent */
	scmi_shmem_clear_channel(chan);
}

static int
scmi_mailbox_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "arm,scmi"))
		return (ENXIO);

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	device_set_desc(dev, "ARM SCMI Mailbox Transport driver");

	return (BUS_PROBE_DEFAULT);
}

static device_method_t scmi_mailbox_methods[] = {
	DEVMETHOD(device_probe,		scmi_mailbox_probe),

	/* SCMI interface */
	DEVMETHOD(scmi_transport_init,		scmi_mailbox_transport_init),
	DEVMETHOD(scmi_transport_cleanup,	scmi_mailbox_transport_cleanup),
	DEVMETHOD(scmi_xfer_msg,		scmi_mailbox_xfer_msg),
	DEVMETHOD(scmi_poll_msg,		scmi_mailbox_poll_msg),
	DEVMETHOD(scmi_collect_reply,		scmi_mailbox_collect_reply),
	DEVMETHOD(scmi_tx_complete,		scmi_mailbox_tx_complete),
	DEVMETHOD(scmi_clear_channel,		scmi_mailbox_clear_channel),

	DEVMETHOD_END
};

DEFINE_CLASS_1(scmi_mailbox, scmi_mailbox_driver, scmi_mailbox_methods,
    sizeof(struct scmi_mailbox_softc), scmi_driver);

DRIVER_MODULE(scmi_mailbox, simplebus, scmi_mailbox_driver, 0, 0);
MODULE_VERSION(scmi_mailbox, 1);
