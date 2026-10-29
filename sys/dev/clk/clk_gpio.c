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
 * Clock gate controlled by a GPIO line ("gpio-gate-clock").
 *
 * The GPIO line is often provided by an I2C expander that attaches after
 * the clock node, so it is looked up when the gate is first changed.
 * Until then the line keeps the state left by firmware.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>

#include <dev/clk/clk.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "clknode_if.h"

struct clk_gpio_gate_sc {
	device_t	dev;
	phandle_t	node;
	gpio_pin_t	pin;
};

static int
clk_gpio_gate_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
clk_gpio_gate_set_gate(struct clknode *clk, bool enable)
{
	struct clk_gpio_gate_sc *sc;
	int error;

	sc = clknode_get_softc(clk);
	if (sc->pin == NULL) {
		error = gpio_pin_get_by_ofw_property(sc->dev, sc->node,
		    "enable-gpios", &sc->pin);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot get enable GPIO: %d\n", error);
			return (error);
		}
		error = gpio_pin_setflags(sc->pin, GPIO_PIN_OUTPUT);
		if (error != 0) {
			gpio_pin_release(sc->pin);
			sc->pin = NULL;
			return (error);
		}
	}
	return (gpio_pin_set_active(sc->pin, enable));
}

static clknode_method_t clk_gpio_gate_clknode_methods[] = {
	CLKNODEMETHOD(clknode_init,	clk_gpio_gate_init),
	CLKNODEMETHOD(clknode_set_gate,	clk_gpio_gate_set_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(clk_gpio_gate_clknode, clk_gpio_gate_clknode_class,
    clk_gpio_gate_clknode_methods, sizeof(struct clk_gpio_gate_sc),
    clknode_class);

static int
clk_gpio_gate_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "gpio-gate-clock"))
		return (ENXIO);

	device_set_desc(dev, "GPIO gated clock");
	if (!bootverbose)
		device_quiet(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
clk_gpio_gate_attach(device_t dev)
{
	struct clknode_init_def def;
	struct clk_gpio_gate_sc *sc;
	struct clkdom *clkdom;
	struct clknode *clk;
	clk_t parent;
	phandle_t node;
	int error;

	node = ofw_bus_get_node(dev);
	error = clk_get_by_ofw_index(dev, node, 0, &parent);
	if (error != 0) {
		device_printf(dev, "cannot get parent clock: %d\n", error);
		return (ENXIO);
	}

	bzero(&def, sizeof(def));
	def.id = 1;
	error = clk_parse_ofw_clk_name(dev, node, &def.name);
	if (error != 0) {
		device_printf(dev, "cannot parse clock name\n");
		clk_release(parent);
		return (ENXIO);
	}
	def.parent_names = malloc(sizeof(char *), M_OFWPROP, M_WAITOK);
	def.parent_names[0] = clk_get_name(parent);
	def.parent_cnt = 1;

	clkdom = clkdom_create(dev);
	clk = clknode_create(clkdom, &clk_gpio_gate_clknode_class, &def);
	if (clk == NULL) {
		device_printf(dev, "cannot create clock\n");
		error = ENXIO;
		goto out;
	}
	sc = clknode_get_softc(clk);
	sc->dev = dev;
	sc->node = node;
	clknode_register(clkdom, clk);

	error = clkdom_finit(clkdom);
	if (error != 0) {
		device_printf(dev, "cannot finalize clock domain\n");
		error = ENXIO;
		goto out;
	}
	if (bootverbose)
		clkdom_dump(clkdom);
out:
	clk_release(parent);
	OF_prop_free(__DECONST(char *, def.name));
	OF_prop_free(def.parent_names);
	return (error);
}

static device_method_t clk_gpio_gate_methods[] = {
	DEVMETHOD(device_probe,		clk_gpio_gate_probe),
	DEVMETHOD(device_attach,	clk_gpio_gate_attach),
	DEVMETHOD_END
};

static DEFINE_CLASS_0(clk_gpio_gate, clk_gpio_gate_driver,
    clk_gpio_gate_methods, 0);

EARLY_DRIVER_MODULE(clk_gpio_gate, simplebus, clk_gpio_gate_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
MODULE_VERSION(clk_gpio_gate, 1);
