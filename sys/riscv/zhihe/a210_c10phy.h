/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Han Gao <gaohan@iscas.ac.cn>
 */

#ifndef _RISCV_ZHIHE_A210_C10PHY_H_
#define	_RISCV_ZHIHE_A210_C10PHY_H_

/* Type-C plug orientation reported by the port controller. */
#define	A210_C10_DETACHED	0
#define	A210_C10_NORMAL		1	/* CC1 */
#define	A210_C10_REVERSE	2	/* CC2 */

int	a210_c10phy_set_orientation(device_t dev, int orientation);

#endif /* _RISCV_ZHIHE_A210_C10PHY_H_ */
