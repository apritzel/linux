/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2017 Icenowy Zheng <icenowy@aosc.xyz>
 */

#ifndef _CCU_SUN55I_A523_CPU_H
#define _CCU_SUN55I_A523_CPU_H

#include <dt-bindings/clock/sun55i-a523-cpu-ccu.h>

/* The PLL clocks itself are not exported. */

#define CLK_PLL_CPU_L		0
#define CLK_PLL_CPU_DSU		1
#define CLK_PLL_CPU_B		2

#define CLK_NUMBER	(CLK_CPU_B + 1)

#endif /* _CCU_SUN55I_A523_CPU_H */
