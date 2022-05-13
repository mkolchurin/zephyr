/*
 * Copyright (c) 2018, Intel Corporation
 * All rights reserved.
 *
 * Author:	Seppo Ingalsuo <seppo.ingalsuo@linux.intel.com>
 *		Sathish Kuttan <sathish.k.kuttan@intel.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>

#include "pdm_decim_fir.h"

static const int32_t fir_int32_02_4375_5100_010_095[101] = {
	-587830,
	-2653881,
	-5154608,
	-4845367,
	-226473,
	4220832,
	2571159,
	-3184701,
	-4043579,
	2206821,
	5554546,
	-750496,
	-6923900,
	-1268584,
	8073360,
	4085183,
	-8546477,
	-7505364,
	8176185,
	11533751,
	-6471060,
	-15704256,
	3359707,
	19852409,
	1635593,
	-23144509,
	-8252640,
	25285013,
	16574479,
	-25723227,
	-26663304,
	23549736,
	38139664,
	-17943366,
	-50446983,
	8141043,
	63090267,
	7051351,
	-75166959,
	-29039894,
	85772627,
	60568979,
	-93167356,
	-106799776,
	94198973,
	180962815,
	-78385592,
	-324820245,
	-12243140,
	742491441,
	1151461281,
	742491441,
	-12243140,
	-324820245,
	-78385592,
	180962815,
	94198973,
	-106799776,
	-93167356,
	60568979,
	85772627,
	-29039894,
	-75166959,
	7051351,
	63090267,
	8141043,
	-50446983,
	-17943366,
	38139664,
	23549736,
	-26663304,
	-25723227,
	16574479,
	25285013,
	-8252640,
	-23144509,
	1635593,
	19852409,
	3359707,
	-15704256,
	-6471060,
	11533751,
	8176185,
	-7505364,
	-8546477,
	4085183,
	8073360,
	-1268584,
	-6923900,
	-750496,
	5554546,
	2206821,
	-4043579,
	-3184701,
	2571159,
	4220832,
	-226473,
	-4845367,
	-5154608,
	-2653881,
	-587830

};

struct pdm_decim pdm_decim_int32_02_4375_5100_010_095 = {
	2, 101, 0, 4375, 5100, 10, 95, fir_int32_02_4375_5100_010_095
};
