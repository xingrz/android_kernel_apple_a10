/*
 * Driver for I2C-based PMUs found on mobile devices with Hx SoCs.
 *
 * Copyright (C) 2020 Corellium LLC
 */

#ifndef _LINUX_INCLUDE_HX_PMU_I2C_H_
#define _LINUX_INCLUDE_HX_PMU_I2C_H_

#include <linux/device.h>
#include <linux/regmap.h>

struct hx_pmu_i2c {
    struct device *dev;
    struct regmap *regmap;
    struct regmap_config config;
};

#endif
