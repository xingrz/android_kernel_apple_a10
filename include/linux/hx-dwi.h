/*
 * DWI backlight bus driver for Hx SoCs
 *
 * Copyright (C) 2020 Corellium LLC
 */

#ifndef _LINUX_INCLUDE_HX_DWI_H_
#define _LINUX_INCLUDE_HX_DWI_H_

#include <linux/device.h>

void hx_dwi_send(struct device *dev, unsigned cmd);
int hx_dwi_register(struct device *dev, struct device *client);

#endif
