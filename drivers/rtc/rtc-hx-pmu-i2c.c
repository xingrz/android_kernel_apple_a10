/*
 * Driver for RTC integrated in PMU of Hx SoC-based mobile devices.
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/hx-pmu-i2c.h>

struct hx_pmu_i2c_rtc {
    struct device *dev;
    struct rtc_device *rtc;
    struct nvmem_device *nvmem;
    u32 base, nvmem_base;
    s64 offs;
};

static int hx_pmu_i2c_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
    struct hx_pmu_i2c *pmu = dev_get_drvdata(dev->parent);
    struct hx_pmu_i2c_rtc *rtc = dev_get_drvdata(dev);
    u8 data[6];
    u64 time;
    int ret;

    ret = regmap_bulk_read(pmu->regmap, rtc->base, data, sizeof(data));
    if(ret < 0)
        goto fail;
    time = data[0] | ((u64)data[1] << 8) | ((u64)data[2] << 16) | ((u64)data[3] << 24) | ((u64)data[4] << 32) | ((u64)data[5] << 40);
    time += rtc->offs;
    rtc_time_to_tm(time >> 16, tm);
    return 0;

fail:
    dev_err(dev, "RTC read time failed: %d\n", ret);
    return ret;
}

static int hx_pmu_i2c_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
    return -EPERM;
}

static const struct rtc_class_ops hx_pmu_i2c_rtc_ops = {
    .read_time = hx_pmu_i2c_rtc_read_time,
    .set_time = hx_pmu_i2c_rtc_set_time,
};

static int hx_pmu_i2c_rtc_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct hx_pmu_i2c_rtc *rtc;
    u8 data[4];
    s32 offs;
    int ret;

    rtc = devm_kzalloc(&pdev->dev, sizeof(struct hx_pmu_i2c_rtc), GFP_KERNEL);
    if(!rtc)
        return -ENOMEM;

    ret = of_property_read_u32_index(node, "reg", 0, &rtc->base);
    if(ret < 0) {
        dev_err(&pdev->dev, "RTC requires 'reg' property: %d.\n", ret);
        return ret;
    }

    rtc->nvmem = of_nvmem_device_get(node, NULL);
    if(IS_ERR(rtc->nvmem)) {
        if(PTR_ERR(rtc->nvmem) == -EPROBE_DEFER)
            return PTR_ERR(rtc->nvmem);
        rtc->nvmem = NULL;
    }

    if(rtc->nvmem) {
        ret = of_property_read_u32_index(node, "nvmem-offset", 0, &rtc->nvmem_base);
        if(ret < 0) {
            dev_err(&pdev->dev, "if NVMEM device is present, 'nvmem-offset' must be supplied in device tree: %d.\n", ret);
            return ret;
        }

        ret = nvmem_device_read(rtc->nvmem, rtc->nvmem_base, sizeof(data), data);
        if(ret == sizeof(data)) {
            offs = (u32)data[0] | ((u32)data[1] << 8) | ((u32)data[2] << 16) | ((u32)data[3] << 24);
            rtc->offs = (s64)offs << 16;
        } else
            dev_warn(&pdev->dev, "failed to read from NVMEM, time may be incorrect.\n");
    } else
        dev_warn(&pdev->dev, "no 'nvmem' device, time may be incorrect.\n");

    rtc->dev = &pdev->dev;
    platform_set_drvdata(pdev, rtc);

    rtc->rtc = devm_rtc_device_register(&pdev->dev, pdev->name, &hx_pmu_i2c_rtc_ops, THIS_MODULE);
    if(IS_ERR(rtc->rtc)) {
        ret = PTR_ERR(rtc->rtc);
        dev_err(&pdev->dev, "failed to register RTC device: %d\n", ret);
        return ret;
    }

    return 0;
}

static const struct of_device_id hx_pmu_i2c_rtc_of_match[] = {
    { .compatible = "hx,i2c-pmu-rtc" },
    { },
};

static struct platform_driver hx_pmu_i2c_rtc_driver = {
    .driver = {
        .name = "rtc-hx-pmu-i2c",
        .of_match_table = of_match_ptr(hx_pmu_i2c_rtc_of_match),
    },
    .probe = hx_pmu_i2c_rtc_probe,
};

module_platform_driver(hx_pmu_i2c_rtc_driver);
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL v2");
