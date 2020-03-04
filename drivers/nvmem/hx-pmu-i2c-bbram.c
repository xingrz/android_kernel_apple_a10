/*
 * Driver for BBRAM inside PMU of Hx SoC-based mobile devices.
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/hx-pmu-i2c.h>

struct hx_pmu_i2c_bbram {
    struct device *dev;
    struct nvmem_config cfg;
    struct nvmem_device *nvmem;
    u32 base, size;
};

static int hx_pmu_i2c_bbram_write(void *context, unsigned int offset, void *val, size_t bytes)
{
    struct hx_pmu_i2c_bbram *bbram = context;
    struct hx_pmu_i2c *pmu = dev_get_drvdata(bbram->dev->parent);
    return regmap_bulk_write(pmu->regmap, bbram->base + offset, val, bytes);
}

static int hx_pmu_i2c_bbram_read(void *context, unsigned int offset, void *val, size_t bytes)
{
    struct hx_pmu_i2c_bbram *bbram = context;
    struct hx_pmu_i2c *pmu = dev_get_drvdata(bbram->dev->parent);
    return regmap_bulk_read(pmu->regmap, bbram->base + offset, val, bytes);
}

static int hx_pmu_i2c_bbram_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct hx_pmu_i2c_bbram *bbram;
    struct nvmem_config *cfg;

    bbram = devm_kzalloc(&pdev->dev, sizeof(struct hx_pmu_i2c_bbram), GFP_KERNEL);
    if(!bbram)
        return -ENOMEM;

    if(of_property_read_u32_index(node, "reg", 0, &bbram->base) < 0 ||
       of_property_read_u32_index(node, "size", 0, &bbram->size) < 0) {
        dev_err(&pdev->dev, "BBRAM requires 'reg' and 'size' properties.\n");
        return -EINVAL;
    }

    bbram->dev = &pdev->dev;
    platform_set_drvdata(pdev, bbram);

    cfg = &bbram->cfg;
    cfg->priv = bbram;
    cfg->name = "bbram";
    cfg->dev = &pdev->dev;
    cfg->root_only = 1;
    cfg->type = NVMEM_TYPE_BATTERY_BACKED;
    cfg->stride = 1;
    cfg->word_size = 1;
    cfg->size = bbram->size,
    cfg->owner = THIS_MODULE;
    cfg->reg_read  = hx_pmu_i2c_bbram_read;
    cfg->reg_write = hx_pmu_i2c_bbram_write;

    bbram->nvmem = devm_nvmem_register(&pdev->dev, cfg);
    return PTR_ERR_OR_ZERO(bbram->nvmem);
}

static const struct of_device_id hx_pmu_i2c_bbram_of_match[] = {
    { .compatible = "hx,i2c-pmu-bbram" },
    { },
};

static struct platform_driver hx_pmu_i2c_bbram_driver = {
    .driver = {
        .name = "hx-pmu-i2c-bbram",
        .of_match_table = of_match_ptr(hx_pmu_i2c_bbram_of_match),
    },
    .probe = hx_pmu_i2c_bbram_probe,
};

module_platform_driver(hx_pmu_i2c_bbram_driver);
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL v2");
