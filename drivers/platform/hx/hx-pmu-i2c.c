/*
 * Driver for I2C-based PMUs found on mobile devices with Hx SoCs.
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/hx-pmu-i2c.h>

struct hx_pmu_i2c_info {
    unsigned reg_bits, val_bits;
};

static const struct hx_pmu_i2c_info hx_pmu_i2c_d2333_info = { .reg_bits = 16, .val_bits = 8 };
static const struct hx_pmu_i2c_info hx_pmu_i2c_chestnut_info = { .reg_bits = 8, .val_bits = 8 };
static const struct of_device_id hx_pmu_i2c_of_match[] = {
    { .compatible = "hx,pmu-d2333", .data = &hx_pmu_i2c_d2333_info },
    { .compatible = "hx,pmu-chestnut", .data = &hx_pmu_i2c_chestnut_info },
    { },
};

static int hx_pmu_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct device_node *node = i2c->dev.of_node, *child;
    const struct of_device_id *of_id;
    const struct hx_pmu_i2c_info *info;
    struct hx_pmu_i2c *pmu;
    int ret;

    pmu = devm_kzalloc(&i2c->dev, sizeof(*pmu), GFP_KERNEL);
    if(!pmu)
        return -ENOMEM;

    of_id = of_match_device(hx_pmu_i2c_of_match, &i2c->dev);
    if(!of_id) {
        dev_err(&i2c->dev, "failed to match device.\n");
        return -ENODEV;
    }
    info = of_id->data;

    pmu->dev = &i2c->dev;
    i2c_set_clientdata(i2c, pmu);

    pmu->config.reg_bits = info->reg_bits;
    pmu->config.val_bits = info->val_bits;
    pmu->config.max_register = (1u << pmu->config.reg_bits) - 1u;

    pmu->regmap = devm_regmap_init_i2c(i2c, &pmu->config);
    if(IS_ERR(pmu->regmap)) {
        ret = PTR_ERR(pmu->regmap);
        dev_err(&i2c->dev, "failed to create regmap: %d.\n", ret);
        return ret;
    }

    for_each_child_of_node(node, child)
        of_platform_device_create(child, NULL, &i2c->dev);

    return 0;
}

static struct i2c_driver hx_pmu_i2c_driver = {
    .driver = {
        .name = "hx-pmu-i2c",
        .of_match_table = of_match_ptr(hx_pmu_i2c_of_match),
    },
    .probe = hx_pmu_i2c_probe,
};

static int __init hx_pmu_i2c_init(void)
{
    return i2c_add_driver(&hx_pmu_i2c_driver);
}

subsys_initcall(hx_pmu_i2c_init);
