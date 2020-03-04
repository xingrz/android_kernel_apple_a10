/*
 * Driver for power switches in PMU of Hx SoC-based mobile devices.
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/hx-pmu-i2c.h>

struct hx_pmu_i2c_pwrsw {
    struct device *dev;
    struct regulator_dev *regu;
    struct regulator_desc desc;
    struct regulator_config config;
    struct regulator_init_data init_data;
    u32 base, mask, enable;
    int was_enabled;
};

static int hx_pmu_i2c_pwrsw_enable(struct regulator_dev *rdev)
{
    int ret;
    struct hx_pmu_i2c_pwrsw *pwrsw = rdev->reg_data;

    ret = regulator_enable_regmap(rdev);
    if(!ret)
        pwrsw->was_enabled = 1;
    return ret;
}

static int hx_pmu_i2c_pwrsw_disable(struct regulator_dev *rdev)
{
    return regulator_disable_regmap(rdev);
}

static int hx_pmu_i2c_pwrsw_is_enabled(struct regulator_dev *rdev)
{
    struct hx_pmu_i2c_pwrsw *pwrsw = rdev->reg_data;
    int ret;
    if(!pwrsw->was_enabled)
        ret = 0;
    else
        ret = regulator_is_enabled_regmap(rdev);
    return ret;
}

static const struct regulator_ops hx_pmu_i2c_pwrsw_ops = {
    .enable = hx_pmu_i2c_pwrsw_enable,
    .disable = hx_pmu_i2c_pwrsw_disable,
    .is_enabled = hx_pmu_i2c_pwrsw_is_enabled,
};

static int hx_pmu_i2c_pwrsw_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct hx_pmu_i2c *pmu = dev_get_drvdata(pdev->dev.parent);
    struct hx_pmu_i2c_pwrsw *pwrsw;
    const char *supply_name = NULL;

    pwrsw = devm_kzalloc(&pdev->dev, sizeof(struct hx_pmu_i2c_pwrsw), GFP_KERNEL);
    if(!pwrsw)
        return -ENOMEM;

    if(of_property_read_u32_index(node, "reg", 0, &pwrsw->base) ||
       of_property_read_u32_index(node, "mask", 0, &pwrsw->mask) ||
       of_property_read_u32_index(node, "enable", 0, &pwrsw->enable)) {
        dev_err(&pdev->dev, "power switch requires 'reg', 'mask' and 'enable' properties.\n");
        return -ENODEV;
    }

    if(of_property_read_string(node, "supply-name", &supply_name))
        supply_name = NULL;

    pwrsw->dev = &pdev->dev;
    platform_set_drvdata(pdev, pwrsw);

    pwrsw->desc.name = node->name;
    pwrsw->desc.supply_name = supply_name;
    pwrsw->desc.ops = &hx_pmu_i2c_pwrsw_ops;
    pwrsw->desc.type = REGULATOR_VOLTAGE;
    pwrsw->desc.owner = THIS_MODULE;
    pwrsw->desc.enable_time = 1000; /* us */
    pwrsw->desc.off_on_delay = 1000; /* us */
    pwrsw->desc.enable_reg = pwrsw->base;
    pwrsw->desc.enable_mask = pwrsw->mask;
    pwrsw->desc.enable_val = pwrsw->enable;
    pwrsw->desc.disable_val = pwrsw->enable ^ pwrsw->mask;
    pwrsw->init_data.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
    pwrsw->config.dev = &pdev->dev;
    pwrsw->config.driver_data = pwrsw;
    pwrsw->config.init_data = &pwrsw->init_data;
    pwrsw->config.of_node = node;
    pwrsw->config.regmap = pmu->regmap;

    pwrsw->regu = devm_regulator_register(&pdev->dev, &pwrsw->desc, &pwrsw->config);
    if(IS_ERR(pwrsw->regu)) {
        dev_err(&pdev->dev, "failed to register regulator: %ld.", PTR_ERR(pwrsw->regu));
        return PTR_ERR(pwrsw->regu);
    }

    return 0;
}

static const struct of_device_id hx_pmu_i2c_pwrsw_of_match[] = {
    { .compatible = "hx,i2c-pmu-switch" },
    { },
};

static struct platform_driver hx_pmu_i2c_pwrsw_driver = {
    .driver = {
        .name = "hx-pmu-i2c-pwrsw",
        .of_match_table = of_match_ptr(hx_pmu_i2c_pwrsw_of_match),
    },
    .probe = hx_pmu_i2c_pwrsw_probe,
};

module_platform_driver(hx_pmu_i2c_pwrsw_driver);
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL v2");
