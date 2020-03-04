/*
 *  Hx I2C PMU GPIO support
 *
 *  Copyright (C) 2020 Corellium LLC
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/hx-pmu-i2c.h>

struct hx_pmu_i2c_gpio {
    struct device *dev;
    struct gpio_chip gc;
    u32 reg_base, *reg_offs, in_base;
    u32 out_mask, out_val;
    u32 dir_mask, dir_in, dir_out;
    u8 *config;
};

static int hx_pmu_i2c_gpio_get_value(struct gpio_chip *chip, unsigned idx)
{
    struct hx_pmu_i2c_gpio *gpio = gpiochip_get_data(chip);
    struct hx_pmu_i2c *pmu = dev_get_drvdata(gpio->dev->parent);
    int ret;
    u8 data;

    ret = regmap_bulk_read(pmu->regmap, gpio->in_base + (idx >> 3), &data, 1);
    if(ret) {
        dev_err(gpio->dev, "Error reading state of GPIO %d: %d.\n", idx, ret);
        return ret;
    }
    return (data >> (idx & 7)) & 1;
}

static int hx_pmu_i2c_gpio_direction_input(struct gpio_chip *chip, unsigned idx)
{
    struct hx_pmu_i2c_gpio *gpio = gpiochip_get_data(chip);
    struct hx_pmu_i2c *pmu = dev_get_drvdata(gpio->dev->parent);
    int ret;
    u8 data;

    if((gpio->config[idx] & gpio->dir_mask) == gpio->dir_in)
        return 0;

    data = (gpio->config[idx] & ~gpio->dir_mask) | gpio->dir_in;
    ret = regmap_bulk_write(pmu->regmap, gpio->reg_base + gpio->reg_offs[idx], &data, 1);
    if(ret)
        return ret;
    gpio->config[idx] = data;
    return 0;
}

static int hx_pmu_i2c_gpio_direction_output(struct gpio_chip *chip, unsigned idx, int level)
{
    struct hx_pmu_i2c_gpio *gpio = gpiochip_get_data(chip);
    struct hx_pmu_i2c *pmu = dev_get_drvdata(gpio->dev->parent);
    int ret;
    u8 data;

    if((gpio->config[idx] & gpio->dir_mask) == gpio->dir_in)
        data = (gpio->config[idx] & ~gpio->dir_mask) | gpio->dir_out;
    else
        data = gpio->config[idx];

    data = (gpio->config[idx] & ~gpio->out_mask) ^ (level ? gpio->out_val : 0);
    if(data == gpio->config[idx])
        return 0;

    ret = regmap_bulk_write(pmu->regmap, gpio->reg_base + gpio->reg_offs[idx], &data, 1);
    if(ret)
        return ret;
    gpio->config[idx] = data;
    return 0;
}

static void hx_pmu_i2c_gpio_set_value(struct gpio_chip *chip, unsigned idx, int value)
{
    struct hx_pmu_i2c_gpio *gpio = gpiochip_get_data(chip);
    int ret = hx_pmu_i2c_gpio_direction_output(chip, idx, value);
    if(ret)
        dev_err(gpio->dev, "Error setting state of GPIO %d: %d.\n", idx, ret);
}

static int hx_pmu_i2c_gpio_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct hx_pmu_i2c *pmu = dev_get_drvdata(pdev->dev.parent);
    struct hx_pmu_i2c_gpio *gpio;
    int ret, idx;
    u32 base;

    gpio = devm_kzalloc(&pdev->dev, sizeof(struct hx_pmu_i2c_gpio), GFP_KERNEL);
    if(!gpio)
        return -ENOMEM;

    ret = of_property_read_u32_index(node, "reg", 0, &gpio->reg_base);
    if(ret < 0) {
        dev_err(&pdev->dev, "GPIO requires 'reg' property: %d.\n", ret);
        return ret;
    }

    ret = of_property_read_u32_index(node, "reg-input", 0, &gpio->in_base);
    if(ret < 0) {
        dev_err(&pdev->dev, "GPIO requires 'reg-input' property: %d.\n", ret);
        return ret;
    }

    ret = of_property_count_elems_of_size(node, "reg-offs", 4);
    if(ret < 1) {
        dev_err(&pdev->dev, "GPIO requires at least one item in 'reg-offs' property: %d.\n", ret);
        return ret;
    }
    gpio->reg_offs = devm_kzalloc(&pdev->dev, 4 * ret, GFP_KERNEL);
    if(!gpio->reg_offs)
        return -ENOMEM;
    gpio->gc.ngpio = ret;
    for(idx=0; idx<ret; idx++)
        of_property_read_u32_index(node, "reg-offs", idx, &gpio->reg_offs[idx]);

    ret = of_property_count_elems_of_size(node, "bit-outval", 4);
    if(ret != 2) {
        dev_err(&pdev->dev, "GPIO requires two items in 'bit-outval' property: %d.\n", ret);
        return ret;
    }
    of_property_read_u32_index(node, "bit-outval", 0, &gpio->out_mask);
    of_property_read_u32_index(node, "bit-outval", 1, &gpio->out_val);

    ret = of_property_count_elems_of_size(node, "bit-direction", 4);
    if(ret != 3) {
        dev_err(&pdev->dev, "GPIO requires three items in 'bit-direction' property: %d.\n", ret);
        return ret;
    }
    of_property_read_u32_index(node, "bit-direction", 0, &gpio->dir_mask);
    of_property_read_u32_index(node, "bit-direction", 1, &gpio->dir_in);
    of_property_read_u32_index(node, "bit-direction", 2, &gpio->dir_out);

    ret = of_property_read_u32_index(node, "gpio-base", 0, &base);
    if(ret)
        base = 0;

    gpio->config = devm_kzalloc(&pdev->dev, gpio->gc.ngpio, GFP_KERNEL);
    if(!gpio->config)
        return -ENOMEM;
    for(idx=0; idx<gpio->gc.ngpio; idx++) {
        ret = regmap_bulk_read(pmu->regmap, gpio->reg_base + gpio->reg_offs[idx], &gpio->config[idx], 1);
        if(ret) {
            dev_err(&pdev->dev, "Failed reading initial state of GPIO %d: %d.\n", idx, ret);
            return ret;
        }
    }

    gpio->dev = &pdev->dev;
    platform_set_drvdata(pdev, gpio);

    gpio->gc.parent = &pdev->dev;
    gpio->gc.label = dev_name(&pdev->dev);
    gpio->gc.base = base;
    gpio->gc.get = hx_pmu_i2c_gpio_get_value;
    gpio->gc.set = hx_pmu_i2c_gpio_set_value;
    gpio->gc.direction_input = hx_pmu_i2c_gpio_direction_input;
    gpio->gc.direction_output = hx_pmu_i2c_gpio_direction_output;

    return devm_gpiochip_add_data(&pdev->dev, &gpio->gc, gpio);
}

static const struct of_device_id hx_pmu_i2c_gpio_of_match[] = {
    { .compatible = "hx,i2c-pmu-gpio" },
    { },
};

static struct platform_driver hx_pmu_i2c_gpio_driver = {
    .driver = {
        .name = "gpio-hx-pmu-i2c",
        .of_match_table = of_match_ptr(hx_pmu_i2c_gpio_of_match),
    },
    .probe = hx_pmu_i2c_gpio_probe,
};

module_platform_driver(hx_pmu_i2c_gpio_driver);
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL v2");
