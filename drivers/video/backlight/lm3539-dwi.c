/*
 * Backlight driver for LM3539 chips on I2C and DWI.
 *
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: bd6107.c
 *   Copyright (C) 2013 Ideas on board SPRL
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hx-dwi.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#define REG_ENABLE      16

#define MODE_SINGLE     0
#define MODE_PRIMARY    1
#define MODE_SECONDARY  2

struct lm3539_dwi {
    struct i2c_client *client;
    int mode;
    struct device *secondary;
    struct backlight_device *backlight;
    struct device *dwi;
    struct regulator *regu_lcd;
    int enabled;
    u8 regs[16];
};

static int lm3539_dwi_secondary_register(struct device *dev, struct device *primary)
{
    struct lm3539_dwi *lm = dev_get_drvdata(dev);
    if(lm->mode != MODE_SECONDARY) {
        dev_err(dev, "attempted to set primary for a non-secondary device.\n");
        return -EINVAL;
    }
    return 0;
}

static int lm3539_dwi_secondary_control(struct device *dev, int enable)
{
    struct lm3539_dwi *lm = dev_get_drvdata(dev);
    int ret;

    if(enable == lm->enabled)
        return 0;

    ret = i2c_smbus_write_byte_data(lm->client, REG_ENABLE, (lm->regs[0] & 0xFE) | (enable ? 0x01 : 0x00));
    if(ret) {
        dev_err(&lm->client->dev, "I2C write failed.\n");
        return ret;
    }
    lm->enabled = enable;
    return 0;
}

static int lm3539_dwi_backlight_update_status(struct backlight_device *backlight)
{
    struct lm3539_dwi *lm = bl_get_data(backlight);
    int brightness = backlight->props.brightness;
    int ret;

    if (backlight->props.power != FB_BLANK_UNBLANK ||
        backlight->props.fb_blank != FB_BLANK_UNBLANK ||
        backlight->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
        brightness = 0;

    if(brightness) {
        if(!lm->enabled) {
            ret = regulator_enable(lm->regu_lcd);
            if(ret)
                return ret;
            ret = i2c_smbus_write_byte_data(lm->client, REG_ENABLE, lm->regs[0] | 0x01);
            if(ret) {
                dev_err(&lm->client->dev, "I2C write failed.\n");
                return ret;
            }
            if(lm->mode == MODE_PRIMARY) {
                ret = lm3539_dwi_secondary_control(lm->secondary, 1);
                if(ret)
                    return ret;
            }
            lm->enabled = 1;
            mdelay(10);
        }
        hx_dwi_send(lm->dwi, 0xA0000000 | brightness);
    } else if(lm->enabled) {
        if(lm->mode == MODE_PRIMARY) {
            ret = lm3539_dwi_secondary_control(lm->secondary, 0);
            if(ret)
                return ret;
        }
        ret = i2c_smbus_write_byte_data(lm->client, REG_ENABLE, lm->regs[0] & 0xFE);
        if(ret) {
            dev_err(&lm->client->dev, "I2C write failed.\n");
            return ret;
        }
        lm->enabled = 0;
        regulator_disable(lm->regu_lcd);
    }

    return 0;
}

static const struct backlight_ops lm3539_dwi_backlight_ops = {
    .update_status = lm3539_dwi_backlight_update_status,
};

static int lm3539_dwi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct backlight_device *backlight;
    struct backlight_properties props;
    struct lm3539_dwi *lm;
    struct platform_device *pdev_dwi;
    struct device_node *node = client->dev.of_node, *of_dwi, *of_sibling;
    int ret, i;

    lm = devm_kzalloc(&client->dev, sizeof(*lm), GFP_KERNEL);
    if(!lm)
        return -ENOMEM;

    lm->mode = MODE_SINGLE;

    of_sibling = of_parse_phandle(node, "secondary", 0);
    if(of_sibling) {
        lm->mode = MODE_PRIMARY;
        lm->secondary = bus_find_device_by_of_node(&i2c_bus_type, of_sibling);
        if(!lm->secondary)
            return -EPROBE_DEFER;
        ret = lm3539_dwi_secondary_register(lm->secondary, &client->dev);
        if(ret)
            return ret;
    }

    of_sibling = of_parse_phandle(node, "primary", 0);
    if(of_sibling) {
        if(lm->mode == MODE_PRIMARY) {
            dev_err(&client->dev, "backlight device cannot be both primary and secondary.\n");
            return -EINVAL;
        }
        lm->mode = MODE_SECONDARY;
    }

    if(lm->mode != MODE_SECONDARY) {
        lm->regu_lcd = devm_regulator_get(&client->dev, "lcd");
        if(IS_ERR(lm->regu_lcd)) {
            if(PTR_ERR(lm->regu_lcd) != -EPROBE_DEFER)
                dev_err(&client->dev, "backlight device requires 'lcd-supply' power supply.\n");
            return PTR_ERR(lm->regu_lcd);
        }

        of_dwi = of_parse_phandle(node, "dwi", 0);
        if(!of_dwi) {
            dev_err(&client->dev, "backlight device requires 'dwi' bus handle.\n");
            return -ENODEV;
        }
        pdev_dwi = of_find_device_by_node(of_dwi);
        if(!pdev_dwi)
            return -EPROBE_DEFER;
        lm->dwi = &pdev_dwi->dev;
        ret = hx_dwi_register(lm->dwi, &client->dev);
        if(ret)
            return ret;
    }

    lm->client = client;

    for(i=0; i<16; i++) {
        ret = i2c_smbus_read_byte_data(client, REG_ENABLE + i);
        if(ret < 0) {
            dev_err(&client->dev, "I2C read failed.\n");
            return ret;
        }
        lm->regs[i] = ret;
    }

    if(lm->mode != MODE_SECONDARY) {
        memset(&props, 0, sizeof(props));
        props.type = BACKLIGHT_RAW;
        props.max_brightness = 0x7D1;
        props.brightness = 0x540;

        backlight = devm_backlight_device_register(&client->dev, dev_name(&client->dev), &lm->client->dev, lm, &lm3539_dwi_backlight_ops, &props);
        if(IS_ERR(backlight)) {
            dev_err(&client->dev, "failed to register backlight.\n");
            return PTR_ERR(backlight);
        }

        backlight_update_status(backlight);
        dev_info(&client->dev, "backlight registered.\n");
        i2c_set_clientdata(client, backlight);
    } else {
        dev_info(&client->dev, "secondary backlight registered.\n");
        i2c_set_clientdata(client, lm);
    }

    return 0;
}

static const struct of_device_id lm3539_dwi_of_match[] = {
    { .compatible = "dwi,lm3534" },
    { .compatible = "dwi,lm3539" },
    { }
};

static struct i2c_driver lm3539_dwi_driver = {
    .driver = {
        .name = "lm3539_dwi",
        .of_match_table = of_match_ptr(lm3539_dwi_of_match),
    },
    .probe = lm3539_dwi_probe,
};

module_i2c_driver(lm3539_dwi_driver);

MODULE_DESCRIPTION("LM3539 DWI Backlight Driver");
MODULE_LICENSE("GPL");
