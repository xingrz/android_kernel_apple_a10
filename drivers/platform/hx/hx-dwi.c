/*
 * DWI backlight bus driver for Hx SoCs
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/hx-dwi.h>

#define REG_SETUP_1             0x000
#define REG_SETUP_3             0x004
#define REG_SETUP_2             0x050
#define REG_CMD                 0x080
#define  REG_CMD_SEND           0x1071
#define REG_DATA                0x084

struct hx_dwi {
    struct device *dev;
    void __iomem *base;
    struct clk *clk;
};

void hx_dwi_send(struct device *dev, unsigned cmd)
{
    struct hx_dwi *dwi = dev_get_drvdata(dev);
    writel(0, dwi->base + REG_CMD);
    writel(cmd, dwi->base + REG_DATA);
    writel(REG_CMD_SEND, dwi->base + REG_CMD);
}

int hx_dwi_register(struct device *dev, struct device *client)
{
    struct hx_dwi *dwi = dev_get_drvdata(dev);
    if(!dwi)
        return -EPROBE_DEFER;
    return 0;
}

static int hx_dwi_probe(struct platform_device *pdev)
{
    struct hx_dwi *dwi;
    struct resource *r;
    int ret = 0;

    dwi = devm_kzalloc(&pdev->dev, sizeof(struct hx_dwi), GFP_KERNEL);
    if(!dwi)
        return -ENOMEM;

    dwi->dev = &pdev->dev;

    dwi->clk = devm_clk_get(&pdev->dev, NULL);
    if(IS_ERR(dwi->clk))
        return PTR_ERR(dwi->clk);

    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    dwi->base = devm_ioremap_resource(&pdev->dev, r);
    if(IS_ERR(dwi->base))
        return PTR_ERR(dwi->base);

    ret = clk_prepare_enable(dwi->clk);
    if(ret < 0)
        return ret;

    writel(1, dwi->base + REG_SETUP_1);
    writel(0x2EE00, dwi->base + REG_SETUP_2);
    writel(0x78, dwi->base + REG_SETUP_3);

    dev_set_drvdata(dwi->dev, dwi);

    return 0;
}

static const struct of_device_id hx_dwi_match[] = {
    { .compatible = "hx,dwi" },
    { },
};
MODULE_DEVICE_TABLE(of, hx_dwi_match);

static struct platform_driver hx_dwi_driver = {
    .probe   = hx_dwi_probe,
    .driver  = {
        .name  = "hx-dwi",
        .of_match_table = hx_dwi_match,
    },
};
module_platform_driver(hx_dwi_driver);

MODULE_DESCRIPTION("Hx SoC DWI backlight bus driver");
MODULE_LICENSE("GPL v2");
