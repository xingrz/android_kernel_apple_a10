/*
 * Hx SoC power-off and reboot code
 *
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: zx-reboot.c
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/nvmem-consumer.h>
#include <linux/reboot.h>

#define REG_RESET_0             0x00
#define REG_UNKNOWN_0           0x04
#define REG_INTERVAL_0          0x08
#define REG_CTRL_0              0x0c
#define REG_RESET_1             0x10
#define REG_UNKNOWN_1           0x14
#define REG_INTERVAL_1          0x18
#define REG_CTRL_1              0x1c
#define  REG_CTRL_ENABLE_IRQ    1
#define  REG_CTRL_ACK_IRQ       2
#define  REG_CTRL_ENABLE_REBOOT 4

#define MAX_POWEROFF            8

struct hx_reboot {
    struct device *dev;
    void __iomem *base;
    struct nvmem_device *nvmem;
    u32 nvmem_poweroff[MAX_POWEROFF*3]; /* addr, mask, val */
    unsigned num_poweroff;

    struct notifier_block restart_nb;
};

static struct hx_reboot *hx_reboot = NULL;

static int hx_reboot_restart(struct notifier_block *this, unsigned long mode, void *cmd)
{
    if(!hx_reboot)
        return NOTIFY_DONE;

    writel(0, hx_reboot->base + REG_RESET_1);
    writel(0, hx_reboot->base + REG_UNKNOWN_1);
    writel(REG_CTRL_ENABLE_REBOOT, hx_reboot->base + REG_CTRL_1);
    writel(0, hx_reboot->base + REG_RESET_1);

    mdelay(500);
    pr_emerg("hx-reboot: Unable to restart!\n");
    return NOTIFY_DONE;
}

static void hx_reboot_poweroff(void)
{
    int ret;
    u8 data;
    unsigned idx;

    if(!hx_reboot)
        return;

    if(hx_reboot->nvmem)
        for(idx=0; idx<hx_reboot->num_poweroff; idx++) {
            ret = nvmem_device_read(hx_reboot->nvmem, hx_reboot->nvmem_poweroff[idx * 3], 1, &data);
            if(ret) {
                pr_err("hx-reboot: %d reading NVMEM %d.\n", ret, idx);
                break;
            }
            data &= ~hx_reboot->nvmem_poweroff[idx * 3 + 1];
            data |= hx_reboot->nvmem_poweroff[idx * 3 + 2];
            ret = nvmem_device_write(hx_reboot->nvmem, hx_reboot->nvmem_poweroff[idx * 3], 1, &data);
            if(ret) {
                pr_err("hx-reboot: %d writing NVMEM %d.\n", ret, idx);
                break;
            }
        }

    mdelay(50);

    writel(0, hx_reboot->base + REG_RESET_1);
    writel(0, hx_reboot->base + REG_UNKNOWN_1);
    writel(REG_CTRL_ENABLE_REBOOT, hx_reboot->base + REG_CTRL_1);
    writel(0, hx_reboot->base + REG_RESET_1);

    mdelay(500);
    pr_emerg("hx-reboot: Unable to power off!\n");
}

static int hx_reboot_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct hx_reboot *hxr;
    struct resource *r;
    int err;

    if(hx_reboot)
        return -EEXIST;

    hxr = devm_kzalloc(&pdev->dev, sizeof(struct hx_reboot), GFP_KERNEL);
    if(!hxr)
        return -ENOMEM;

    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    hxr->base = devm_ioremap_resource(&pdev->dev, r);
    if(IS_ERR(hxr->base))
        return PTR_ERR(hxr->base);

    hxr->nvmem = of_nvmem_device_get(node, NULL);
    if(IS_ERR(hxr->nvmem) || !hxr->nvmem) {
        if(PTR_ERR(hxr->nvmem) == -EPROBE_DEFER)
            return PTR_ERR(hxr->nvmem);
        dev_warn(&pdev->dev, "No 'nvmem' parameter, will not be able to power off: %ld.\n", PTR_ERR(hxr->nvmem));
        hxr->nvmem = NULL;
    }

    if(hxr->nvmem) {
        err = of_property_read_variable_u32_array(node, "nvmem-poweroff", hxr->nvmem_poweroff, 3, MAX_POWEROFF * 3);
        if(err < 3 || (err % 3)) {
            dev_err(&pdev->dev, "'nvmem-poweroff' parameter must have triplets of elements: %d.\n", err);
            return err;
        }
        hxr->num_poweroff = err / 3;
    }

    hxr->restart_nb.notifier_call = hx_reboot_restart;
    hxr->restart_nb.priority = 128;
    err = register_restart_handler(&hxr->restart_nb);
    if(err) {
        dev_err(&pdev->dev, "Registering restart handler failed: %d.\n", err);
        return err;
    }

    writel(0, hxr->base + REG_CTRL_0);
    writel(0, hxr->base + REG_CTRL_1);

    pm_power_off = &hx_reboot_poweroff;

    hx_reboot = hxr;
    return 0;
}

static const struct of_device_id hx_reboot_of_match[] = {
    { .compatible = "hx,reboot" },
    { }
};
MODULE_DEVICE_TABLE(of, hx_reboot_of_match);

static struct platform_driver hx_reboot_driver = {
    .probe = hx_reboot_probe,
    .driver = {
        .name = "hx-reboot",
        .of_match_table = hx_reboot_of_match,
    },
};
module_platform_driver(hx_reboot_driver);

MODULE_DESCRIPTION("Hx SoC reset driver");
MODULE_LICENSE("GPL v2");
