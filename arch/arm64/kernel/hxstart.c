/*
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <asm/cpu_ops.h>
#include <asm/errno.h>
#include <asm/smp_plat.h>
#include <asm/io.h>

#define MAGIC_UNLOCK 0xc5acce55

struct cpu_hxstart_info {
    void __iomem *pmgr_start;
    void __iomem *cputrc_rvbar;
    void __iomem *dbg_unlock;
};

extern void hx_aic_cpu_prepare(unsigned int cpu);

static int cpu_hxstart_cpu0_unlocked = 0;
static DEFINE_PER_CPU(struct cpu_hxstart_info, cpu_hxstart_info);

static int __init cpu_hxstart_cpu_init(unsigned int cpu)
{
    return 0;
}

static int cpu_hxstart_cpu_prepare(unsigned int cpu)
{
    struct device_node *node;
    struct cpu_hxstart_info *info;

    info = per_cpu_ptr(&cpu_hxstart_info, cpu);

    if(info->pmgr_start && info->cputrc_rvbar && info->dbg_unlock)
        return 0;

    node = of_find_compatible_node(NULL, NULL, "hx,startcpu");
    if(!node) {
        pr_err("%s: missing startcpu node in device tree.\n", __func__);
        return -EINVAL;
    }

    if(!info->pmgr_start) {
        info->pmgr_start = of_iomap(node, cpu * 3);
        if(!info->pmgr_start) {
            pr_err("%s: failed to map start register for CPU %d.\n", __func__, cpu);
            return -EINVAL;
        }
    }

    if(!info->cputrc_rvbar) {
        info->cputrc_rvbar = of_iomap(node, cpu * 3 + 1);
        if(!info->cputrc_rvbar) {
            pr_err("%s: failed to map reset address register for CPU %d.\n", __func__, cpu);
            return -EINVAL;
        }
    }

    if(!info->dbg_unlock) {
        info->dbg_unlock = of_iomap(node, cpu * 3 + 2);
        if(!info->dbg_unlock) {
            pr_err("%s: failed to map unlock register for CPU %d.\n", __func__, cpu);
            return -EINVAL;
        }
    }

    if(cpu)
        hx_aic_cpu_prepare(cpu);

    return 0;
}

static int cpu_hxstart_cpu_boot(unsigned int cpu)
{
    struct cpu_hxstart_info *info;
    unsigned long addr;

    if(!cpu_hxstart_cpu0_unlocked) {
        if(!cpu_hxstart_cpu_prepare(0)) {
            info = per_cpu_ptr(&cpu_hxstart_info, 0);
            writel(MAGIC_UNLOCK, info->dbg_unlock);
            cpu_hxstart_cpu0_unlocked = 1;
        } else
            pr_err("%s: failed to unlock boot CPU\n", __func__);
    }

    info = per_cpu_ptr(&cpu_hxstart_info, cpu);

    if(!info->pmgr_start || !info->cputrc_rvbar || !info->dbg_unlock)
        return -EINVAL;

    writeq(__pa_symbol(secondary_entry) | 1, info->cputrc_rvbar);
    readq(info->cputrc_rvbar);
    writeq(__pa_symbol(secondary_entry) | 1, info->cputrc_rvbar);
    addr = readq(info->cputrc_rvbar) & 0xFFFFFFFFFul;

    if(addr != (__pa_symbol(secondary_entry) | 1))
        pr_err("%s: CPU%d reset address: 0x%lx, failed to set to 0x%lx.\n", __func__, cpu, addr, (unsigned long)(__pa_symbol(secondary_entry) | 1));

    writel(MAGIC_UNLOCK, info->dbg_unlock);

    writel(1 << cpu, info->pmgr_start);
    writel(1 << cpu, info->pmgr_start + 4);

    return 0;
}

const struct cpu_operations cpu_hxstart_ops = {
    .name = "hx,startcpu",
    .cpu_init = cpu_hxstart_cpu_init,
    .cpu_prepare = cpu_hxstart_cpu_prepare,
    .cpu_boot = cpu_hxstart_cpu_boot,
};
