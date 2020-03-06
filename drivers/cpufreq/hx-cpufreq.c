/*
 * CPU power state driver for Hx SoCs
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <soc/tegra/bpmp.h>
#include <soc/tegra/bpmp-abi.h>

#define REG_ACC_CLK_L2FEAT      0x0018
#define REG_ACC_CLK_PSTATE      0x0020
#define REG_ACC_CLK_ENABLE2     0x0048
#define REG_ACC_CLK_ENABLE1     0x0660
#define REG_ACC_CLK_ENABLE3     0x06b8
#define REG_DVFM_VSET_LO        0x1220
#define REG_DVFM_VSET_HI        0x1228
#define REG_DVFM_SET1_LO        0x1100
#define REG_DVFM_SET1_HI        0x1108
#define REG_DVFM_SET2_LO        0x1120
#define REG_DVFM_SET2_HI        0x1128
#define REG_DVFM_MASK_LO        0x1420
#define REG_DVFM_MASK_HI        0x1428
#define REG_ACC_DBG_CTRL        0x2000
#define REG_PSINFO_SET1_DFLT    0x3000
#define REG_PSINFO_SET2_DFLT    0x3008
#define REG_PSINFO_GET1(ps)    (0x3000+0x20*(ps))
#define REG_PSINFO_GET2(ps)    (0x3008+0x20*(ps))
#define REG_PSINFO_SET1         0x31E0
#define REG_PSINFO_SET2         0x31E8
#define REG_MCCPS_IDX           0x4000
#define REG_MCCPS_GET(ps)      (0x4018+0x10*(ps))
#define REG_PSTATE_DISABLE      0x5080
#define REG_PSTATE_CTRL         0x6000
#define REG_PSTATE_SET3         0x6008
#define REG_PSTATE_SET2         0x6010
#define REG_PSTATE_SET1         0x6018
#define REG_CPUPM_CPU0          0x7000
#define REG_CPUPM_CPU1          0x7200

struct hx_cpufreq_config {
    unsigned num_states;
    const u8 *dvfm_vset;
    const u8 *dvfm_set1;
    const u8 *dvfm_set2;
    struct cpufreq_frequency_table *cpufreq_table;
};

#define NUM_BASES       8
struct hx_cpufreq_data {
    void __iomem *base[NUM_BASES];
    const struct hx_cpufreq_config *cfg;
};

/* frequency table for fast devices */

#define NUM_STATES_FAST         12
static const u8 hx_cpufreq_dvfm_vset_fast[NUM_STATES_FAST] = { 0x00, 0x00, 0x37, 0x3B, 0x4F, 0x40, 0x4C, 0x5F, 0x71, 0x8F, 0xB5, 0xB5 };
static const u8 hx_cpufreq_dvfm_set1_fast[NUM_STATES_FAST] = { 0x00, 0x00, 0x0A, 0x05, 0x04, 0x1F, 0x15, 0x10, 0x0D, 0x0B, 0x09, 0x09 };
static const u8 hx_cpufreq_dvfm_set2_fast[NUM_STATES_FAST] = { 0x00, 0x00, 0x12, 0x0A, 0x06, 0x09, 0x07, 0x05, 0x04, 0x03, 0x03, 0x03 };

/* E-core frequencies are adjusted to match measured performance to produce a linear "frequency" table */
static struct cpufreq_frequency_table hx_cpufreq_table_fast[NUM_STATES_FAST + 1] = {
    { .frequency = CPUFREQ_ENTRY_INVALID },
    { .frequency = CPUFREQ_ENTRY_INVALID },
    { .frequency =  149000 }, /*  396 MHz, E-core */
    { .frequency =  275000 }, /*  732 MHz, E-core */
    { .frequency =  410000 }, /* 1092 MHz, E-core */
    { .frequency =  756000 }, /*  756 MHz, P-core */
    { .frequency = 1056000 }, /* 1056 MHz, P-core */
    { .frequency = 1356000 }, /* 1356 MHz, P-core */
    { .frequency = 1644000 }, /* 1644 MHz, P-core */
    { .frequency = 1944000 }, /* 1944 MHz, P-core */
    { .frequency = 2244000 }, /* 2244 MHz, P-core */
    { .frequency = 2340000 }, /* 2340 MHz, P-core */
    { .frequency = CPUFREQ_TABLE_END } };

static const struct hx_cpufreq_config hx_cpufreq_config_fast = {
    .num_states = NUM_STATES_FAST,
    .dvfm_vset = hx_cpufreq_dvfm_vset_fast,
    .dvfm_set1 = hx_cpufreq_dvfm_set1_fast,
    .dvfm_set2 = hx_cpufreq_dvfm_set2_fast,
    .cpufreq_table = hx_cpufreq_table_fast };

/* frequency table for slow devices */

#define NUM_STATES_SLOW         9
static const u8 hx_cpufreq_dvfm_vset_slow[NUM_STATES_SLOW] = { 0x00, 0x00, 0x37, 0x3B, 0x4F, 0x40, 0x4C, 0x5F, 0x71 };
static const u8 hx_cpufreq_dvfm_set1_slow[NUM_STATES_SLOW] = { 0x00, 0x00, 0x0A, 0x05, 0x04, 0x1F, 0x15, 0x10, 0x0D };
static const u8 hx_cpufreq_dvfm_set2_slow[NUM_STATES_SLOW] = { 0x00, 0x00, 0x12, 0x0A, 0x06, 0x09, 0x07, 0x05, 0x04 };

static struct cpufreq_frequency_table hx_cpufreq_table_slow[NUM_STATES_SLOW + 1] = {
    { .frequency = CPUFREQ_ENTRY_INVALID },
    { .frequency = CPUFREQ_ENTRY_INVALID },
    { .frequency =  149000 }, /*  396 MHz, E-core */
    { .frequency =  275000 }, /*  732 MHz, E-core */
    { .frequency =  410000 }, /* 1092 MHz, E-core */
    { .frequency =  756000 }, /*  756 MHz, P-core */
    { .frequency = 1056000 }, /* 1056 MHz, P-core */
    { .frequency = 1356000 }, /* 1356 MHz, P-core */
    { .frequency = 1644000 }, /* 1644 MHz, P-core */
    { .frequency = CPUFREQ_TABLE_END } };

static const struct hx_cpufreq_config hx_cpufreq_config_slow = {
    .num_states = NUM_STATES_SLOW,
    .dvfm_vset = hx_cpufreq_dvfm_vset_slow,
    .dvfm_set1 = hx_cpufreq_dvfm_set1_slow,
    .dvfm_set2 = hx_cpufreq_dvfm_set2_slow,
    .cpufreq_table = hx_cpufreq_table_slow };

static inline void __iomem *hx_cpufreq_reg(struct hx_cpufreq_data *hc, unsigned reg)
{
    return hc->base[reg >> 12] + (reg & 0xFFF);
}
static u32 hx_cpufreq_readl(struct hx_cpufreq_data *hc, unsigned reg)
{
    return readl(hx_cpufreq_reg(hc, reg));
}
static u64 hx_cpufreq_readq(struct hx_cpufreq_data *hc, unsigned reg)
{
    return readq(hx_cpufreq_reg(hc, reg));
}
static void hx_cpufreq_writel(struct hx_cpufreq_data *hc, unsigned reg, u32 val)
{
    writel(val, hx_cpufreq_reg(hc, reg));
}
static void hx_cpufreq_writeq(struct hx_cpufreq_data *hc, unsigned reg, u64 val)
{
    writeq(val, hx_cpufreq_reg(hc, reg));
}
static void hx_cpufreq_rmwl(struct hx_cpufreq_data *hc, unsigned reg, u32 clr, u32 set)
{
    void __iomem *ptr = hx_cpufreq_reg(hc, reg);
    writel((readl(ptr) & ~clr) | set, ptr);
}
static void hx_cpufreq_rmwq(struct hx_cpufreq_data *hc, unsigned reg, u64 clr, u64 set)
{
    void __iomem *ptr = hx_cpufreq_reg(hc, reg);
    writeq((readq(ptr) & ~clr) | set, ptr);
}

static int hx_cpufreq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
    struct hx_cpufreq_data *hc = policy->driver_data;

    if(index < 2)
        index = 2;
    if(index > hc->cfg->num_states - 1)
        index = hc->cfg->num_states - 1;

    hx_cpufreq_writeq(hc, REG_PSINFO_SET1, (hx_cpufreq_readq(hc, REG_PSINFO_GET1(index)) & 0xFF00000000800000ul) |
                                           (hx_cpufreq_readq(hc, REG_PSINFO_SET1_DFLT)  & ~0xFF00000000800000ul));
    hx_cpufreq_writeq(hc, REG_PSINFO_SET2, (hx_cpufreq_readq(hc, REG_PSINFO_GET2(index)) & 0x000000FF00000000ul) |
                                           (hx_cpufreq_readq(hc, REG_PSINFO_SET2_DFLT)  & ~0x000000FF00000000ul));
    hx_cpufreq_rmwq(hc, REG_ACC_CLK_PSTATE, 0xF00Ful, index | (index << 12) | 0x20F0000ul);

    return 0;
}

static void hx_cpufreq_write_dvfm_set(struct hx_cpufreq_data *hc, unsigned rlo, unsigned rhi, const u8 *set, unsigned len)
{
    u64 val[2] = { 0 }, ent;
    unsigned i;
    for(i=0; i<len; i++) {
        ent = set ? set[i] : 0xFF;
        val[i >> 3] |= ent << ((i & 7) << 3);
    }
    hx_cpufreq_writeq(hc, rlo, val[0]);
    hx_cpufreq_writeq(hc, rhi, val[1]);
}

static int hx_cpufreq_init(struct cpufreq_policy *policy)
{
    struct hx_cpufreq_data *hc = cpufreq_get_driver_data();

    hx_cpufreq_write_dvfm_set(hc, REG_DVFM_VSET_LO, REG_DVFM_VSET_HI, hc->cfg->dvfm_vset, hc->cfg->num_states);
    hx_cpufreq_write_dvfm_set(hc, REG_DVFM_SET1_LO, REG_DVFM_SET1_HI, hc->cfg->dvfm_set1, hc->cfg->num_states);
    hx_cpufreq_write_dvfm_set(hc, REG_DVFM_SET2_LO, REG_DVFM_SET2_HI, hc->cfg->dvfm_set2, hc->cfg->num_states);
    hx_cpufreq_write_dvfm_set(hc, REG_DVFM_MASK_LO, REG_DVFM_MASK_HI, NULL, hc->cfg->num_states);
    hx_cpufreq_writeq(hc, REG_ACC_CLK_ENABLE1, 0x15);
    hx_cpufreq_rmwl(hc, REG_PSTATE_CTRL, 0x1F00, 0);
    hx_cpufreq_writel(hc, REG_PSTATE_SET1, hx_cpufreq_readq(hc, REG_PSINFO_GET1(2)) >> 56);
    hx_cpufreq_writel(hc, REG_PSTATE_SET2, (hx_cpufreq_readq(hc, REG_PSINFO_GET2(2)) >> 32) & 0xFF);
    hx_cpufreq_writel(hc, REG_PSTATE_SET3, (hx_cpufreq_readl(hc, REG_MCCPS_GET(hx_cpufreq_readl(hc, REG_MCCPS_IDX) & 15)) >> 12) & 0xFF);
    hx_cpufreq_rmwq(hc, REG_ACC_CLK_PSTATE, 0x400000, 0);
    hx_cpufreq_rmwq(hc, REG_ACC_CLK_ENABLE2, 0, 1);
    hx_cpufreq_rmwl(hc, REG_CPUPM_CPU0, 0, 1);
    hx_cpufreq_rmwl(hc, REG_CPUPM_CPU1, 0, 1);
    hx_cpufreq_rmwq(hc, REG_ACC_CLK_ENABLE3, 0, 1ul << 63);
    hx_cpufreq_writel(hc, REG_ACC_CLK_L2FEAT, 0xFFFFFFFF);
    hx_cpufreq_rmwq(hc, REG_ACC_DBG_CTRL, 0, 0x7C000);
    hx_cpufreq_rmwl(hc, REG_PSTATE_DISABLE, 0x80000000u, 0);

    udelay(10000);

    hx_cpufreq_rmwq(hc, REG_PSINFO_SET1_DFLT, 0xFFul << 56, hx_cpufreq_readq(hc, REG_PSINFO_GET1(2)) & (0xFFul << 56));
    hx_cpufreq_rmwq(hc, REG_PSINFO_SET2_DFLT, 0xFFul << 32, hx_cpufreq_readq(hc, REG_PSINFO_GET2(2)) & (0xFFul << 32));
    hx_cpufreq_rmwq(hc, REG_ACC_CLK_PSTATE, 0, 0x60002000);

    udelay(10000);

    cpumask_copy(policy->cpus, cpu_online_mask);
    policy->driver_data = hc;
    policy->freq_table = hc->cfg->cpufreq_table;
    policy->cpuinfo.transition_latency = 100;
    policy->cur = hc->cfg->cpufreq_table[2].frequency;

    return 0;
}

static struct cpufreq_driver hx_cpufreq_driver = {
    .name = "hx-pmgr",
    .flags = CPUFREQ_STICKY | CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
    .verify = cpufreq_generic_frequency_table_verify,
    .target_index = hx_cpufreq_set_target,
    .init = hx_cpufreq_init,
    .attr = cpufreq_generic_attr,
};

static int hx_cpufreq_probe(struct platform_device *pdev)
{
    struct hx_cpufreq_data *hc;
    struct resource *res;
    unsigned int i = 0, err;
    const char *mode = NULL;

    hc = devm_kzalloc(&pdev->dev, sizeof(*hc), GFP_KERNEL);
    if(!hc)
        return -ENOMEM;

    for(i=0; i<NUM_BASES; i++) {
        res = platform_get_resource(pdev, IORESOURCE_MEM, i);
        hc->base[i] = devm_ioremap_resource(&pdev->dev, res);
        if(IS_ERR(hc->base[i])) {
            err = PTR_ERR(hc->base[i]);
            return err;
        }
    }

    hc->cfg = &hx_cpufreq_config_fast;
    if(!of_property_read_string(pdev->dev.of_node, "mode", &mode)) {
        if(!strcmp(mode, "slow"))
            hc->cfg = &hx_cpufreq_config_slow;
    }

    hx_cpufreq_driver.driver_data = hc;

    err = cpufreq_register_driver(&hx_cpufreq_driver);
    if(err)
        return err;

    dev_warn(&pdev->dev, "registered cpufreq driver.\n");
    return 0;
}

static const struct of_device_id hx_cpufreq_of_match[] = {
    { .compatible = "hx,pmgr-cpufreq" },
    { }
};
MODULE_DEVICE_TABLE(of, hx_cpufreq_of_match);

static struct platform_driver hx_cpufreq_platform_driver = {
    .driver = {
        .name = "hx-cpufreq",
        .of_match_table = hx_cpufreq_of_match,
    },
    .probe = hx_cpufreq_probe,
};
module_platform_driver(hx_cpufreq_platform_driver);

MODULE_DESCRIPTION("Hx SoC cpufreq driver");
MODULE_LICENSE("GPL v2");
