/*
 * MIPI DSI driver for Hx SoCs
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/gpio/consumer.h>

struct hx_mipi_dsi {
    struct device *dev;
    void __iomem *base, *disp_base, *pmgr_base;
    struct clk *clk;
    struct regulator_dev *regu;
    struct regulator_desc desc;
    struct regulator_config config;
    struct regulator_init_data init_data;
    struct gpio_desc *gpiod_reset;
    int was_enabled, current_state;
    struct {
        u32 pkt_size;
        u32 hsa_time;
        u32 hbp_time;
        u32 hline_time;
        u32 vsa_lines;
        u32 vbp_lines;
        u32 vactive_lines;
        u32 vfp_lines;
        u32 clkmgr_cfg;
        u32 phy_if_cfg;
        u32 vid_if_cfg[5];
    } setup;
};

#define DWM_VERSION                             0x00
#define DWM_PWR_UP                              0x04
#define   DWM_PWR_UP_RESET                      0
#define   DWM_PWR_UP_ON                         1
#define DWM_CLKMGR_CFG                          0x08
#define   DWM_CLKMGR_CFG_TO_CLK_DIV(x)          ((x) << 8)
#define   DWM_CLKMGR_CFG_TX_ESC_CLK_DIV(x)      (x)
#define DWM_DPI_VCID                            0x0c
#define   DWM_DPI_VCID_VID(x)                   (x)
#define DWM_DPI_COLOR_CODING                    0x10
#define   DWM_DPI_COLOR_CODING_EN18_LOOSE       (1 << 8)
#define   DWM_DPI_COLOR_CODING_16BIT_1          0
#define   DWM_DPI_COLOR_CODING_16BIT_2          1
#define   DWM_DPI_COLOR_CODING_16BIT_3          2
#define   DWM_DPI_COLOR_CODING_18BIT_1          3
#define   DWM_DPI_COLOR_CODING_18BIT_2          4
#define   DWM_DPI_COLOR_CODING_24BIT            5
#define DWM_DPI_CFG_POL                         0x14
#define   DWM_DPI_CFG_POL_COLORM_ACTIVE_LOW     (1 << 4)
#define   DWM_DPI_CFG_POL_SHUTD_ACTIVE_LOW      (1 << 3)
#define   DWM_DPI_CFG_POL_HSYNC_ACTIVE_LOW      (1 << 2)
#define   DWM_DPI_CFG_POL_VSYNC_ACTIVE_LOW      (1 << 1)
#define   DWM_DPI_CFG_POL_DATAEN_ACTIVE_LOW     (1 << 0)
#define DWM_DPI_LP_CMD_TIM                      0x18
#define   DWM_DPI_LP_CMD_TIM_OUTVACT(x)         ((x) << 16)
#define   DWM_DPI_LP_CMD_TIM_INVACT(x)          (x)
#define DWM_DBI_CFG                             0x20
#define DWM_DBI_CMDSIZE                         0x28
#define DWM_PKCHDL_CFG                          0x2c
#define   DWM_PKCHDL_CFG_EN_CRC_RX              (1 << 4)
#define   DWM_PKCHDL_CFG_EN_ECC_RX              (1 << 3)
#define   DWM_PKCHDL_CFG_EN_BTA                 (1 << 2)
#define   DWM_PKCHDL_CFG_EN_EOTP_RX             (1 << 1)
#define   DWM_PKCHDL_CFG_EN_EOTP_TX             (1 << 0)
#define DWM_MODE_CFG                            0x34
#define   DWM_MODE_CFG_VIDEO                    0
#define   DWM_MODE_CFG_COMMAND                  1
#define DWM_VID_MODE_CFG                        0x38
#define   DWM_VID_MODE_CFG_FRM_BTA_ACK          (1 << 14)
#define   DWM_VID_MODE_CFG_EN_LP_MASK           (63 << 8)
#define   DWM_VID_MODE_CFG_EN_LP(x)             ((x) << 8)
#define   DWM_VID_MODE_CFG_NON_BRST_SYNC_PULSE  0
#define   DWM_VID_MODE_CFG_NON_BRST_SYNC_EVENT  1
#define   DWM_VID_MODE_CFG_BRST_SYNC_PULSE_A    2
#define   DWM_VID_MODE_CFG_BRST_SYNC_PULSE_B    3
#define DWM_VID_PKT_SIZE                        0x3c
#define   DWM_VID_PKT_MAX_SIZE                  0x3fff
#define DWM_VID_HSA_TIME                        0x48
#define DWM_VID_HBP_TIME                        0x4c
#define DWM_VID_HLINE_TIME                      0x50
#define DWM_VID_VSA_LINES                       0x54
#define DWM_VID_VBP_LINES                       0x58
#define DWM_VID_VFP_LINES                       0x5c
#define DWM_VID_VACTIVE_LINES                   0x60
#define DWM_CMD_MODE_CFG                        0x68
#define   DWM_CMD_MODE_CFG_MAX_RD_PKT_SIZE_LP   (1 << 24)
#define   DWM_CMD_MODE_CFG_DCS_LW_TX_LP         (1 << 19)
#define   DWM_CMD_MODE_CFG_DCS_SR_0P_TX_LP      (1 << 18)
#define   DWM_CMD_MODE_CFG_DCS_SW_1P_TX_LP      (1 << 17)
#define   DWM_CMD_MODE_CFG_DCS_SW_0P_TX_LP      (1 << 16)
#define   DWM_CMD_MODE_CFG_GEN_LW_TX_LP         (1 << 14)
#define   DWM_CMD_MODE_CFG_GEN_SR_2P_TX_LP      (1 << 13)
#define   DWM_CMD_MODE_CFG_GEN_SR_1P_TX_LP      (1 << 12)
#define   DWM_CMD_MODE_CFG_GEN_SR_0P_TX_LP      (1 << 11)
#define   DWM_CMD_MODE_CFG_GEN_SW_2P_TX_LP      (1 << 10)
#define   DWM_CMD_MODE_CFG_GEN_SW_1P_TX_LP      (1 << 9)
#define   DWM_CMD_MODE_CFG_GEN_SW_0P_TX_LP      (1 << 8)
#define   DWM_CMD_MODE_CFG_EN_ACK_RQST          (1 << 1)
#define   DWM_CMD_MODE_CFG_EN_TEAR_FX           (1 << 0)
#define DWM_GEN_HDR                             0x6c
#define   DWM_GEN_HDR_HDATA(x)                  ((x) << 8)
#define   DWM_GEN_HDR_HTYPE(x)                  (x)
#define DWM_GEN_PLD_DATA                        0x70
#define DWM_CMD_PKT_STATUS                      0x74
#define   DWM_CMD_PKT_STATUS_CMD_EMPTY          (1 << 0)
#define   DWM_CMD_PKT_STATUS_CMD_FULL           (1 << 1)
#define   DWM_CMD_PKT_STATUS_PLD_W_EMPTY        (1 << 2)
#define   DWM_CMD_PKT_STATUS_PLD_W_FULL         (1 << 3)
#define   DWM_CMD_PKT_STATUS_PLD_R_EMPTY        (1 << 4)
#define   DWM_CMD_PKT_STATUS_PLD_R_FULL         (1 << 5)
#define   DWM_CMD_PKT_STATUS_RD_CMD_BUSY        (1 << 6)
#define DWM_TO_CNT_CFG                          0x78
#define   DWM_TO_CNT_CFG_HSTX(x)                ((x) << 16)
#define   DWM_TO_CNT_CFG_LPRX(x)                (x)
#define DWM_HS_RD_TO_CNT                        0x7c
#define DWM_LP_RD_TO_CNT                        0x80
#define DWM_HS_WR_TO_CNT                        0x84
#define DWM_LP_WR_TO_CNT                        0x88
#define DWM_BTA_TO_CNT                          0x8c
#define DWM_SDF_3D                              0x90
#define DWM_LPCLK_CTRL                          0x94
#define   DWM_LPCLK_CTRL_AUTO_CLKLANE           (1 << 1)
#define   DWM_LPCLK_CTRL_PHY_TXREQCLKHS         (1 << 0)
#define DWM_PHY_TMR_LPCLK_CFG                   0x98
#define   DWM_PHY_TMR_LPCLK_CFG_HS2LP(x)        ((x) << 16)
#define   DWM_PHY_TMR_LPCLK_CFG_LP2HS(x)        (x)
#define DWM_PHY_TMR_CFG                         0x9c
#define   DWM_PHY_TMR_CFG_HS2LP(x)              ((x) << 24)
#define   DWM_PHY_TMR_CFG_LP2HS(x)              ((x) << 16)
#define   DWM_PHY_TMR_CFG_MAX_RD(x)             (x)
#define DWM_PHY_RSTZ                            0xa0
#define   DWM_PHY_RSTZ_ENFORCEPLL               (1 << 3)
#define   DWM_PHY_RSTZ_ENABLECLK                (1 << 2)
#define   DWM_PHY_RSTZ_UNRSTZ                   (1 << 1)
#define   DWM_PHY_RSTZ_UNSHUTDOWNZ              (1 << 0)
#define DWM_PHY_IF_CFG                          0xa4
#define   DWM_PHY_IF_CFG_N_LANES(x)             ((x) - 1)
#define   DWM_PHY_IF_CFG_PHY_STOP_WAIT(x)       (x << 8)
#define DWM_PHY_STATUS                          0xb0
#define   DWM_PHY_STATUS_LOCK                   (1 << 0)
#define   DWM_PHY_STATUS_STOP_STATE_CLK_LANE    (1 << 2)
#define   DWM_PHY_STATUS_STOP_STATE_LANE_0      (1 << 4)
#define   DWM_PHY_STATUS_STOP_STATE_LANE_1      (1 << 7)
#define   DWM_PHY_STATUS_STOP_STATE_LANE_2      (1 << 9)
#define   DWM_PHY_STATUS_STOP_STATE_LANE_3      (1 << 11)
#define DWM_PHY_TST_CTRL0                       0xb4
#define   DWM_PHY_TST_CTRL0_TESTCLK             (1 << 1)
#define   DWM_PHY_TST_CTRL0_TESTCLR             (1 << 0)
#define DWM_PHY_TST_CTRL1                       0xb8
#define   DWM_PHY_TST_CTRL1_TESTEN              (1 << 16)
#define   DWM_PHY_TST_CTRL1_TESTDOUT(n)         (((n) & 0xff) << 8)
#define   DWM_PHY_TST_CTRL1_TESTDOUT_MASK       (0xff << 8)
#define   DWM_PHY_TST_CTRL1_TESTDIN(n)          (((n) & 0xff) << 0)
#define   DWM_PHY_TST_CTRL1_TESTDIN_MASK        (0xff << 0)
#define DWM_INT_ST0                             0xbc
#define DWM_INT_ST1                             0xc0
#define DWM_INT_MSK0                            0xc4
#define DWM_INT_MSK1                            0xc8
#define DWM_VID_IF_CFG0                         0x80028
#define DWM_VID_IF_CFG1                         0x8002c
#define DWM_VID_IF_CFG2                         0x80030
#define DWM_VID_IF_CFG3                         0x80034
#define DWM_VID_IF_CFG4                         0x80038

#define DTYPE_DCS_WRITE                         0x05 /* short write, 0 parameter */
#define DTYPE_DCS_WRITE1                        0x15 /* short write, 1 parameter */
#define DTYPE_DCS_READ                          0x06 /* read */
#define DTYPE_DCS_LWRITE                        0x39 /* long write */
#define DTYPE_DSC_LWRITE                        0x0A /* dsc dsi1.2 vase3x long write */
#define DTYPE_GEN_WRITE                         0x03 /* short write, 0 parameter */
#define DTYPE_GEN_WRITE1                        0x13 /* short write, 1 parameter */
#define DTYPE_GEN_WRITE2                        0x23 /* short write, 2 parameter */
#define DTYPE_GEN_LWRITE                        0x29 /* long write */
#define DTYPE_GEN_READ                          0x04 /* long read, 0 parameter */
#define DTYPE_GEN_READ1                         0x14 /* long read, 1 parameter */
#define DTYPE_GEN_READ2                         0x24 /* long read, 2 parameter */
#define DTYPE_TEAR_ON                           0x35 /* set tear on */
#define DTYPE_MAX_PKTSIZE                       0x37 /* set max packet size */
#define DTYPE_NULL_PKT                          0x09 /* null packet, no data */
#define DTYPE_BLANK_PKT                         0x19 /* blankiing packet, no data */

#define DISP_STATE                              0x08
#define  DISP_STATE_RUN_CMD                     (1u << 31)
#define  DISP_STATE_RUN_STAT                    (1u << 30)

static void hx_mipi_dsi_writel(struct hx_mipi_dsi *md, unsigned reg, u32 val)
{
    writel(val, md->base + reg);
}

static u32 hx_mipi_dsi_readl(struct hx_mipi_dsi *md, unsigned reg)
{
    return readl(md->base + reg);
}

static void hx_mipi_dsi_rmwl(struct hx_mipi_dsi *md, unsigned reg, u32 clr, u32 set)
{
    hx_mipi_dsi_writel(md, reg, (hx_mipi_dsi_readl(md, reg) & ~clr) | set);
}

static int hx_mipi_dsi_disp_run(struct hx_mipi_dsi *md, unsigned state)
{
    unsigned timeout = 100;
    u32 val;
    writel((readl(md->disp_base + DISP_STATE) & ~DISP_STATE_RUN_CMD) | (state ? DISP_STATE_RUN_CMD : 0), md->disp_base + DISP_STATE);
    while(1) {
        val = readl(md->disp_base + DISP_STATE);
        if((val & DISP_STATE_RUN_STAT) == (state ? DISP_STATE_RUN_STAT : 0))
            return 0;
        usleep_range(1000, 2000);
        timeout --;
        if(!timeout) {
            dev_warn(md->dev, "display %sable timed out, 0x%x\n", state ? "en" : "dis", val);
            return -ETIMEDOUT;
        }
    }
}

static int hx_mipi_dsi_wait_cmd(struct hx_mipi_dsi *md)
{
    unsigned timeout = 100;
    u32 val;
    while(1) {
        val = hx_mipi_dsi_readl(md, DWM_CMD_PKT_STATUS);
        if(val & DWM_CMD_PKT_STATUS_CMD_EMPTY)
            return 0;
        usleep_range(1000, 2000);
        timeout --;
        if(!timeout) {
            dev_warn(md->dev, "command timed out, 0x%x\n", val);
            return -ETIMEDOUT;
        }
    }
}

static int hx_mipi_dsi_send_short(struct hx_mipi_dsi *md, unsigned cmd, unsigned data)
{
    hx_mipi_dsi_writel(md, DWM_GEN_HDR, DWM_GEN_HDR_HTYPE(DTYPE_DCS_WRITE) | DWM_GEN_HDR_HDATA(cmd | (data << 8)));
    return hx_mipi_dsi_wait_cmd(md);
}

static void hx_mipi_dsi_reset(struct hx_mipi_dsi *md)
{
    u32 val = readl(md->pmgr_base);
    if(val & 0x10000000u) {
        writel(val & 0xEFFFFCFFu, md->pmgr_base);
        udelay(1);
    }
    writel((val & 0xEFFFFCFFu) | 0x400u, md->pmgr_base);
    udelay(1);
    writel((val & 0xEFFFFCFFu) | 0x80000400u, md->pmgr_base);
    udelay(1);
    writel(val & 0xEFFFFCFFu, md->pmgr_base);
    if(val & 0x10000000u) {
        udelay(1);
        writel(val, md->pmgr_base);
    }
}

static void hx_mipi_dsi_turn_off(struct hx_mipi_dsi *md)
{
    hx_mipi_dsi_send_short(md, 0x28, 0);
    hx_mipi_dsi_send_short(md, 0x10, 0);
    usleep_range(25000, 50000);
    hx_mipi_dsi_disp_run(md, 0);
    gpiod_direction_output(md->gpiod_reset, 0);
    hx_mipi_dsi_rmwl(md, DWM_LPCLK_CTRL, DWM_LPCLK_CTRL_PHY_TXREQCLKHS, 0);
    hx_mipi_dsi_writel(md, DWM_PHY_TST_CTRL0, DWM_PHY_TST_CTRL0_TESTCLR);
    usleep_range(1500, 5000);
    hx_mipi_dsi_writel(md, DWM_PWR_UP, DWM_PWR_UP_RESET);
    usleep_range(1500, 5000);
    hx_mipi_dsi_rmwl(md, DWM_PHY_RSTZ, DWM_PHY_RSTZ_ENFORCEPLL | DWM_PHY_RSTZ_ENABLECLK | DWM_PHY_RSTZ_UNSHUTDOWNZ, 0);
    usleep_range(1500, 5000);
    hx_mipi_dsi_rmwl(md, DWM_PHY_RSTZ, DWM_PHY_RSTZ_UNRSTZ, 0);
    usleep_range(1000, 2000);
    hx_mipi_dsi_reset(md);
    usleep_range(7500, 15000);
}

static int hx_mipi_dsi_wait_reset(struct hx_mipi_dsi *md)
{
    unsigned timeout = 100;
    u32 val;
    while(1) {
        val = hx_mipi_dsi_readl(md, 0x80004);
        if(val & 0x10)
            return 0;
        usleep_range(1000, 2000);
        timeout --;
        if(!timeout) {
            dev_warn(md->dev, "reset wait timed out, 0x%x\n", val);
            return -ETIMEDOUT;
        }
    }
}

static int hx_mipi_dsi_wait_lock(struct hx_mipi_dsi *md, u32 mask)
{
    unsigned timeout = 100;
    u32 val;
    while(1) {
        val = hx_mipi_dsi_readl(md, DWM_PHY_STATUS);
        if((val & mask) == mask)
            return 0;
        usleep_range(1000, 2000);
        timeout --;
        if(!timeout) {
            dev_warn(md->dev, "lock wait timed out, 0x%x/0x%x\n", val, mask);
            return -ETIMEDOUT;
        }
    }
}

static int hx_mipi_dsi_wait_phy_init(struct hx_mipi_dsi *md)
{
    unsigned timeout = 100;
    u32 val;
    while(1) {
        val = hx_mipi_dsi_readl(md, 0x80034);
        if(!(val & 4))
            return 0;
        usleep_range(1000, 2000);
        timeout --;
        if(!timeout) {
            dev_warn(md->dev, "phy init wait timed out, 0x%x\n", val);
            return -ETIMEDOUT;
        }
    }
}

static void hx_mipi_dsi_phy_write(struct hx_mipi_dsi *md, u32 cmd, u32 val)
{
    hx_mipi_dsi_writel(md, DWM_PHY_TST_CTRL0, DWM_PHY_TST_CTRL0_TESTCLK);
    hx_mipi_dsi_writel(md, DWM_PHY_TST_CTRL1, DWM_PHY_TST_CTRL1_TESTDIN(cmd));
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL1, 0, DWM_PHY_TST_CTRL1_TESTEN);
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL0, DWM_PHY_TST_CTRL0_TESTCLK, 0);
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL1, DWM_PHY_TST_CTRL1_TESTEN, 0);
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL1, DWM_PHY_TST_CTRL1_TESTDIN_MASK, 0);
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL1, 0, DWM_PHY_TST_CTRL1_TESTDIN(val));
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL0, 0, DWM_PHY_TST_CTRL0_TESTCLK);
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, DWM_PHY_TST_CTRL0, DWM_PHY_TST_CTRL0_TESTCLK, 0);
}

static void hx_mipi_dsi_turn_on(struct hx_mipi_dsi *md)
{
    unsigned lanes = (md->setup.phy_if_cfg & 3) + 1;

    if(hx_mipi_dsi_readl(md, DWM_PWR_UP) & DWM_PWR_UP_ON)
        return;

    hx_mipi_dsi_writel(md, 0x80034, 0x80000000);
    hx_mipi_dsi_writel(md, DWM_CLKMGR_CFG, md->setup.clkmgr_cfg);
    hx_mipi_dsi_writel(md, DWM_MODE_CFG, DWM_MODE_CFG_COMMAND);
    hx_mipi_dsi_writel(md, DWM_PHY_IF_CFG, md->setup.phy_if_cfg);
    hx_mipi_dsi_writel(md, DWM_PKCHDL_CFG, DWM_PKCHDL_CFG_EN_CRC_RX | DWM_PKCHDL_CFG_EN_ECC_RX | DWM_PKCHDL_CFG_EN_BTA);
    hx_mipi_dsi_writel(md, DWM_DPI_VCID, DWM_DPI_VCID_VID(0));
    hx_mipi_dsi_writel(md, DWM_DPI_COLOR_CODING, DWM_DPI_COLOR_CODING_24BIT);
    hx_mipi_dsi_writel(md, 0x8000c, 5);
    hx_mipi_dsi_writel(md, DWM_DPI_CFG_POL, 0);
    hx_mipi_dsi_writel(md, DWM_VID_MODE_CFG, 0x8000 | DWM_VID_MODE_CFG_EN_LP(60));
    hx_mipi_dsi_rmwl(md, DWM_VID_MODE_CFG, 0, DWM_VID_MODE_CFG_EN_LP(3));
    hx_mipi_dsi_rmwl(md, DWM_VID_MODE_CFG, 0, DWM_VID_MODE_CFG_BRST_SYNC_PULSE_B);
    hx_mipi_dsi_writel(md, DWM_VID_PKT_SIZE, md->setup.pkt_size);
    hx_mipi_dsi_writel(md, DWM_VID_HSA_TIME, md->setup.hsa_time);
    hx_mipi_dsi_writel(md, DWM_VID_HBP_TIME, md->setup.hbp_time);
    hx_mipi_dsi_writel(md, DWM_VID_HLINE_TIME, md->setup.hline_time);
    hx_mipi_dsi_writel(md, DWM_VID_VSA_LINES, md->setup.vsa_lines);
    hx_mipi_dsi_writel(md, DWM_VID_VBP_LINES, md->setup.vbp_lines);
    hx_mipi_dsi_writel(md, DWM_VID_VACTIVE_LINES, md->setup.vactive_lines);
    hx_mipi_dsi_writel(md, DWM_VID_VFP_LINES, md->setup.vfp_lines);
    hx_mipi_dsi_writel(md, DWM_LPCLK_CTRL, 0);
    hx_mipi_dsi_writel(md, DWM_DPI_LP_CMD_TIM, 0);
    hx_mipi_dsi_writel(md, DWM_SDF_3D, 0);
    hx_mipi_dsi_writel(md, 0x108, 0);
    hx_mipi_dsi_writel(md, DWM_PHY_TMR_CFG, DWM_PHY_TMR_CFG_MAX_RD(936) | DWM_PHY_TMR_CFG_LP2HS(60) | DWM_PHY_TMR_CFG_HS2LP(35));
    hx_mipi_dsi_writel(md, DWM_PHY_TMR_LPCLK_CFG, DWM_PHY_TMR_LPCLK_CFG_HS2LP(67) | DWM_PHY_TMR_LPCLK_CFG_LP2HS(159));
    hx_mipi_dsi_writel(md, DWM_CMD_MODE_CFG, DWM_CMD_MODE_CFG_GEN_LW_TX_LP | DWM_CMD_MODE_CFG_GEN_SR_2P_TX_LP | DWM_CMD_MODE_CFG_GEN_SR_1P_TX_LP |
        DWM_CMD_MODE_CFG_GEN_SR_0P_TX_LP | DWM_CMD_MODE_CFG_GEN_SW_2P_TX_LP | DWM_CMD_MODE_CFG_GEN_SW_1P_TX_LP | DWM_CMD_MODE_CFG_GEN_SW_0P_TX_LP);
    hx_mipi_dsi_writel(md, DWM_PHY_TST_CTRL0, 0);
    usleep_range(100, 500);
    hx_mipi_dsi_writel(md, DWM_PHY_TST_CTRL0, 0);
    usleep_range(100, 500);
    hx_mipi_dsi_writel(md, DWM_PHY_TST_CTRL0, 1);
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, 0x80034, 0, 0x10000000);
    hx_mipi_dsi_writel(md, DWM_PWR_UP, DWM_PWR_UP_ON);
    hx_mipi_dsi_writel(md, DWM_PHY_RSTZ, DWM_PHY_RSTZ_ENABLECLK);
    hx_mipi_dsi_rmwl(md, 0x80034, 0, 0x02000000 | (md->setup.vid_if_cfg[3] & 0xFF0));
    hx_mipi_dsi_rmwl(md, 0x80034, 0, 4);
    hx_mipi_dsi_wait_phy_init(md);
    hx_mipi_dsi_writel(md, DWM_VID_IF_CFG4, md->setup.vid_if_cfg[4]);
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, 0x80034, 0, 0x08000000);
    usleep_range(100, 500);
    hx_mipi_dsi_rmwl(md, 0x80034, 0x08000000, 0);
    usleep_range(100, 500);
    hx_mipi_dsi_writel(md, DWM_VID_IF_CFG1, md->setup.vid_if_cfg[1]);
    hx_mipi_dsi_writel(md, DWM_VID_IF_CFG2, md->setup.vid_if_cfg[2]);
    hx_mipi_dsi_rmwl(md, 0x80034, 0x10000000, 0);
    hx_mipi_dsi_writel(md, DWM_VID_IF_CFG0, md->setup.vid_if_cfg[0]);
    hx_mipi_dsi_writel(md, DWM_PHY_RSTZ, DWM_PHY_RSTZ_ENFORCEPLL | DWM_PHY_RSTZ_ENABLECLK | DWM_PHY_RSTZ_UNSHUTDOWNZ);
    usleep_range(100, 500);
    hx_mipi_dsi_writel(md, DWM_PHY_RSTZ, DWM_PHY_RSTZ_ENFORCEPLL | DWM_PHY_RSTZ_ENABLECLK | DWM_PHY_RSTZ_UNRSTZ | DWM_PHY_RSTZ_UNSHUTDOWNZ);
    usleep_range(100, 500);
    hx_mipi_dsi_wait_reset(md);
    hx_mipi_dsi_writel(md, DWM_LP_RD_TO_CNT, 25);
    hx_mipi_dsi_writel(md, DWM_BTA_TO_CNT, 25);
    switch(lanes) {
    case 1:
        hx_mipi_dsi_wait_lock(md, DWM_PHY_STATUS_LOCK | DWM_PHY_STATUS_STOP_STATE_LANE_0);
        break;
    case 2:
        hx_mipi_dsi_wait_lock(md, DWM_PHY_STATUS_LOCK | DWM_PHY_STATUS_STOP_STATE_LANE_0 | DWM_PHY_STATUS_STOP_STATE_LANE_1);
        break;
    case 3:
        hx_mipi_dsi_wait_lock(md, DWM_PHY_STATUS_LOCK | DWM_PHY_STATUS_STOP_STATE_LANE_0 | DWM_PHY_STATUS_STOP_STATE_LANE_1 | DWM_PHY_STATUS_STOP_STATE_LANE_2);
        break;
    case 4:
        hx_mipi_dsi_wait_lock(md, DWM_PHY_STATUS_LOCK | DWM_PHY_STATUS_STOP_STATE_LANE_0 | DWM_PHY_STATUS_STOP_STATE_LANE_1 | DWM_PHY_STATUS_STOP_STATE_LANE_2 | DWM_PHY_STATUS_STOP_STATE_LANE_3);
        break;
    }
    usleep_range(10000, 20000);
    gpiod_direction_output(md->gpiod_reset, 1);
    usleep_range(7500, 10000);
    hx_mipi_dsi_send_short(md, 0x11, 0);
    usleep_range(500, 1500);
    hx_mipi_dsi_phy_write(md, 0x71, (md->setup.vid_if_cfg[0] == 2) ? 0x85 : 0x8D);
    hx_mipi_dsi_phy_write(md, 0x32, 0x80);
    hx_mipi_dsi_phy_write(md, 0x42, 0x80);
    hx_mipi_dsi_phy_write(md, 0x52, 0x80);
    hx_mipi_dsi_phy_write(md, 0x82, 0x80);
    hx_mipi_dsi_phy_write(md, 0x52, 0x80);
    usleep_range(500, 1500);
    hx_mipi_dsi_writel(md, DWM_LPCLK_CTRL, DWM_LPCLK_CTRL_PHY_TXREQCLKHS);
    usleep_range(25000, 30000);
    hx_mipi_dsi_disp_run(md, 1);
    usleep_range(75000, 100000);
    hx_mipi_dsi_send_short(md, 0x29, 0);
    hx_mipi_dsi_writel(md, DWM_MODE_CFG, DWM_MODE_CFG_VIDEO);
}

static int hx_mipi_dsi_enable(struct regulator_dev *rdev)
{
    struct hx_mipi_dsi *md = rdev->reg_data;
    usleep_range(25000, 35000);
    hx_mipi_dsi_turn_on(md);
    md->was_enabled = 1;
    md->current_state = 1;
    return 0;
}

static int hx_mipi_dsi_disable(struct regulator_dev *rdev)
{
    struct hx_mipi_dsi *md = rdev->reg_data;
    hx_mipi_dsi_turn_off(md);
    md->current_state = 0;
    return 0;
}

static int hx_mipi_dsi_is_enabled(struct regulator_dev *rdev)
{
    struct hx_mipi_dsi *md = rdev->reg_data;
    int ret;
    if(!md->was_enabled)
        ret = 0;
    else
        ret = md->current_state;
    return ret;
}

static const struct regulator_ops hx_mipi_dsi_ops = {
    .enable = hx_mipi_dsi_enable,
    .disable = hx_mipi_dsi_disable,
    .is_enabled = hx_mipi_dsi_is_enabled,
};

static int hx_mipi_dsi_probe(struct platform_device *pdev)
{
    struct hx_mipi_dsi *md;
    struct resource *r;
    int ret = 0, i;
    const char *supply_name = NULL;

    md = devm_kzalloc(&pdev->dev, sizeof(struct hx_mipi_dsi), GFP_KERNEL);
    if(!md)
        return -ENOMEM;

    md->dev = &pdev->dev;

    md->clk = devm_clk_get(&pdev->dev, NULL);
    if(IS_ERR(md->clk))
        return PTR_ERR(md->clk);

    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    md->base = devm_ioremap_resource(md->dev, r);
    if(IS_ERR(md->base))
        return PTR_ERR(md->base);

    r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    md->disp_base = devm_ioremap_resource(md->dev, r);
    if(IS_ERR(md->disp_base))
        return PTR_ERR(md->disp_base);

    r = platform_get_resource(pdev, IORESOURCE_MEM, 2);
    md->pmgr_base = devm_ioremap_resource(md->dev, r);
    if(IS_ERR(md->pmgr_base))
        return PTR_ERR(md->pmgr_base);

    if(of_property_read_string(pdev->dev.of_node, "supply-name", &supply_name)) {
        dev_err(md->dev, "LCD power supply must be given\n");
        return -ENODEV;
    }

    md->gpiod_reset = devm_gpiod_get_index(md->dev, "reset", 0, 0);
    if(IS_ERR(md->gpiod_reset)) {
        if(PTR_ERR(md->gpiod_reset) != -EPROBE_DEFER)
            dev_err(md->dev, "failed to get 'reset-gpios': %ld\n", PTR_ERR(md->gpiod_reset));
        return PTR_ERR(md->gpiod_reset);
    }

    dev_set_drvdata(md->dev, md);

    md->current_state = 1;

    ret = clk_prepare_enable(md->clk);
    if(ret < 0)
        return ret;

    md->desc.name = "mipi_dsi";
    md->desc.supply_name = supply_name;
    md->desc.ops = &hx_mipi_dsi_ops;
    md->desc.type = REGULATOR_VOLTAGE;
    md->desc.owner = THIS_MODULE;
    md->desc.enable_time = 1000; /* us */
    md->desc.off_on_delay = 1000; /* us */
    md->init_data.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
    md->config.dev = &pdev->dev;
    md->config.driver_data = md;
    md->config.init_data = &md->init_data;
    md->config.of_node = pdev->dev.of_node;

    md->setup.pkt_size = hx_mipi_dsi_readl(md, DWM_VID_PKT_SIZE);
    md->setup.hsa_time = hx_mipi_dsi_readl(md, DWM_VID_HSA_TIME);
    md->setup.hbp_time = hx_mipi_dsi_readl(md, DWM_VID_HBP_TIME);
    md->setup.hline_time = hx_mipi_dsi_readl(md, DWM_VID_HLINE_TIME);
    md->setup.vsa_lines = hx_mipi_dsi_readl(md, DWM_VID_VSA_LINES);
    md->setup.vbp_lines = hx_mipi_dsi_readl(md, DWM_VID_VBP_LINES);
    md->setup.vactive_lines = hx_mipi_dsi_readl(md, DWM_VID_VACTIVE_LINES);
    md->setup.vfp_lines = hx_mipi_dsi_readl(md, DWM_VID_VFP_LINES);
    md->setup.clkmgr_cfg = hx_mipi_dsi_readl(md, DWM_CLKMGR_CFG);
    md->setup.phy_if_cfg = hx_mipi_dsi_readl(md, DWM_PHY_IF_CFG);
    for(i=0; i<5; i++)
        md->setup.vid_if_cfg[i] = hx_mipi_dsi_readl(md, DWM_VID_IF_CFG0 + 4 * i);

    md->regu = devm_regulator_register(md->dev, &md->desc, &md->config);
    if(IS_ERR(md->regu)) {
        dev_err(&pdev->dev, "failed to register regulator: %ld.", PTR_ERR(md->regu));
        return PTR_ERR(md->regu);
    }

    return 0;
}

static const struct of_device_id hx_mipi_dsi_match[] = {
    { .compatible = "hx,mipi-dsi" },
    { },
};
MODULE_DEVICE_TABLE(of, hx_mipi_dsi_match);

static struct platform_driver hx_mipi_dsi_driver = {
    .probe   = hx_mipi_dsi_probe,
    .driver  = {
        .name  = "hx-mipi-dsi",
        .of_match_table = hx_mipi_dsi_match,
    },
};
module_platform_driver(hx_mipi_dsi_driver);

MODULE_DESCRIPTION("MIPI DSI driver for Hx SoCs");
MODULE_LICENSE("GPL v2");
