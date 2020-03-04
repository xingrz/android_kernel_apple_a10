/*
 * SPI controller driver for Hx SoCs
 *
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on spi-mt7621.c:
 *   Copyright (C) 2011 Sergiy <piratfm@gmail.com>
 *   Copyright (C) 2011-2013 Gabor Juhos <juhosg@openwrt.org>
 *   Copyright (C) 2014-2015 Felix Fietkau <nbd@nbd.name>
 *   Copyright (C) 2007-2008 Marvell Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

#define REG_CLKCFG                      0x00
#define  REG_CLKCFG_ENABLE              0xD
#define REG_CONFIG                      0x04
#define  REG_CONFIG_AUTOTX              (1 << 0)
#define  REG_CONFIG_PIOEN               (1 << 5)
#define  REG_CONFIG_SIOEN               (1 << 6)
#define  REG_CONFIG_IE_RXRDY            (1 << 7)
#define  REG_CONFIG_IE_TXEMPTY          (1 << 8)
#define  REG_CONFIG_16BIT               (1 << 15)
#define  REG_CONFIG_32BIT               (1 << 16)
#define  REG_CONFIG_IE_COMPL            (1 << 21)
#define  REG_CONFIG_SET                 (0x0010401E)
#define REG_STATUS                      0x08
#define  REG_STATUS_RXRDY               (1 << 0)
#define  REG_STATUS_TXEMPTY             (1 << 1)
#define  REG_STATUS_TXFIFO_MASK         (31 << 6)
#define   REG_STATUS_TXFIFO_SHIFT       6
#define  REG_STATUS_RXFIFO_MASK         (31 << 11)
#define   REG_STATUS_RXFIFO_SHIFT       11
#define  REG_STATUS_COMPL               (1 << 22)
#define REG_PIN                         0x0C
#define  REG_PIN_CS                     (1 << 1)
#define REG_TXDATA                      0x10
#define REG_RXDATA                      0x20
#define REG_CLKDIV                      0x30
#define  REG_CLKDIV_MAX                 2047
#define REG_RXCNT                       0x34
#define REG_CLKIDLE                     0x38
#define REG_TXCNT                       0x4C

#define SPI_FIFO                        16
#define TIMEOUT_MS                      1000

struct hx_spi {
    struct spi_controller *master;
    void __iomem *base;
    unsigned int clkfreq;
    unsigned int speed;
    struct clk *clk;
    struct gpio_descs *csgpio;

    spinlock_t lock;
    const unsigned char *tx_buf;
    unsigned char *rx_buf;
    unsigned int tx_compl, rx_compl, len;
    struct completion done;
};

static inline struct hx_spi *spidev_to_hx_spi(struct spi_device *spi)
{
    return spi_controller_get_devdata(spi->master);
}

static void hx_spi_continue_tx(struct hx_spi *spi, unsigned status)
{
    unsigned maxtx = SPI_FIFO - ((status & REG_STATUS_TXFIFO_MASK) >> REG_STATUS_TXFIFO_SHIFT);
    unsigned data;

    while(spi->tx_compl < spi->len && maxtx) {
        if(spi->tx_buf)
            data = spi->tx_buf[spi->tx_compl];
        else
            data = 0x00;
        writel(data, spi->base + REG_TXDATA);
        spi->tx_compl ++;
        maxtx --;
    }

    if(spi->tx_compl < spi->len)
        writel(REG_CONFIG_SET | REG_CONFIG_PIOEN | REG_CONFIG_IE_TXEMPTY | REG_CONFIG_IE_RXRDY, spi->base + REG_CONFIG);
    else
        writel(REG_CONFIG_SET | REG_CONFIG_PIOEN | REG_CONFIG_IE_RXRDY, spi->base + REG_CONFIG);
}

static int hx_spi_continue_rx(struct hx_spi *spi, unsigned status)
{
    unsigned maxrx = (status & REG_STATUS_RXFIFO_MASK) >> REG_STATUS_RXFIFO_SHIFT;
    unsigned data;

    while(spi->rx_compl < spi->len && maxrx) {
        data = readl(spi->base + REG_RXDATA);
        if(spi->rx_buf)
            spi->rx_buf[spi->rx_compl] = data;
        spi->rx_compl ++;
        maxrx --;
    }

    if(spi->rx_compl >= spi->len) {
        writel(REG_CONFIG_SET, spi->base + REG_CONFIG);
        writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY, spi->base + REG_STATUS);
        return 1;
    }
    return 0;
}

static irqreturn_t hx_spi_irq(int irq, void *dev_id)
{
    struct hx_spi *spi = dev_id;
    unsigned long flags;
    unsigned status;

    spin_lock_irqsave(&spi->lock, flags);

    status = readl(spi->base + REG_STATUS);
    writel(status, spi->base + REG_STATUS);

    hx_spi_continue_tx(spi, status);

    if(hx_spi_continue_rx(spi, status))
        complete(&spi->done);

    spin_unlock_irqrestore(&spi->lock, flags);

    return IRQ_HANDLED;
}

static void hx_spi_set_cs(struct spi_device *spid, int enable)
{
    struct hx_spi *spi = spidev_to_hx_spi(spid);
    int cs = spid->chip_select;

    if(!spi->csgpio || cs >= spi->csgpio->ndescs)
        return;

    gpiod_direction_output(spi->csgpio->desc[cs], !enable);
}

static int hx_spi_prepare(struct spi_device *spid, unsigned int speed)
{
    struct hx_spi *spi = spidev_to_hx_spi(spid);
    u32 rate;

    rate = DIV_ROUND_UP(spi->clkfreq, speed);
    if(rate > REG_CLKDIV_MAX + 1)
        return -EINVAL;
    if(rate < 2)
        rate = 2;

    writel(0, spi->base + REG_CLKCFG);
    writel(0, spi->base + REG_PIN);
    writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY, spi->base + REG_STATUS);
    writel(rate, spi->base + REG_CLKDIV);
    writel(2, spi->base + REG_CLKIDLE); /* ? */
    writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY, spi->base + REG_STATUS);
    writel(REG_CONFIG_SET, spi->base + REG_CONFIG);
    writel(REG_CLKCFG_ENABLE, spi->base + REG_CLKCFG);
    readl(spi->base + REG_CONFIG);

    spi->speed = speed;

    return 0;
}

static int hx_spi_transfer_one_message(struct spi_controller *master, struct spi_message *m)
{
    unsigned long timeout = msecs_to_jiffies(TIMEOUT_MS);
    struct hx_spi *spi = spi_controller_get_devdata(master);
    struct spi_device *spid = m->spi;
    unsigned int speed = spid->max_speed_hz;
    struct spi_transfer *t = NULL;
    int status = 0;
    unsigned long flags;

    list_for_each_entry(t, &m->transfers, transfer_list)
        if(t->speed_hz < speed)
            speed = t->speed_hz;

    if(hx_spi_prepare(spid, speed)) {
        status = -EIO;
        goto msg_done;
    }

    hx_spi_set_cs(spid, 1);

    m->actual_length = 0;
    list_for_each_entry(t, &m->transfers, transfer_list) {
        spin_lock_irqsave(&spi->lock, flags);

        reinit_completion(&spi->done);

        spi->len = t->len;
        spi->tx_compl = spi->rx_compl = 0;
        spi->tx_buf = t->tx_buf;
        spi->rx_buf = t->rx_buf;

        writel(t->len, spi->base + REG_RXCNT);
        writel(t->len, spi->base + REG_TXCNT);
        writel(REG_CONFIG_SET | REG_CONFIG_PIOEN, spi->base + REG_CONFIG);

        hx_spi_continue_tx(spi, readl(spi->base + REG_STATUS));

        spin_unlock_irqrestore(&spi->lock, flags);

        timeout = wait_for_completion_timeout(&spi->done, timeout);

        spin_lock_irqsave(&spi->lock, flags);

        writel(REG_CONFIG_SET, spi->base + REG_CONFIG);
        writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY, spi->base + REG_STATUS);

        if(timeout == 0) {
            dev_err(&spid->dev, "transfer timed out with %d/%d remaining.\n", spi->len - spi->tx_compl, spi->len - spi->rx_compl);
            status = -ETIMEDOUT;
        }

        m->actual_length += t->len;

        spin_unlock_irqrestore(&spi->lock, flags);

        if(status)
            break;
    }

    hx_spi_set_cs(spid, 0);

msg_done:
    m->status = status;
    spi_finalize_current_message(master);

    return 0;
}

static int hx_spi_setup(struct spi_device *spid)
{
    struct hx_spi *spi = spidev_to_hx_spi(spid);

    if(!spid->max_speed_hz || spid->max_speed_hz > (spi->clkfreq / 2))
        spid->max_speed_hz = spi->clkfreq / 2;

    if(spid->max_speed_hz < spi->clkfreq / (REG_CLKDIV_MAX + 1)) {
        dev_err(&spid->dev, "setup: requested speed is too low: %d Hz\n", spid->max_speed_hz);
        return -EINVAL;
    }

    return 0;
}

static const struct of_device_id hx_spi_match[] = {
    { .compatible = "hx,spi" },
    {},
};
MODULE_DEVICE_TABLE(of, hx_spi_match);

static int hx_spi_probe(struct platform_device *pdev)
{
    struct spi_controller *master;
    struct hx_spi *spi;
    void __iomem *base;
    struct clk *clk;
    struct gpio_descs *csgpio = NULL;
    int ret, ncs, irq;

    base = devm_platform_ioremap_resource(pdev, 0);
    if(IS_ERR(base))
        return PTR_ERR(base);

    clk = devm_clk_get(&pdev->dev, NULL);
    if(IS_ERR(clk)) {
        dev_err(&pdev->dev, "unable to get clock: %ld.\n", PTR_ERR(clk));
        return PTR_ERR(clk);
    }

    ncs = gpiod_count(&pdev->dev, "cs");
    if(ncs > 0) {
        csgpio = devm_gpiod_get_array(&pdev->dev, "cs", 0);
        if(IS_ERR(csgpio)) {
            if(PTR_ERR(csgpio) != -EPROBE_DEFER)
                dev_err(&pdev->dev, "failed to get chip select gpios: %ld\n", PTR_ERR(csgpio));
            return PTR_ERR(csgpio);
        }
    } else
        ncs = 0;

    ret = clk_prepare_enable(clk);
    if(ret)
        return ret;

    master = spi_alloc_master(&pdev->dev, sizeof(*spi));
    if(!master) {
        dev_info(&pdev->dev, "master allocation failed.\n");
        return -ENOMEM;
    }

    master->mode_bits = SPI_LSB_FIRST;
    master->flags = 0;
    master->setup = hx_spi_setup;
    master->transfer_one_message = hx_spi_transfer_one_message;
    master->bits_per_word_mask = SPI_BPW_MASK(8);
    master->dev.of_node = pdev->dev.of_node;
    master->num_chipselect = ncs ? ncs : 1;

    dev_set_drvdata(&pdev->dev, master);

    spi = spi_controller_get_devdata(master);
    spi->base = base;
    spi->clk = clk;
    spi->master = master;
    spi->clkfreq = clk_get_rate(spi->clk);
    spi->csgpio = csgpio;

    dev_err(&pdev->dev, "Hx SPI at %d MHz, %d chip select GPIO%s.\n", spi->clkfreq / 1000000, ncs, ncs == 1 ? "" : "s");

    spin_lock_init(&spi->lock);
    init_completion(&spi->done);

    irq = platform_get_irq(pdev, 0);
    if(irq < 0)
        return irq;

    ret = devm_request_irq(&pdev->dev, irq, hx_spi_irq, 0, dev_name(&pdev->dev), spi);
    if(ret < 0)
        return ret;

    return devm_spi_register_controller(&pdev->dev, master);
}

static int hx_spi_remove(struct platform_device *pdev)
{
    struct spi_controller *master;
    struct hx_spi *spi;

    master = dev_get_drvdata(&pdev->dev);
    spi = spi_controller_get_devdata(master);

    clk_disable_unprepare(spi->clk);

    return 0;
}

MODULE_ALIAS("platform:spi-hx");

static struct platform_driver hx_spi_driver = {
    .driver = {
        .name = "spi-hx",
        .of_match_table = hx_spi_match,
    },
    .probe = hx_spi_probe,
    .remove = hx_spi_remove,
};

module_platform_driver(hx_spi_driver);

MODULE_DESCRIPTION("Hx SoC SPI driver");
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL");
