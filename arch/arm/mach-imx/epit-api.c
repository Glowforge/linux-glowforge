/**
 * Low-level i.MX6 EPIT API.
 * Copyright (C) 2015-2018 Glowforge, Inc. <opensource@glowforge.com>
 */

#include <linux/platform_data/epit-imx.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>

/* Register offsets */
#define EPITCR              0x00  /* control register */
#define EPITSR              0x04  /* status register */
#define EPITLR              0x08  /* load register */
#define EPITCMPR            0x0c  /* compare register */
#define EPITCNR             0x10  /* counter register */

/* Control register bits */
#define EPITCR_EN                 (1 << 0)
#define EPITCR_ENMOD              (1 << 1)
#define EPITCR_OCIEN              (1 << 2)
#define EPITCR_RLD                (1 << 3)
#define EPITCR_PRESC(x)           (((x) & 0xfff) << 4)
#define EPITCR_SWR                (1 << 16)
#define EPITCR_IOVW               (1 << 17)
#define EPITCR_DBGEN              (1 << 18)
#define EPITCR_WAITEN             (1 << 19)
#define EPITCR_RES                (1 << 20)
#define EPITCR_STOPEN             (1 << 21)
#define EPITCR_OM_DISCON          (0 << 22)
#define EPITCR_OM_TOGGLE          (1 << 22)
#define EPITCR_OM_CLEAR           (2 << 22)
#define EPITCR_OM_SET             (3 << 22)
#define EPITCR_CLKSRC_OFF         (0 << 24)
#define EPITCR_CLKSRC_PERIPHERAL  (1 << 24)
#define EPITCR_CLKSRC_REF_HIGH    (2 << 24)
#define EPITCR_CLKSRC_REF_LOW     (3 << 24)

/* Status register bits */
#define EPITSR_OCIF               (1 << 0)

struct epit {
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	u32 base_phys;
	int irq;
	int sdma_event;
	epit_cb callback;
	void *cb_arg;
};

static irqreturn_t epit_irq_handler_freerunning(int irq, void *dev_id)
{
	struct epit *epit = dev_id;
	/* clear previous interrupt status */
	__raw_writel(EPITSR_OCIF, epit->base+EPITSR);
	/* call the callback */
	epit->callback(epit->cb_arg);
	return IRQ_HANDLED;
}

static irqreturn_t epit_irq_handler_oneshot(int irq, void *dev_id)
{
	struct epit *epit = dev_id;
	u32 val;

	/* clear previous interrupt status */
	__raw_writel(EPITSR_OCIF, epit->base+EPITSR);
	/* stop counting */
	val = __raw_readl(epit->base+EPITCR);
	val &= ~EPITCR_EN;
	__raw_writel(val, epit->base+EPITCR);
	/* call the callback */
	epit->callback(epit->cb_arg);
	return IRQ_HANDLED;
}

static int epit_init(struct epit *epit, epit_cb callback,
	void *cb_arg, irqreturn_t (*irq_handler)(int, void *))
{
	u32 val;
	int ret = 0;

	/* zero out control register */
	__raw_writel(0, epit->base+EPITCR);
	/* clear previous interrupt status */
	__raw_writel(EPITSR_OCIF, epit->base+EPITSR);
	/* set counter reset value */
	__raw_writel(0, epit->base+EPITLR);
	/* zero out compare register */
	__raw_writel(0, epit->base+EPITCMPR);
	/* set mode and clock source; enable interrupt */
	/* peripheral clock is 66MHz */
	val = EPITCR_CLKSRC_PERIPHERAL|EPITCR_WAITEN|EPITCR_RLD|EPITCR_OCIEN
		|EPITCR_ENMOD;
	__raw_writel(val, epit->base+EPITCR);
	/* set up callback, register an IRQ handler if necessary */
	epit->callback = callback;
	epit->cb_arg = cb_arg;
	if (callback) {
		ret = request_irq(epit->irq, irq_handler,
			IRQF_TIMER|IRQF_IRQPOLL, "epit", epit);
		if (ret) { epit->callback = NULL; }
	}
	return ret;
}

int epit_init_freerunning(struct epit *epit, epit_cb callback, void *cb_arg)
{
	return epit_init(epit, callback, cb_arg, epit_irq_handler_freerunning);
}
EXPORT_SYMBOL(epit_init_freerunning);

int epit_init_oneshot(struct epit *epit, epit_cb callback, void *cb_arg)
{
	return epit_init(epit, callback, cb_arg, epit_irq_handler_oneshot);
}
EXPORT_SYMBOL(epit_init_oneshot);

u32 epit_hz_to_divisor(struct epit *epit, u32 hz)
{
	return (hz) ? epit->rate/hz : 0;
}
EXPORT_SYMBOL(epit_hz_to_divisor);

void epit_start(struct epit *epit, u32 divisor)
{
	u32 val;
	/* clear previous interrupt status */
	__raw_writel(EPITSR_OCIF, epit->base+EPITSR);
	/* set divisor */
	__raw_writel(divisor, epit->base+EPITLR);
	/* start counting */
	val = __raw_readl(epit->base+EPITCR);
	val |= EPITCR_EN;
	__raw_writel(val, epit->base+EPITCR);
}
EXPORT_SYMBOL(epit_start);

void epit_start_hz(struct epit *epit, u32 hz)
{
	epit_start(epit, epit_hz_to_divisor(epit, hz));
}
EXPORT_SYMBOL(epit_start_hz);

void epit_stop(struct epit *epit)
{
	/* stop counting */
	u32 val = __raw_readl(epit->base+EPITCR);
	val &= ~EPITCR_EN;
	__raw_writel(val, epit->base+EPITCR);
}
EXPORT_SYMBOL(epit_stop);

void epit_set_divisor(struct epit *epit, u32 divisor)
{
	__raw_writel(divisor, epit->base+EPITLR);
}
EXPORT_SYMBOL(epit_set_divisor);

void epit_set_hz(struct epit *epit, u32 hz)
{
	epit_set_divisor(epit, epit_hz_to_divisor(epit, hz));
}
EXPORT_SYMBOL(epit_set_hz);

u32 epit_count(struct epit *epit)
{
	return __raw_readl(epit->base+EPITCNR);
}
EXPORT_SYMBOL(epit_count);

int epit_is_running(struct epit *epit)
{
	return (__raw_readl(epit->base+EPITCR) & EPITCR_EN) != 0;
}
EXPORT_SYMBOL(epit_is_running);

int epit_irq(struct epit *epit)
{
	return epit->irq;
}
EXPORT_SYMBOL(epit_irq);

int epit_sdma_event(struct epit *epit)
{
	return epit->sdma_event;
}
EXPORT_SYMBOL(epit_sdma_event);

u32 epit_status_register_address(struct epit *epit)
{
	return epit->base_phys+EPITSR;
}
EXPORT_SYMBOL(epit_status_register_address);

struct epit *epit_get(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	struct epit *epit = NULL;
	if (!pdev) { return NULL; }
	epit = platform_get_drvdata(pdev);
	return epit;
}
EXPORT_SYMBOL(epit_get);

static int epit_probe(struct platform_device *pdev)
{
	struct epit *epit;
	struct device_node *np = pdev->dev.of_node;

	/* allocate driver data */
	epit = devm_kzalloc(&pdev->dev, sizeof(*epit), GFP_KERNEL);
	if (!epit) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, epit);

	/* map registers */
	epit->base = devm_ioremap_resource(&pdev->dev, pdev->resource);
	if (unlikely(IS_ERR(epit->base))) {
		return PTR_ERR(epit->base);
	}
	epit->base_phys = pdev->resource->start;

	/* get irq */
	epit->irq = platform_get_irq(pdev, 0);
	if (unlikely(epit->irq < 0)) {
		return -EINVAL;
	}

	/* get clock */
	epit->clk = of_clk_get(np, 0);
	if (IS_ERR(epit->clk)) {
		return -EINVAL;
	}
	clk_prepare_enable(epit->clk);

	/* get SDMA event */
	if (unlikely(of_property_read_u32(np, "sdma-event",
		&epit->sdma_event) != 0)) {
		epit->sdma_event = -1;
	}
	/* event may need to be explicitly selected */
	if (of_find_property(np, "sdma-event-select", NULL)) {
		struct of_phandle_args args;
		if (of_parse_phandle_with_fixed_args(np,
		  "sdma-event-select", 3, 0, &args) == 0) {
			struct regmap *gpr = syscon_node_to_regmap(args.np);
			if (!IS_ERR(gpr)) {
				regmap_update_bits(gpr,
					args.args[0],
					args.args[1],
					args.args[2]);
			} else {
				dev_err(&pdev->dev, "regmap not found, "
					"cannot enable SDMA event\n");
			}
			of_node_put(args.np);
		} else {
			dev_err(&pdev->dev, "invalid sdma-event-select params");
		}
	}

	/* zero out control register and clear previous interrupt status */
	__raw_writel(0, epit->base+EPITCR);
	__raw_writel(EPITSR_OCIF, epit->base+EPITSR);

	epit->rate = clk_get_rate(epit->clk);
	dev_info(&pdev->dev, "irq=%d, rate=%ld, sdma_event=%d\n",
		epit->irq, epit->rate, epit->sdma_event);

	return 0;
}

static int epit_remove(struct platform_device *pdev)
{
	/* all resources released by devm */
	struct epit *epit = platform_get_drvdata(pdev);
	epit_stop(epit);
	if (epit->callback) {
		free_irq(epit->irq, epit);
	}
	clk_disable_unprepare(epit->clk);
	return 0;
}

static const struct of_device_id epit_dt_ids[] = {
	{ .compatible = "fsl,imx6qdl-epit" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, epit_dt_ids);

static struct platform_driver epit_driver = {
	.driver = {
		.name = "epit",
		.owner = THIS_MODULE,
		.of_match_table = epit_dt_ids
	},
	.probe = epit_probe,
	.remove = epit_remove
};
module_platform_driver(epit_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Glowforge, Inc. <opensource@glowforge.com>");

