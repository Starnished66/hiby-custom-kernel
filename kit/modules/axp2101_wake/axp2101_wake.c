// SPDX-License-Identifier: GPL-2.0
//
// axp2101_wake -- wake the SoC from suspend when a charger is plugged in.
// Loaded after the stock axp2101 module. Shares the PMIC IRQ pin, unmasks
// only VBUS insert (REG41h bit 7), and marks that IRQ as a wake source.
// VBUS remove (REG41h bit 6) stays masked so unplugging never wakes.
//
// On the R1 the IRQ net has no external pull-up and floats low. The SoC
// pull on PB23 is a pull-up, so the module enables it before LEVEL_LOW.
// There is no exported reader for that bit: gpio_set_func only writes
// PXPENS/PXPENC. One read of PxPEN remembers the previous state.
// Refuse and exit leave that pull-up on. The pin stays a falling-edge
// interrupt for the stock handler, and a float on that edge is a storm.
// The blob requested the irq with no trigger, and IRQ_TYPE_NONE does not
// clear the INT bit, so the previous trigger cannot be put back.
//
// Parameters (axp2101_wake.sh passes them):
//   pmu_int_gpio   interrupt pin, same name as axp2101 (PB23 on the R1)
//   i2c_bus_num    I2C bus of the PMIC at address 0x34 (0)

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <asm/addrspace.h>

extern const struct kernel_param_ops param_gpio_ops;
extern int gpio_set_func(int gpio, int func);

#define AXP_I2C_ADDR		0x34
#define AXP_IRQ_EN2		0x41
#define AXP_IRQ_STATUS1		0x48
#define AXP_IRQ_STATUS2		0x49
#define AXP_IRQ_STATUS3		0x4a
#define AXP_VBUS_INSERT		BIT(7)
#define AXP_IRQ_FLAGS		(IRQF_SHARED | IRQF_ONESHOT)
#define AXP_WAKE_STORM_LIMIT	20
#define AXP_WAKE_I2C_FAIL_LIMIT	20
/* utils gpio_set_func pull bits. X1600 has one pull switch; on PB23 it is up. */
#define GPIO_PULL_HIZ		0x8000
#define GPIO_PULL		0xa000
#define GPIO_PORT_BYTES		0x100
#define GPIO_PX_PEN		0x80
#define GPIO_IO_PHYS		0x10010000

static int pmu_int_gpio = -1;
static int i2c_bus_num;

module_param_cb(pmu_int_gpio, &param_gpio_ops, &pmu_int_gpio, 0644);
MODULE_PARM_DESC(pmu_int_gpio, "AXP2101 IRQ pin (PB23 on the R1)");
module_param(i2c_bus_num, int, 0644);
MODULE_PARM_DESC(i2c_bus_num, "I2C bus of the AXP2101 (address 0x34)");

/* 1 only after VBUS-insert wake is enabled. Cleared when that wake stops.
 * Readable so userspace can tell a loaded module from an armed one.
 * A refuse that stays resident, and the storm stop, both leave this 0. */
static int armed;
module_param(armed, int, 0444);
MODULE_PARM_DESC(armed, "1 while VBUS-insert wake is armed");

struct axp2101_wake {
	struct i2c_adapter *adap;
	int irq;
	int storm;
	int i2c_fails;
	bool irq_requested;
	bool wake_enabled;
	bool vbus_unmasked;
	bool line_disabled;
	bool pull_set;
	bool pull_was_on;
	bool level_armed;
};

static struct axp2101_wake wake_priv;
static DEFINE_MUTEX(wake_lock);

static int axp_xfer(struct axp2101_wake *w, u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	struct i2c_msg msgs[2];
	int n = 0, ret;

	if (!w->adap)
		return -ENODEV;
	msgs[n].addr = AXP_I2C_ADDR;
	msgs[n].flags = 0;
	msgs[n].len = wlen;
	msgs[n++].buf = wbuf;
	if (rlen) {
		msgs[n].addr = AXP_I2C_ADDR;
		msgs[n].flags = I2C_M_RD;
		msgs[n].len = rlen;
		msgs[n++].buf = rbuf;
	}
	ret = i2c_transfer(w->adap, msgs, n);
	if (ret == n)
		return 0;
	return ret < 0 ? ret : -EIO;
}

static int axp_write(struct axp2101_wake *w, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };

	return axp_xfer(w, buf, 2, NULL, 0);
}

static int axp_read(struct axp2101_wake *w, u8 reg, u8 *val)
{
	return axp_xfer(w, &reg, 1, val, 1);
}

static int axp_set_vbus_insert_irq(struct axp2101_wake *w, bool enable)
{
	u8 en;
	int ret;

	ret = axp_read(w, AXP_IRQ_EN2, &en);
	if (ret)
		return ret;
	if (enable)
		en |= AXP_VBUS_INSERT;
	else
		en &= ~AXP_VBUS_INSERT;
	ret = axp_write(w, AXP_IRQ_EN2, en);
	if (!ret)
		w->vbus_unmasked = enable;
	return ret;
}

static int axp_ack_all(struct axp2101_wake *w)
{
	int ret;

	ret = axp_write(w, AXP_IRQ_STATUS1, 0xff);
	if (ret)
		return ret;
	ret = axp_write(w, AXP_IRQ_STATUS2, 0xff);
	if (ret)
		return ret;
	return axp_write(w, AXP_IRQ_STATUS3, 0xff);
}

static int axp_read_status(struct axp2101_wake *w, u8 st[3])
{
	int ret;

	ret = axp_read(w, AXP_IRQ_STATUS1, &st[0]);
	if (ret)
		return ret;
	ret = axp_read(w, AXP_IRQ_STATUS2, &st[1]);
	if (ret)
		return ret;
	return axp_read(w, AXP_IRQ_STATUS3, &st[2]);
}

/* Write-1-to-clear: write back only the bits that were set. */
static int axp_ack_status(struct axp2101_wake *w, const u8 st[3], bool *did)
{
	static const u8 regs[3] = {
		AXP_IRQ_STATUS1, AXP_IRQ_STATUS2, AXP_IRQ_STATUS3
	};
	int i, ret;

	*did = false;
	for (i = 0; i < 3; i++) {
		if (!st[i])
			continue;
		ret = axp_write(w, regs[i], st[i]);
		if (ret)
			return ret;
		*did = true;
	}
	return 0;
}

static void axp_stop_wake(struct axp2101_wake *w)
{
	mutex_lock(&wake_lock);
	armed = 0;
	if (w->vbus_unmasked)
		axp_set_vbus_insert_irq(w, false);
	if (w->wake_enabled) {
		disable_irq_wake(w->irq);
		w->wake_enabled = false;
	}
	mutex_unlock(&wake_lock);
}

/* PxPEN, port offset 0x80. No getter is exported beside this register. */
static bool axp_pull_enabled(int gpio)
{
	unsigned int port = (unsigned int)gpio / 32;
	unsigned int pin = (unsigned int)gpio % 32;
	void __iomem *pen;

	if (port > 3)
		return false;
	pen = (void __iomem *)CKSEG1ADDR(GPIO_IO_PHYS + port * GPIO_PORT_BYTES +
					 GPIO_PX_PEN);
	return !!(readl(pen) & BIT(pin));
}

static void axp_restore_pull(struct axp2101_wake *w)
{
	if (!w->pull_set || w->pull_was_on)
		return;
	gpio_set_func(pmu_int_gpio, GPIO_PULL_HIZ);
	w->pull_set = false;
}

/* Stop VBUS wake and leave the shared line off until reboot. Logged once. */
static void axp_disable_line(struct axp2101_wake *w, const char *reason)
{
	axp_stop_wake(w);
	disable_irq_nosync(w->irq);
	w->line_disabled = true;
	pr_err("axp2101_wake: %s, disabling irq %d until reboot\n",
	       reason, w->irq);
}

/* Consecutive I2C failures on a level-low line would otherwise refire
 * forever: the storm counter only moves after a successful all-zero read.
 * Give up through axp_disable_line, and log once. */
static irqreturn_t axp_i2c_failed(struct axp2101_wake *w, irqreturn_t how)
{
	msleep(5);
	if (w->line_disabled)
		return how;
	if (++w->i2c_fails < AXP_WAKE_I2C_FAIL_LIMIT)
		return how;
	axp_disable_line(w, "I2C failed " __stringify(AXP_WAKE_I2C_FAIL_LIMIT)
			 " times");
	return how;
}

static irqreturn_t axp2101_wake_thread(int irq, void *data)
{
	struct axp2101_wake *w = data;
	u8 st[3];
	bool acked = false;
	int ret;

	if (w->line_disabled)
		return IRQ_NONE;
	ret = axp_read_status(w, st);
	if (ret)
		return axp_i2c_failed(w, IRQ_NONE);
	if (!st[0] && !st[1] && !st[2]) {
		w->i2c_fails = 0;
		if (++w->storm >= AXP_WAKE_STORM_LIMIT)
			axp_disable_line(w, "status clear for "
					 __stringify(AXP_WAKE_STORM_LIMIT) " IRQs");
		return IRQ_NONE;
	}
	w->storm = 0;
	ret = axp_ack_status(w, st, &acked);
	if (ret)
		return axp_i2c_failed(w, acked ? IRQ_HANDLED : IRQ_NONE);
	w->i2c_fails = 0;
	if (st[1] & AXP_VBUS_INSERT)
		pr_info_ratelimited("axp2101_wake: VBUS inserted\n");
	return IRQ_HANDLED;
}

static int __init axp2101_wake_init(void)
{
	struct axp2101_wake *w = &wake_priv;
	struct gpio_desc *desc;
	int level, ret;

	armed = 0;
	if (pmu_int_gpio < 0) {
		pr_err("axp2101_wake: pmu_int_gpio is required\n");
		return -EINVAL;
	}
	desc = gpio_to_desc((unsigned int)pmu_int_gpio);
	if (!desc) {
		pr_err("axp2101_wake: gpio %d has no descriptor\n", pmu_int_gpio);
		return -ENODEV;
	}
	w->irq = gpiod_to_irq(desc);
	if (w->irq < 0) {
		pr_err("axp2101_wake: gpio %d has no irq (%d)\n", pmu_int_gpio, w->irq);
		return w->irq;
	}
	w->adap = i2c_get_adapter(i2c_bus_num);
	if (!w->adap) {
		pr_err("axp2101_wake: no I2C adapter %d\n", i2c_bus_num);
		return -ENODEV;
	}

	/* Raise the line before LEVEL_LOW. GPIO_PULL does not change the function. */
	w->pull_was_on = axp_pull_enabled(pmu_int_gpio);
	ret = gpio_set_func(pmu_int_gpio, GPIO_PULL);
	if (ret) {
		pr_err("axp2101_wake: gpio_set_func pull failed: %d\n", ret);
		goto err_adap;
	}
	w->pull_set = true;

	ret = request_threaded_irq(w->irq, NULL, axp2101_wake_thread,
				   AXP_IRQ_FLAGS, "axp2101_wake", w);
	if (ret) {
		pr_err("axp2101_wake: request_threaded_irq(%d) failed: %d\n", w->irq, ret);
		goto err_pull;
	}
	w->irq_requested = true;
	disable_irq(w->irq);

	ret = irq_set_irq_type(w->irq, IRQ_TYPE_LEVEL_LOW);
	if (ret) {
		pr_err("axp2101_wake: irq_set_irq_type LEVEL_LOW failed: %d\n", ret);
		goto err_disabled;
	}
	w->level_armed = true;

	ret = axp_ack_all(w);
	if (ret) {
		pr_err("axp2101_wake: failed to ack IRQ status: %d\n", ret);
		goto err_disabled;
	}
	/* Electrical level, the same reading as /sys/class/gpio/gpioN/value. */
	level = gpiod_get_raw_value(desc);
	if (level < 0) {
		pr_err("axp2101_wake: could not read IRQ line (%d), not arming\n", level);
		ret = -EIO;
		goto err_disabled;
	}
	if (!level) {
		pr_err("axp2101_wake: IRQ line low with no PMIC status, not arming\n");
		ret = -EIO;
		goto err_disabled;
	}

	ret = axp_set_vbus_insert_irq(w, true);
	if (ret) {
		pr_err("axp2101_wake: failed to unmask VBUS insert: %d\n", ret);
		goto err_disabled;
	}
	ret = enable_irq_wake(w->irq);
	if (ret) {
		pr_err("axp2101_wake: enable_irq_wake failed: %d\n", ret);
		goto err_disabled;
	}
	w->wake_enabled = true;
	armed = 1;
	enable_irq(w->irq);
	pr_info("axp2101_wake: gpio %d irq %d, VBUS insert wakes\n", pmu_int_gpio, w->irq);
	return 0;

err_disabled:
	axp_stop_wake(w);
	/* A low line produces no edges, so the stock handler cannot storm.
	 * The pull-up stays: see the file header. */
	if (w->level_armed) {
		irq_set_irq_type(w->irq, IRQ_TYPE_EDGE_FALLING);
		w->level_armed = false;
	}
	enable_irq(w->irq);
	free_irq(w->irq, w);
	w->irq_requested = false;
	goto err_adap;
err_pull:
	axp_restore_pull(w);
err_adap:
	i2c_put_adapter(w->adap);
	w->adap = NULL;
	return ret;
}

static void __exit axp2101_wake_exit(void)
{
	struct axp2101_wake *w = &wake_priv;

	axp_stop_wake(w);
	if (w->irq_requested) {
		if (w->level_armed)
			irq_set_irq_type(w->irq, IRQ_TYPE_EDGE_FALLING);
		free_irq(w->irq, w);
		w->irq_requested = false;
		/* line_disabled: disable_irq_nosync left depth > 0. free_irq
		 * does not re-enable a shared line, so irq 72 stays off
		 * until reboot. The stock regmap masks every status bit.
		 * The pull-up stays on. See the file header. */
	}
	if (w->adap) {
		i2c_put_adapter(w->adap);
		w->adap = NULL;
	}
}

module_init(axp2101_wake_init);
module_exit(axp2101_wake_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Wake on AXP2101 VBUS insert");
MODULE_SOFTDEP("pre: utils axp2101");
