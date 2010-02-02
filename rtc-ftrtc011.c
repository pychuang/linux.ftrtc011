/*
 * Real Time Clock interface for Faraday RTC011
 *
 * Copyright (c) 2009 Po-Yu Chuang <ratbert@faraday-tech.com>
 *
 * Copyright (c) 2000 Nils Faerber
 *
 * Based on rtc.c by Paul Gortmaker
 *
 * Original Driver by Nils Faerber <nils@kernelconcepts.de>
 *
 * Modifications from:
 *   CIH <cih@coventive.com>
 *   Nicolas Pitre <nico@cam.org>
 *   Andrew Christian <andrew.christian@hp.com>
 *
 * Converted to the RTC subsystem and Driver Model
 *   by Richard Purdie <rpurdie@rpsys.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <asm/io.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
#include <mach/hardware.h>
#else
#include <asm/hardware.h>
#endif

/******************************************************************************
 * register definitions
 *****************************************************************************/

/*
 * we cannot use CONFIG_FTRTC011_WRITABLE_TIME on A360
 * The design of RTC011 of A360 (1.15) is weird (read broken).
 * After write WSEC, WMIN...etc. and set bit 6 of CR, new values reflect 
 * at next second.
 */
#define	FTRTC011_OFFSET_SEC		0x00
#define	FTRTC011_OFFSET_MIN		0x04
#define	FTRTC011_OFFSET_HOUR		0x08
#define	FTRTC011_OFFSET_DAY		0x0c
#define	FTRTC011_OFFSET_ALARM_SEC	0x10
#define	FTRTC011_OFFSET_ALARM_MIN	0x14
#define	FTRTC011_OFFSET_ALARM_HOUR	0x18

#ifndef	CONFIG_FTRTC011_WRITABLE_TIME
#define	FTRTC011_OFFSET_RECORD		0x1c
#endif

#define	FTRTC011_OFFSET_CR		0x20

#ifdef	CONFIG_FTRTC011_WRITABLE_TIME
#define	FTRTC011_OFFSET_WSEC		0x24	/* ds1.1 - */
#define	FTRTC011_OFFSET_WMIN		0x28	/* ds1.1 - */
#define	FTRTC011_OFFSET_WHOUR		0x2c	/* ds1.1 - */
#define	FTRTC011_OFFSET_WDAY		0x30	/* ds1.1 - */
#endif

#define	FTRTC011_OFFSET_INTR_STATE	0x34	/* ds1.1 - */
#define	FTRTC011_OFFSET_REVISION	0x3c	/* ds1.1 - */
#define FTRTC011_OFFSET_RWSTATUS	0x40
#define FTRTC011_OFFSET_CURRENT		0x44
#define FTRTC011_OFFSET_SLEEPTIME	0x48

/*
 * RTC Control Register
 */
#define	FTRTC011_CR_ENABLE		(1 << 0)
#define	FTRTC011_CR_INTERRUPT_SEC	(1 << 1)	/* enable interrupt per second */
#define	FTRTC011_CR_INTERRUPT_MIN	(1 << 2)	/* enable interrupt per minute */
#define	FTRTC011_CR_INTERRUPT_HR	(1 << 3)	/* enable interrupt per hour */
#define	FTRTC011_CR_INTERRUPT_DAY	(1 << 4)	/* enable interrupt per day */

#define	FTRTC011_CR_ALARM_INTERRUPT	(1 << 5)	/* ds1.1 - */

#ifdef	CONFIG_FTRTC011_WRITABLE_TIME
#define	FTRTC011_CR_COUNTER_LOAD	(1 << 6)	/* ds1.1 - */
#endif
#define FTRTC011_CR_REFRESH		(1 << 7)

/*
 * IntrState
 */
#define	FTRTC011_INTR_STATE_SEC		(1 << 0)
#define	FTRTC011_INTR_STATE_MIN		(1 << 1)
#define	FTRTC011_INTR_STATE_HOUR	(1 << 2)
#define	FTRTC011_INTR_STATE_DAY		(1 << 3)
#define	FTRTC011_INTR_STATE_ALARM	(1 << 4)

/*
 * RtcDivide
 */
#define	FTRTC011_DIVIDE_CYCLE(x)	((x) & 0x7fffffff)	/* ds1.2 - */
#define	FTRTC011_DIVIDE_ENABLE		(1 << 31)		/* ds1.2 - */

/******************************************************************************
 * FTRTC011 private data
 *****************************************************************************/
struct ftrtc011 {
	spinlock_t		lock;
	struct platform_device	*pdev;
	struct rtc_device	*rtc;
	void __iomem		*base;
	int			alarm_irq;
	int			periodic_irq;
	unsigned int		periodic_count;
};

/******************************************************************************
 * internal functions
 *****************************************************************************/
static void ftrtc011_show_tm(struct ftrtc011 *ftrtc011, const char *s, 
	struct rtc_time *tm)
{
	dev_info(&ftrtc011->pdev->dev, "%s: day %d, %d:%d:%d\n",
		s, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

#ifdef	CONFIG_FTRTC011_WRITABLE_TIME
static unsigned long ftrtc011_raw_time_to_user_time(struct ftrtc011 *ftrtc011,
		unsigned long time)
{
	return time;
}

static void ftrtc011_user_tm_to_raw_tm(struct ftrtc011 *ftrtc011,
	struct rtc_time *user_tm, struct rtc_time *raw_tm)
{
	raw_tm->tm_sec		= user_tm->tm_sec;
	raw_tm->tm_min		= user_tm->tm_min;
	raw_tm->tm_hour		= user_tm->tm_hour;
	raw_tm->tm_mday		= user_tm->tm_mday;
	raw_tm->tm_mon		= user_tm->tm_mon;
	raw_tm->tm_year		= user_tm->tm_year;
	raw_tm->tm_wday		= user_tm->tm_wday;
	raw_tm->tm_yday		= user_tm->tm_yday;
	raw_tm->tm_isdst	= user_tm->tm_isdst;
}
#else	/* !CONFIG_FTRTC011_WRITABLE_TIME */
static unsigned long ftrtc011_raw_time_to_user_time(struct ftrtc011 *ftrtc011,
		unsigned long time)
{
	return time + inl(ftrtc011->base + FTRTC011_OFFSET_RECORD);
}

static unsigned long ftrtc011_user_time_to_raw_time(struct ftrtc011 *ftrtc011,
		unsigned long time)
{
	return time - inl(ftrtc011->base + FTRTC011_OFFSET_RECORD);
}

static void ftrtc011_user_tm_to_raw_tm(struct ftrtc011 *ftrtc011,
		struct rtc_time *user_tm, struct rtc_time *raw_tm)
{
	unsigned long	time;

	rtc_tm_to_time(user_tm, &time);
	time = ftrtc011_user_time_to_raw_time(ftrtc011, time);
	rtc_time_to_tm(time, raw_tm);
}
#endif	/* CONFIG_FTRTC011_WRITABLE_TIME */

static unsigned long ftrtc011_get_raw_uptime(struct ftrtc011 *ftrtc011)
{
	unsigned long	sec, sec2, min, hour, day;

	do {
		sec	= inl(ftrtc011->base + FTRTC011_OFFSET_SEC);
		min	= inl(ftrtc011->base + FTRTC011_OFFSET_MIN);
		hour	= inl(ftrtc011->base + FTRTC011_OFFSET_HOUR);
		day	= inl(ftrtc011->base + FTRTC011_OFFSET_DAY);
		sec2	= inl(ftrtc011->base + FTRTC011_OFFSET_SEC);
	} while (sec != sec2);

	dev_info(&ftrtc011->pdev->dev, "raw uptime: %ld day, %ld:%ld:%ld\n",
		day, hour, min, sec);

	return sec + min * 60 + hour * 60 * 60 + day * 24 * 60 * 60;
}

static unsigned long ftrtc011_get_uptime(struct ftrtc011 *ftrtc011)
{
	unsigned long	time = ftrtc011_get_raw_uptime(ftrtc011);

	return ftrtc011_raw_time_to_user_time(ftrtc011, time);
}

#ifdef	CONFIG_FTRTC011_WRITABLE_TIME
static void ftrtc011_set_uptime(struct ftrtc011 *ftrtc011, unsigned long time)
{
	unsigned long	sec, min, hour, day;
	unsigned int	cr;

	sec	= time % 60;
	min	= (time / 60) % 60;
	hour	= (time / 60 / 60) % 24;
	day	= time / 60 / 60 / 24;

	outl(sec,  ftrtc011->base + FTRTC011_OFFSET_WSEC);
	outl(min,  ftrtc011->base + FTRTC011_OFFSET_WMIN);
	outl(hour, ftrtc011->base + FTRTC011_OFFSET_WHOUR);
	outl(day,  ftrtc011->base + FTRTC011_OFFSET_WDAY);

	cr = inl(ftrtc011->base + FTRTC011_OFFSET_CR);
	cr |= FTRTC011_CR_COUNTER_LOAD;
	outl(cr, ftrtc011->base + FTRTC011_OFFSET_CR);
}
#else	/* !CONFIG_FTRTC011_WRITABLE_TIME */
static void ftrtc011_set_uptime(struct ftrtc011 *ftrtc011, unsigned long time)
{
	unsigned long	now = ftrtc011_get_raw_uptime(ftrtc011);

	dev_info(&ftrtc011->pdev->dev, "now: %lx, new: %lx\n", now, time);

	outl(time - now, ftrtc011->base + FTRTC011_OFFSET_RECORD);
}
#endif	/* CONFIG_FTRTC011_WRITABLE_TIME */

static void ftrtc011_get_alarm_tm(struct ftrtc011 *ftrtc011, struct rtc_time *tm)
{
	unsigned long	sec	= inl(ftrtc011->base + FTRTC011_OFFSET_ALARM_SEC);
	unsigned long	min	= inl(ftrtc011->base + FTRTC011_OFFSET_ALARM_MIN);
	unsigned long	hour	= inl(ftrtc011->base + FTRTC011_OFFSET_ALARM_HOUR);
	unsigned long	day	= inl(ftrtc011->base + FTRTC011_OFFSET_DAY);
	unsigned long	time;

	dev_info(&ftrtc011->pdev->dev, "%s() raw day %ld, %ld:%ld:%ld\n",
		__func__, day, hour, min, sec);

	/* raw alarm time */
	time = sec + min * 60 + hour * 60 * 60 + day * 24 * 60 * 60;

	/* converted alarm time */
	time = ftrtc011_raw_time_to_user_time(ftrtc011, time);

	rtc_time_to_tm(time, tm);
}

static void ftrtc011_set_raw_alarm_tm(struct ftrtc011 *ftrtc011, struct rtc_time *tm)
{
	ftrtc011_show_tm(ftrtc011, __func__, tm);

	outl(tm->tm_sec,  ftrtc011->base + FTRTC011_OFFSET_ALARM_SEC);
	outl(tm->tm_min,  ftrtc011->base + FTRTC011_OFFSET_ALARM_MIN);
	outl(tm->tm_hour, ftrtc011->base + FTRTC011_OFFSET_ALARM_HOUR);
}

static void ftrtc011_set_alarm_tm(struct ftrtc011 *ftrtc011, struct rtc_time *tm)
{
	struct rtc_time	raw_tm;

	ftrtc011_show_tm(ftrtc011, __func__, tm);

	ftrtc011_user_tm_to_raw_tm(ftrtc011, tm, &raw_tm);
	ftrtc011_set_raw_alarm_tm(ftrtc011, &raw_tm);
}

static void ftrtc011_enable_alarm(struct ftrtc011 *ftrtc011)
{
	unsigned int	cr;

	dev_info(&ftrtc011->pdev->dev, "%s()\n", __func__);

	cr = inl(ftrtc011->base + FTRTC011_OFFSET_CR);
	cr |= FTRTC011_CR_ALARM_INTERRUPT;
	outl(cr, ftrtc011->base + FTRTC011_OFFSET_CR);
}

static void ftrtc011_disable_alarm(struct ftrtc011 *ftrtc011)
{
	unsigned int	cr;

	dev_info(&ftrtc011->pdev->dev, "%s()\n", __func__);

	cr = inl(ftrtc011->base + FTRTC011_OFFSET_CR);
	cr &= ~FTRTC011_CR_ALARM_INTERRUPT;
	outl(cr, ftrtc011->base + FTRTC011_OFFSET_CR);
}

static void ftrtc011_enable_per_second_interrupt(struct ftrtc011 *ftrtc011)
{
	unsigned int	cr;

	cr = inl(ftrtc011->base + FTRTC011_OFFSET_CR);
	cr |= FTRTC011_CR_INTERRUPT_SEC;
	outl(cr, ftrtc011->base + FTRTC011_OFFSET_CR);
}

static void ftrtc011_disable_per_second_interrupt(struct ftrtc011 *ftrtc011)
{
	unsigned int	cr;

	cr = inl(ftrtc011->base + FTRTC011_OFFSET_CR);
	cr &= ~FTRTC011_CR_INTERRUPT_SEC;
	outl(cr, ftrtc011->base + FTRTC011_OFFSET_CR);
}

static void ftrtc011_enable(struct ftrtc011 *ftrtc011)
{
	/* make sure RTC is enabled and no interrupt will be issued */
	outl(FTRTC011_CR_ENABLE | FTRTC011_CR_REFRESH, ftrtc011->base + FTRTC011_OFFSET_CR);
}

/******************************************************************************
 * alarm interrupt handler
 *****************************************************************************/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19))
static irqreturn_t ftrtc011_ai_handler(int irq, void *dev_id)
#else
static irqreturn_t ftrtc011_ai_handler(int irq, void *dev_id,
		struct pt_regs *regs)
#endif
{
	struct ftrtc011		*ftrtc011 = dev_id;
	struct rtc_device	*rtc = ftrtc011->rtc;
	unsigned long		events = 0;

	dev_info(&ftrtc011->pdev->dev, "%s()\n", __func__);

	spin_lock(&ftrtc011->lock);

	ftrtc011_disable_alarm(ftrtc011);

	events |= RTC_AF | RTC_IRQF;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22))
	rtc_update_irq(rtc, 1, events);
#else
	rtc_update_irq(&rtc->class_dev, 1, events);
#endif

	spin_unlock(&ftrtc011->lock);

	outl(FTRTC011_INTR_STATE_ALARM, ftrtc011->base + FTRTC011_OFFSET_INTR_STATE);
	return IRQ_HANDLED;
}

/******************************************************************************
 * periodic interrupt handler
 *****************************************************************************/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19))
static irqreturn_t ftrtc011_pi_handler(int irq, void *dev_id)
#else
static irqreturn_t ftrtc011_pi_handler(int irq, void *dev_id,
		struct pt_regs *regs)
#endif
{
	struct ftrtc011		*ftrtc011 = dev_id;
	struct rtc_device	*rtc = ftrtc011->rtc;
	unsigned long		events = 0;

	dev_info(&ftrtc011->pdev->dev, "%s()\n", __func__);

	spin_lock(&ftrtc011->lock);

	if (--ftrtc011->periodic_count == 0) {
		events |= RTC_PF | RTC_IRQF;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22))
		rtc_update_irq(rtc, 1, events);
#else
		rtc_update_irq(&rtc->class_dev, 1, events);
#endif
		ftrtc011->periodic_count = rtc->irq_freq;
	}

	spin_unlock(&ftrtc011->lock);

	outl(FTRTC011_INTR_STATE_SEC, ftrtc011->base + FTRTC011_OFFSET_INTR_STATE);
	return IRQ_HANDLED;
}

/******************************************************************************
 * struct rtc_class_ops functions
 *****************************************************************************/
static int ftrtc011_open(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);
	int			ret;

	dev_info(dev, "%s()\n", __func__);

	ret = request_irq(ftrtc011->alarm_irq, ftrtc011_ai_handler, IRQF_DISABLED,
		"rtc ai", ftrtc011);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", ftrtc011->alarm_irq);
		goto fail_ai;
	}

	ret = request_irq(ftrtc011->periodic_irq, ftrtc011_pi_handler, IRQF_DISABLED,
		"rtc pi", ftrtc011);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", ftrtc011->periodic_irq);
		goto fail_pi;
	}

	return 0;

fail_pi:
	free_irq(ftrtc011->alarm_irq, ftrtc011);
fail_ai:
	return ret;
}

static void ftrtc011_release(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);

	dev_info(dev, "%s()\n", __func__);

	spin_lock_irq(&ftrtc011->lock);

	ftrtc011_disable_alarm(ftrtc011);

	spin_unlock_irq(&ftrtc011->lock);

	free_irq(ftrtc011->alarm_irq, ftrtc011);
	free_irq(ftrtc011->periodic_irq, ftrtc011);
}

static int ftrtc011_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
	struct rtc_device	*rtc = ftrtc011->rtc;
	void __user *uarg = (void __user *) arg;
#endif

	switch (cmd) {
	case RTC_AIE_OFF:
		spin_lock_irq(&ftrtc011->lock);
		ftrtc011_disable_alarm(ftrtc011);
		spin_unlock_irq(&ftrtc011->lock);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irq(&ftrtc011->lock);
		ftrtc011_enable_alarm(ftrtc011);
		spin_unlock_irq(&ftrtc011->lock);
		return 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
	case RTC_PIE_ON:
		return rtc->ops->irq_set_state(dev, 1);

	case RTC_PIE_OFF:
		return rtc->ops->irq_set_state(dev, 0);

	case RTC_IRQP_SET:
		return rtc->ops->irq_set_freq(dev, arg);

	case RTC_IRQP_READ:
		return put_user(rtc->irq_freq, (unsigned long __user *)uarg);
#endif

	default:
		return -ENOIOCTLCMD;
	}
}

static int ftrtc011_read_time(struct device *dev, struct rtc_time *tm)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);
	unsigned long		time;

	spin_lock_irq(&ftrtc011->lock);

	time = ftrtc011_get_uptime(ftrtc011);
	rtc_time_to_tm(time, tm);

	spin_unlock_irq(&ftrtc011->lock);

	return 0;
}

static int ftrtc011_set_time(struct device *dev, struct rtc_time *tm)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);
	unsigned long		new;
	int			ret;

	spin_lock_irq(&ftrtc011->lock);

	ret = rtc_tm_to_time(tm, &new);
	if (ret == 0) {
		ftrtc011_set_uptime(ftrtc011, new);
	}

	spin_unlock_irq(&ftrtc011->lock);

	return ret;
}

static int ftrtc011_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);

	ftrtc011_get_alarm_tm(ftrtc011, &alrm->time);
	return 0;
}

static int ftrtc011_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);

	ftrtc011_show_tm(ftrtc011, __func__, &alrm->time);

	spin_lock_irq(&ftrtc011->lock);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
	{
		struct rtc_time	tm;
		unsigned long	now, then;
		int		err;

		ftrtc011_read_time(&ftrtc011->pdev->dev, &tm);
		rtc_tm_to_time(&tm, &now);

		alrm->time.tm_mday	= tm.tm_mday;
		alrm->time.tm_mon	= tm.tm_mon;
		alrm->time.tm_year	= tm.tm_year;

		err  = rtc_valid_tm(&alrm->time);
		if (err < 0) {
			spin_unlock_irq(&ftrtc011->lock);
			return err;
		}

		rtc_tm_to_time(&alrm->time, &then);

		/* alarm may need to wrap into tomorrow */
		if (then < now) {
			rtc_time_to_tm(now + 24 * 60 * 60, &tm);
			alrm->time.tm_mday	= tm.tm_mday;
			alrm->time.tm_mon	= tm.tm_mon;
			alrm->time.tm_year	= tm.tm_year;
		}
	}
#endif

	ftrtc011_set_alarm_tm(ftrtc011, &alrm->time);
	spin_unlock_irq(&ftrtc011->lock);

	return 0;
}

static int ftrtc011_irq_set_state(struct device *dev, int enabled)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);

	dev_info(dev, "%s(%d)\n", __func__, enabled);

	spin_lock_irq(&ftrtc011->lock);

	/* per second alarm supported only */
	if (enabled) {
		ftrtc011_enable_per_second_interrupt(ftrtc011);
	} else {
		ftrtc011_disable_per_second_interrupt(ftrtc011);
	}

	spin_unlock_irq(&ftrtc011->lock);

	return 0;
}

static int ftrtc011_irq_set_freq(struct device *dev, int freq)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct ftrtc011		*ftrtc011 = platform_get_drvdata(pdev);

	dev_info(dev, "%s(%d)\n", __func__, freq);

	spin_lock_irq(&ftrtc011->lock);
	ftrtc011->rtc->irq_freq		= freq;
	ftrtc011->periodic_count	= freq;
	spin_unlock_irq(&ftrtc011->lock);

	return 0;
}

static struct rtc_class_ops ftrtc011_ops = {
	.open		= ftrtc011_open,
	.release	= ftrtc011_release,
	.ioctl		= ftrtc011_ioctl,
	.read_time	= ftrtc011_read_time,
	.set_time	= ftrtc011_set_time,
	.read_alarm	= ftrtc011_read_alarm,
	.set_alarm	= ftrtc011_set_alarm,
	.irq_set_state	= ftrtc011_irq_set_state,
	.irq_set_freq	= ftrtc011_irq_set_freq,
};

/******************************************************************************
 * struct platform_driver functions
 *****************************************************************************/
static int ftrtc011_probe(struct platform_device *pdev)
{
	struct ftrtc011		*ftrtc011;
	struct resource		*res;
	int			ret;
	struct rtc_device	*rtc;

	dev_info(&pdev->dev, "%s()\n", __func__);

	if ((ftrtc011 = kzalloc(sizeof(*ftrtc011), GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}

	if ((res = platform_get_resource(pdev, IORESOURCE_MEM, 0)) == 0) {
		ret = -ENXIO;
		goto err_dealloc;
	}

	if ((ftrtc011->alarm_irq = platform_get_irq(pdev, 0)) < 0) {
		ret = ftrtc011->alarm_irq;
		goto err_dealloc;
	}

	if ((ftrtc011->periodic_irq = platform_get_irq(pdev, 1)) < 0) {
		ret = ftrtc011->periodic_irq;
		goto err_dealloc;
	}

	if ((ftrtc011->base = ioremap(res->start, res->end - res->start)) == NULL) {
		ret = -ENOMEM;
		goto err_dealloc;
	}

	rtc = rtc_device_register(pdev->name, &pdev->dev, &ftrtc011_ops,
		THIS_MODULE);
	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "register rtc device failed\n");
		ret = PTR_ERR(rtc);
		goto err_unmap;
	}
	ftrtc011->rtc	= rtc;
	ftrtc011->pdev	= pdev;

	spin_lock_init(&ftrtc011->lock);

	platform_set_drvdata(pdev, ftrtc011);

	ftrtc011_enable(ftrtc011);

	return 0;

err_unmap:
	iounmap(ftrtc011->base);
err_dealloc:
	kfree(ftrtc011);
	dev_err(&pdev->dev, "%s() = %d\n", __func__, ret);
	return ret;
}

static int ftrtc011_remove(struct platform_device *pdev)
{
	struct ftrtc011 *ftrtc011 = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s()\n", __func__);

	rtc_device_unregister(ftrtc011->rtc);
	iounmap(ftrtc011->base);
	kfree(ftrtc011);

	return 0;
}

static struct platform_driver ftrtc011_driver = {
	.probe	= ftrtc011_probe,
	.remove	= ftrtc011_remove,
	.driver	= {
		.name = "ftrtc011",
	},
};

/******************************************************************************
 * initialization / finalization
 *****************************************************************************/
static int __init ftrtc011_init(void)
{
	int	ret;

	printk(KERN_DEBUG "%s()\n", __func__);

	ret = platform_driver_register(&ftrtc011_driver);

	if (ret)
		printk(KERN_ERR "register platform driver failed (%d)\n", ret);

	return ret;
}

static void __exit ftrtc011_exit(void)
{
	printk(KERN_DEBUG "%s()\n", __func__);
	platform_driver_unregister(&ftrtc011_driver);
}

module_init(ftrtc011_init);
module_exit(ftrtc011_exit);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("FTRTC011 Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
