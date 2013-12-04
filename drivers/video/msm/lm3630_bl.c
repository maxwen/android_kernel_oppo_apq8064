/**************************************************************
* Copyright (c)	 200X- 2013	 Oppo Mobile communication Corp.ltd.//?
* VENDOR_EDIT
*File		: lm3630a.c
* Description: Source file for backlight IC LM630.
*			To control backlight brightness.
* Version	: 1.0
* Date		: 2013-5
* Author	: goushengjun@oppo.com
* ---------------------------------- Revision History: ----------------------------------
*	<version>	<date>		< author >			<desc>
****************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/vreg.h>
#include <linux/err.h>
#include <linux/i2c/lm3630_bl.h>
#include <mach/board.h>
#include <mach/device_info.h>




#define LM3630_DRV_NAME "lm3630"
#define LM3630_I2C_READ
#define LM3630_ENABLE_GPIO	 86
#define LM3630_PWM_ON		0x19
#define LM3630_PWM_TEST		0x1d
#define LM3630_PWM_OFF		0x18

static char *DEVICE_VERSION = "3630";
static char *DEVICE_MANUFACUTRE = "ti";
static bool sleep_mode = true;
static bool backlight_pwm_state_change = false;
static bool pwm_switch_enable = true;
static struct kobject *auto_sysfs_kobject;

static struct i2c_client *lm3630_client;

static const unsigned char pwm_off_map[] = {0,1,2,3,4,5,6,7,8,9,10,11,11,11,11,11,11,11,11,11};

static unsigned char get_bkl_level_pwm_off(unsigned char bkl_level)
{
	if(bkl_level > 20)
		return bkl_level;

	return pwm_off_map[bkl_level];
}

#ifdef LM3630_I2C_READ
static int lm3630_i2c_rxdata(unsigned char saddr, unsigned char *rxdata, int length)
{
	int rc;
	int retry_cnt = 0;
	struct i2c_msg msgs[] =
	{
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};

	do
	{
		rc = i2c_transfer(lm3630_client->adapter, msgs, 2);
		if (rc > 0)
			break;
		retry_cnt++;
	}
	while (retry_cnt < 3);

	if (rc < 0)
	{
		pr_err("%s: failed!:%d %d\n", __func__, rc, retry_cnt);
		return -EIO;
	}

	return 0;
}
#endif

static int32_t lm3630_i2c_txdata(unsigned short saddr,
								 unsigned char *txdata, int length)
{
	int retry_cnt = 0;
	int rc;

	struct i2c_msg msg[] =
	{
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	do
	{
		rc = i2c_transfer(lm3630_client->adapter, msg, 1);
		if (rc > 0)
			break;
		retry_cnt++;
	}
	while (retry_cnt < 3);

	if (rc < 0)
	{
		pr_err("%s :failed: %d %d\n", __func__, rc, retry_cnt);
		return -EIO;
	}

	return 0;
}

#ifdef LM3630_I2C_READ
static int32_t lm3630_i2c_read(unsigned char raddr, unsigned char *rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[1];

	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = lm3630_i2c_rxdata(lm3630_client->addr, buf, 1);
	if (rc < 0)
	{
		pr_err("%s : read reg addr 0x%x failed!\n", __func__, raddr);
		return rc;
	}
	*rdata = buf[0];

	return rc;
}
#endif

static int lm3630_i2c_write(unsigned char	raddr, unsigned char  rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];

	if (!rdata)
		return 0;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	buf[1] = rdata;
	rc = lm3630_i2c_txdata(lm3630_client->addr , buf, 2);
	if (rc < 0)
		pr_err("%s :failed, addr = 0x%x, val = 0x%x!\n", __func__, raddr, rdata);

	return rc;
}

/* OPPO 2013-10-04 gousj Add begin for back light flicker */
int set_backlight_pwm(int state)
{
	int rc = 0;

	if(state == 1) {
		if(pwm_switch_enable){
			if (lm3630_bkl_readout() > 20)
				rc = lm3630_i2c_write(0x01, LM3630_PWM_ON);
		} else
			rc = lm3630_i2c_write(0x01, LM3630_PWM_ON);
	} else {
		rc = lm3630_i2c_write(0x01, LM3630_PWM_OFF);
		backlight_pwm_state_change = true;
	}

	return rc;
}
/* OPPO 2013-10-04 gousj Add end */

int lm3630_bkl_control(unsigned char bkl_level)
{
	int rc = 0;


	if (!lm3630_client)
	{
		pr_err("%s: lm3630_client == null!\n", __func__);
		return rc;
	}

	if(bkl_level == 0)
	{
		rc = lm3630_i2c_write(0x03, 0x00);
		rc = lm3630_i2c_write(0x00, 0x9f);
		sleep_mode = true;
		pr_debug("%s: sleep rc = %d\n", __func__,rc);
		return rc;
	}

	if(sleep_mode == true)
	{
		pr_debug("%s backlight IC weke up.\n",__func__);
		rc = lm3630_i2c_write(0x00, 0x1f);
		sleep_mode= false;
		mdelay(10);
	}
	
	if(pwm_switch_enable){
		/* OPPO 2013-10-28 gousj Add begin for pwm flicker */
		if(bkl_level <= 19){
			rc = lm3630_i2c_write(0x01, LM3630_PWM_OFF);
			backlight_pwm_state_change = true;
			bkl_level = get_bkl_level_pwm_off(bkl_level);
		} else {
			if(backlight_pwm_state_change == true){
				rc = lm3630_i2c_write(0x01, LM3630_PWM_ON);
				backlight_pwm_state_change = false;
			}
		}
		/* OPPO 2013-10-28 gousj Add end */
	}
	
	rc = lm3630_i2c_write(0x03, bkl_level);
	pr_info("%s: set bkl level = %d, read level after write = %d ,rc = %d\n",
		 __func__,(int)bkl_level,lm3630_bkl_readout(),rc);

	return rc;
}

int lm3630_bkl_readout(void)
{
	int rc = 0;
#ifdef LM3630_I2C_READ
	unsigned char bkl_level = 0;
	rc = lm3630_i2c_read(0x03,&bkl_level);
	rc = bkl_level;
	pr_debug("%s bkl read out = %d\n",__func__,(int)rc);
#endif
	return rc;
}

int lm3630_pwm_readout(void)
{
	int rc = 0;
	unsigned char bkl_level = 0;
	rc = lm3630_i2c_read(0x01,&bkl_level);
	rc = bkl_level;
	pr_debug("%s pwm read out = %d\n",__func__,(int)rc);
	return rc;
}


static int lm3630_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	pr_debug("%s: backlight probe\n", __func__);

/* OPPO 2013-11-13 gousj Add begin for device information */
#ifdef CONFIG_VENDOR_EDIT
	register_device_proc("backlight", DEVICE_VERSION, DEVICE_MANUFACUTRE);
#endif
/* OPPO 2013-11-13 gousj Add end */

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("%s: i2c_check_functionality failed\n", __func__);
		rc = -EACCES;
		return rc;
	}

	lm3630_client = client;

	if (!lm3630_client)
	{
		pr_err("%s: lm3630_client == null!\n", __func__);
		rc = -ENODEV;
		goto exit;
	}

	rc = gpio_request(LM3630_ENABLE_GPIO, "lm3630_enable");
	if (rc)
	{
		pr_err("lm3630_enable gpio_request failed: %d\n", rc);
		goto exit;
	}
	rc = gpio_direction_output(LM3630_ENABLE_GPIO, 1);
	if (rc)
	{
		pr_err("%s: unable to enable LM3630_ENABLE_GPIO\n", __func__);
		goto exit;
	}

	//initialize bkl to max when system boot

	rc = lm3630_i2c_write(0x00, 0x1f);
	rc = lm3630_i2c_write(0x50, 0x03);
	if(get_boot_mode() == MSM_BOOT_MODE__NORMAL ||
		get_boot_mode() == MSM_BOOT_MODE__RECOVERY)
	{
		rc = lm3630_i2c_write(0x01, LM3630_PWM_ON);
	}
	else
	{
		rc = lm3630_i2c_write(0x01, LM3630_PWM_OFF);
	}
	rc = lm3630_i2c_write(0x02, 0x38);
	rc = lm3630_i2c_write(0x03, 0xff);
	rc = lm3630_i2c_write(0x05, 0x14);
	rc = lm3630_i2c_write(0x06, 0x14);
	rc = lm3630_i2c_write(0x07, 0x00);
	rc = lm3630_i2c_write(0x08, 0x00);
	rc = lm3630_i2c_write(0x09, 0xe3);
	rc = lm3630_i2c_write(0x10, 0xe3);
	rc = lm3630_i2c_write(0x11, 0xe3);
	rc = lm3630_i2c_write(0x12, 0xe3);
	rc = lm3630_i2c_write(0x13, 0xe3);

exit:
	if (rc)
		lm3630_client = 0;
	return rc;

}

static int lm3630_i2c_remove(struct i2c_client *client)
{
	return 0;
}
static int lm3630_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc ;

	pr_debug("%s: backlight suspend.\n", __func__);
	rc = gpio_direction_output(LM3630_ENABLE_GPIO, 0);
	if (rc)
	{
		pr_err("%s: unable to disable LM3630_ENABLE_GPIO\n", __func__);
		return rc;
	}

	return 0;
}
static int lm3630_resume(struct i2c_client *client)
{
	int rc ;
	pr_debug("%s: backlight resume.\n", __func__);

	rc = gpio_direction_output(LM3630_ENABLE_GPIO, 1);
	if (rc)
	{
		pr_err("%s: unable to enable LM3630_ENABLE_GPIO\n", __func__);
		return rc;
	}

	mdelay(10);
	rc = lm3630_i2c_write(0x50, 0x03);
	rc = lm3630_i2c_write(0x01, LM3630_PWM_OFF);
	backlight_pwm_state_change = true;
	/* OPPO 2013-10-04 gousj Modify begin for Radio Frequency Interference and over voltage */
#ifndef CONFIG_VENDOR_EDIT
	rc = lm3630_i2c_write(0x02, 0x79);
#else
	rc = lm3630_i2c_write(0x02, 0x38);
#endif
	/* OPPO 2013-10-04 gousj Modify end */
	rc = lm3630_i2c_write(0x05, 0x14);
	rc = lm3630_i2c_write(0x06, 0x14);
	rc = lm3630_i2c_write(0x07, 0x00);
	rc = lm3630_i2c_write(0x08, 0x00);

	return 0;
}

static const struct i2c_device_id lm3630_i2c_id[] =
{
	{ LM3630_DRV_NAME, 0},
	{}
};

static ssize_t show_pwm_switch_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", pwm_switch_enable);

	return out - buf;
}

static ssize_t store_pwm_switch_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int n;
		
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1) || n < 0 || n > 1)
		return -EINVAL;

	pwm_switch_enable = n;	
	return count;
}

static struct kobj_attribute pwm_switch_enable_attr =
	__ATTR(pwm_switch_enable, 0644, show_pwm_switch_enable, store_pwm_switch_enable);

static struct attribute *lm3630_attr[] = {
	&pwm_switch_enable_attr.attr,
	NULL,
};

static struct attribute_group lm3630_attr_group = {
	.attrs = lm3630_attr,
};

static int lm3630_auto_sysfs(void)
{
	int err;

	auto_sysfs_kobject= kobject_create_and_add("lm3630", kernel_kobj);
	if (!auto_sysfs_kobject)
		return -ENOMEM;


	err = sysfs_create_group(auto_sysfs_kobject, &lm3630_attr_group);
	if (err){
		kobject_put(auto_sysfs_kobject);
		auto_sysfs_kobject = NULL;
	}

	return err;
}

static struct i2c_driver lm3630_i2c_driver =
{
	.id_table = lm3630_i2c_id,
	.probe	= lm3630_i2c_probe,
	.remove = lm3630_i2c_remove,
	.suspend =lm3630_suspend,
	.resume = lm3630_resume,
	.driver = {
		.name = LM3630_DRV_NAME,
	},
};

static int __init lm3630_init(void)
{
	lm3630_auto_sysfs();
	return i2c_add_driver(&lm3630_i2c_driver);
}

static void __exit lm3630_eixt(void)
{
	if(auto_sysfs_kobject)
		kobject_put(auto_sysfs_kobject);
	i2c_del_driver(&lm3630_i2c_driver);
}

module_init(lm3630_init);
module_exit(lm3630_eixt);

MODULE_DESCRIPTION("lm3630 I2C Driver");
MODULE_LICENSE("GPL");
