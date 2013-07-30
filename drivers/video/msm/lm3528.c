/**************************************************************
* Copyright (c)  200X- 2012  Oppo Mobile communication Corp.ltd.£¬
* VENDOR_EDIT
*File       : lm3528.c
* Description: Source file for backlight IC LM3528.
*           To control backlight brightness.
* Version   : 1.0
* Date      : 2012-10-14
* Author    : zhengzekai@oppo.com
* ---------------------------------- Revision History: ----------------------------------
*  	<version>	<date>		< author >			<desc>
* 	Revision 1.0  2012-10-14   zhengzekai@oppo.com	creat
****************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/vreg.h>
#include <linux/err.h>
#include <linux/i2c/lm3528.h>

#define LM3528_DRV_NAME	"lm3528"
#define SSL3252_CHIP_ID		0x41
#define LM3528_GP			0x10
#define LM3528_GP_MODE		0xc7
#define LM3528_BMAIN		0xa0
#define LM3528_BMAIN_MAX	127

#define LM3528_I2C_READ

static struct i2c_client *lm3528_client;

#ifdef LM3528_I2C_READ
static int lm3528_i2c_rxdata(unsigned char saddr, unsigned char *rxdata, int length)
{
	int rc;
	int retry_cnt = 0;
	struct i2c_msg msgs[] = {
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
	
	do {
		rc = i2c_transfer(lm3528_client->adapter, msgs, 2);
		if (rc > 0)
			break;	
		retry_cnt++;
	} while (retry_cnt < 3);
	
	if (rc < 0) {
		pr_err("%s: failed!:%d %d\n", __func__, rc, retry_cnt);
		return -EIO;
	}

	return 0;
}
#endif

static int32_t lm3528_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	int retry_cnt = 0;
	int rc;

	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	
	do {
		rc = i2c_transfer(lm3528_client->adapter, msg, 1);
		if (rc > 0)
			break;
		retry_cnt++;
	} while (retry_cnt < 3);
	
	if (rc < 0) {
		pr_err("%s :failed: %d %d\n", __func__, rc, retry_cnt); 	
		return -EIO;
	}
	
	return 0;
}

#ifdef LM3528_I2C_READ
static int32_t lm3528_i2c_read(unsigned char raddr, unsigned char *rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[1];
	
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = lm3528_i2c_rxdata(lm3528_client->addr, buf, 1);
	if (rc < 0) {
		pr_err("%s : read reg addr 0x%x failed!\n", __func__, raddr);
		return rc;
	}
	*rdata = buf[0];

	return rc;
}
#endif

static int lm3528_i2c_write(unsigned char	raddr, unsigned char  rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];

	if (!rdata)
		return 0;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	buf[1] = rdata;
	rc = lm3528_i2c_txdata(lm3528_client->addr , buf, 2);
	if (rc < 0)
		pr_err("%s :failed, addr = 0x%x, val = 0x%x!\n", __func__, raddr, rdata);
	
	return rc;
}

int lm3528_bkl_control(unsigned char bkl_level)
{
	int rc = 0;

	if (!lm3528_client) {
		pr_err("%s: lm3528_client == null!\n", __func__);
		return rc;
	}
	
	if(bkl_level == 0)
	{
		rc = lm3528_i2c_write(LM3528_GP, 0xc0);
		return rc;
	}
	
	rc = lm3528_i2c_write(LM3528_GP, LM3528_GP_MODE);
	rc = lm3528_i2c_write(LM3528_BMAIN, bkl_level);

	return rc;
}

int lm3528_bkl_readout(void)
{
	int rc = 0;
#ifdef LM3528_I2C_READ
	unsigned char bkl_level = 0;
	rc = lm3528_i2c_read(LM3528_BMAIN, &bkl_level);
	rc = bkl_level;
#endif
	return rc;
}

#define LM3528_ENABLE_GPIO   86
static int lm3528_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c_check_functionality failed\n", __func__);
		rc = -EACCES;
		return rc;
	}

	lm3528_client = client;

	if (!lm3528_client) {
		pr_err("%s: lm3528_client == null!\n", __func__);
		rc = -ENODEV;
		goto exit;
	}
	
	rc = gpio_request(LM3528_ENABLE_GPIO, "lm3528_enable");
	if (rc) {
		pr_err("lm3528_enable gpio_request failed: %d\n", rc);
		goto exit;
	}
	rc = gpio_direction_output(LM3528_ENABLE_GPIO, 1);
	if (rc) {
		pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
		goto exit;
	}
	
	//initialize bkl to max when system boot
	rc = lm3528_i2c_write(LM3528_GP, LM3528_GP_MODE);
	rc = lm3528_i2c_write(LM3528_BMAIN, LM3528_BMAIN_MAX);

exit:
	if (rc)
       lm3528_client = 0;
	return rc;

}

static int lm3528_i2c_remove(struct i2c_client *client)
{
	return 0;
}
static int lm3528_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc ;
	
	printk("%s:backlight suspend.\n", __func__);
	rc = gpio_direction_output(LM3528_ENABLE_GPIO, 0);
	if (rc) {
		pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
		return rc;
	}

	return 0;
}
static int lm3528_resume(struct i2c_client *client)
{
	int rc ;

	printk("%s: backlight resume.\n", __func__);
	rc = gpio_direction_output(LM3528_ENABLE_GPIO, 1);
	if (rc) {
		pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
		return rc;
	}

	return 0;
}
static const struct i2c_device_id lm3528_i2c_id[] = {
	{ LM3528_DRV_NAME, 0},
	{}
};

static struct i2c_driver lm3528_i2c_driver = {
	.id_table = lm3528_i2c_id,
	.probe	= lm3528_i2c_probe,
	.remove = lm3528_i2c_remove,
	.suspend = lm3528_suspend,
	.resume = lm3528_resume,
	.driver = {
		.name = LM3528_DRV_NAME,
	},
};

static int __init lm3528_init(void) 
{
	return i2c_add_driver(&lm3528_i2c_driver);
}

static void __exit lm3528_eixt(void)
{
	i2c_del_driver(&lm3528_i2c_driver);
}

module_init(lm3528_init);
module_exit(lm3528_eixt);

MODULE_DESCRIPTION("lm3528 I2C Driver");
MODULE_LICENSE("GPL");
