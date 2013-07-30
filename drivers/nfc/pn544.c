/*
 * Driver for the PN544 NFC chip.
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Jari Vanhala <ext-jari.vanhala@nokia.com>
 * Contact: Matti Aaltonen <matti.j.aaltonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/nfc/pn544.h>
#include <linux/regulator/consumer.h>
#include <linux/pcb_version.h>

//TODO:replace and include corresponding head file for VEN/IRQ/FIRM I/O configuration
//#include <plat/gpio-core.h>
//#include <plat/gpio-cfg.h>
//#include <plat/gpio-cfg-helpers.h>


//#define pr_err printk
//#define pr_debug printk
//#define pr_warning printk




#define MAX_BUFFER_SIZE	512
/* OPPO 2012-07-20 liuhd Add begin for reason */
#define NFC_POWER_ON 1
#define NFC_POWER_OFF 0
static struct regulator *ldo121;
static struct regulator *ldol23;
static struct regulator *lvs5;
extern int get_pcb_version(void);
/* OPPO 2012-07-20 liuhd Add end */
struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool				irq_enabled;
	spinlock_t			irq_enabled_lock;
};

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;
	
/* OPPO 2012-08-13 liuhd Modify begin for nfc */
#ifndef PN544
	if(gpio_get_value(pn544_dev->irq_gpio)){
		pn544_disable_irq(pn544_dev);

		/* Wake up waiting readers */
		wake_up(&pn544_dev->read_wq);
	}
#else
	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);
#endif
/* OPPO 2012-08-13 liuhd Modify end */
	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret ;
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 0
	printk("%s : reading %zu bytes.\n", __func__, count);
#endif
/* OPPO 2012-08-13 liuhd Delete end */

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) 
	{
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	/* pn544 seems to be slow in handling I2C read requests
	 * so add 1ms delay after recv operation */
	if (ret < 0) 
	{
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) 
	{
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	
/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 0
	printk("IFD->PC:");
	for(i = 0; i < ret; i++)
	{
		printk(" %02X", tmp[i]);
	}
	printk("\n");
#endif
/* OPPO 2012-08-13 liuhd Delete end */
	
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret ;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
	}
	if (copy_from_user(tmp, buf, count)) 
	{
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 0
	printk("%s : writing %zu bytes.\n", __func__, count);
#endif
/* OPPO 2012-08-13 liuhd Delete end */
	
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) 
	{
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	
	/* pn544 seems to be slow in handling I2C write requests
	 * so add 1ms delay after I2C send oparation */
	
/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 0
	printk("PC->IFD:");
	for(i = 0; i < count; i++)
	{
		printk(" %02X", tmp[i]);
	}
	printk("\n");
#endif
/* OPPO 2012-08-13 liuhd Delete end */
	
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{

	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);
	
	filp->private_data = pn544_dev;

	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	
	switch (cmd) 
	{
	case PN544_SET_PWR:
		if (arg == 2) 
		{
			/* power on with firmware download (requires hw reset)
			 */
			printk("%s power on with firmware\n", __func__);
			
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(20);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(20);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(100);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(20);
		} else if (arg == 1) {
			/* power on */
			printk("%s power on\n", __func__);
			
			gpio_set_value(pn544_dev->firm_gpio, 0);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(50);
		} else  if (arg == 0) {
			/* power off */
			printk("%s power off\n", __func__);
			
			gpio_set_value(pn544_dev->firm_gpio, 0);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = 
{
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl = pn544_dev_ioctl,
};

/* OPPO 2012-07-20 liuhd Add begin for reason */
static int pn544_power(int on)
{
	if (on) {
		//l23
		ldol23 = regulator_get(NULL, "8921_l23");
		if (IS_ERR(ldol23)){
			pr_err("%s: VREG ldol23 get failed\n", __func__);
			ldol23 = NULL;
			goto ldo123_get_failed;
		}
		//printk("8921_l23 power on1 regulator yuyi+++++++++++++++++++++++++++++++++++++++++");
		if (regulator_set_voltage(ldol23, 1800000, 1800000)) {
			pr_err("%s: VREG ldol23 set voltage failed\n",  __func__);
			goto ldo123_get_failed;
		}
		//printk("8921_l23 power on2 voltage yuyi+++++++++++++++++++++++++++++++++++++++++");
		if (regulator_enable(ldol23)) {
			pr_err("%s: VREG ldol23 enable failed\n", __func__);
			goto ldo123_get_failed;
		}
		//printk("8921_l23 power on3 enable yuyi+++++++++++++++++++++++++++++++++++++++++");

	    if (get_pcb_version() <= PCB_VERSION_EVT) { 
			//evt---lvs5
		    lvs5 = regulator_get(NULL, "8921_lvs5");
			if (IS_ERR(lvs5)){
				pr_err("%s: VREG LVS5 get failed\n", __func__);
				lvs5 = NULL;
				goto lvs5_get_failed;
			}
			if (regulator_enable(lvs5)) {
				pr_err("%s: VREG LVS5 enable failed\n", __func__);
				goto lvs5_get_failed;
			}
		}
		else {
		//dvt---l21
			ldo121 = regulator_get(NULL, "8921_l21");
			if (IS_ERR(ldo121)){
				pr_err("%s: VREG ldo121 get failed\n", __func__);
				ldo121 = NULL;
				goto ldo121_get_failed;
			}
			if (regulator_set_voltage(ldo121, 1800000, 1800000)) {
				pr_err("%s: VREG ldo121 set voltage failed\n",	__func__);
				goto ldo121_get_failed;
			}
			if (regulator_enable(ldo121)) {
				pr_err("%s: VREG ldo121 enable failed\n", __func__);
				goto ldo121_get_failed;
			}		
		}
	}else if (!on) {
		if (ldol23) {
			regulator_disable(ldol23);
			regulator_put(ldol23);
		}
		//printk(" power down nfc l23 yuyi-----------------------------------------\n");
	    if (get_pcb_version() <= PCB_VERSION_EVT) {  //evt
			if (lvs5){
				regulator_disable(lvs5);
				regulator_put(lvs5);
			}
	    }else { //dvt
			if (ldo121) {
				regulator_disable(ldo121);
				regulator_put(ldo121);
			}
			//printk(" power down  nfc l21 yuyi----------------------------------------\n");
		}
	}
	return 0 ;
	
ldo121_get_failed:
	regulator_disable(ldo121);
ldo123_get_failed:
	regulator_disable(ldol23);
lvs5_get_failed:
	regulator_put(lvs5);
	return -1 ;
}			

/* OPPO 2012-07-20 liuhd Add end */

static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pn544_dev;

	platform_data = client->dev.platform_data;

	
/* OPPO 2012-07-20 liuhd Add begin for reason */
	if (pn544_power(NFC_POWER_ON)) {
			printk("%s: ---nfc failed to power up nfc\n", __func__);
			return	-ENODEV;
		}
	//printk(" power up nfc yuyi-------------------------------------\n");
/* OPPO 2012-07-20 liuhd Add end */

	if (platform_data == NULL) {
		pr_err("%s : nfc probe fail\n", __func__);
		return  -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	//IRQ 
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret){
		pr_err("gpio_nfc_int request error\n");
		return  -ENODEV;
	}
	//printk(" power up IRQ yuyi---------------------------------------\n");

	//VEN
	ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
	if (ret){
		pr_err("gpio_nfc_ven request error\n");
		return  -ENODEV;
	}
	//printk(" power up VEN yuyi------------------------------------------\n");

	//FIRM
	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret){
		pr_err("gpio_nfc_firm request error\n");	
		return  -ENODEV;
	}

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
	pn544_dev->client   = client;
	
/* OPPO 2012-07-11 liuhd Add begin for reason */
	ret = gpio_direction_input(pn544_dev->irq_gpio);
	if (ret < 0) {
		pr_err("%s :not able to set irq_gpio as input\n", __func__);
		goto err_exit;
	}
	if (platform_data->firm_gpio) {
		ret = gpio_direction_output(pn544_dev->firm_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set firm_gpio as output\n",
				 __func__);
			goto err_exit;
		}
	}
	ret = gpio_direction_output(pn544_dev->ven_gpio, 1);
	if (ret < 0) {
		pr_err("%s : not able to set ven_gpio as output\n", __func__);
		goto err_exit;
	}
/* OPPO 2012-07-11 liuhd Add end */
			
	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	
	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_exit:
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}
static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= 
	{
		.owner	= THIS_MODULE,
		.name	= "pn544",
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
