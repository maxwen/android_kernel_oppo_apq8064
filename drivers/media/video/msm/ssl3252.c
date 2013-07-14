
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/vreg.h>
#include <linux/err.h>
#include <mach/camera.h>
#include <linux/i2c/ssl3252.h>
#include <linux/leds.h>
#include <linux/gpio.h>

//#define SSL3252_DEBUG 1
//#define CAMERA_FLASH_SSL3252_DEBUG
#ifdef CAMERA_FLASH_SSL3252_DEBUG
#define CDBG_FLASH(fmt, args...) printk(KERN_INFO "ssl3252.c: " fmt, ##args) 
#else
#define CDBG_FLASH(fmt, args...) do { } while (0)
#endif

#define SSL3252_DRV_NAME "ssl3252"
#define SSL3252_I2C_ADDR 	0x30
#define SSL3252_CHIP_ID	0x41

/****************************************************************************
 * device addr functions
 ***************************************************************************/
#define REG_CHIPID		0x00
#define REG_VREF_TIMER 	0x02
#define REG_CUR			0x03
#define REG_OUTPUT_MODE 0x04
#define REG_FAULT_INFO	0x05

//Output Mode Register bit definition
#define MOD_LED_OFF		0
#define STR_MODE_OFF	2
#define OUTPUT_EN_OFF	3
#define FREQ_FB_OFF		4
#define STR_LV_OFF		5
#define IL_PEAK_OFF		6

//vref and timer bit definition
#define FL_TIM_OFF 		0
#define IO1_CFG_OFF		4
#define IO2_CFG_OFF		6

//current bit definition
#define I_FL_OFF		3
#define I_TOR_OFF		0

#define SSL3252_MAX_TIME		850
#define SSL3252_MIN_TIME		100
#define SSL3252_TIME_STEP		50

#define SSL3252_MAX_FLASH_CUR	400  //for two leds
#define SSL3252_MIN_FLASH_CUR	200
#define SSL3252_FLASH_CUR_STEP	20

#define SSL3252_MAX_TORCH_CUR	160
#define SSL3252_MIN_TORCH_CUR	20
#define SSL3252_TORCH_CUR_STEP	20

#define LED_DETECTION_BIT       0x08

#define SSL3252_TORCH_CURRENT   40
#define SSL3252_FLASH_CURRENT   400
/* OPPO 2012-12-15 yxq modified begin for flash current is too large */
//#define SSL3252_FLASH_CURRENT   340
/* OPPO 2012-12-15 yxq modified end */

/* OPPO 2012-12-17 yxq Add begin for reason */
#define SSL3252_FLASH_TIME   450
/* OPPO 2012-12-17 yxq Add end */

#define MSM_CAMERA_LED_OFF  0
#define MSM_CAMERA_LED_LOW  1
#define MSM_CAMERA_LED_HIGH 2
#define MSM_CAMERA_LED_INIT 3
#define MSM_CAMERA_LED_RELEASE 4

/****************************************************************************
 * structures
 ***************************************************************************/
struct ssl3252_data {
	bool enable;
	uint8_t torch_cur;
	uint8_t flash_cur;
	uint8_t flash_time;
	int gpio_en;
	int gpio_tx1;
	int gpio_tx2;
	void (*dev_startup)(void);
	void (*dev_shutdown)(void);
	struct i2c_client *client;
};

static struct led_classdev ldev;
static int current_britness = 0;
static struct i2c_client *ssl3252_client;
/* OPPO 2012-08-11 yxq modified begin for ssl3252 */
#if 0
static struct regulator *lvs0;
#else
static struct regulator *lvs5;
#endif
/* OPPO 2012-08-11 yxq modified end */
static int current_state = 0;
static int inited = 0;
/****************************************************************************
 * internal functions
 ***************************************************************************/
static int ssl3252_i2c_rxdata(unsigned char saddr, unsigned char *rxdata, int length)
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
		rc = i2c_transfer(ssl3252_client->adapter, msgs, 2);
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

static int32_t ssl3252_i2c_txdata(unsigned short saddr,
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
		rc = i2c_transfer(ssl3252_client->adapter, msg, 1);
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

static int32_t ssl3252_i2c_read(unsigned char raddr, unsigned char *rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[1];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = raddr;

	rc = ssl3252_i2c_rxdata(ssl3252_client->addr, buf, 1);

	if (rc < 0) {
		pr_err("%s : read reg addr 0x%x failed!\n", __func__, raddr);
		return rc;
	}

	*rdata = buf[0];

	return rc;
}



static int ssl3252_i2c_write(unsigned char	raddr, unsigned char  rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];

	if (!rdata)
		return 0;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	buf[1] = rdata;
	rc = ssl3252_i2c_txdata(ssl3252_client->addr , buf, 2);
	if (rc < 0)
		pr_err("%s :failed, addr = 0x%x, val = 0x%x!\n", __func__, raddr, rdata);
	
	return rc;
}

static void ssl3252_poweron(void)
{
/* OPPO 2012-08-11 yxq modified begin for ssl3252 */
#if 0
	CDBG_FLASH("%s: E\n", __func__);
	lvs0 = regulator_get(NULL, "8058_lvs0");
	if (IS_ERR(lvs0)) {
		pr_err("%s: VREG LVS0 get failed\n", __func__);
		lvs0 = NULL;
	}
	if (regulator_enable(lvs0)) {
		pr_err("%s: VREG LVS0 enable failed\n", __func__);
	}
#else
	CDBG_FLASH("%s: E\n", __func__);
	lvs5= regulator_get(NULL, "8921_lvs5");
	if (IS_ERR(lvs5)) {
		pr_err("%s: VREG_LVS5 get failed\n", __func__);
		lvs5= NULL;
	}
	if (regulator_enable(lvs5)) {
		pr_err("%s: VREG_LVS5 enable failed\n", __func__);
	}
#endif
/* OPPO 2012-08-11 yxq modified end */
}
static void ssl3252_poweroff(void){
/* OPPO 2012-08-11 yxq modified begin for ssl3252 */
#if 0
	if (lvs0) {
		regulator_disable(lvs0);
		regulator_put(lvs0);
	}
#else
if (lvs5) {
	regulator_disable(lvs5);
	regulator_put(lvs5);
}
#endif
/* OPPO 2012-08-11 yxq modified end */
}

static void ssl3252_startup(void)
{
	int rc;

	CDBG_FLASH("%s: E\n", __func__);
/* OPPO 2012-08-11 yxq modified begin for ssl3252 */
#if 0
	rc = gpio_request(125, "flash_en");
	if (rc < 0) {
		pr_err("%s not able to get gpio 125\n", __func__);
	}
	gpio_direction_output(125, 1);

	gpio_request(47, "ssl3252_scl");
	if (rc < 0) {
		pr_err("%s not able to get gpio 47\n", __func__);
	}
	
	gpio_request(48, "ssl3252_sda");
	if (rc < 0) {
		pr_err("%s not able to get gpio 48\n", __func__);
	}

	rc = gpio_request(50, "torch_tx1");
	if (rc < 0) {
		pr_err("%s not able to get gpio\n", __func__);
		gpio_free(125);
	}
	rc = gpio_request(49, "led_tx2");
	if (rc < 0) {
		pr_err("%s not able to get gpio\n", __func__);
		gpio_free(125);
		gpio_free(50);
	}
#else
	rc = gpio_request(36, "led_torch");
	if (rc < 0) {
		pr_err("%s not able to get gpio 36\n", __func__);
	}
	rc = gpio_request(1, "flash_now");
	if (rc < 0) {
		pr_err("%s not able to get gpio 1\n", __func__);
		gpio_free(36);
	}
#endif
/* OPPO 2012-08-11 yxq modified end */
	CDBG_FLASH("%s: X\n", __func__);
}
static void ssl3252_shutdown(void)
{
/* OPPO 2012-08-11 yxq modified begin for ssl3252 */
#if 0
	//gpio_direction_output(125, 0);
	gpio_free(125);
	gpio_free(50);
	gpio_free(49);
	gpio_free(48);
	gpio_free(47);
#else
	gpio_free(36);
	gpio_free(1);
#endif
/* OPPO 2012-08-11 yxq modified end */
}

static int ssl3252_flash_sw_triger(struct i2c_client *client, int cur)
{
	int ret = 0;
	unsigned int cur_reg_val = 0; 
	unsigned char val = 0;
/* OPPO 2012-12-17 yxq Add begin for flash time is */
    unsigned char flash_time_reg_val = 0;
/* OPPO 2012-12-17 yxq Add end */
	
	if(cur >= SSL3252_MIN_FLASH_CUR && cur <= SSL3252_MAX_FLASH_CUR) {
		CDBG("%s: the current is %dmA\n", __func__, cur);

		cur_reg_val = ((cur-200)/20) << 4;
		ret = ssl3252_i2c_read( REG_CUR, &val);
		if(ret)
			pr_err("%s: got REG_CUR error 0x%x!\n", __func__,val);
		cur_reg_val |= (val&0x0F);	

/* OPPO 2012-12-17 yxq Add begin for flash time */
        flash_time_reg_val = (SSL3252_FLASH_TIME-100)/50;
		ret = ssl3252_i2c_read( REG_VREF_TIMER, &val);
		if(ret)
			pr_err("%s: got REG_VREF_TIMER error 0x%x!\n", __func__,val);
		flash_time_reg_val |= (val&0xF0);
/* OPPO 2012-12-17 yxq Add end */
		
		CDBG("%s : current reg val will be set as 0x%x\n", __func__, cur_reg_val);
		ret |= ssl3252_i2c_write( REG_CUR, cur_reg_val);
/* OPPO 2012-12-17 yxq Add begin for flash time */
        ret |= ssl3252_i2c_write( REG_VREF_TIMER, flash_time_reg_val);
/* OPPO 2012-12-17 yxq Add end */
		ret |= ssl3252_i2c_write( REG_OUTPUT_MODE, 0xBB);
		if(ret)
			pr_err("%s: write to REG_CUR error!\n", __func__ );
	} else {
		pr_err("%s: the flash current seted is out of range!\n", __func__);
	}
	
	return ret;
}


static int ssl3252_torch_control(struct i2c_client *client, int cur)
{
 	int ret = 0;
		
	if(cur>=0 && cur<=SSL3252_MAX_TORCH_CUR) {
		if(cur == 0) {
			ret = ssl3252_i2c_write(REG_OUTPUT_MODE, 0xB0);
		} else { 
			unsigned char curr;

			curr = (cur-20)/20;
			ret |= ssl3252_i2c_write(REG_CUR, curr | LED_DETECTION_BIT); 
			/*now power on torch mode*/
			ret |= ssl3252_i2c_write(REG_OUTPUT_MODE, 0xAA); 
		}
	} else {
		pr_err("%s: the torch current seted is out of range!\n", __func__);
	}

	return ret;
}

void ssl3252_set_torch_control(unsigned long cur)
{
	if(cur > 0) {
		CDBG_FLASH("cur is > 0\n");
		if(current_state == 0){
			ssl3252_poweron();
			ssl3252_startup();
			}
			ssl3252_torch_control(ssl3252_client, cur);
		current_state = 1;
	} else if(cur == 0) {
		CDBG_FLASH("cur is 0\n");
		if(current_state == 1){
			ssl3252_torch_control(ssl3252_client, cur);
			ssl3252_shutdown();
			ssl3252_poweroff();
		}
		current_state = 0;
	}
}

static ssize_t ssl3252_torch_control_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	unsigned long cur = simple_strtoul(buf, NULL, 10);
	ssl3252_set_torch_control(cur);
	return count;
}

void ssl3252_set_flash(int cur)
{
	
	ssl3252_poweron();
	ssl3252_startup();
	ssl3252_flash_sw_triger(ssl3252_client, cur);
	msleep(850);
	ssl3252_shutdown();
	ssl3252_poweroff();
}

static ssize_t ssl3252_test_flash_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	unsigned long cur = simple_strtoul(buf, NULL, 10);
	ssl3252_set_flash(cur);
	return count;
}

static DEVICE_ATTR(test_flash, S_IWUSR | S_IRUGO,
		   NULL, ssl3252_test_flash_store);

static DEVICE_ATTR(torch_control, S_IWUSR | S_IRUGO,
		   NULL, ssl3252_torch_control_store);

#ifdef SSL3252_DEBUG
ssize_t regs_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int regs[] = {
		REG_CHIPID,		
		REG_VREF_TIMER,	
		REG_CUR,			
		REG_OUTPUT_MODE, 
		REG_FAULT_INFO,	
		REG_INPUT_CNTL,	
	};
	int idx;
	char *p = buf;
	unsigned char val;

	for (idx=0; idx<sizeof(regs)/sizeof(regs[0]); idx++) {
		ssl3252_i2c_read(regs[idx], &val);
		p += sprintf(p, "reg %x = 0x%02x\n", regs[idx], val);
	}
	
	return (p-buf);
}

static DEVICE_ATTR(regs, S_IWUSR | S_IRUGO,
		   regs_show, NULL);
#endif

static struct attribute *ssl3252_attributes[] = {
	&dev_attr_test_flash.attr,
	&dev_attr_torch_control.attr,
#ifdef SSL3252_DEBUG
	&dev_attr_regs.attr,	
#endif	
	NULL
};

static const struct attribute_group ssl3252_attr_group = {
	.attrs = ssl3252_attributes,
};

//soft triger
int oppo_led_control(unsigned state)
{
	int ret = 0;
#if 1
        unsigned gpio_strobe = 1;
	unsigned char val;
	if (!ssl3252_client) {
		pr_err("%s: ssl3252_client == null!\n", __func__);
		return ret;
	}

	switch(state) {
	case MSM_CAMERA_LED_INIT:
		if(inited) break;
		inited = 1;
		CDBG_FLASH("%s: INIT flash led\n", __func__);
		ssl3252_i2c_write( REG_OUTPUT_MODE, 0xA4);
		/*init flash time out*/
		ret = ssl3252_i2c_write( REG_VREF_TIMER, (SSL3252_FLASH_TIME-100)/50);
		/*init current*/
		ret |= ssl3252_i2c_write( REG_CUR,  
			(((SSL3252_FLASH_CURRENT-200)/20) << 4)|
			((SSL3252_TORCH_CURRENT-20)/20)|LED_DETECTION_BIT);
		gpio_request_one(gpio_strobe, GPIOF_OUT_INIT_LOW, "strobe");
		break;
	case MSM_CAMERA_LED_LOW:
		__gpio_set_value(gpio_strobe, 0);
		ret = ssl3252_i2c_read(REG_FAULT_INFO, &val);
		CDBG("%s: the fault info value is 0x%X\n", __func__, val);	
		ret |= ssl3252_i2c_write(REG_OUTPUT_MODE, 0xAE); 
		CDBG_FLASH("LOW flash led\n");
		break;
	case MSM_CAMERA_LED_HIGH:
		ssl3252_i2c_write( REG_OUTPUT_MODE, 0xAF);
		__gpio_set_value(gpio_strobe, 1);
		CDBG_FLASH("HIGH flash led\n");
		break;
	case MSM_CAMERA_LED_OFF:	
		__gpio_set_value(gpio_strobe, 0);
		ret = ssl3252_i2c_write( REG_OUTPUT_MODE, 0xA6);
		CDBG_FLASH("OFF flash led\n");
		break;
	case MSM_CAMERA_LED_RELEASE:	
		if(inited == 0) break;
		inited = 0;
		__gpio_set_value(gpio_strobe, 0);
		CDBG_FLASH("%s: RELEASE flash led\n", __func__);
		ret = ssl3252_i2c_write( REG_OUTPUT_MODE, 0xA0);
		gpio_free(gpio_strobe);
		break;
	default:
		break;
	}
#endif
	CDBG("%s:state is %d, ret is %d\n", __func__, state, ret);
	return ret;
}

/*-----------------------------------------------------------------*/
#if 0
int ssl3252_dev_init(int power_on)
{
	int ret = 0;
	//struct ssl3252_platform_data *pdata ;
	
	if (!ssl3252_client) {
		return -ENODEV;
	}
	
	//pdata = ssl3252_client->dev.platform_data;
	
	if(power_on){
		//gpio_direction_output(pdata->flash_en, 1);
		msleep(20);
		CDBG_FLASH("%s : Power on Flash led\n", __func__);
		/*GPIO0,GOIO1:impedance;flash time 1000ms*/
		//ret |= ssl3252_i2c_write( REG_VREF_TIMER, 0x09); 
		/*torch current 75mA, flash current 700mA*/
		//ret |= ssl3252_i2c_write( REG_CUR, 0x42); 
		//ret = ssl3252_i2c_read( REG_INPUT_CNTL, NULL); 
		//ret = ret & 
		ret |= ssl3252_i2c_write( REG_INPUT_CNTL, 0x02); 
		ret |= ssl3252_i2c_write( REG_OUTPUT_MODE, 0x9B); 
		
	} else {
		CDBG_FLASH("%s : power down flash led!\n", __func__);
		//gpio_direction_output(pdata->flash_en, 0);
		ret |= ssl3252_i2c_write( REG_OUTPUT_MODE, 0x94); 
	}
	
	if(ret)
		pr_err("%s : Adp1650 flash led init failed power_on %d, ret %d\n", __func__, power_on,ret);
	
	return ret;
}
#endif

static void ssl3252_set_brightness(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	CDBG_FLASH("%s", __func__);
	ssl3252_set_torch_control(brightness);
	current_britness = brightness;
}

static enum led_brightness ssl3252_get_brightness(struct led_classdev *led_cdev)
{
	CDBG_FLASH("%s", __func__);
	return current_britness;
}

static int ssl3252_set_blink(struct led_classdev *led_cdev,
				unsigned long *delay_on,unsigned long *delay_off)
{
	CDBG_FLASH("%s", __func__);
	return 0;
}

void ssl3252_add_led_class(void)
{
	int  err = 0;
		
	ldev.name = "spotlight";
	ldev.max_brightness = SSL3252_MAX_TORCH_CUR;
	ldev.brightness_set = ssl3252_set_brightness;
	ldev.brightness_get = ssl3252_get_brightness;
	ldev.blink_set = ssl3252_set_blink;
	ldev.flags = LED_CORE_SUSPENDRESUME;
	ldev.brightness = LED_OFF ;
	
	err = led_classdev_register(&ssl3252_client->dev, &ldev);
	if (err < 0) {
		pr_err("%s: register led drive error\n",__func__);
		}
}

static int ssl3252_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct ssl3252_platform_data *pdata = client->dev.platform_data;
	unsigned char val;
	int ret = 0;

	CDBG_FLASH( "%s :E\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c_check_functionality failed\n", __func__);
		ret = -EACCES;
		return ret;
	}

	ssl3252_client = client;
#if 0
	if (pdata && pdata->dev_startup) {
		pdata->dev_startup();
	} else {
		pr_err("%s : invalid pdata!!!\n", __func__);
		return -EINVAL;
	}
#endif

	ssl3252_poweron();
	ssl3252_startup();
	msleep(5);

	if (ssl3252_i2c_read(REG_CHIPID, &val) < 0) {
		pr_err("%s : read chip ID failed! line = %d\n", __func__,  __LINE__);
		ret = -EIO;
		goto exit;
	}
	CDBG_FLASH("%s: CHIP ID is 0x%x", __func__, val);
	//ssl3252_shutdown();
	//ssl3252_poweroff();

/* OPPO 2013-03-09 kangjian modify begin for led */
#if 0
	if (val != SSL3252_CHIP_ID) {
#else	
    if ((val != SSL3252_CHIP_ID) && (val != 0x21)) {
#endif
/* OPPO 2013-03-09 kangjian modify end for led */
		pr_err("%s: chipID %d is wrong\n", __func__, val);
		ret = -ENODEV;
		goto exit;
	}
	
	//gpio_direction_output(pdata->flash_en, 0);

	/*add led class*/
	ssl3252_add_led_class();

	/* Register sysfs hooks */
	ret |= sysfs_create_group(&client->dev.kobj, &ssl3252_attr_group);
	if (ret) {
		pr_err( "%s : sysfs_create_group failed!\n", __func__);
		goto exit;
	}
	
	CDBG_FLASH("%s: X\n", __func__);
	
	//return 0;
	
exit:
	//gpio_direction_output(pdata->flash_en, 0);
	//release the resources after probe failed
	if (ret)
        ssl3252_client = 0;
	ssl3252_shutdown();
	ssl3252_poweroff();
	return ret;
}

static int ssl3252_i2c_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ssl3252_attr_group);
	return 0;
}

/*-----------------------------------------------------------------*/
#if 0
static int ssl3252_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl3252_platform_data *pdata = client->dev.platform_data;

	if (pdata && pdata->dev_shutdown) {
		pdata->dev_shutdown();
	}
	return 0;
}

static int ssl3252_resume(struct i2c_client *client)
{
	struct ssl3252_platform_data *pdata = client->dev.platform_data;
	
	if (pdata && pdata->dev_startup) {
		pdata->dev_startup();
	}
	return 0;
}
#endif
/*-----------------------------------------------------------------*/
static const struct i2c_device_id ssl3252_i2c_id[] = {
	{ SSL3252_DRV_NAME, 0},
	{ }
};
/*-----------------------------------------------------------------*/
static struct i2c_driver ssl3252_i2c_driver = {
	.id_table = ssl3252_i2c_id,
	.probe	= ssl3252_i2c_probe,
	.remove = ssl3252_i2c_remove,
	//.suspend = ssl3252_suspend,
	//.resume	= ssl3252_resume,
	.driver = {
		.name = SSL3252_DRV_NAME,
	},
};
/*-----------------------------------------------------------------*/
static int __init ssl3252_init(void) 
{
	return i2c_add_driver(&ssl3252_i2c_driver);
}

static void __exit ssl3252_eixt(void)
{
	i2c_del_driver(&ssl3252_i2c_driver);
}
/*-----------------------------------------------------------------*/
module_init(ssl3252_init);
module_exit(ssl3252_eixt);

MODULE_DESCRIPTION("ssl3252 I2C Driver");
MODULE_LICENSE("GPL");
