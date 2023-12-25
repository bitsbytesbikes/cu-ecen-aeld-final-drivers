#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#define I2C_BUS_AVAILABLE (1)
#define AESD_DEVICE_NAME ("aesd_bm280")
#define I2C_SLAVE_ADDR (0x76)

static struct i2c_adapter *aesd_bme_i2c_adapter = NULL;
static struct i2c_client *aesd_bme_i2c_client = NULL;

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		     uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		    uint16_t length);

static int __init aesd_bme_driver_init(void);
static void __exit aesd_bme_driver_exit(void);

static dev_t aesd_bme280_dev = 0;
static struct class *dev_class;
static struct cdev aesd_bme_cdev;


/* fops function declarations */
static int aesd_bme_open(struct inode *inode, struct file *file);
static int aesd_bme_release(struct inode *inode, struct file *file);
static ssize_t aesd_bme_read(struct file *filp, char __user *buf, size_t len,
			     loff_t *off);
static ssize_t aesd_bme_write(struct file *filp, const char *buf, size_t len,
			      loff_t *off);

/* fops structure */
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aesd_bme_read,
	.write = aesd_bme_write,
	.open = aesd_bme_open,
	.release = aesd_bme_release,
};

static int aesd_bme_open(struct inode *inode, struct file *file)
{
	pr_info("aesd_bme_open called\n");
	return 0;
}

static int aesd_bme_release(struct inode *inode, struct file *file)
{
	pr_info("aesd_bme_release called\n");
	return 0;
}

static ssize_t aesd_bme_read(struct file *filp, char __user *buf, size_t len,
			     loff_t *off)
{
	pr_info("aesd_bme_read called\n");
	return 0;
}

static ssize_t aesd_bme_write(struct file *filp, const char __user *buf,
			      size_t len, loff_t *off)
{
	pr_info("Wrting the device file of aesd_bme280 not supported.\n");
	return 0;
}


/*
 *  Function for writing the sensor's registers through I2C bus.
 *
 *  returns 0 on success
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		     uint16_t length)
{
	uint8_t tx_data[100] = { 0 };
	uint16_t i = 0;
	int ret;

	tx_data[0] = reg_addr;
	for (i = 0; i < length; i++)
		tx_data[i + 1] = reg_data[i];
	ret = i2c_master_send(aesd_bme_i2c_client, tx_data, length + 1);

	if (ret >= 0)
		return 0;
	else
		return ret;
}

/*
 * Function for reading the sensor's registers through I2C bus.
 *
 * returns 0 on success
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		    uint16_t length)
{
	/* Implement the I2C read routine according to the target machine. */
	int ret = i2c_master_send(aesd_bme_i2c_client, &reg_addr, 1);
	ret |= i2c_master_recv(aesd_bme_i2c_client, reg_data, length);

	if (ret >= 0)
		return 0;
	else
		return ret;
}

/* this function is called once on driver load */
static int aesd_bme280_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	//TODO: Configure sensor
	//		- set power mode
	//		- read calibration data?

	pr_info("aesd_bme280_probe called\n");

	return 0;
}

/* this function is called once on driver removal */
static void aesd_bme280_remove(struct i2c_client *client)
{
	// TODO: put sensor in sleep mode
	pr_info("aesd_bme280_remove called\n");
}

/* supported slave ids */
static const struct i2c_device_id aesd_bme280_id[] = { { AESD_DEVICE_NAME, 0 },
						       {} };
MODULE_DEVICE_TABLE(i2c, aesd_bme280_id);

/* i2c platform driver struct */
static struct i2c_driver aesd_bmp280_driver = {
        .driver = {
            .name   = AESD_DEVICE_NAME,
            .owner  = THIS_MODULE,
        },
        .probe          = aesd_bme280_probe,
        .remove         = aesd_bme280_remove,
        .id_table       = aesd_bme280_id,
};

/* board info */
static struct i2c_board_info bme_i2c_board_info = { I2C_BOARD_INFO(
	AESD_DEVICE_NAME, I2C_SLAVE_ADDR) };

/* module initialization */
static int __init aesd_bme_driver_init(void)
{
	int ret = -1;

	/* allocate a major number*/
	if ((alloc_chrdev_region(&aesd_bme280_dev, 0, 1, "aesd_bme_dev")) < 0) {
		pr_err("Error during allocation of  major number\n");
		return -1;
	}

	/* create cdev structure*/
	cdev_init(&aesd_bme_cdev, &fops);

	/* create a character device */
	if ((cdev_add(&aesd_bme_cdev, aesd_bme280_dev, 1)) < 0) {
		pr_err("Error while adding character device for aesd_bme280\n");
		return -1;
	}

	/* create device classe */
	if ((dev_class = class_create(THIS_MODULE, "aesd_bme_class")) == NULL) {
		pr_err("Error while creating the device class\n");
		return -1;
	}

	/* create device*/
	if ((device_create(dev_class, NULL, aesd_bme280_dev, NULL,
			   "aesd_bme_device")) == NULL) {
		pr_err("Error while creating device for aesd_bme280");
		return -1;
	}
	aesd_bme_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

	if (aesd_bme_i2c_adapter != NULL) {
		pr_info("i2c_adapter received for aesd_bme280\n");
		aesd_bme_i2c_client = i2c_new_client_device(
			aesd_bme_i2c_adapter, &bme_i2c_board_info);

		if (aesd_bme_i2c_client != NULL) {
			pr_info("i2c_client device created for aesd_bme280\n");
			i2c_add_driver(&aesd_bmp280_driver);
			ret = 0;
		}

		i2c_put_adapter(aesd_bme_i2c_adapter);
	}

	pr_info("aesd bme280 module initialized\n");
	return ret;
}

/* module exit function */
static void __exit aesd_bme_driver_exit(void)
{
	i2c_unregister_device(aesd_bme_i2c_client);
	i2c_del_driver(&aesd_bmp280_driver);
	device_destroy(dev_class, aesd_bme280_dev);
	class_destroy(dev_class);
	cdev_del(&aesd_bme_cdev);
	unregister_chrdev_region(aesd_bme280_dev, 1);
	pr_info("aesd bme280 module removed");
}

module_init(aesd_bme_driver_init);
module_exit(aesd_bme_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Eichinger");
MODULE_DESCRIPTION("Kernel module for the Bosch BMP280 - Coursera AESD course");
MODULE_VERSION("1.0");
