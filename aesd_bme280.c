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
#include <linux/jiffies.h>
#include <linux/timer.h>

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
static struct cdev aesd_bme280_cdev;

static struct timer_list aesd_bme280_timer;

/* timer callback function before taking measurements */
static void aesd_bme280_callback(struct timer_list *timer);

/* bme280 sensor data */
static struct bme280_sensor_data_s
{
	struct bme280_comp_data{
		u16 dig_T1;
		s16 dig_T2;
		s16 dig_T3;
		u16 dig_P1;
		s16 dig_P2;
		s16 dig_P3;
		s16 dig_P4;
		s16 dig_P5;
		s16 dig_P6;
		s16 dig_P7;
		s16 dig_P8;
		s16 dig_P9;
		u8  dig_H1;
		s16 dig_H2;
		u8  dig_H3;
		s16 dig_H4;
		s16 dig_H5;
		s8  dig_H6;
	} comp_data;
	int32_t temperature;
	uint32_t pressure;
	uint32_t humidity;
}aesd_bme280_sensor_data;


/* fops function declarations */
static int aesd_bme_open(struct inode *inode, struct file *file);
static int aesd_bme_release(struct inode *inode, struct file *file);
static ssize_t aesd_bme_read(struct file *filp, char __user *buf, size_t len,
			     loff_t *off);
static ssize_t aesd_bme_write(struct file *filp, const char *buf, size_t len,
			      loff_t *off);

/* bme280 function declarations */
static int aesd_initialize_bm280_chip(void);
static int aesd_read_comp_data(void);
static int aesd_start_measurement(void);
static int aesd_read_sensor_data(void);

/* Compensation functions from Bosch BME280 data sheet */
static int32_t BME280_compensate_T_int32(int32_t adc_T, int32_t *t_fine);
static uint32_t BME280_compensate_P_int64(int32_t adc_P, int32_t t_fine);
static uint32_t BME280_compensate_H_int32(int32_t adc_H, int32_t t_fine);

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
	pr_info("writing the device file of aesd_bme280 not supported.\n");
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
	cdev_init(&aesd_bme280_cdev, &fops);

	/* create a character device */
	if ((cdev_add(&aesd_bme280_cdev, aesd_bme280_dev, 1)) < 0) {
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

	
	if(aesd_initialize_bm280_chip() != 0) {
		pr_err("aesd_initialize_bm280_chip() failed");
		ret = -1;
	}

	timer_setup(&aesd_bme280_timer, aesd_bme280_callback, 0);
	mod_timer(&aesd_bme280_timer, jiffies + msecs_to_jiffies(1000));
	aesd_start_measurement();

	pr_info("aesd bme280 module initialized\n");
	return ret;
}

/* module exit function */
static void __exit aesd_bme_driver_exit(void)
{
	del_timer(&aesd_bme280_timer);
	i2c_unregister_device(aesd_bme_i2c_client);
	i2c_del_driver(&aesd_bmp280_driver);
	device_destroy(dev_class, aesd_bme280_dev);
	class_destroy(dev_class);
	cdev_del(&aesd_bme280_cdev);
	unregister_chrdev_region(aesd_bme280_dev, 1);
	pr_info("aesd bme280 module removed");
}

/*
 * BME 280 functions
 */

static int aesd_initialize_bm280_chip(void)
{ 
	int8_t chip_id;
	pr_info("aesd_initialize_bm280_chip called.");
	
	//int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
	if(i2c_reg_read(I2C_SLAVE_ADDR, 0xD0, &chip_id, 1) != 0)
	{
		pr_info("reading chip id for bme280 failed.");
	}

	// check chip_id just in case
	if(chip_id != 0x60)
	{
		pr_info("reading wrong chip_id for bme280");
	}
	else
	{
		pr_info("reading correct chip_id for bme280");
	}

	if(aesd_read_comp_data() != 0)
	{
		pr_err("reading compensation data failed");
		return 1;
	}

	return 0;
}

static int aesd_read_comp_data(void)
{
	uint8_t register_contents[33];
	if(i2c_reg_read(I2C_SLAVE_ADDR, 0x88, register_contents, 33) != 0)
	{
		pr_err("aesd_read_comp_data() failed.");
	}
	aesd_bme280_sensor_data.comp_data.dig_T1 = (register_contents[1] << 8) | register_contents[0];
	aesd_bme280_sensor_data.comp_data.dig_T2 = (register_contents[3] << 8) | register_contents[2];
	aesd_bme280_sensor_data.comp_data.dig_T3 = (register_contents[5] << 8) | register_contents[4];

	aesd_bme280_sensor_data.comp_data.dig_P1 = (register_contents[7] << 8) | register_contents[6];
	aesd_bme280_sensor_data.comp_data.dig_P2 = (register_contents[9] << 8) | register_contents[8];
	aesd_bme280_sensor_data.comp_data.dig_P3 = (register_contents[11] << 8) | register_contents[10];
	aesd_bme280_sensor_data.comp_data.dig_P4 = (register_contents[13] << 8) | register_contents[12];
	aesd_bme280_sensor_data.comp_data.dig_P5 = (register_contents[15] << 8) | register_contents[14];
	aesd_bme280_sensor_data.comp_data.dig_P6 = (register_contents[17] << 8) | register_contents[16];
	aesd_bme280_sensor_data.comp_data.dig_P7 = (register_contents[19] << 8) | register_contents[18];
	aesd_bme280_sensor_data.comp_data.dig_P8 = (register_contents[21] << 8) | register_contents[20];
	aesd_bme280_sensor_data.comp_data.dig_P9 = (register_contents[23] << 8) | register_contents[22];
	
	aesd_bme280_sensor_data.comp_data.dig_H1 = register_contents[24];
	aesd_bme280_sensor_data.comp_data.dig_H2 = (register_contents[26] << 8) | register_contents[25];
	aesd_bme280_sensor_data.comp_data.dig_H3 = register_contents[27];
	aesd_bme280_sensor_data.comp_data.dig_H4 = (register_contents[29] << 8) | register_contents[28];
	aesd_bme280_sensor_data.comp_data.dig_H5 = (register_contents[31] << 8) | register_contents[30];
	aesd_bme280_sensor_data.comp_data.dig_H6 = register_contents[32];
	
	pr_info("aesd_read_comp_data() finished");
	return 0;
}

static int aesd_start_measurement(void)
{
	uint8_t reg_val;
	reg_val = 1; // osrs_h[2:0] oversampling 1x for humidity (recomended value for weather monitoring)
	if(i2c_reg_write(I2C_SLAVE_ADDR, 0xF2, &reg_val, 1) != 0)
	{
		pr_err("aesd_start_measurement: writing the ctrl_hum register failed");
		return -1;
	}
	reg_val = 0x25;	// ctrl_meas register: 1x temp oversampling, 1x pressure oversampling, forced mode
	if(i2c_reg_write(I2C_SLAVE_ADDR, 0xF4, &reg_val, 1) != 0)
	{
		pr_err("aesd_start_measurement: writing the ctrl_hum register failed");
		return -1;
	}
	return 0;
}

static int aesd_read_sensor_data(void)
{
	// Data readout is done by starting a burst read from 0xF7 to 0xFC (temperature and pressure) or
	// from 0xF7 to 0xFE (temperature, pressure and humidity). The data are read out in an unsigned
	// 20-bit format both for pressure and for temperature and in an unsigned 16-bit format for
	// humidity. It is strongly recommended to use the BME280 API, available from Bosch Sensortec,
	// for readout and compensation. For details on memory map and interfaces, please consult
	// chapters 5 and 6 respectively.
	uint8_t sensor_data[8];
	int32_t T_fine;	// for compensation acc. to data sheet
	int32_t adc_P;
	int32_t adc_T;
	int32_t adc_H;

	pr_info("aesd_read_sensor_data");
	
	if(i2c_reg_read(I2C_SLAVE_ADDR, 0xF7, sensor_data, 8)!= 0)
	{
		pr_err("Error reading sensor data");
		return -1;
	}

	adc_P = (sensor_data[0]<<12) | (sensor_data[1] << 4) | (sensor_data[2] >> 4);
	adc_T = (sensor_data[3]<<12) | (sensor_data[4] << 4) | (sensor_data[5] >> 4);
	adc_H = (sensor_data[6]<<8) | (sensor_data[7]) ;

	aesd_bme280_sensor_data.temperature = BME280_compensate_T_int32(adc_T, &T_fine);
	aesd_bme280_sensor_data.pressure = BME280_compensate_P_int64(adc_P, T_fine);
	aesd_bme280_sensor_data.humidity = BME280_compensate_H_int32(adc_H, T_fine);

	pr_info("Read sensor data: %i, P: %u, H: %u", aesd_bme280_sensor_data.temperature, aesd_bme280_sensor_data.pressure, aesd_bme280_sensor_data.humidity);

	return 0;
}

/* Compensation functions from Bosch BME280 data sheet */

/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. */
static int32_t BME280_compensate_T_int32(int32_t adc_T, int32_t *t_fine)
{
	struct bme280_comp_data *c = &aesd_bme280_sensor_data.comp_data;
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)c->dig_T1<<1))) * ((int32_t)c->dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)c->dig_T1)) * ((adc_T>>4) - ((int32_t)c->dig_T1))) >> 12) *
	((int32_t)c->dig_T3)) >> 14;
	*t_fine = var1 + var2;
	T = (*t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t BME280_compensate_P_int64(int32_t adc_P, int32_t t_fine)
{
	int64_t var1, var2, p;
	struct bme280_comp_data *c = &aesd_bme280_sensor_data.comp_data;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)c->dig_P6;
	var2 = var2 + ((var1*(int64_t)c->dig_P5)<<17);
	var2 = var2 + (((int64_t)c->dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)c->dig_P3)>>8) + ((var1 * (int64_t)c->dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)c->dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)c->dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)c->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)c->dig_P7)<<4);
	return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t BME280_compensate_H_int32(int32_t adc_H, int32_t t_fine)
{
	int32_t v_x1_u32r;
	struct bme280_comp_data *c = &aesd_bme280_sensor_data.comp_data;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)c->dig_H4) << 20) - (((int32_t)c->dig_H5) * v_x1_u32r)) +
	((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)c->dig_H6)) >> 10) * (((v_x1_u32r *
	((int32_t)c->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)c->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)c->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

static void aesd_bme280_callback(struct timer_list *timer)
{
	pr_info("aesd_bme280_callback");
	if(aesd_read_sensor_data() != 0)
	{
		pr_err("Error while reading sensor data");
	}
	mod_timer(&aesd_bme280_timer, jiffies + msecs_to_jiffies(1000));
}

module_init(aesd_bme_driver_init);
module_exit(aesd_bme_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Eichinger");
MODULE_DESCRIPTION("Kernel module for the Bosch BMP280 - Coursera AESD course");
MODULE_VERSION("1.0");
