#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define DRIVER_NAME "bmp180"
#define DRIVER_CLASS "bmp180Class"

static struct i2c_adapter *bmp180_i2c_adapter = NULL;
static struct i2c_client *bmp180_i2c_client = NULL;

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nick GNU/Linux");
MODULE_DESCRIPTION("A simple driver to access the Hardware PWM IP");

/* Defines for device identification */
#define I2C_BUS_AVAILABLE	2		/* The I2C Bus available on the rock4A */
#define SLAVE_DEVICE_NAME	"BMP180"	/* Device and Driver Name */
#define BMP180_SLAVE_ADDRESS	0x77		/* BMP180 I2C address */

static const struct i2c_device_id bmp180_id[] = {
	{ SLAVE_DEVICE_NAME, 0 },
	{ }
};


static struct i2c_driver bmp180_driver = {
	.driver = {
		.name = SLAVE_DEVICE_NAME,
		.owner = THIS_MODULE
	}
};

static struct i2c_board_info bmp180_i2c_board_info = {
	I2C_BOARD_INFO(SLAVE_DEVICE_NAME, BMP180_SLAVE_ADDRESS)
};

/* Variables for device and device class */
static dev_t my_device_nr;
static struct class *my_class;
static struct cdev my_cdev;

static struct bmp180_data_t {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
	long B5;
	short oss;
} bmp180_data;

static long read_ut(void)
{
	long UT;
	i2c_smbus_write_byte_data(bmp180_i2c_client, 0xF4, 0x2E);
	msleep(4.5);
	UT = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF6) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF7);
	return UT;
}

static long read_up(void)
{
	long UP;
	bmp180_data.oss = 0;	/* oversam pling_setting (ultra low power mode) */
	i2c_smbus_write_byte_data(bmp180_i2c_client, 0xF4, 0x34 + (bmp180_data.oss<<6));
	msleep(4.5);
	UP = ((i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF6) << 16) + (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF7) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF7))>>(8-bmp180_data.oss);
	return UP;

}

static long read_temperature(void)
{
	long X1, X2, T, UT;
	UT = read_ut();
	X1 = (UT-bmp180_data.AC6) * bmp180_data.AC5 / (1<<15);
	X2 = bmp180_data.MC * (1<<11) / (X1 +bmp180_data.MD);
	bmp180_data.B5 = X1 + X2;
	T = (bmp180_data.B5 + 8) / (1<<4);
	return T;	
}

static long read_pressure(void)
{
	long B6,X1,X2,X3,B3,B4,B7,P,UP;
	UP = read_up();
	B6 = bmp180_data.B5 - 4000;
	X1 = (bmp180_data.B2 * (B6 * B6 / (1<<12)))/(1<<11);
	X2 = bmp180_data.AC2 * B6 / (1<<11);
	X3 = X1 + X2;
	B3 = (((bmp180_data.AC1 * 4 + X3)<<bmp180_data.oss) + 2) / 4;
	X1 = bmp180_data.AC3 * B6 / (1<<13);
	X2 = (bmp180_data.B1 * (B6 * B6 / (1<<12)))/(1<<16);
	X3 = ((X1 + X2) + 2) / 4;
	B4 = bmp180_data.AC4 * (unsigned long)(X3 + 32768) / (1<<15);
	B7 = ((unsigned long)UP - B3) * (50000>>bmp180_data.oss);
	if (B7 < 0x80000000) {
		P = (B7 * 2) / B4;
	}
	else {
		P = (B7 / B4) * 2;
	}
	X1 = (P / (1<<8)) * (P / (1<<8));
	X1 = (X1 * 3038) / (1<<16);
	X2 = (-7357 * P) / (1<<16);
	P = P + (X1 + X2 + 3791) / (1<<4);
	return P;
}

/**
 * @brief Get data out of buffer
 */
static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char out_string[40];
	long temperature, pressure;

	/* Get amount of bytes to copy */
	to_copy = min(sizeof(out_string), count);

	/* Get temperature and pressure*/
	pressure = read_pressure();
	temperature = read_temperature();
	snprintf(out_string, sizeof(out_string), "%ld.%ld , %ld\n", temperature/10, temperature%10, pressure);

	/* Copy Data to user */
	not_copied = copy_to_user(user_buffer, out_string, to_copy);

	/* Calculate delta */
	delta = to_copy - not_copied;

	return delta;
}

static int driver_open(struct inode *device_file, struct file *instance) {
	printk("dev_nr - open was called!\n");
	return 0;
}

static int driver_close(struct inode *device_file, struct file *instance) {
	printk("dev_nr - close was called!\n");
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.release = driver_close,
	.read = driver_read,
};

#define MYMAJOR 91

static int __init ModuleInit(void) {
	int ret = -1;
	u8 id;

	printk("MyDeviceDriver - Hello, Kernel!\n");
	
	/* Allocate a device nr */
	if( alloc_chrdev_region(&my_device_nr, 0, 1, DRIVER_NAME) < 0 ) {
		printk("Device Nr. could not be allocated!\n");
		return -1;
	}
	printk("read_write - Device Nr.Major %d, Minor: %d was registered!\n", my_device_nr >> 20, my_device_nr && 0xfffff);

	/* Create device class */
	if((my_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
		printk("Device class cat not e created!\n");
		goto ClassError;
	}
	
	/* create drvice file */
	if(device_create(my_class, NULL, my_device_nr, NULL, DRIVER_NAME) == NULL) {
		printk("Can not create device file!\n");
		goto FileError;
	}

	/* Initialize device file */
	cdev_init(&my_cdev, &fops);

	/* Registering device to kernel */
	if(cdev_add(&my_cdev, my_device_nr, 1) == -1) {
		printk("Registering of device to kernel failed!\n");
		goto AddError;
	}
	
	bmp180_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);
	
	if (bmp180_i2c_adapter != NULL) {
		bmp180_i2c_client = i2c_new_client_device(bmp180_i2c_adapter, &bmp180_i2c_board_info);
		if(bmp180_i2c_client != NULL) {
			if(i2c_add_driver(&bmp180_driver) != -1) {
				ret = 0;
			}
			else 
				printk("Can't add driver...\n");
		}
	}

	printk("BMP180 Driver added!\n");

	/* Read ID */
	id = i2c_smbus_read_byte_data(bmp180_i2c_client, 0xD0);
	printk("ID: 0x%x\n", id);

	bmp180_data.AC1 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAA) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAB);
	bmp180_data.AC2 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAC) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAD);
	bmp180_data.AC3 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAE) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAF);
	bmp180_data.AC4 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB0) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB1);
	bmp180_data.AC5 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB2) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB3);
	bmp180_data.AC6 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB4) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB5);
	bmp180_data.B1 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB6) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB7);
	bmp180_data.B2 = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB8) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xB9);
	bmp180_data.MB = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xBA) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xBB);
	bmp180_data.MC = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xBC) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xBD);
	bmp180_data.MD = (i2c_smbus_read_byte_data(bmp180_i2c_client, 0xBE) << 8) + i2c_smbus_read_byte_data(bmp180_i2c_client, 0xBF);
	return 0;
AddError:
	device_destroy(my_class, my_device_nr);
FileError:
	class_destroy(my_class);
ClassError:
	unregister_chrdev(my_device_nr, DRIVER_NAME);
	return -1;
}

static void __exit ModuleExit(void)
{
	i2c_unregister_device(bmp180_i2c_client);
	i2c_del_driver(&bmp180_driver);
	cdev_del(&my_cdev);
	device_destroy(my_class, my_device_nr);
	class_destroy(my_class);
	unregister_chrdev(my_device_nr, DRIVER_NAME);
	unregister_chrdev(MYMAJOR, "my_dev_nr");
	printk("Goodbye, Kernel!\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

