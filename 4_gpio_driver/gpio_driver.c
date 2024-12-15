#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nick GNU/Linux");
MODULE_DESCRIPTION("A simple gpio driver for setting a LED and reading a button");

/* Buffer for data*/
static size_t buffer_pointer;

/* Variables for device and device class */
static dev_t my_device_nr;
static struct class *my_class;
static struct cdev my_cdev;
static struct gpio_desc *gpio_11;
static struct gpio_desc *gpio_13;

#define DRIVER_NAME "my_gpio_driver"
#define DRIVER_CLASS "MyModuleClass"

/**
 * @brief Read data out of the buffer
 */
static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char tmp[3] = " \n";

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(tmp));

	/* Read value of button */
	printk("Value of button: %d\n", gpiod_get_value(gpio_13));
	tmp[0] = gpiod_get_value(gpio_13) + '0';

	/* Copy data to user */
	not_copied = copy_to_user(user_buffer, &tmp, to_copy);

	/* Calculate data */
	delta = to_copy - not_copied;

	return delta;
}

/**
 * @brief Write data to buffer
 */
static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char value;

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(value));

	/* Copy data to user */
	not_copied = copy_from_user(&value, user_buffer, to_copy);
	buffer_pointer = (size_t)to_copy;

	/* Setting the LED */
	switch(value) {
		case '0':
			gpiod_set_value(gpio_11, 0);
			break;
		case '1':
			gpiod_set_value(gpio_11, 1);
			break;
		default:
			printk("Invalid Input!\n");
			break;
	}

	/* Calculate data */
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
	.write = driver_write,
};

#define MYMAJOR 91

static int __init ModuleInit(void) {
	printk("hello - Hello, Kernel!\n");
	
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

	/* GPIO 11 get */
	if((gpio_11 =  gpio_to_desc(146)) == NULL) {
		printk("Cat not get GPIO 11!\n");
		goto AddError;
	}

	/* Set GPIO 11 direction */
	if(gpiod_direction_output(gpio_11, 0) < 0) {
		printk("Cat not set GPIO 11 to output!\n");
		goto Gpio11Error;
	}
	
	/* GPIO 13 get */
	if((gpio_13 =  gpio_to_desc(150)) == NULL) {
		printk("Cat not get GPIO 13!\n");
		goto Gpio11Error;
	}

	if(gpiod_direction_input(gpio_13) < 0) {
		printk("Cat not set GPIO 13 to input!\n");
		goto Gpio13Error;
	}
		
	return 0;
Gpio13Error:
	gpiod_put(gpio_13);
Gpio11Error:
	gpiod_put(gpio_11);
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
	gpiod_set_value(gpio_11, 0);
	gpiod_put(gpio_11);
	gpiod_put(gpio_13);
	cdev_del(&my_cdev);
	device_destroy(my_class, my_device_nr);
	class_destroy(my_class);
	unregister_chrdev(my_device_nr, DRIVER_NAME);
	unregister_chrdev(MYMAJOR, "my_dev_nr");
	printk("Goodbye, Kernel!\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

