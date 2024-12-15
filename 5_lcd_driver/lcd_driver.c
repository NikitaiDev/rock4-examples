#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nick GNU/Linux");
MODULE_DESCRIPTION("A driver to write to a LCD text display");

/* Variables for device and device class */
static dev_t my_device_nr;
static struct class *my_class;
static struct cdev my_cdev;

#define DRIVER_NAME "my_gpio_driver"
#define DRIVER_CLASS "MyModuleClass"

/* LCD char buffer */
static char lcd_buffer[17];

static struct gpio_desc* gpios[10];

static uint8_t gpios_num[10] = {
	40,
	39,
	41,
	42,
	64,
	74,
	73,
	76,
	133,
	158
};
/*	ROCK 4A
	gpio_19,	Enable Pin
	gpio_21,	Register Select Pin
	gpio_23,	Data Pin 0
	gpio_24,	Data Pin 1	
	gpio_27,	Data Pin 2		
	gpio_29,	Data Pin 3		
	gpio_31,	Data Pin 4		
	gpio_33,	Data Pin 5		
	gpio_35,	Data Pin 6		
	gpio_37,	Data Pin 7		
*/

#define ENABLE_PIN gpios[0]
#define REGISTER_SELECT gpios[1]

/**
 * @brief generates a pulse on the enable signal
 */
static void lcd_enable(void) {
	gpiod_set_value(ENABLE_PIN, 1);
	msleep(5);
	gpiod_set_value(ENABLE_PIN, 0);
}

/**
 * @brief set the 8 bit data bus
 * @param data: Data to set 
 */
void lcd_send_byte(char data) {
	int i;
	for(i=0; i<8; i++)
		gpiod_set_value(gpios[i+2], ((data) & (1<<i)) >> i);
	lcd_enable();
	msleep(5);
}

/**
 * @brief send a command to the LCD
 *
 * @param data: command to send
 */
void lcd_command(uint8_t data) {
	gpiod_set_value(REGISTER_SELECT, 0);	/* RS to Instruction */
	lcd_send_byte(data);
}

/**
 * @brief send a command to the LCD
 *
 * @param data: command to send
 */
void lcd_data(uint8_t data) {
	gpiod_set_value(REGISTER_SELECT, 1);	/* RS to data */
	lcd_send_byte(data);
}

/**
 * @brief Write data to buffer
 */
static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta, i;

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(lcd_buffer));

	/* Copy data to user */
	not_copied = copy_from_user(lcd_buffer, user_buffer, to_copy);

	/* Calculate data */
	delta = to_copy - not_copied;

	/* Set the new data to the display */
	lcd_command(0x1);

	for(i=0; i<to_copy; i++)
		lcd_data(lcd_buffer[i]);

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
	.write = driver_write,
};

#define MYMAJOR 91

static int __init ModuleInit(void) {
	int i;
	char text[] = "Hello world!";

	printk("Hello, Kernel!\n");	

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

	/* GPIO get */
	for(i=0;i<10;i++) {
		if((gpios[i] =  gpio_to_desc(gpios_num[i])) == NULL) {
			printk("Cat not get LCD GPIO #%d!\n", gpios_num[i]);
			goto GpioInitError;
		}
	}

	/* Set GPIO direction */
	for(i=0;i<10;i++) {
		if(gpiod_direction_output(gpios[i], 0) < 0) {
			printk("Cat not set GPIO[%d] to output!\n",i);
			goto GpioDirectionError;
		}
	}

	/* Init the display */
	lcd_command(0x30);	/* Set the display for 8 bit data interface */
	
	lcd_command(0xf);	/* Turn display on, turn cursor on, set cursor blinking */

	lcd_command(0x1);
	
	for(i=0; i<sizeof(text)-1;i++)
		lcd_data(text[i]);

	return 0;
GpioDirectionError:
	i=9;
GpioInitError:
	for(;i>=0;i--)
		gpiod_put(gpios[i]);
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
	int i;
	lcd_command(0x1);	/* Clear the display */
	for(i=0;i<10;i++) {
		gpiod_set_value(gpios[i], 0);
		gpiod_put(gpios[i]);
	}
	cdev_del(&my_cdev);
	device_destroy(my_class, my_device_nr);
	class_destroy(my_class);
	unregister_chrdev(my_device_nr, DRIVER_NAME);
	unregister_chrdev(MYMAJOR, "my_dev_nr");
	printk("Goodbye, Kernel!\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

