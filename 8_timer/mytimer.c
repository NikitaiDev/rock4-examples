#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nick 4 GNU/Linux");
MODULE_DESCRIPTION("A simple test for LKM's timer");

/* variable for timer */
static struct timer_list my_timer;

static struct gpio_desc *gpio_11;

void timer_callback(struct timer_list * data) {
	gpiod_set_value(gpio_11, 0);
}

static int __init ModuleInit(void)
{
	printk("hello - Hello, Kernel!\n");
	
	/* GPIO 11 get */
	if((gpio_11 =  gpio_to_desc(146)) == NULL) {
		printk("Cat not get GPIO 11!\n");
		return -1;
	}

	/* Set GPIO 11 direction */
	if(gpiod_direction_output(gpio_11, 0) < 0) {
		printk("Cat not set GPIO 11 to output!\n");
		gpiod_put(gpio_11);
		return -1;
	}

	/* Turn LED on*/
	gpiod_set_value(gpio_11, 1);

	/* Initialize timer */
	timer_setup(&my_timer, timer_callback, 0);
	mod_timer(&my_timer, jiffies + msecs_to_jiffies(1000));
	
	return 0;
}

static void __exit ModuleExit(void)
{
	gpiod_put(gpio_11);
	printk("hello - Goodmye, Kernel!\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

