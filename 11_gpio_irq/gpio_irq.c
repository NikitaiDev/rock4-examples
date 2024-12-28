#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nick 4 GNU/Linux");
MODULE_DESCRIPTION("A simple LKM for a gpio interrupt"); 

/** variable contains pin number o interrupt controller to which GPIO 11 is mapped to */
static struct gpio_desc *gpio_11;
unsigned int irq_number;

/**
 *  @brief Interrupt service routine is called, when interrupt is triggered
 */
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
	printk("gpio_irq: Interrupt was triggered and ISR was called!\n");
	return (irq_handler_t) IRQ_HANDLED;
}

static int __init ModuleInit(void)
{
	printk("gpio_irq: Loading module\n");

	/* GPIO 11 get */
	if((gpio_11 = gpio_to_desc(146)) == NULL) {
		printk("Can not get GPIO 11!\n");
		return -1;
	}

	/* Set GPIO 11 direction */
	if(gpiod_direction_input(gpio_11) < 0) {
		printk("Can not set GPIO 11 to output!\n");
		gpiod_put(gpio_11);
		return -1;
	}

	/* Setup the interrupt */
	irq_number = gpiod_to_irq(gpio_11);

	if(request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING, "my_gpio_irq", NULL) != 0) { 
		printk("Can not request interrupt nr.: %d\n", irq_number);
		gpiod_put(gpio_11);
		return -1;
	}

	printk("Done!\n");
	printk("GPIO 11 is mapped to IRQ Nr.: %d\n", irq_number);

	return 0;
}

static void __exit ModuleExit(void)
{
	printk("gpio_req: Unloading module...");
	free_irq(irq_number, NULL);
	gpiod_put(gpio_11);
	printk("Goodbye, Kernel!\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

