#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ioctl.h>

 // GPIO pin definitions
#define GPIOSERDEV_STRB_PINID (531) // GPIO19, based on the /sys/kernel/debug/gpio output
#define GPIOSERDEV_DATA_PINID (528) // GPIO16, based on the /sys/kernel/debug/gpio output

// Bit transaction delay
#define GPIOSERDEV_DELAY_US 750

// IOCTL command definitions
#define GPIOSERDEV_IOC_MAGIC 'k'
#define GPIOSERDEV_STRBPIN _IOW(GPIOSERDEV_IOC_MAGIC, 1, int)
#define GPIOSERDEV_DATAPIN _IOW(GPIOSERDEV_IOC_MAGIC, 2, int)

// Device structure
struct gpioserdev_t {
    dev_t devnum;
    struct class *class;
    struct cdev cdev;
};

struct gpioserdev_t gpioserdev;

// Configurable module parameters
static int delay_us = GPIOSERDEV_DELAY_US;
static char byte_order[4] = "lsb";  // Default value

// Function prototypes
static int gpioserdev_open(struct inode *device_file, struct file *instance);
static int gpioserdev_close(struct inode *device_file, struct file *instance);
static ssize_t gpioserdev_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs);
static long gpioserdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int gpioserdev_pinsetup(void);
static void gpioserdev_pinfree(void);

static int gpioserdev_validate_byteorder(const char *val, const struct kernel_param *kp);
static int gpioserdev_get_byteorder(char *buffer, const struct kernel_param *kp);

// File operations
static struct file_operations gpioserdev_fops = {
	.owner = THIS_MODULE,
	.open = gpioserdev_open,
	.release = gpioserdev_close,
	.write = gpioserdev_write,
    .unlocked_ioctl = gpioserdev_ioctl,
};

static const struct kernel_param_ops param_ops = {
    .set = gpioserdev_validate_byteorder,
    .get = gpioserdev_get_byteorder,
};

static int gpioserdev_validate_byteorder(const char *val, const struct kernel_param *kp)
{
    if (strcmp(val, "lsb\n") == 0 || strcmp(val, "msb\n") == 0) {
        strscpy((char *)kp->arg, val, sizeof(byte_order));  // Update value
        return 0;
    }

    pr_err("Invalid value for %s. Allowed values: lsb, msb\n", kp->name);
    return -EINVAL;
}

static int gpioserdev_get_byteorder(char *buffer, const struct kernel_param *kp)
{
    return sprintf(buffer, "%s\n", byte_order);
}

/**
 * Sets up GPIO pins by getting their numbers from the device tree and configuring them as outputs.
 * Handles pin allocation, validation, and initial low-state setup.
 */
static int gpioserdev_pinsetup(void) {

	if(gpio_request(GPIOSERDEV_STRB_PINID, "gpio-strobe")) {
		printk("Allocation failed for gpio-strobe: %d\n", GPIOSERDEV_STRB_PINID);
		return -1;
	}

	if(gpio_direction_output(GPIOSERDEV_STRB_PINID, 0)) {
		printk("Direction set output failed for gpio-strobe\n");
		gpio_free(GPIOSERDEV_STRB_PINID);
		return -1;
	}

	if(gpio_request(GPIOSERDEV_DATA_PINID, "gpio-data")) {
		printk("Allocation failed for gpio-data: %d\n", GPIOSERDEV_DATA_PINID);
		gpio_free(GPIOSERDEV_STRB_PINID);
		return -1;
	}

	if(gpio_direction_output(GPIOSERDEV_DATA_PINID, 0)) {
		printk("Direction set output failed for gpio-data\n");
		gpio_free(GPIOSERDEV_STRB_PINID);
		gpio_free(GPIOSERDEV_DATA_PINID);
		return -1;
	}

	return 0;
}

/**
 * Releases GPIO pins, setting them to low and freeing kernel resources.
 * Ensures clean GPIO state when the module is unloaded.
 */
static void gpioserdev_pinfree(void) {
	gpio_set_value(GPIOSERDEV_STRB_PINID, 0);
	gpio_set_value(GPIOSERDEV_DATA_PINID, 0);
	gpio_free(GPIOSERDEV_STRB_PINID);
	gpio_free(GPIOSERDEV_DATA_PINID);
}

/**
 * Opens the gpioserdev device file.
 */
int gpioserdev_open(struct inode *device_file, struct file *instance) {
	return 0;
}

/**
 * Closes the gpioserdev device file.
 */
int gpioserdev_close(struct inode *device_file, struct file *instance) {
	return 0;
}

// IOCTL handler function
static long gpioserdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long value;

	if(copy_from_user(&value ,(int32_t*) arg, sizeof(value))) {
		printk("data write : Err!\n");
	}

    switch(cmd) {
        case GPIOSERDEV_STRBPIN:
            gpio_set_value(GPIOSERDEV_STRB_PINID, value);
			printk("set strobe_pin value = %ld\n", value);
            break;

        case GPIOSERDEV_DATAPIN:
            gpio_set_value(GPIOSERDEV_DATA_PINID, value);
			printk("set data_pin value = %ld\n", value);
            break;

        default:
            break; // Invalid command
    }
	return 0;
}

/**
 * Writes a byte of data to the GPIO pins.
 *
 * This function takes a byte of data and writes it to the GPIO data pin, one bit
 * at a time. It sets the strobe pin high, waits for the delay_us
 * delay, then sets the strobe pin low again. This sequence is repeated for each
 * bit in the byte, from the least significant bit to the most significant bit.
 *
 * After all 8 bits have been written, the function sets the data pin low.
 */
void gpioserdev_write_byte(char byte) {
	char value;
	int i;

	if (strcmp(byte_order, "lsb") == 0) {
		for (i = 0; i < 8; i++) {
			value = (byte >> i) & 0x01;
			gpio_set_value(GPIOSERDEV_DATA_PINID, value); // set the data
			gpio_set_value(GPIOSERDEV_STRB_PINID, 1); // set strobe 
			udelay(delay_us);
			gpio_set_value(GPIOSERDEV_STRB_PINID, 0); // clear strobe
			udelay(delay_us);
		}
	} else {
		for (i = 7; i >= 0; i--) {
			value = (byte >> i) & 0x01;
			gpio_set_value(GPIOSERDEV_DATA_PINID, value); // set the data
			gpio_set_value(GPIOSERDEV_STRB_PINID, 1); // set strobe 
			udelay(delay_us);
			gpio_set_value(GPIOSERDEV_STRB_PINID, 0); // clear strobe
			udelay(delay_us);
		}
	}
	gpio_set_value(GPIOSERDEV_DATA_PINID, 0);
}

/**
 * Handles write system calls to the device file.
 * Copies user data and calls byte writing function.
 */
ssize_t gpioserdev_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char value;
	
    to_copy = min(count, sizeof(value));
	not_copied = copy_from_user(&value, user_buffer, to_copy);
	gpioserdev_write_byte(value);
	delta = to_copy - not_copied;
	return delta;
}

/**
 * Initialize the gpioserdev module
 */
static int __init gpioserdev_init(void)
{
	if(alloc_chrdev_region(&gpioserdev.devnum, 0, 1, "gpioserdev") < 0) {
		printk("Device number could not be allocated!\n");
		return -1;
	}

	if((gpioserdev.class = class_create("gpioserdev_class")) == NULL) {
		printk("Device class can not be created!\n");
		goto class_cleanup;
	}

	if(device_create(gpioserdev.class, NULL, gpioserdev.devnum, NULL, "gpioserdev") == NULL) {
		printk("Can not create device file!\n");
		goto device_cleanup;
	}

	cdev_init(&gpioserdev.cdev, &gpioserdev_fops);

	if(cdev_add(&gpioserdev.cdev, gpioserdev.devnum, 1) == -1) {
		printk("Registering of device to kernel failed!\n");
		goto cdevadd_cleanup;
	}

	if (gpioserdev_pinsetup() != 0)
		goto gpiopin_cleanup;

    printk("gpioserdev module loaded\n");
    return 0;

gpiopin_cleanup:
	cdev_del(&gpioserdev.cdev);
cdevadd_cleanup:
	device_destroy(gpioserdev.class, gpioserdev.devnum);
device_cleanup:
	class_destroy(gpioserdev.class);
class_cleanup:
	unregister_chrdev_region(gpioserdev.devnum, 1);
	return -1;
}

/**
 * Cleanup and unregister the gpioserdev module
 */
static void __exit gpioserdev_exit(void)
{
	gpioserdev_pinfree();
	cdev_del(&gpioserdev.cdev);
	device_destroy(gpioserdev.class, gpioserdev.devnum);
	class_destroy(gpioserdev.class);
	unregister_chrdev_region(gpioserdev.devnum, 1);
	printk("gpioserdev module removed\n");
}

module_init(gpioserdev_init);
module_exit(gpioserdev_exit);

module_param(delay_us, int, 0644);
MODULE_PARM_DESC(delay_us, "Bit transaction delay in microseconds");

module_param_cb(data_order, &param_ops, byte_order, 0644);
MODULE_PARM_DESC(data_order, "Byte order: lsb or msb");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pravin Raghul S");
MODULE_DESCRIPTION("A Custom GPIO Serial Communication Driver");
