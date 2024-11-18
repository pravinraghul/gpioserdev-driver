#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#define GPIOSERDEV_DELAY_US (500) // data was reliable in 200 us as well

#define GPIOSERDEV_IOC_MAGIC 0x15
#define GPIOSERDEV_STRBPIN _IOWR(GPIOSERDEV_IOC_MAGIC, 1, unsigned long)
#define GPIOSERDEV_DATAPIN _IOWR(GPIOSERDEV_IOC_MAGIC, 2, unsigned long)

// Device structure
struct gpioserdev_t {
  dev_t devnum;
  struct class *class;
  struct cdev cdev;
  int strobe_pin;
  int data_pin;
};

struct gpioserdev_t gpioserdev;

// Function prototypes
static int gpioserdev_open(struct inode *device_file, struct file *instance);
static int gpioserdev_close(struct inode *device_file, struct file *instance);
static ssize_t gpioserdev_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs);
static long gpioserdev_ioctl(struct file *File, unsigned int cmd, unsigned long arg);

static int gpioserdev_pinsetup(struct device_node *node);
static void gpioserdev_pinfree(void);

static int delay_us = GPIOSERDEV_DELAY_US;

static struct file_operations gpioserdev_fops = {
  .owner = THIS_MODULE,
  .open = gpioserdev_open,
  .release = gpioserdev_close,
  .write = gpioserdev_write,
  .unlocked_ioctl = gpioserdev_ioctl
};

/**
 * Sets up GPIO pins by getting their numbers from the device tree and configuring them as outputs.
 * Handles pin allocation, validation, and initial low-state setup.
 */
static int gpioserdev_pinsetup(struct device_node *node) {
  // Get GPIO pins from device tree
  gpioserdev.strobe_pin = of_get_named_gpio(node, "strobe-pin", 0);
  if (!gpio_is_valid(gpioserdev.strobe_pin)) {
    printk("Failed to get strobe-pin from DT\n");
    return -EINVAL;
  }

  gpioserdev.data_pin = of_get_named_gpio(node, "data-pin", 0);
  if (!gpio_is_valid(gpioserdev.data_pin)) {
    printk("Failed to get data-pin from DT\n");
    return -EINVAL;
  }

  if(gpio_request(gpioserdev.strobe_pin, "gpio-strobe")) {
    printk("Allocation failed for gpio-strobe: %d\n", gpioserdev.strobe_pin);
    return -1;
  }

  if(gpio_direction_output(gpioserdev.strobe_pin, 0)) {
    printk("Direction set output failed for gpio-strobe\n");
    gpio_free(gpioserdev.strobe_pin);
    return -1;
  }

  if(gpio_request(gpioserdev.data_pin, "gpio-data")) {
    printk("Allocation failed for gpio-data: %d\n", gpioserdev.data_pin);
    gpio_free(gpioserdev.strobe_pin);
    return -1;
  }

  if(gpio_direction_output(gpioserdev.data_pin, 0)) {
    printk("Direction set output failed for gpio-data\n");
    gpio_free(gpioserdev.strobe_pin);
    gpio_free(gpioserdev.data_pin);
    return -1;
  }

  return 0;
}

/**
 * Releases GPIO pins, setting them to low and freeing kernel resources.
 * Ensures clean GPIO state when the module is unloaded.
 */
static void gpioserdev_pinfree(void) {
  gpio_set_value(gpioserdev.strobe_pin, 0);
  gpio_set_value(gpioserdev.data_pin, 0);
  gpio_free(gpioserdev.strobe_pin);
  gpio_free(gpioserdev.data_pin);
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
  for (i = 0; i < 8; i++) {
    value = (byte >> i) & 0x01;
    gpio_set_value(gpioserdev.data_pin, value);
    gpio_set_value(gpioserdev.strobe_pin, 1);
    udelay(delay_us);
    gpio_set_value(gpioserdev.strobe_pin, 0);
    udelay(delay_us);
  }
  gpio_set_value(gpioserdev.data_pin, 0);
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
 * Processes ioctl commands for manually setting strobe or data pin values.
 * Supports direct GPIO pin control for testing and debugging.
 */
long gpioserdev_ioctl(struct file *File, unsigned int cmd, unsigned long arg) {
  unsigned long value;

  if(copy_from_user(&value ,(int32_t*) arg, sizeof(value))) {
    printk("data write : Err!\n");
  }

  switch(cmd) {
  case GPIOSERDEV_STRBPIN:
    gpio_set_value(gpioserdev.strobe_pin, value);
    printk("set strobe_pin value = %ld\n", value);
    break;
  case GPIOSERDEV_DATAPIN:
    gpio_set_value(gpioserdev.data_pin, value);
    printk("set data_pin value = %ld\n", value);
    break;
  default:
    printk("default\n");
    break;
  }
  return 0;
}

/**
 * Initialize the gpioserdev module
 */
static int __init gpioserdev_init(void)
{
  struct device_node *node = of_find_node_by_name(NULL, "gpioserdev");
  if (!node) {
    printk("Device tree node 'gpioserdev' not found, module not loaded\n");
    return -ENODEV;
  }

  // Initialize the character device
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

  // Setup GPIO pins
  if (gpioserdev_pinsetup(node) != 0)
    goto gpiopin_cleanup;

  printk("gpioserdev initialized successfully\n");
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
module_param(delay_us, int, 0776);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pravin Raghul S");
MODULE_DESCRIPTION("A Custom GPIO Serial Communication Driver");
MODULE_PARM_DESC(delay_us, "Delay in microseconds(default: 500us)");
