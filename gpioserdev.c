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

#define GPIOSERDEV_DELAY_US 25

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

static int gpioserdev_pinsetup(void);
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
 * gpioserdev_probe - Probe function for the gpioserdev platform device
 *
 * This function is called when the gpioserdev platform device is probed. It performs the following tasks:
 *
 * - Retrieves the GPIO pin IDs for the strobe and data pins from the device tree
 * - Allocates a character device region
 * - Creates a device class and device file
 * - Initializes the character device
 * - Registers the character device with the kernel
 * - Sets up the GPIO pins used by the module
 *
 * If any of these steps fail, the function performs cleanup and returns an error code.
 *
 * @pdev: The platform device structure
 *
 * @return: 0 on success, negative error code on failure
 */
static int gpioserdev_probe(struct platform_device *pdev)
{
  struct device_node *node = pdev->dev.of_node;

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
  if (gpioserdev_pinsetup() != 0)
    goto gpiopin_cleanup;

  printk("gpioserdev probe successful\n");
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
 * gpioserdev_remove - Remove the gpioserdev module
 *
 * This function is called when the gpioserdev module is being unloaded. It performs the necessary
 * cleanup tasks, including:
 *
 * - Freeing the GPIO pins used by the module
 * - Removing the character device from the kernel
 * - Destroying the device file and device class
 * - Unregistering the character device region
 *
 * @pdev: The platform device associated with the gpioserdev module
 *
 * Returns: 0 on success, negative value on failure
 */
static int gpioserdev_remove(struct platform_device *pdev)
{
  gpioserdev_pinfree();
  cdev_del(&gpioserdev.cdev);
  device_destroy(gpioserdev.class, gpioserdev.devnum);
  class_destroy(gpioserdev.class);
  unregister_chrdev_region(gpioserdev.devnum, 1);
  printk("gpioserdev removed\n");
  return 0;
}

/**
 * Initializes and configures the GPIO pins used by the gpioserdev device.
 *
 * This function requests the GPIO pins for the strobe and data signals, sets them
 * to output mode, and initializes them to a low state.
 *
 * Returns 0 on success, or a negative error code on failure.
 */
static int gpioserdev_pinsetup(void) {
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
 * Frees the GPIO pins used by the gpioserdev device.
 *
 * This function sets the strobe and data pins to a low state, and then frees the
 * GPIO pins that were requested in the gpioserdev_pinsetup() function.
 */
static void gpioserdev_pinfree(void) {
  gpio_set_value(gpioserdev.strobe_pin, 0);
  gpio_set_value(gpioserdev.data_pin, 0);
  gpio_free(gpioserdev.strobe_pin);
  gpio_free(gpioserdev.data_pin);
}

/**
 * Opens the gpioserdev device file.
 *
 * This function is called when the gpioserdev device file is opened. It does not
 * perform any additional actions and simply returns 0 to indicate success.
 */
int gpioserdev_open(struct inode *device_file, struct file *instance) {
  return 0;
}

/**
 * Closes the gpioserdev device file.
 *
 * This function is called when the gpioserdev device file is closed. It does not
 * perform any additional actions and simply returns 0 to indicate success.
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
 *
 * @param byte The byte of data to write to the GPIO pins.
 */
void gpioserdev_write_byte(char byte) {
  char value;
  int i;
  for (i = 0; i < 8; i++) {
    value = (byte >> i) & 0x01;
    gpio_set_value(gpioserdev.data_pin, value);
    gpio_set_value(gpioserdev.strobe_pin, 1);
    msleep(delay_us);
    gpio_set_value(gpioserdev.strobe_pin, 0);
    msleep(delay_us);
  }
  gpio_set_value(gpioserdev.data_pin, 0);
}

/**
 * Writes to the gpioserdev device file.
 *
 * This function is called from the write() system call handler for the gpioserdev
 * device. It takes the data from the user-space buffer, copies it to a local
 * variable, and then calls the gpioserdev_write_byte() function to write the
 * byte to the GPIO pins.
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
 * Handles ioctl commands for the gpioserdev device, for testing purposes
 *
 * This function is called when an ioctl system call is made on the gpioserdev
 * device file. It supports two ioctl commands:
 *
 * - GPIOSERDEV_STRBPIN: Sets the value of the strobe pin.
 * - GPIOSERDEV_DATAPIN: Sets the value of the data pin.
 *
 * The function copies the value from the user-provided argument, sets the
 * corresponding GPIO pin.
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
 * Device tree compatible string for the gpioserdev platform device.
 */
static const struct of_device_id gpioserdev_of_match[] = {
  { .compatible = "pravin,gpioserdev" },
  { }
};
MODULE_DEVICE_TABLE(of, gpioserdev_of_match);

/**
 * @brief Platform driver for the gpioserdev device
 *
 */
static struct platform_driver gpioserdev_driver = {
  .probe = gpioserdev_probe,
  .remove = gpioserdev_remove,
  .driver = {
    .name = "gpioserdev",
    .of_match_table = gpioserdev_of_match,
  },
};

/**
 * @brief Initialize the gpioserdev module
 *
 * This function is called when the gpioserdev module is loaded. It first checks if the
 * device tree node "gpioserdev" exists.
 *
 * @return 0 on success, negative value on failure
 */
static int __init gpioserdev_init(void)
{
  struct device_node *node = of_find_node_by_name(NULL, "gpioserdev");
  if (!node) {
    printk("Device tree node 'gpioserdev' not found, module not loaded\n");
    return -ENODEV;
  }

  int ret = platform_driver_register(&gpioserdev_driver);
  if (ret) {
    printk("Failed to register the plaform device\n");
    return ret;
  }

  return 0;
}

/**
 * @brief Cleanup and unregister the gpioserdev module
 *
 * This function is called when the gpioserdev module is unloaded. It unregisters the
 * gpioserdev platform driver.
 */
static void __exit gpioserdev_exit(void)
{
  platform_driver_unregister(&gpioserdev_driver);
}

module_init(gpioserdev_init);
module_exit(gpioserdev_exit);
module_param(delay_us, int, 0644);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pravin Raghul S");
MODULE_DESCRIPTION("A Custom GPIO Serial Communication Driver");
MODULE_PARM_DESC(delay_us, "Delay in microseconds(default: 25us)");
