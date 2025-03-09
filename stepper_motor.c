/*
 * stepper-motor.c - Generic device driver for STEP/DIR stepper motor drivers
 *
 * Copyright (C) 2025 Aeybel varghese
 * License: GPL v2
 */

#define DRIVER_NAME "generic_stepper"

#include <linux/cdev.h>
#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/device/class.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/gfp_types.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "stepper_motor.h"

struct stepper_motor_device
{
    /* Device information  */
    struct device *dev;           // Pointer to paret device
    struct gpio_desc *step_gpiod; // Required gpio line for the STEP wire on a driver
    struct gpio_desc *dir_gpiod;  // Required gpio line for the DIR wire on a driver

    /* char dev interface */
    dev_t devt; // Device number
    struct cdev cdev;
    struct class *class;
    struct mutex lock;

    /* Stepping */
    struct hrtimer timer;
    bool stepping;          // Whether we are presently stepping.
    bool step_pin_state;    // Logical (HIGH/LOW) state of the STEP pin
    unsigned int remaining; // Remaining steps
    ktime_t step_period;    // Interrval between pulses
    stepper_direction direction;
    wait_queue_head_t waitq; // A wait queue for having userspace  programs wait for a completed movement
};

static enum hrtimer_restart stepper_timer_cb(struct hrtimer *timer);

/**
 * @brief Timer callback, toggles the STEP pin ad decrement the remaining steps
 *
 * @param timer htimer object
 * @return error code
 */
static enum hrtimer_restart stepper_timer_cb(struct hrtimer *timer)
{
    struct stepper_motor_device *sd = container_of(timer, struct stepper_motor_device, timer);
    dev_info(sd->dev, "Timer callback\n");

    // If we stopped stepping through a halt or run out remaining steps, we are done
    if (!sd->stepping || (sd->remaining == 0))
    {
        dev_info(sd->dev, "Completed movement\n");
        sd->stepping = false;
        wake_up_interruptible(&sd->waitq);
        // Makes sure STEP is driven low
        if (sd->step_gpiod)
            gpiod_set_value(sd->step_gpiod, 0);

        return HRTIMER_NORESTART;
    }

    // Toggle steps
    sd->step_pin_state = !sd->step_pin_state;
    if (sd->step_gpiod)
        gpiod_set_value(sd->step_gpiod, sd->step_pin_state);

    // Counts step done on rising edge
    if (sd->step_pin_state)
    {
        sd->remaining--;
    }

    // Schedule the next pulse
    hrtimer_forward_now(timer, sd->step_period);
    return HRTIMER_RESTART;
}

/**
 * @brief Function to start a movement given the parametes for the move.
 *
 * @param sd Device private data
 * @param movement destired movement
 *
 * @return error code
 */
static int stepper_move(struct stepper_motor_device *sd, struct stepdrv_move movement)
{
    dev_info(sd->dev, "Performing movement of stepper motor\n");

    // Check if we have the required STEP/DIR lines
    if (!sd->step_gpiod || !sd->dir_gpiod)
    {
        dev_err(sd->dev, "Unable to peform a movement. No STEP/DIR pins configured\n");
        return -ENOTTY;
    }

    // Cancel any ongoing movement
    // TODO: Maybe queue the movement instead?
    sd->stepping = false;
    hrtimer_cancel(&sd->timer);
    sd->remaining = 0;

    // Set direction line
    gpiod_set_value(sd->dir_gpiod, movement.direction ? 1 : 0);
    sd->direction = movement.direction;

    unsigned int speed_hz = movement.speed_hz;
    // Handle the scenario where speed_hz is zero to avoid dividing by zero. Force it to 1hz
    if (movement.speed_hz == 0)
        speed_hz = 1;

    /*
     * One approach:
     *   - We do 1 step per "rising edge."
     *   - Therefore step period = 1 / speed (in seconds).
     *   - If speed=500 steps/sec => period=2ms => 2e6 ns
     */
    unsigned long long period_ns = 1000000000ULL / (speed_hz * 2);
    sd->step_period = ktime_set(0, period_ns);

    // Setup the move
    sd->remaining = movement.steps;
    sd->step_pin_state = false;
    sd->stepping = true;

    // Clear the step pin initially
    gpiod_set_value(sd->step_gpiod, 0);

    // Start the timer
    hrtimer_start(&sd->timer, sd->step_period, HRTIMER_MODE_REL_PINNED);
    // TODO: Handle errors better
    return 0;
}

/**
 * @brief IOCTL handler for the driver
 *
 * @param file file handle for the opened device
 * @param cmd IOCTL cmd code
 * @param arg associated argument for the cmd
 * @return ioctl status code
 */
static long stepper_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct stepper_motor_device *sd = file->private_data;
    long ret = 0;

    mutex_lock(&sd->lock);

    // Handle the respective IOCTL cmd
    switch (cmd)
    {
    case STEPDRV_IOC_MOVE: {
        struct stepdrv_move mv;
        if (copy_from_user(&mv, (void __user *)arg, sizeof(mv)))
        {
            dev_err(sd->dev, "Unable to copy user provided argument to kernel space");
            ret = -EFAULT;
            break;
        }
        stepper_move(sd, mv);
        break;
    }
    case STEPDRV_IOC_STOP: {
        sd->stepping = false;
        sd->remaining = 0;
        hrtimer_cancel(&sd->timer);
        // Set STEP low to be safe
        if (sd->step_gpiod)
            gpiod_set_value(sd->step_gpiod, 0);
        break;
    }
    default:
        ret = -ENOTTY;
        break;
    }

    mutex_unlock(&sd->lock);
    return ret;
}

/**
 * @brief read file_op handler for stepper motor. Does nothing for now
 *
 * @param file  file handle for the opene device
 * @param buf Userspace write back buffer
 * @param count count read
 * @param ppos pos
 * @return bytes
 */
static ssize_t stepper_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct stepper_motor_device *sd = file->private_data;
    // Idk what i could use the read callback for. Maybe to have userspace apps read from the device, and the device
    // blocks until the stepper move operation is done? and if there is no movement in progess, it can return.
    // Doesnt actually return data, so itll just return 0 to inidicate a EOF
    if (sd->stepping)
    {
        wait_event_interruptible(sd->waitq, !sd->stepping);
    }

    return 0;
}

/**
 * @brief open file_op handler for steppe motor.
 *
 * @param inode
 * @param file
 * @return
 */
static int stepper_open(struct inode *inode, struct file *file)
{
    struct stepper_motor_device *sd = container_of(inode->i_cdev, struct stepper_motor_device, cdev);
    file->private_data = sd;
    return 0;
}

/**
 * @brief release fipe handler for stepper motor
 *
 * @param inode
 * @param file
 * @return
 */
static int stepper_release(struct inode *inode, struct file *file)
{
    return 0;
}

/**
 * @brief Linux  file_operations struct with assigned handlers for file operations
 */
static const struct file_operations stepper_fops = {
    .owner = THIS_MODULE,
    .open = stepper_open,
    .release = stepper_release,
    .unlocked_ioctl = stepper_ioctl,
    .read = stepper_read,
};

/**
 * @brief Parses the device entry to get the required GPIO lines
 *
 * @param sd stepper motor device
 * @return error code
 */
static int stepper_parse_gpios(struct stepper_motor_device *sd)
{
    struct device *dev = sd->dev;
    dev_info(dev, "Reading device entry and fetching gpio lines\n");
    /* Gets the "STEP" gpio line entry */
    sd->step_gpiod = devm_gpiod_get(dev, "step", GPIOD_OUT_LOW);
    if (IS_ERR(sd->step_gpiod))
    {
        dev_err(dev, "Failed to  get STEP pin from step-gpio entry\n");
        return PTR_ERR(sd->step_gpiod);
    }

    /* Gets the "DIR" gpio line entry */
    sd->dir_gpiod = devm_gpiod_get(dev, "dir", GPIOD_OUT_LOW);
    if (IS_ERR(sd->dir_gpiod))
    {
        dev_err(dev, "Failed to get DIR pin from dir-gpio entry\n");
        return PTR_ERR(sd->dir_gpiod);
    }

    return 0;
}

/**
 * @brief Creates a character device for a stepper motor
 *
 * @param sd stepper motor device private data
 * @return error code
 */
static int stepper_create_cdev(struct stepper_motor_device *sd)
{
    int ret;

    // Allocate the device number
    ret = alloc_chrdev_region(&sd->devt, 0, 1, "stepper");
    if (ret < 0)
    {
        dev_err(sd->dev, "Allocating a device number failed\n");
        return ret;
    }

    // Initializate and add the cdev
    cdev_init(&sd->cdev, &stepper_fops);
    sd->cdev.owner = THIS_MODULE;

    ret = cdev_add(&sd->cdev, sd->devt, 1);
    if (ret)
    {
        dev_err(sd->dev, "Failed to add chaacter device");
        unregister_chrdev_region(sd->devt, 1);
        return ret;
    }

    // Creates a sysfs class for the device /dev/stepperX
    sd->class = class_create("stepper");
    if (IS_ERR(sd->class))
    {
        dev_err(sd->dev, "Failed to create sysfs class");
        cdev_del(&sd->cdev);
        unregister_chrdev_region(sd->devt, 1);
        return PTR_ERR(sd->class);
    }

    // Creates the device node
    // TODO: Have it be more than one, ie stepperX
    device_create(sd->class, NULL, sd->devt, NULL, "stepper");

    return 0;
}

/**
 * @brief Cleans up and removes the character device
 *
 * @param sd stepper motor private data
 */
static void stepper_destroy_cdev(struct stepper_motor_device *sd)
{
    device_destroy(sd->class, sd->devt);
    class_destroy(sd->class);
    cdev_del(&sd->cdev);
    unregister_chrdev_region(sd->devt, 1);
}

/**
 * @brief Driver probe function
 *
 * @param pdev Platform device handle
 * @return error code
 */
static int stepper_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct stepper_motor_device *sd;
    int ret;

    // Allocates memory for the private data
    sd = devm_kzalloc(dev, sizeof(*sd), GFP_KERNEL);
    if (!sd)
        return -ENOMEM;

    sd->dev = dev;
    platform_set_drvdata(pdev, sd);
    dev_info(sd->dev, "Initialiazing stepper\n");

    // Initializes mutex
    mutex_init(&sd->lock);

    // Parse the device entry for gpio lines
    ret = stepper_parse_gpios(sd);
    if (ret)
        return ret;

    // Init the hrtimer
    sd->stepping = false;
    sd->remaining = 0;
    hrtimer_init(&sd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_PINNED);
    sd->timer.function = stepper_timer_cb;

    // Init the waitqueue
    init_waitqueue_head(&sd->waitq);

    // Create char device node
    dev_info(sd->dev, "Creating char device node\n");
    ret = stepper_create_cdev(sd);
    if (ret)
        return ret;

    dev_info(sd->dev, "Generic stepper driver probed\n");
    return 0;
}

static int stepper_remove(struct platform_device *pdev)
{
    struct stepper_motor_device *sd = platform_get_drvdata(pdev);

    // Clean up any stepping activity
    sd->stepping = false;
    sd->remaining = 0;
    hrtimer_cancel(&sd->timer);
    gpiod_set_value(sd->step_gpiod, 0);
    wake_up_interruptible(&sd->waitq);

    // remove char device
    stepper_destroy_cdev(sd);

    dev_info(&pdev->dev, "Stepper driver removed\n");
    return 0;
}

// of_match table if using device tree
#ifdef CONFIG_OF
static const struct of_device_id stepper_of_match[] = {{.compatible = "generic,stepper-motor"}, {}};
MODULE_DEVICE_TABLE(of, stepper_of_match);
#endif

// TODO: Mayhbe optionally have a ACPI table for x86 platform? idk what i would do though

static struct platform_driver stepper_driver = {
    .probe = stepper_probe,
    .remove = stepper_remove,
    .driver =
        {
            .name = DRIVER_NAME,
            .of_match_table = of_match_ptr(stepper_of_match),
        },
};

module_platform_driver(stepper_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aeybel Varghese");
MODULE_DESCRIPTION("Generic stepper motor device dive for STEP/DIR motor drive IC's");
