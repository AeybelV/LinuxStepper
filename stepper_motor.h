/*
 * stepper-motor.h - Generic device driver for STEP/DIR stepper motor drivers. Shared ioctl interface
 *
 * Copyright (C) 2025 Aeybel varghese
 * License: GPL v2
 */

#ifndef _STEPPER_IOCTL_H
#define _STEPPER_IOCTL_H

#include <linux/ioctl.h>

#define STEPDRV_IOC_MAGIC 's'

typedef enum stepper_direction
{
    DIR_LOW,
    DIR_HIGH,
} stepper_direction;

struct stepdrv_move
{
    unsigned int steps;          // Number of steps to move
    unsigned int speed_hz;       // Steps per second
    stepper_direction direction; // Direction to move repesented as logical state
};

#define STEPDRV_IOC_MOVE _IOW(STEPDRV_IOC_MAGIC, 1, struct stepdrv_move)
#define STEPDRV_IOC_STOP _IO(STEPDRV_IOC_MAGIC, 2)

#endif
