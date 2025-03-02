/*
 * example_move.c
 *
 * Example user-space test program for the generic stepper driver.
 *
 * Compile with something like:
 *    gcc -Wall -o example_move example_move.c
 *
 * Usage:
 *    ./example_move /dev/stepper0 move <steps> <speed_hz> <dir>
 *    ./example_move /dev/stepper0 stop
 *
 * If you call 'move', the program will then do a blocking read()
 * so it won't exit until the move is finished (assuming the driver
 * blocks in .read when steps remain).
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* Include our driver’s ioctl definitions */
#include "stepper_motor.h"

static void usage(const char *prog)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s <device> stop\n", prog);
    fprintf(stderr, "  %s <device> move <steps> <speed_hz> <dir>\n", prog);
    fprintf(stderr, "    - steps: number of steps (unsigned int)\n");
    fprintf(stderr, "    - speed_hz: steps per second (unsigned int)\n");
    fprintf(stderr, "    - dir: 0 or 1 (for DIR line low or high)\n");
}

int main(int argc, char *argv[])
{
    int fd;
    const char *devpath;
    int ret;

    if (argc < 3)
    {
        usage(argv[0]);
        return 1;
    }

    devpath = argv[1];

    /* Open the stepper device */
    fd = open(devpath, O_RDWR);
    if (fd < 0)
    {
        perror("Unable to open stepper motor device file");
        return 1;
    }

    /* Parse the command */
    if (strcmp(argv[2], "stop") == 0)
    {
        /* Stop any ongoing motion */
        ret = ioctl(fd, STEPDRV_IOC_STOP);
        if (ret < 0)
        {
            perror("ioctl STOP failed");
            close(fd);
            return 1;
        }
        printf("Motion stopped.\n");
    }
    else if (strcmp(argv[2], "move") == 0)
    {
        /* Move command requires 3 extra args: <steps> <speed_hz> <dir> */
        if (argc < 6)
        {
            usage(argv[0]);
            close(fd);
            return 1;
        }
        struct stepdrv_move mv;
        mv.steps = atoi(argv[3]);
        mv.speed_hz = atoi(argv[4]);
        mv.direction = atoi(argv[5]); /* 0 or 1 */

        ret = ioctl(fd, STEPDRV_IOC_MOVE, &mv);
        if (ret < 0)
        {
            perror("ioctl MOVE failed");
            close(fd);
            return 1;
        }
        printf("Issued move: %u steps @ %u steps/sec dir=%d\n", mv.steps, mv.speed_hz, mv.direction);

        /*
         * The driver’s .read() method blocks until the current move completes.
         * We'll do a dummy read so that the program doesn’t exit
         * until the move finishes.
         */
        char dummy;
        ssize_t n = read(fd, &dummy, 1);
        if (n < 0)
        {
            perror("read failed");
            close(fd);
            return 1;
        }
        printf("Move completed!\n");
    }
    else
    {
        fprintf(stderr, "Unknown command: %s\n", argv[2]);
        usage(argv[0]);
        close(fd);
        return 1;
    }

    close(fd);
    return 0;
}
