#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define GPIOSERDEV_IOC_MAGIC 'k'
#define GPIOSERDEV_STRBPIN _IOW(GPIOSERDEV_IOC_MAGIC, 1, int)
#define GPIOSERDEV_DATAPIN _IOW(GPIOSERDEV_IOC_MAGIC, 2, int)

void print_usage(const char* program) {
    printf("Usage: %s <strobe|data> <on|off>\n", program);
    printf("Example: %s strobe on\n", program);
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        print_usage(argv[0]);
        return 1;
    }

    int fd = open("/dev/gpioserdev", O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    int value = (strcmp(argv[2], "on") == 0) ? 1 : 0;
    unsigned int cmd;

    if (strcmp(argv[1], "strobe") == 0) {
        cmd = GPIOSERDEV_STRBPIN;
    } else if (strcmp(argv[1], "data") == 0) {
        cmd = GPIOSERDEV_DATAPIN;
    } else {
        print_usage(argv[0]);
        close(fd);
        return 1;
    }

    if (ioctl(fd, cmd, &value) < 0) {
        perror("IOCTL failed");
        close(fd);
        return -1;
    }

    printf("%s pin set to %s\n", argv[1], argv[2]);
    close(fd);
    return 0;
}