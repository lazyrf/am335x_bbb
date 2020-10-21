#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#define SYS_FS_LEDS_PATH    "/sys/class/leds"
#define USR_LED_NUMBER      3
#define SOME_BYTES          100

int write_trigger_values(uint8_t led_no, char *value)
{
    int fd, ret = 0;
    char buf[SOME_BYTES];

    snprintf(buf, sizeof(buf), SYS_FS_LEDS_PATH "/beaglebone:green:usr%d/trigger", led_no);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("write trigger error\n");
        return -1;
    }

    ret = write(fd, value, strlen(value));
    if (ret <= 0) {
        printf("trigger value write error (%s)\n", buf);
        return -1;
    }
    return 0;
}

int write_brightness_values(uint8_t led_no, char *value)
{
    int fd, ret = 0;
    char buf[SOME_BYTES];

    snprintf(buf, sizeof(buf), SYS_FS_LEDS_PATH "/beaglebone:green:usr%d/brightness", led_no);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("write brightness error\n");
        return -1;
    }

    ret = write(fd, value, strlen(value));
    if (ret <= 0) {
        printf("brightness value write error (%s)\n", buf);
        return -1;
    }
    return 0;
}

void process_trigger_values(char *value)
{
    if(strcmp(value, "timer") || strcmp(value, "heartbeat") || \
            strcmp(value, "none") || strcmp(value, "oneshot") || \
            strcmp(value, "default-on")) {
        write_trigger_values(USR_LED_NUMBER, value);
    } else {
        printf("Invalid value\n");
        printf("valid trigger values: heartbeat, timer, none, oneshot, default-on\n");
    }
}

void process_brightness_values(int value)
{
    switch (value) {
        case 1:
            write_brightness_values(USR_LED_NUMBER, "1");
            break;
        case 0:
            write_brightness_values(USR_LED_NUMBER, "0");
            break;
        default:
            printf("Invalid value\n");
            printf("valid brightness value: 0, 1\n");
    }
}

int main(int argc, char *argv[])
{
    printf("This application controls the UER LED3\n");

    if (argc != 3) {
        printf("usage: %s <control_option> <value>\n", argv[0]);
        printf("valid control_options : brightness, trigger\n");
        printf("valid 'brightness' value: 0, 1\n");
        printf("valid 'trigger' values: heartbeat, timer, none, oneshot, deafault-on\n");
    } else {
        if (strcmp(argv[1], "trigger") == 0) {
            process_trigger_values(argv[2]);
        } else if (strcmp(argv[1], "brightness") == 0) {
            int value = atoi(argv[2]);
            process_brightness_values(value);
        } else {
            printf("Invalid control option\n");
            printf("valid control_options : brightness, trigger\n");
        }
    }

    return 0;
}
