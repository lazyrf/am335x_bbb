#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_DEVICE_FILE                 "/dev/i2c-2"

#define SHT3X_SLAVE_ADDRESS             0x44

#define SHT3X_REG_STATUS                0xF32D
#define SHT3X_REG_PERIODIC_MEA          0x2032
#define SHT3X_REG_BREAK                 0x3093
#define SHT3X_REG_PERIODIC_DATA_READ    0xE000
#define SHT3X_REG_SINGLE_DATA_READ      0x2C10

int fd;

int sht3x_write(uint16_t cmd)
{
    int ret;
    char buf[2];
    buf[0] = (cmd & 0xFF00) >> 8;
    buf[1] = cmd & 0x00FF;
    ret = write(fd, buf, 2);
    if (ret <= 0) {
        perror("write command failed\n");
        return -1;
    }
    return 0;
}

int sht3x_read(uint16_t cmd, uint8_t *p_buf, uint32_t len)
{
    int ret;
    char buf[2];
    buf[0] = (cmd & 0xFF00) >> 8;
    buf[1] = cmd & 0x00FF;
    ret = write(fd, buf, 2);
    if (ret <= 0) {
        perror("write command failed\n");
        return -1;
    }

    ret = read(fd, p_buf, len);
    if (ret <= 0) {
        perror("read failed\n");
    }
    return 0;
}

void sht3x_status_get(void)
{
    uint8_t buf[2];
    sht3x_read(SHT3X_REG_STATUS, buf, 2);
    printf("status = %u\r\n", buf[0] << 8 | buf[1]);
}

int main(void)
{
    double temp, humi;
    uint8_t buf[6];
    uint16_t temp_raw, humi_raw;

    if ((fd = open(I2C_DEVICE_FILE, O_RDWR)) < 0) {
        perror("Failed to open I2C device file.\n");
        return -1;
    }

    /* Set the I2C slave address using iotcl I2C_SLAVE comamnd */
    if (ioctl(fd, I2C_SLAVE, SHT3X_SLAVE_ADDRESS) < 0) {
        perror("Failed to set I2C slave address.\n");
        close(fd);
        return -1;
    }

    while (1) {
        sht3x_read(SHT3X_REG_SINGLE_DATA_READ, buf, 6);
        temp_raw = buf[0] << 8 | buf[1];
        humi_raw = buf[3] << 8 | buf[4];
        temp = -45 + 175 * (temp_raw / 65535.0);
        humi = 100 * (humi_raw / 65535.0);
        printf("temp = %lf, humi = %lf\n", temp, humi);
        usleep(500000);
    }
}

