#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>


const char *device = "/dev/i2c-1";
const int mpu9265_addr = 0x68; /*ad0 = gnd */

int write_byte(int fd, char reg, char val)
{
    char buffer[2];

    buffer[0] = reg;
    buffer[1] = val;
    return write(fd, buffer, 2);
    
}

int read_byte(int fd, char reg,char *val)
{
    int ret;
    int _reg;
    _reg = reg;
    ret = write(fd, &_reg, 1);
    if(ret != 1)
    {
        printf("Something went wrong while reading %x\n",reg);
        return -1;
    }
    return read(fd, val, 1);
}

int main(void)
{
    int fd;
    int ret;
    char buf[16];

    fd = open(device,O_RDWR);
    if(fd < 0)
    {
        printf("Cannot open device %s\n", device);
        return 1;
    }

    ret = ioctl(fd, I2C_SLAVE, mpu9265_addr);
    if(ret < 0)
    {
        printf("Couldn't set slave address\n");
        return 2;
    }
    
    short x,y,z;
    char valh, vall;
    while(!0)
    {
        x = 0;
        y = 0;
        z = 0;

        read_byte(fd, 0x3B, &valh);
        read_byte(fd, 0x3C, &vall);
        x = (short)(valh << 8 | vall);

        read_byte(fd, 0x3D, &valh);
        read_byte(fd, 0x3E, &vall);
        y = (short)(valh << 8 | vall);

        read_byte(fd, 0x3F, &valh);
        read_byte(fd, 0x40, &vall);
        z = (short)(valh << 8 | vall);
        printf("x=%d\ty=%d\tz=%d\n", x, y, z);

        /* sleep for 50msec */
        usleep(50000);
        
    }

    /* do not forget to close the handle */
    close(fd);

    return 0;
}

