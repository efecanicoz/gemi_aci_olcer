#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>

#define RAD_TO_DEG (180 / M_PI)

const char *device = "/dev/i2c-1";
const int mpu9265_addr = 0x68; /*ad0 = gnd */

struct axis
{
    double x;
    double y;
    double z;
};

int write_byte(const int fd, const char reg, const char val)
{
    char buffer[2];

    buffer[0] = reg;
    buffer[1] = val;
    return write(fd, buffer, 2);
    
}

int read_byte(const int fd, const char reg, char * const val)
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

void read_accel(const int fd, struct axis * const acc)
{
    char valh,vall;

    /*Read raw data and divide to 16284 (AFSEL=00 sensivity scale factor)
      x,y,z will be in unit "G"*/
    read_byte(fd, 0x3B, &valh);
    read_byte(fd, 0x3C, &vall);
    acc->x = ((short)(valh << 8 | vall)/16384.0);

    read_byte(fd, 0x3D, &valh);
    read_byte(fd, 0x3E, &vall);
    acc->y = ((short)(valh << 8 | vall)/16384.0);

    read_byte(fd, 0x3F, &valh);
    read_byte(fd, 0x40, &vall);
    acc->z = ((short)(valh << 8 | vall)/16384.0);
    return;
}

void read_gyro(const int fd, struct axis * const gyro)
{
    char valh,vall;

    /*Read raw data and divide it by 131.0 (FSEL=00 sensivity scale factor)
      By this x,y,z variables will be in "deg/second" unit */
    read_byte(fd, 0x43, &valh);
    read_byte(fd, 0x44, &vall);
    gyro->x = ((short)(valh << 8 | vall)/131.0);

    read_byte(fd, 0x45, &valh);
    read_byte(fd, 0x46, &vall);
    gyro->y = ((short)(valh << 8 | vall)/131.0);

    read_byte(fd, 0x47, &valh);
    read_byte(fd, 0x48, &vall);
    gyro->z = ((short)(valh << 8 | vall)/131.0);
    return;
}

int main(void)
{
    double roll,pitch,comp_roll,comp_pitch;
    int fd;
    int ret;
    char buf[16];
    struct axis accel, gyro;

    fd = open(device,O_RDWR);
    if(fd < 0)
    {
        printf("Cannot open device %s Maybe try with sudo ?\n", device);
        return 1;
    }

    ret = ioctl(fd, I2C_SLAVE, mpu9265_addr);
    if(ret < 0)
    {
        printf("Couldn't set slave address\n");
        return 2;
    }

    comp_roll= 0;
    comp_pitch=0;

    while(!0)
    {
        read_accel(fd, &accel);
        read_gyro(fd, &gyro);

        roll = atan2(accel.y, accel.z) * RAD_TO_DEG;
        pitch = atan(-accel.x / sqrt(accel.x * accel.x + accel.z * accel.z)) * RAD_TO_DEG;

        comp_roll = 0.93 * (comp_roll + gyro.x * 0.05) + 0.07 * roll;
        comp_pitch = 0.93 * (comp_pitch + gyro.y * 0.05) + 0.07 * pitch;
        //printf("Acc : x=%f\ty=%f\tz=%f\n", accel.x, accel.y, accel.z);
        //printf("Gyro: x=%f\ty=%f\tz=%f\n", gyro.x, gyro.y, gyro.z);
        printf("     Roll: %f\tPitch: %f\n", roll, pitch);
        printf("Comp Roll: %f\tPitch: %f\n", comp_roll, comp_pitch);

        

        /* sleep for 50msec */
        usleep(50000);
        
    }

    /* do not forget to close the handle */
    close(fd);

    return 0;
}

