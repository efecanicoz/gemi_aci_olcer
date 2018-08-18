#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

struct _axis
{
    double x;
    double y;
    double z;
};

struct _data
{
    struct _axis acc;
    struct _axis ang_vel;
    struct _axis angle;
    double temp;
};

int error_count;

int set_interface_attribs (int , int speed, int );
void synchronize(int );
void read_frame(int , struct _data * );


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 1;            // minimum message length to wait in blocking read function
        tty.c_cc[VTIME] = 0;            // time between each byte to fire select() event 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void change_vmin(int fd)
{
struct termios options;

/*
 * Get the current options for the port...
 */

tcgetattr(fd, &options);

options.c_cc[VMIN]=11;
options.c_cc[VTIME]=0;

/*
 * Set the new options for the port...
 */
tcsetattr(fd, TCSANOW, &options);
}

void synchronize(int fd)
{
    
    int n;
    char buffer[16];
    
    printf("synchronizing\n"); 
    /* read until find a start message to synchronize with sensor */
    while(!0)
    {
        n = read(fd, buffer, 1); /* read 1 byte */
        if(n != 1)
        {
            /* n = 0, doesnt expected but better to guard */ 
            continue;
        }
        else if(0x55 != buffer[0]) /* expecting start byte */
        {
            printf("Expecting start byte but got %x\n",buffer[0]);
            continue;
        }
        
        /* this may be start byte, check next start frame to be sure*/
        n = read(fd, buffer, 11);
        if(n == 11 || 0x55 == buffer[10])
        {
            /* read the rest of the frame and return */
            n = read(fd, buffer, 10);
            break;
        }
        
        printf("next frame header expected but something else came\n");
        continue;
        
    }
    error_count=0;
    return;
}

void read_frame(int fd, struct _data * sensor_data)
{
    int n;
    int i;
    char buffer[11];
    char sum;
    
    n = read(fd, buffer, 11);

    if(n != 11)
    {
        printf("Expected 11 bytes of frame but didnt get\n");
        return;
    }
    

    /* check checksum */
    for(i = 0, sum = 0; i < 10; i++)
    {
        sum += buffer[i]; /* let it overflow */
    }
    if(sum != buffer[10])
    {
        printf("Checksum doesn't match, ignoring frame\n");
	error_count++;
	if(100 == error_count)
		synchronize(fd);
        //return;
    }
    
    /* Frame is valid */
    if(0x51 == buffer[1])
    {
        sensor_data->acc.x = (((short)((buffer[3]<<8) | buffer[2]))/32768.0 * 16); /* 32768.0 * 16*/
        sensor_data->acc.y = (((short)((buffer[5]<<8) | buffer[4]))/32768.0 * 16);
        sensor_data->acc.z = (((short)((buffer[7]<<8) | buffer[6]))/32768.0 * 16);
        sensor_data->temp = (((short)((buffer[9]<<8) | buffer[8]))/340.0 + 36.25);
    }
    else if(0x52 == buffer[1])
    {
        sensor_data->ang_vel.x = (((short)((buffer[3]<<8) | buffer[2]))/32768.0 * 2000);
        sensor_data->ang_vel.y = (((short)((buffer[5]<<8) | buffer[4]))/32768.0 * 2000);
        sensor_data->ang_vel.z = (((short)((buffer[7]<<8) | buffer[6]))/32768.0 * 2000);
        sensor_data->temp = (((short)((buffer[9]<<8) | buffer[8]))/340.0 + 36.25);
    }
    else if(0x53 == buffer[1])
    {
        sensor_data->angle.x = (((short)((buffer[3]<<8) | buffer[2]))/32768.0*180);
        sensor_data->angle.y = (((short)((buffer[5]<<8) | buffer[4]))/32768.0*180);
        sensor_data->angle.z = (((short)((buffer[7]<<8) | buffer[6]))/32768.0*180);
        sensor_data->temp = (((short)((buffer[9]<<8) | buffer[8]))/340.0 + 36.25);
    }
    else
    {
        printf("Invalid frame type");
    }
    
    return;
}

int main(void)
{
    int n;
    int fd;
    /* port name for usart2 */
    char *portname = "/dev/ttyS2";
    char buf [100];
    
    struct _data sensor_data;
    
    printf("%s will be opened\n",portname);
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }
    printf("Port has opened, now it will be configured");
    set_interface_attribs(fd, B115200, 0);

    
    synchronize(fd);
    change_vmin(fd);
    /* read sensor data */
    while(!0)
    {
        read_frame(fd, &sensor_data);
        /* clear the terminal */
        printf("\033[2J\033[1;1H");
        printf("Acc   x:%f\ty:%f\tz:%f\n", sensor_data.acc.x, sensor_data.acc.y, sensor_data.acc.z);
        printf("Ang.v x:%f\ty:%f\tz:%f\n", sensor_data.ang_vel.x, sensor_data.ang_vel.y, sensor_data.ang_vel.z);
        printf("Angle x:%f\ty:%f\tz:%f\n", sensor_data.angle.x, sensor_data.angle.y, sensor_data.angle.z);
        printf("Temp %f\n", sensor_data.temp);
    }
    
    /* Don't forget to close */
    close(fd);
    
    return 0;
}
