#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdio.h>

static const char *device = "/dev/spidev0.0";


int main(void)
{
	int ret;
	int fd;
	int i;
	unsigned int mode;
	unsigned int speed;
	unsigned char bits;
	unsigned char tx_buffer[8] = {0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA};
	unsigned char rx_buffer[8] = {0};


	/* open spi device */
	fd = open(device, O_RDWR);
	if(fd < 0)
	{
		printf("Cannot open device: %s",device);
		return 1;
	}

	/* set the spi mode 
	 * check spidev.h for details
	 *
	 * SPI_CPHA | SPI_CPOL | SPI_CS_HIGH 
	 * SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP 
	 * SPI_NO_CS | SPI_READY | SPI_TX_DUAL 
	 * SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD
	 */

	/*ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if(ret == -1)
	{
		printf("Cannot set the spi mode");
		return 2;
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
	if(ret == -1)
	{
		printf("Cannot get the spi mode");
		return 3;
	}*/


	/* set bits per word 
	 * 0 : 8 bits
	 */
	bits = 0;
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if(ret == -1)
	{
		printf("Cannot set the bit per word");
		return 4;
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if(ret == -1)
	{
		printf("Cannot get the bit per word");
		return 5;
	}

	/* set max speed in hz */
	speed = 1000000;
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if(ret == -1)
	{
		printf("Cannot set the max speed");
		return 6;
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if(ret == -1)
	{
		printf("Cannot get the max speed");
		return 7;
	}

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buffer,
		.rx_buf = (unsigned long)rx_buffer,
		.len = 8,
		.delay_usecs = 0,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

	if (ret < 1)
	{
	    printf("can't send spi message");
	}


	/*ret = write(fd, tx_buffer, 8);
	printf("written %d\n", ret);
	ret = read(fd, rx_buffer, 8);
	printf("read %d\n",ret);*/

	printf("write ");
	for(i = 0; i < 8; i++)
	{
		printf("%x ", tx_buffer[i]);
	}
	printf("\nRead: ");
	for(i = 0; i < 8; i++)
	{
		printf("%x ", rx_buffer[i]);
	}

	/* Don't forget to close device handle */
	close(fd);

	return 0;
}
