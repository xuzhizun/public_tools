//Laser Measurement Code,
//send the order, and receive the distance;
//author:zhizun xu
//

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <math.h>

int open_port(const char* port_address)
{
	int fd;
	fd = open(port_address, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		printf("open_port: Unable to open %s", port_address);
	else
		fcntl(fd, F_SETFL, 0);

	return fd;
}
	


void config_port(int fd)
{
	struct termios options;
	
	tcgetattr(fd, &options);
	
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);

	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	tcsetattr(fd, TCSANOW, &options);
}

//received hex number should be sparsed, the method is proposed from instruction book
double sparse_hex(unsigned char* buffer, int size){
	int decimal[size];
	for(int i = 0; i<size; i++)
		decimal[i]=*(buffer+i);

	double sum = 0;
	printf("decimal[0]=%d\n", decimal[3]);
	for(int i = 1; !(i > decimal[2]); i++)
	{	
				sum += decimal[2+i]*pow(16, 2*(decimal[2]-i));
	
	}

	return sum/10000.0; //return the measured distance (unit: meter)
}	
	

int main(int argc, char** argv)
{
	if(argc < 2)
	{
			printf("Needed port address\n");
			return 0;
	}

  int fd = open_port(argv[1]);
	
	config_port(fd);

	char st_buffer[8];
	unsigned char rec_buffer[255];
	unsigned char valid_buffer[9];
	st_buffer[0]=0x01;
	st_buffer[1]=0x03;
	st_buffer[2]=0x00;
	st_buffer[3]=0x15;
	st_buffer[4]=0x00;
	st_buffer[5]=0x02;
	st_buffer[6]=0xd5;
	st_buffer[7]=0xcf;//the message to acquire the measured distance.
  //st_buffer[8]=0xb9;
	
	write(fd, st_buffer, sizeof(st_buffer)); 

	int length = 0;
	while(1){
		int t = read(fd, rec_buffer, sizeof(rec_buffer)); //read the received message
		//printf("bytes_read= %d\n", t);
		if(t>0){	
				for(int i=0; i<t; i++){
					printf("received %dth buffer: %d\n", i, rec_buffer[i]);
					valid_buffer[length + i]=rec_buffer[i];//extract valid data from received messaged, for the laser each time transmitted 3 bytes,
				}
		      length+=t;
		}
		if(length > 8)//get all data, finished.
			break;
	}

	printf("measured distance = %lf\n", sparse_hex(valid_buffer, sizeof(valid_buffer)));

	return 0;
}
