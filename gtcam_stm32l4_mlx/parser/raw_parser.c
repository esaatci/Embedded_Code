#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#define SIZE 1548
#define TEMP_SIZE 1536
int buf[SIZE];

void extract_frame(void);

int main() 
{

	int c, flag, c_count, i, total;
	uint8_t lsb, msb;
	uint16_t packed;
	uint32_t times_stamp;
	float f_packed;
	flag = c_count = 1;
	
	while((c = getchar()) != EOF) 
	{
		
		buf[c_count-1] = c;
		if(c_count == 1548) 
		{
			extract_frame();
			total++;
			c_count = 1;
		}
		else
		{
			c_count++;
		}
		
	}

	
	return 0;
}

void extract_frame(void) 
{
	
	char str[256];
	uint8_t lsb, msb;
	uint16_t packed, milliseconds;
	uint32_t times_stamp, save_time;
	float f_packed;
	int i ,sz, count;
	count = 0;
	
	for(i=0; i < TEMP_SIZE; i+=2) 
	{
		
		msb = buf[i];
		lsb = buf[i+1];
		
		packed = 0;
		packed = (msb << 8) | lsb;
		
		f_packed = packed;
		f_packed /= 100;
		
		sz = sprintf(str, "%.2f,",f_packed);
		printf("%s", str);
		memset(str, 0, sz);
		count++;
	}

	times_stamp = 0;
	times_stamp = (buf[i] << 24) | (buf[i+1] << 16) | (buf[i+2] << 8) | (buf[i+3]);
	printf("%u," , times_stamp);
	i += 4;

	milliseconds = 0;
	milliseconds = (buf[i] << 8) | (buf[i+1]);
	printf("%u,", milliseconds);
	i += 2;

	save_time = 0;
	save_time = (buf[i] << 24) | (buf[i+1] << 16) | (buf[i+2] << 8) | (buf[i+3]);
	printf("%u\n", save_time);

	printf("\n");


}