#include<stdio.h>
typedef unsigned char       uint8;
typedef unsigned int        uint32;
typedef struct checksum32bit_scatter_s 
{
	uint8 *buf;
	uint32 size;
} checksum32bit_scatter_t;
unsigned char temp[4075];

#define MAX_SCATTERS_CHECKSUM  1
uint8 *checksum_scatter_addr[MAX_SCATTERS_CHECKSUM];
uint32 checksum_scatter_len[MAX_SCATTERS_CHECKSUM];

static uint32 checksum_addition(uint8 *buf, uint32 size, uint32 prev_sum)
{
	uint32 sum = prev_sum;
	uint32 cnt;
	uint32 cnt_limit;
	uint32 dword;

	if (size == 0)
	{
 		return sum;
	}

	cnt_limit = (size & (~0x3));
	/* Accumulate checksum */
	for (cnt = 0; cnt < cnt_limit; cnt += 4)
	{
		dword = *(uint32 *) &buf[cnt];
		sum += dword;
		if(sum < dword)
		{
			/* In addition operation, if result is lesser than any one of the operand
			 * it means carry is generated. 
			 * Incrementing the sum to get ones compliment addition */
			sum++;
		}
	}

	/* Handle non dword-sized case */
  if(size & 0x3) {
		dword = 0xffffffff;
		dword = ~(dword << (8 * (size & 0x3)));
		/* Keeping only valid bytes and making upper bytes zeroes. */
		dword = (*(uint32 *) &buf[cnt]) & dword;
		sum += dword;
		if(sum < dword)
		{
			sum++;
		}
	}

	return sum;
}

uint32 checksum_32bit(uint8 **scatter_addr, uint32 *scatter_len, uint32 no_of_scatters)
{


	uint32 sum = 0;
	uint32 cnt;
	uint8 *buf;
	uint32 size;
	for (cnt = 0; cnt < no_of_scatters; cnt++)
	{
		buf  = scatter_addr[cnt];
		size = scatter_len[cnt];
		sum = checksum_addition(buf, size, sum);
	}
	/* Invert to get the negative in ones-complement arithmetic */
	return ~sum;
}

int main()
{
	/* Example of one scatter */
	struct checksum32bit_scatter_s scatters_checksum;
	uint32 checksum_result = 0,length=4075,j;
	uint8 value[2],i,checksum_byte;

	FILE *pfile;
	/*Opening the file to read the contents and store in buffer from 16 offset to caluclate CRC*/
	pfile = fopen("RS9113_RS8111_calib_values.txt", "r");
	if(pfile == NULL)
	{
		printf("WARNING Unable to create a file\n");
	}
	fseek(pfile, (16*6), SEEK_SET);

	printf("Filling the buffer.....\n");
	for(j=0;j<length;j++)
	{
		fscanf(pfile,"%x%c\n",&temp[j],&value[0]);
	}
    
	checksum_scatter_addr[0] = &temp[0];
	checksum_scatter_len[0]  = length+1;

//	checksum_scatter_addr[1] = &temp[0];
//	checksum_scatter_len[1]  = length+1;
        checksum_result= checksum_32bit(&checksum_scatter_addr[0], &checksum_scatter_len[0], 1);
	fclose(pfile);

	/*Opening the file to write caluclated CRC into the 4092 location*/

	pfile = fopen("RS9113_RS8111_calib_values.txt", "rw+");
	if(pfile == NULL)
	{
		printf("WARNING Unable to create a file_1\n");
	}
	printf("CRC = %08X \n", checksum_result);
	fseek(pfile, (4092*6), SEEK_SET);

	for(i=0;i<4;i++)
	{   
		checksum_byte = (uint8)checksum_result;
		fprintf(pfile , "0x%.2x,\n", checksum_byte);
		printf("\nchecksum_byte[%d]: %0.2X\n",i,checksum_byte);
		checksum_result = (checksum_result >> 8);
	}
	fclose(pfile);
	return 0;
}

