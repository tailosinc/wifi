/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include <linux/unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>


#define FILE_NAME "eeprom.txt"
#define BACKUP_FILE_NAME "eeprom_backup.txt"
#define UPDATED_FILE_NAME "eeprom_updated.txt"
#define INPUT_FILE "input.txt"
#define NEW_FILE "eeprom_new.txt"
#define RMMOD_COMMAND "sh remove_all.sh"
#define INSMOD_COMMAND "sh wlan_enable.sh"
#define INSMOD_FLASH_COMMAND "sh remove_all.sh 1"
#define MAX_READ                4096
#define MAX_WRITE                4096
#define RSI_EEPROM_READ         26
#define MAX_LENGTH              256
#define EEPROM_READ_IOCTL       22
#define EEPROM_WRITE_IOCTL      23
#define ONEBOX_HOST_IOCTL SIOCIWLASTPRIV - 0x0B
#define LOCATION               2048      
#define INTERFACE_NAME         "rpine0"
#define START_LOCATION         4001792

extern int check_sum();
typedef struct eepromrw_info_s
{
	unsigned int    offset;
	unsigned int    length;
	unsigned char   write;
	unsigned short  eeprom_erase;
	unsigned char   data[480];
}eepromrw, EEPROMRW;

FILE *fp, *new_fp, *backup_fp;
struct iwreq wrq;
char *ifName;
//int cmdNo = -1;
EEPROMRW  eeprom;
int sfd, call_count;
unsigned char buff[102];

int check_interface(char *interface_name){

	int state = -1;
	int socId = socket(AF_INET, SOCK_DGRAM, 0);
	if (socId < 0){
		printf("Socket failed. Errno = %d\n", errno);
		return state;
	}

	struct ifreq if_req;
	(void) strncpy(if_req.ifr_name, interface_name, sizeof(if_req.ifr_name));
	state = ioctl(socId, SIOCGIFFLAGS, &if_req);
	
	close(socId);

	if (state != 0){
		 printf("Ioctl failed. Errno = %d\n", errno);
		 return errno;
	}
	return state;
}

int read_eeprom(FILE *fp_in){

	int rsi_cmd_flags = 0, ii;
	int i; 
	
	if(!call_count){
  		backup_fp = fopen(BACKUP_FILE_NAME, "w+");
  		if(!backup_fp)
    			printf("Unable to open file for backup\n");
	}
	printf("Reading EEPROM Values...\n");
	for(i = 0;i < MAX_READ; i = i + MAX_LENGTH){
		//if(i != 4000)
			eeprom.length = MAX_LENGTH;
		//else
		//	eeprom.length = 96;
		eeprom.offset = i + START_LOCATION;
		//eeprom.offset = i;
		rsi_cmd_flags = eeprom.length;
		memset (&wrq, 0, sizeof (struct iwreq));
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		wrq.u.data.flags = EEPROM_READ_IOCTL;
		wrq.u.data.pointer = &eeprom;
		wrq.u.data.length = eeprom.length;
		if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0){ 
			printf("%s\n", strerror(errno));
			printf("Unable to read EEPROM\n");
	if(!call_count){
      if(backup_fp)
        fclose(backup_fp);
	}
			return -1;
		}
		else
		{
			//sleep(2);
#ifdef DEBUG
			printf("Read Values in %d iteration :\n", (i+1));
#endif
			for (ii = 0; ii < rsi_cmd_flags; ii++ ){
				fprintf(fp_in,"0x%02x,\n", eeprom.data[ii]);
				if(!call_count)
					fprintf(backup_fp,"0x%02x,\n", eeprom.data[ii]);
				//temp_buff[i+ii] = eeprom.data[ii];
#ifdef DEBUG
				printf("0x%02x ", eeprom.data[ii]);
#endif
			}
#ifdef DEBUG
			printf("\n");
#endif
		}

	}
	if(!call_count){
  if(backup_fp)
    fclose(backup_fp);
	}

	return 0;
}

int read_input_file(){
	FILE *input_fp;
	int ii;

	input_fp = fopen(INPUT_FILE, "r");
	if(!input_fp){
		printf("Unable to Open file to read...\n");
		return -1;
	}

	printf("Reading Input Values to be written...\n");
	for(ii = 0; ((ii < 42) && (!feof(input_fp))); ii++){
		fscanf(input_fp, "%d", &buff[ii]);
	}
  if(ii < 42){
    printf("Less content is there in file as compare to expected...\n");
    return -2;
  }
	if(input_fp)
		fclose(input_fp);
#ifdef DEBUG
	printf("Dumping Read buffer...\n");
	for(ii = 0; ii < 42 ; ii++)
		printf("0x%x ", buff[ii]);
	printf("\n");
#endif
	return 0;
}

int write_to_eeprom_file(){

	int ii;
	unsigned char value;
	unsigned char temp_buff[4097];
	fp = fopen(FILE_NAME, "r");
	if(!fp){
		printf("Error in Open File...\n");
		return -1;
	}
	
	for(ii = 0; ii < 4096; ii++)
		fscanf(fp, "0x%02x%c\n", &temp_buff[ii], &value);
	fclose(fp);
#ifdef DEBUG
	printf("Dumping Temp buffer...\n");
	for(ii = 0; ii < 4096; ii++)
		printf("0x%02x\n", temp_buff[ii]);
	printf("\n");
#endif
	printf("Updating EEPROM values as per input file values...\n");
	for(ii = 0; ii < 42; ii++)
		temp_buff[ii] = buff[ii];
#ifdef DEBUG
	printf("Dumping Temp buffer After changes...\n");
	for(ii = 0; ii < 4096; ii++)
		printf("0x%x ", temp_buff[ii]);
	printf("\n");
#endif
	fp = fopen(FILE_NAME, "w+");
	if(!fp){
		printf("Error in Open File...\n");
		return -1;
	}

	for(ii = 0; ii < 4096; ii++)
		fprintf(fp,"0x%02x,\n", temp_buff[ii]);

	fclose(fp);

	return 0;
}

int write_eeprom(){

	int /*rsi_cmd_flags = 0,*/ ii;
	int i; 
	unsigned char value;

	fp = fopen(FILE_NAME, "r");
	//fp = fopen(UPDATED_FILE_NAME, "r");
	if(!fp){
		printf("Error in Open File\n");
		return -1;
	}
	printf("Writing updated values back in EEPROM...\n");
	for(i = 0;i < MAX_WRITE; i = i + MAX_LENGTH){

		memset (&eeprom, 0, sizeof (struct eepromrw_info_s));
		if(i == 0)
			eeprom.eeprom_erase = 1;
		else
			eeprom.eeprom_erase = 0;
		eeprom.length = MAX_LENGTH;
		eeprom.offset = i + START_LOCATION;
		eeprom.write = 1;
#ifdef DEBUG	
		printf("In iteration %d :\n", (i+1));
#endif
		for(ii = 0; ii < MAX_LENGTH; ii++){
			fscanf(fp, "0x%02x%c\n", &eeprom.data[ii], &value);
#ifdef DEBUG	
			printf("0x%02x ", eeprom.data[ii]);
#endif
		}	
#ifdef DEBUG	
		printf("\n");
#endif
		memset (&wrq, 0, sizeof (struct iwreq));
		//rsi_cmd_flags = eeprom.length;
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		wrq.u.data.flags = EEPROM_WRITE_IOCTL;
		wrq.u.data.pointer = &eeprom;
		wrq.u.data.length = sizeof(eeprom);
		if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0){ 
			printf("Unable to Write EEPROM\nError is %d\n", errno);
			printf("%s\n", strerror(errno));
			if(fp)
				fclose(fp);
			return -1;
		}
	}
	if(fp)
		fclose(fp);

	return 0;
}

int compare_eeprom_files(){
	int length1 = 0, length2 = 0, ii;
	unsigned int content1 = 0, content2 = 0;
	fp = fopen(FILE_NAME, "r");
	if(!fp){
		printf("Unable to Open File...\n");
		return -1;
	}
	new_fp = fopen(NEW_FILE, "r");
	if(!new_fp){
		printf("Unable to Open File...\n");
		if(fp)
			fclose(fp);
		return -2;
	}

	fseek(fp, 0, SEEK_END);
	length1 = ftell(fp);
	
	fseek(fp, 0, SEEK_END);
	length2 = ftell(fp);

#ifdef DEBUG
	printf("Length of Both Files are:\nSource File : %d\nTarget File : %d\n", length1, length2);
#endif

	if(length1 != length2){
		if(fp)
			fclose(fp);
		if(new_fp)
			fclose(new_fp);
		return 1;
	}

	rewind(fp);
	rewind(new_fp);

	for(ii =0; ii < MAX_READ; ii++){
		fscanf(fp, "0x%02x,\n", &content1);	
		fscanf(new_fp, "0x%02x,\n", &content2);	

		if(content1 != content2){
			if(fp)
				fclose(fp);
			if(new_fp)
				fclose(new_fp);

			return 2;
		}
	}
	
	if(fp)
		fclose(fp);
	if(new_fp)
		fclose(new_fp);

	return 0;
	
}

int main(int argc, char *argv[]){

	int ret;
	if (argc < 1)
	{
		printf("More arguments required...\n");
		return -1;
	}
	else if (argc <= 50)
	{
		ifName = INTERFACE_NAME;
		// Check Whether Interface is there or not.
		ret = check_interface(ifName);
		if(ret == -1){
			printf("Failed to create Socket for checking interface...\n");
			return ret;
		}
		if(ret == ENODEV){
			printf("Either Chip is not connected to driver is not inserted properly.\nPlease Check Chip or Driver...\n");
			return -1;
		}
		// Open a file to write Content of EEPROM.
		fp = fopen(FILE_NAME, "w+");
		if(!fp){
			printf("Error in Open File.\n");
			return -1;
		}

		if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
		{
			printf("socket creation error\n");
			if(fp)
				fclose(fp);
			return -2;
		}

		//Reading EEPROM data of 4K.
		ret = read_eeprom(fp);
		if(fp)
			fclose(fp);
		if(sfd)
			close(sfd);
		if(ret == -1){
			printf("Error in Reading EEPROM...\n");
			return ret;
		}
		call_count++;
		//Writing 100 bytes to file at specific offset. So First Read Input file to a buffer and write it to EEPROM file.
		ret = read_input_file(); //Reading Input File.

		if(ret == -1){
			printf("Error in Reading Input File...\n");
			return ret;
		}
		write_to_eeprom_file(); //Writing to EEPROM file to a specific location.

		if(ret == -1){
			printf("Error in Open EEPROM File for doing update...\n");
			return ret;
		}

		// do rmmod driver and insert driver with flash script.
#if 0
		ret = system(RMMOD_COMMAND);
		if(ret < 0){
			printf("Error in executing system command...\n");
			return -1;
		}
		else{
			if(ret != 0){
				if(ret == 1)
					printf("Unable to do rmmod. May be Driver is not inserted yet...\n");
				else if(ret == 127)
					printf("rmmod script is not present in current path...\n");
				return -1;
			}
		}

		//Do insmod driver using flash script.
		ret = system(INSMOD_FLASH_COMMAND);
		if(ret < 0){
			printf("Error in executing system command...\n");
			return -1;
		}
		else{
			if(ret != 0){
				if(ret == 255)
					printf("Unable to do insmod in flash mode. May be Driver is already inserted...\n");
				else if(ret == 127)
					printf("insmod script is not present in current path...\n");
				return -1;
			}
		}

#endif
		if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
		{
			printf("socket creation error\n");
			return 2;
		}
		//Now Write file back to EEPROM.
		ret = write_eeprom();

		if(ret == -1){
			printf("Error in Writing File to EEPROM...\n");
			if(sfd)
				close(sfd);
			return ret;
		}
		//Do rmmod and insmod for normal mode.
#if 0
		ret = system(RMMOD_COMMAND);
		if(ret < 0){
			printf("Error in executing system command...\n");
			return -1;
		}
		else{
			if(ret != 0){
				if(ret == 1)
					printf("Unable to do rmmod. May be Driver is not inserted yet...\n");
				else if(ret == 127)
					printf("rmmod script is not present in current path...\n");
				return -1;
			}
		}
		//Doing Insmod again.
		ret = system(INSMOD_COMMAND);
		if(ret < 0){
			printf("Error in executing system command...\n");
			return -1;
		}
		else{
			if(ret != 0){
				if(ret == 255)
					printf("Unable to do insmod in flash mode. May be Driver is already inserted...\n");
				else if(ret == 127)
					printf("insmod script is not present in current path...\n");
				return -1;
			}
		}

#endif
		// Reading EEPROM After write Values for Verification.
		printf("Going to Read in EEPROM again...\n");
		//memset (&wrq, 0, sizeof (struct iwreq));
		//strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		new_fp = fopen(NEW_FILE, "w+");
		ret = read_eeprom(new_fp);

		if(ret == -1){
			printf("Error in Reading New Values from EEPROM...\n");
			close(sfd);
			fclose(new_fp);
			return ret;
		}

		close(sfd);
		fclose(new_fp);

		//Now Match both Files
		ret = compare_eeprom_files();

		if(ret < 0){
			if(ret == -1)
				printf("Error in Opening Source File...\n");
			if(ret == -2)
				printf("Error in Opening Target File...\n");
			return ret;
		}
		else{
			if(!ret)
				printf("Both Files are same. Hence wrtitng to EEPROM is correct...\n");
			else{
				printf("Both Files are not same...\n");
				if(ret == 1)
					printf("Length of both Files are not same...\n");
				if(ret == 2)
					printf("Content of Files are different...\n");
			}
		}
	
	}
	return ret;
}
