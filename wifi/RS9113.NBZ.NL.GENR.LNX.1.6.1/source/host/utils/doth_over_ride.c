
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include "onebox_util.h"

#define INPUT_FILE "input.txt"

#define EOF_CHECK if (ch == EOF) { \
	    printf("Reached EOF\n"); \
      goto end_of_file; \
    }

unsigned char Ascii2Hex(unsigned char c)
{
	if (c >= '0' && c <= '9')
	{
		return (unsigned char)(c - '0');
	}
	if (c >= 'A' && c <= 'F')
	{
		return (unsigned char)(c - 'A' + 10);
	}
	if (c >= 'a' && c <= 'f')
	{
        return (unsigned char)(c - 'a' + 10);
	}

	return 0;  // this "return" will never be reached, but some compilers give a warning if it is not present
}

//int main ()
int main (int argc, char *argv[])
{
  FILE *fp;
  char ch;
  static char file_name[200];
  unsigned short DATA[500];
  unsigned ADDR;
  int sfd ,len = 0 ,i =0;
  struct iwreq wrq;
  char ifName[32] = "rpine0";
  struct bb_rf_param_t  bb_rf_params;
  prog_structure_t *prog_structure;

  if(argc < 3) {
    printf("Please Enter the prog_type \n");
    return -1;
  }
  fp = fopen( argv[2], "r" );
  //fp = fopen(INPUT_FILE, "r");

  if(fp == NULL){
    printf("Unable to Open file to read\n");
    return;
  }

  prog_structure = (prog_structure_t *)DATA;
       prog_structure->prog_type = atoi(argv[1]);
       prog_structure->structure_present = 1; 

  i = (sizeof(prog_structure_t)/2 );
  i += (sizeof(prog_structure_t) % 2);  

  if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    ONEBOX_PRINT ("socket creation error\n");
    return 2;
  }

  do {
    ch = fgetc(fp);
    EOF_CHECK;
    if (ch == '{') {
      break;
    }
  } while (1);

  ADDR = 0;

  do {
    ch = fgetc(fp);
    EOF_CHECK;
    if (ch == '0') {
      ch = fgetc(fp);
      EOF_CHECK;
      if (ch == 'x' || ch == 'X') {
        DATA[i] = Ascii2Hex(fgetc(fp));
        DATA[i] = (DATA[i] << 4) | Ascii2Hex(fgetc(fp));
        DATA[i]= (DATA[i] << 4) | Ascii2Hex(fgetc(fp));
        DATA[i] = (DATA[i] << 4) | Ascii2Hex(fgetc(fp));
        len ++;
        i++;
      }
    }

  } while (1);

end_of_file:
       prog_structure->TA_RAM_ADDRESS = 0;
       prog_structure->bb_rf_flags = 0;
			wrq.u.data.flags = PROG_STRUCTURE ;
			wrq.u.data.pointer = DATA;
      wrq.u.data.length = (&DATA[i] - &DATA[0]) * 2;
      prog_structure->len = (wrq.u.data.length - (sizeof(prog_structure_t)));  

			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

        fclose(fp);  
			if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0) 
			{  
				printf("Error in PROG_STRUCTURE from driver\n");
				return -1;
			}
}

