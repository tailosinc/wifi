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

#include <linux/fs.h>
#include <linux/file.h>
#include "onebox_common.h"
#include "onebox_linux.h"
#include "onebox_zone.h"

//ONEBOX_EXTERN uint8 firmware_path[256];

/**
 * This function checks if the file is of the UNIX/DOS type.
 *
 * @param  Pointer to the opened file desc.
 * @param  Pointer to a data buffer .  
 * @return Returns 8 if the file is of UNIX type else returns 9 if
 *         the file is of DOS  type . 
 */

static int32 onebox_check_file_type (struct file *filp, int8   *ptr)
{
	loff_t pos = 0;
	int32  temp;

	temp = vfs_read(filp,(char __user*)ptr,8, &pos);
	if (temp != 8)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Unable to check %d\n"), __func__, temp));
		return ONEBOX_STATUS_FAILURE;
	}

	if (ptr[7] == 10)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT( "%s: UNIX file type\n"),__func__));
		return UNIX_FILE_TYPE;
	}
	else /* if(ptr[7] == 13) */
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT( "%s: DOS file type\n"), __func__));
		return DOS_FILE_TYPE;
	} /* End if <condition> */
}

/**
 * This function opens a file and reads data.
 *
 * @param  File name to be opened for reading.
 * @param  Pointer to the read length.  
 * @param  File Format.  
 * @param  Firmware path(dummy for linux).  
 * @return On success, a pointer to the data read is returned
 *         else a NULL pointer is returned . 
 */
PUCHAR get_firmware (const int8 *fn, uint32 *read_length, uint32 type,
                     uint8 *fw_path)
{
	uint32 length; 
	uint32 temp;
	uint16 *dp;
	loff_t pos;
	uint8  file_to_open[266];
	struct file* filp;
	mm_segment_t fs = get_fs();

	set_fs(get_ds());
	/* prepend file name with path */
	strcpy(file_to_open, fw_path);
	strcat(file_to_open,fn); 
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("%s: trying to open '%s'.\n"),__func__, file_to_open));
	filp = filp_open(file_to_open,0,0);
	if (IS_ERR(filp))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to open file '%s'.\n"),
		             __func__, file_to_open));
		return NULL;
	}

#if ((LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0))) 
	length = filp->f_dentry->d_inode->i_size;
#else
	length = filp->f_path.dentry->d_inode->i_size;
#endif
	if (length <= 0 )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Too big file '%s'\n"),__func__, file_to_open));
		filp_close(filp, current->files);
		return NULL;
	}
	dp = vmalloc(length);
	if (dp == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Low on memory '%s'.\n"), __func__, fn));
		filp_close(filp, current->files);
		return NULL;
	}
	pos = 0;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: reading size %d.\n"),__func__, length));

	/* LMAC firmware will be in HEX file format */
	if (type == HEX_FILE)   
	{
		uint8 read_data[10];
		int32 ii = 0,num ;
		num = UNIX_FILE_TYPE;

		for (ii = 0; ii < (LMAC_INSTRUCTIONS_SIZE / 2); ii++)
		{
			memset(read_data,0,10);
			temp = vfs_read(filp,(char __user*)read_data,num, &pos);
			if (!ii) 
			{
				/*check for the file type either DOS/UNIX */
				num = onebox_check_file_type(filp,read_data);      
				if (num == -1)
				{
					vfree(dp);
					filp_close(filp, current->files);
					return NULL;
				}  
				else if (num == DOS_FILE_TYPE)
				{ 
					temp = vfs_read(filp,(char __user*)&read_data[8],1,&pos);
					if (temp != 1)
					{            
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
						             (TEXT("%s: Failed to read '%s %d %d'.\n"),
						               __func__, fn, temp, ii));
						vfree(dp);
						filp_close(filp, current->files);
						return NULL;
					}
				}                             
				else
				{
					num = UNIX_FILE_TYPE;
				} /* End if <condition> */        
			}      
			else
			{
				if (temp != num)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					             (TEXT("%s: Failed to read '%s %d %d'.\n"),
					              __func__, fn, temp, ii));
					vfree(dp);
					filp_close(filp, current->files);
					return NULL;
				}
			} /* End if <condition> */  
			*(dp+ii) = (uint16 )simple_strtol(&read_data[2],NULL,16);
		}
	}
	/*For tadm & taim cases*/
	else 
	{
		temp = vfs_read(filp,(char __user*)dp,length, &pos);
		if (temp != length)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("%s: Failed to read '%s %d'.\n"), 
			              __func__, fn,temp));
			vfree(dp);
			filp_close(filp, current->files);
			return NULL;
		}
	} /* End if <condition> */    
	filp_close(filp, current->files);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Successfully read the f/w\n"),
	             __func__));
	set_fs(fs);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s %d: Length of the firmware read =%d\n"), __func__, __LINE__, length));
	*read_length = length;
	return (uint8 *)dp;
}

/**
 * This function opens/creates a file and writes data.
 *
 * @param  File name to be opened for writing.
 * @param  Data to be written
 * @param  length to be written.  
 * @param  File Format.  
 * @param  File path(dummy for linux).  
 * @return On success, 0 is returned
 *         else -1 pointer is returned . 
 */
int write_to_file (const int8 *fn, uint16 *dp, uint32 write_len, uint32 type,
                     uint8 *file_path)
{
	loff_t pos = 0;
	uint8  file_to_open[266];
	struct file* filp;
	uint32 len_written = 0;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	/* prepend file name with path */
	strcpy(file_to_open, file_path);
	strcat(file_to_open, fn); 

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: trying to open/create '%s'.\n"),__func__, file_to_open));
	filp = filp_open(file_to_open, O_WRONLY|O_CREAT, 644);
	if (IS_ERR(filp))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to open file '%s'.\n"),
		             __func__, file_to_open));
		return ONEBOX_STATUS_FAILURE;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("%s: writing data of size %d.\n"),__func__, write_len));

	len_written = vfs_write(filp,(char __user *)dp,write_len, &pos);
	if (len_written != write_len)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Failed to write '%s %d'.\n"), 
				 __func__, fn,len_written));
		filp_close(filp, current->files);
		return ONEBOX_STATUS_FAILURE;
	}

	filp_close(filp, current->files);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: Successfully wrote the data to file\n"),
				__func__));
	set_fs(fs);
	return ONEBOX_STATUS_SUCCESS;
}
