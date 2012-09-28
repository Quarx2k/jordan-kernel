/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * Date         Author          Comment
 * ===========  ==============  ==============================================
 * 16-Apr-2009  Motorola        Initial revision.
 *
 */

/*Header for external kernel space SecAPIs*/

#define SEC_PROC_ID_SIZE 16
#define MODEL_ID_SIZE 4


typedef enum {
	SEC_ENGINEERING = 0x77,
	SEC_PRODUCTION = 0x44
} SEC_MODE_T;


typedef enum {
	SEC_SUCCESS = 0x33,
	SEC_FAIL = 0x00
} SEC_STAT_T;

SEC_STAT_T SecProcessorID(unsigned char *buffer, int length);

SEC_STAT_T SecModelID(unsigned char *buffer, int length);

SEC_MODE_T SecProcessorType(void);
