/*
 * lw.h FPGA logiWIN Video Input driver header file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Supports:
 *  FPGA logiWIN Video Input
 */

#ifndef __LW_H__
#define __LW_H__

struct logiwin_platform_data
{
	unsigned long vmem_base_addr;
	unsigned long vmem_high_addr;
	unsigned short vmem_width;
	unsigned short vmem_height;
	unsigned short input_res_x;
	unsigned short input_res_y;
	unsigned char input_num;
	unsigned char input_format;
	/*
		supported:
		- V4L2_PIX_FMT_RGB565
		- V4L2_PIX_FMT_RGB32
		- V4L2_PIX_FMT_YUYV
	*/
	unsigned long output_format;
	unsigned long out_byte_align;
	unsigned char scale_fraction_bits;
};

#endif /* __LW_H__ */
