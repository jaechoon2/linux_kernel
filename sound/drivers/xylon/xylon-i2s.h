/*
 * xylon-i2s.h FPGA logiI2S driver header file
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
 *  FPGA logiI2S
 */

#ifndef __XYLONI2S_H__
#define __XYLONI2S_H__


struct xylon_i2s_instance_platform_data {
	unsigned long i2s_clock;
	unsigned short fifo_size;
	unsigned short almost_full;
	unsigned short almost_empty;
};

struct xylon_i2s_platform_data {
	unsigned char instances;
	unsigned long system_clock;
	struct xylon_i2s_instance_platform_data *instance_pdata;
};

#endif /* __XYLONI2S_H__ */
