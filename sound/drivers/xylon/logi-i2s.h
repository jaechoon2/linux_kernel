/*****************************************************************************
** Copyright 2012 - Xylon d.o.o.
**
**  This program is free software: you can redistribute it and/or modify
**  it under the terms of the GNU Lesser General Public License version 3
**  as published by the Free Software Foundation.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU Lesser General Public License for more details.
**
**  You should have received a copy of the GNU Lesser General Public License
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************/

/**
* @file logi-i2s.h
*
* This file contains the low-level driver functions for controling logiI2S
* device. It is the driver for an logiI2S master or slave device.
* The abbreviation I2S bus stands for Inter-IC Sound bus.
* The I2S bus transports audio data using 3 lines: bit clock, word select signal
* and serial data line. Each individual transmitter/receiver can be configured
* as either clock master or slave. If a device is configured to be a clock
* master, the bit clock and word select signal are derived from the system
* clock.
*
* The driver handles the following interrupts:
*  - RX/TX FIFO Full
*  - RX/TX FIFO Almost Full
*  - RX/TX FIFO Almost Empty
*  - TX FIFO Empty
*
* The logiI2S peripheral device driver handles logical functionality for
* controlling the logiI2S FPGA IP core device.
* logiI2S driver parameters are stored inside of driver internal structures
* (struct logii2s_device and struct logii2s_port), which are set at
* initialization procedure.
* Parameters are passed from upper layer (upper device driver) and stored.
* In moment when peripheral device is initialized, parameters are properly
* formatted and written into peripheral device.
* Parameters are stored inside of structures for easier interfacing with the
* upper layer, that is when peripheral configuration must be read for various
* purposes.
*
*****************************************************************************/

#ifndef __LOGII2S_H__
#define __LOGII2S_H__


#include <asm/io.h>


/* I2S device parameters */
#define LOGII2S_MAX_INSTANCES  8
/* Number of registers per I2S device port */
#define LOGII2S_INSTANCE_REGS  5
/* Register size (4bytes) */
#define LOGII2S_REG_STEP       8
/* Instance registers distance */
#define LOGII2S_INSTANCE_REG_STEP (8 * LOGII2S_REG_STEP)

/* Register definitions */
/* I2S Hardware Version Register */
#define LOGII2S_HW_VERSION_REG (0 * LOGII2S_REG_STEP)
/* Instance Interrupt Register */
#define LOGII2S_INT_REG        (1 * LOGII2S_REG_STEP)

/* Holds prescale value, if clock master */
#define LOGII2S_PRESCALE       (0 * LOGII2S_REG_STEP)
/* Control Register */
#define LOGII2S_CTRL           (1 * LOGII2S_REG_STEP)
 /* Interrupt Mask Register */
#define LOGII2S_IMR            (2 * LOGII2S_REG_STEP)
/* Interrupt Status Register */
#define LOGII2S_ISR            (3 * LOGII2S_REG_STEP)
/* Read/Write FIFO Register */
#define LOGII2S_FIFO           (4 * LOGII2S_REG_STEP)

/* Interrupt Register bit masks */
#define LOGII2S_INT_FF       0x01 /* RX/TX FIFO Full Mask */
#define LOGII2S_INT_FAF      0x02 /* RX/TX FIFO Almost Full Mask */
#define LOGII2S_INT_FE       0x04 /* RX/TX FIFO Empty Mask */
#define LOGII2S_INT_FAE      0x08 /* RX/TX FIFO Almost Empty Mask */
#define LOGII2S_INT_MASK_ALL 0xFF /* All interrupts masked */
#define LOGII2S_INT_ACK_ALL  0xFF /* All interrupts cleared */

/* Control Register masks */
/* I2S Instance Enable */
#define LOGII2S_CTRL_ENABLE    0x00000001
/* FIFO Clear Mask */
#define LOGII2S_CTRL_FIFO_CLR  0x00000002
/* Soft reset Mask */
#define LOGII2S_CTRL_SWR       0x00000004
/* Not used Mask */
#define LOGII2S_CTRL_NONE      0x00000008
/* Left/right channel swap */
#define LOGII2S_CTRL_LRSWAP    0x01000000
/* I2S instance is clock master */
#define LOGII2S_CTRL_CLKMASTER 0x10000000
/* I2S instance is word select master */
#define LOGII2S_CTRL_WSMASTER  0x20000000
/* I2S instance direction (0=RX, 1=TX) */
#define LOGII2S_CTRL_DIR       0x40000000
/* I2S instance current word select */
#define LOGII2S_CTRL_WS        0x80000000


#define LOGII2S_RX_INSTANCE 0
#define LOGII2S_TX_INSTANCE 1

#define LOGII2S_LEFT_JUSTIFY  0x04000000
#define LOGII2S_RIGHT_JUSTIFY 0x08000000


/* Default FIFO size in words (512 * 4 = 2048 bytes) */
#define LOGII2S_FIFO_SIZE         512
#define LOGII2S_FIFO_SIZE_MAX     4096
/* Default FIFO levels */
#define LOGII2S_ALMOST_EMPTY_SIZE 80
#define LOGII2S_ALMOST_FULL_SIZE  170
/* Default I2S system hw clocks */
#define LOGII2S_SYSTEM_CLOCK 100000000
#define LOGII2S_BUS_CLOCK    12500000

/* struct logii2s_device type declaration */
struct logii2s_device;

/* struct logii2s_port private parameter structure */
struct logii2s_port {
    struct logii2s_device *device;
    void *port_base_virt;
    void *private;
    unsigned long i2s_clock;
    unsigned short fifo_size;
    unsigned short almost_full;
    unsigned short almost_empty;
    unsigned char port_id;
};

/* struct logii2s_device private parameter structure */
struct logii2s_device {
    unsigned long regs_base_phys;
    unsigned long regs_size;
    void *regs_base_virt;
    struct logii2s_port **port;
    int irq;
    unsigned long system_clock;
    unsigned char instances;
};


#define logii2s_read32(base_virt, offset) \
    readl(base_virt + offset)
#define logii2s_write32(base_virt, offset, val) \
    writel(val, (base_virt + offset))


/* HW init functions */
void logii2s_port_init(struct logii2s_device *device, int instance);
void logii2s_port_reset(struct logii2s_port *port);
int logii2s_check_sample_rate(unsigned long sample_rate);
void logii2s_port_init_clock(struct logii2s_port *port,
    unsigned long sample_rate);

/* transfer direction functions */
unsigned int logii2s_port_direction(struct logii2s_port *port);

/* register access functions */
unsigned long logii2s_port_get_version(struct logii2s_port *port);

/* interrupt functions */
unsigned long logii2s_get_device_ir(struct logii2s_device *device);
void logii2s_port_unmask_int(struct logii2s_port *port, unsigned long bit_mask);
void logii2s_port_mask_int(struct logii2s_port *port, unsigned long bit_mask);
unsigned long logii2s_port_get_isr(struct logii2s_port *port);
void logii2s_port_clear_isr(struct logii2s_port *port, unsigned long bit_mask);

/* functions to start transfer */
void logii2s_port_enable_xfer(struct logii2s_port *port);
void logii2s_port_disable_xfer(struct logii2s_port *port);

unsigned long logii2s_port_read_fifo_word(struct logii2s_port *port);
void logii2s_port_write_fifo_word(struct logii2s_port *port,
    unsigned long data);
void logii2s_port_read_fifo(struct logii2s_port *port,
    unsigned long *data, unsigned long count);
void logii2s_port_write_fifo(struct logii2s_port *port,
    unsigned long *data, unsigned long count);
unsigned long logii2s_port_transfer_data(struct logii2s_port *port,
    unsigned long *data, unsigned long size);

#endif /* __LOGII2S_H__ */
