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


#include <linux/kernel.h>
#include "logi-i2s.h"


/* Default sampling rate in Hz. */
#define DEFAULT_SAMPLE_RATE 44100


/**
*
* This function initializes I2S device port register offsets.
*
* @param    device is a pointer to the struct logii2s_device structure
*
* @return   Pointer to the logii2s_port structure.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_init(struct logii2s_device *device, int instance)
{
    int id = instance;

    device->port[id]->device = device;
    device->port[id]->port_base_virt = device->regs_base_virt +
        LOGII2S_INSTANCE_REG_STEP + (id * LOGII2S_INSTANCE_REG_STEP);

    device->port[id]->port_id = id;
}

/**
*
* This function resets the I2S device port - resets the FIFO, bit clock and
* transmission/reception logic, clears Interrupt Enable Register and
* Interrupt Clear Register.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_reset(struct logii2s_port *port)
{
    /* Reset I2S port */
    logii2s_write32(port->port_base_virt, LOGII2S_CTRL, LOGII2S_CTRL_SWR);
    /* Mask INT */
    logii2s_write32(port->port_base_virt, LOGII2S_IMR, LOGII2S_INT_MASK_ALL);
    /* Clear INT */
    logii2s_write32(port->port_base_virt, LOGII2S_ISR, LOGII2S_INT_ACK_ALL);
}

/**
*
* This function returns sample rate index.
*
* @param    sample_rate is a desired sampling rate in Hz.
*
* @return   If sampling rate is valid, it returns sample rate index.
*           If sampling rate is not supported, it returns -1.
*
* @note     None.
*
******************************************************************************/
int logii2s_port_check_sample_rate(unsigned long sample_rate)
{
    unsigned long logii2s_sample_rates[] = {
        8000,
        11025,
        16000,
        22050,
        44100,
        48000,
        96000,
        192000
    };
    int i, size;

    size = sizeof(logii2s_sample_rates) / sizeof(logii2s_sample_rates[0]);

    for (i = 0; i < size; i++) {
        if (sample_rate == logii2s_sample_rates[i])
            return i;
    }

    return -1;
}

/**
*
*
*
* @param    system_clock is I2S HW system clock.
* @param    i2s_clock is I2S HW system clock.
* @param    ws_master is I2S word select master flag.
* @param    prescale is prescaler value set in HW device.
* @param    sample_rate is sampling rate.
* @param    ws_left
* @param    ws_right
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
static void logii2s_port_calc_ws_value(
    unsigned long system_clock, unsigned long i2s_clock,
    unsigned long clock_master,
    unsigned long prescale, unsigned long sample_rate,
    unsigned long *ws_left, unsigned long *ws_right)
{
    if (clock_master)
        i2s_clock = system_clock / (2 * prescale);

    *ws_left = i2s_clock / (2 * sample_rate);
    *ws_right = *ws_left;
}

/**
*
* This function sets the word select length for the right and the left channel
* depending on the selected sample rate, only if the I2S device port is
* configured as a clock master. The word select length parameter decides the
* number of cycles before the word select signal switches for right respective
* left side.
* If the I2S device port can't drive requested sampling rate, sampling rate will
* be set to 48000Hz.
*
* @param    port is a pointer to the logii2s_port information structure.
* @param    sample_rate is a desired sampling rate.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_init_clock(struct logii2s_port *port,
    unsigned long sample_rate)
{
    struct logii2s_device *device;
    unsigned long ctrl;
    unsigned long prescale;
    unsigned long ws_left;
    unsigned long ws_right;
    unsigned int direction;
    int i;

    device = port->device;
    direction = logii2s_port_direction(port);
    ctrl = logii2s_read32(port->port_base_virt, LOGII2S_CTRL);

    i = logii2s_port_check_sample_rate(sample_rate);
    if (i < 0) {
        sample_rate = DEFAULT_SAMPLE_RATE;
        pr_info("Unsupported sampling rate, set to default %dkHz\n",
            DEFAULT_SAMPLE_RATE);
    }

    prescale = (device->system_clock / (2 * port->i2s_clock));
    logii2s_port_calc_ws_value(
        device->system_clock, port->i2s_clock,
        (ctrl & LOGII2S_CTRL_CLKMASTER),
        prescale, sample_rate,
        &ws_left, &ws_right);
    ws_left <<= 16;
    ws_right <<= 8;

    ctrl &= 0xFF0000FF;
    ctrl |= (ws_left | ws_right);

    logii2s_write32(port->port_base_virt, LOGII2S_PRESCALE, prescale);
    logii2s_write32(port->port_base_virt, LOGII2S_CTRL, ctrl);
}

/**
*
* This function gets the content of the Interrupt Unit Register.
* This register indicates which device instance has generated an interrupt.
* This information is used in further devices registers accessing.
*
* @param    device is a pointer to the struct logii2s_device structure.
*
* @return   I2S device instance interrupt register.
*
* @note     None.
*
******************************************************************************/
unsigned long logii2s_get_device_ir(struct logii2s_device *device)
{
    return logii2s_read32(device->regs_base_virt, LOGII2S_INT_REG);
}

/**
*
* This function enables the specified interrupts in the Interrupt Mask Register.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    bit_mask is the bitmask of the interrupts to be enabled.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_unmask_int(struct logii2s_port *port, unsigned long bit_mask)
{
    unsigned long imr;

    imr = logii2s_read32(port->port_base_virt, LOGII2S_IMR);
    imr &= (~bit_mask);
    logii2s_write32(port->port_base_virt, LOGII2S_IMR, imr);
}

/**
*
* This function disables specified interrupts in the Interrupt Enable Register.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    bit_mask is the bitmask of the interrupts to be enabled.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_mask_int(struct logii2s_port *port, unsigned long bit_mask)
{
    unsigned long imr;

    imr = logii2s_read32(port->port_base_virt, LOGII2S_IMR);
    imr |= bit_mask;
    logii2s_write32(port->port_base_virt, LOGII2S_IMR, imr);
}

/**
*
* This function gets the content of the Interrupt Status Register.
* This register indicates the status of interrupt sources for the device.
* The status is independent of whether interrupts are enabled such
* that the status register may also be polled when interrupts are not enabled.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   A status which contains the value read from the Interrupt
*           Status Register.
*
* @note     None.
*
******************************************************************************/
unsigned long logii2s_port_get_isr(struct logii2s_port *port)
{
    return logii2s_read32(port->port_base_virt, LOGII2S_ISR);
}

/**
*
* This function clears the specified interrupts in the Interrupt Clear Register.
* The interrupt is cleared by writing to this register with the bits
* to be cleared set to a 1 and all others to 0.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    bit_mask is the bitmask for interrupts to be cleared.
*           "1" clears the interrupt.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_clear_isr(struct logii2s_port *port, unsigned long bit_mask)
{
    logii2s_write32(port->port_base_virt, LOGII2S_ISR, bit_mask);
}

/**
*
* This function enables reception.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_enable_xfer(struct logii2s_port *port)
{
    unsigned long ctrl;

    ctrl = logii2s_read32(port->port_base_virt, LOGII2S_CTRL);
    ctrl |= LOGII2S_CTRL_ENABLE;
    logii2s_write32(port->port_base_virt, LOGII2S_CTRL, ctrl);
}

/**
*
* This function disables reception.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   None.
*
* @note     When reception is disabled, the corresponding right side word is
*           received before stopping.
*
******************************************************************************/
void logii2s_port_disable_xfer(struct logii2s_port *port)
{
    unsigned long ctrl;

    ctrl = logii2s_read32(port->port_base_virt, LOGII2S_CTRL);
    ctrl &= (~LOGII2S_CTRL_ENABLE);
    logii2s_write32(port->port_base_virt, LOGII2S_CTRL, ctrl);
}

/**
*
* This function checks I2S device port direction configuration.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   0 if receiver or 1 if transmitter.
*
* @note     None.
*
******************************************************************************/
unsigned int logii2s_port_direction(struct logii2s_port *port)
{
    return (logii2s_read32(port->port_base_virt, LOGII2S_CTRL)
        & LOGII2S_CTRL_DIR) ? LOGII2S_TX_INSTANCE : LOGII2S_RX_INSTANCE;
}

/**
*
* This function gets the I2S core's hardware version.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   A 32-bit value which contains the content of the
*           Hardware Version Register.
*
* @note     16 upper bits represents the major version and the 16 lower
*           bits represent the minor version.
*
******************************************************************************/
unsigned long logii2s_port_get_version(struct logii2s_port *port)
{
    return logii2s_read32(port->port_base_virt, LOGII2S_HW_VERSION_REG);
}

/**
*
* This function reads one data word from the FIFO register.
*
* @param    port is a pointer to the logii2s_port structure.
*
* @return   A 32-bit value representing the content read from the FIFO register.
*
* @note     None.
*
******************************************************************************/
unsigned long logii2s_port_read_fifo_word(struct logii2s_port *port)
{
    return logii2s_read32(port->port_base_virt, LOGII2S_FIFO);
}

/**
*
* This function writes one data word to the FIFO register.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    data is a 32-bit value to write to the FIFO register.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_write_fifo_word(struct logii2s_port *port, unsigned long data)
{
    logii2s_write32(port->port_base_virt, LOGII2S_FIFO, data);
}

/**
*
* This function reads count of the data words from the FIFO register.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    data is a pointer to 32-bit value data for reading from the FIFO
*           register.
* @param    count is number of data words.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_read_fifo(struct logii2s_port *port,
    unsigned long *data, unsigned long count)
{
    int i;

    for (i = 0; i < count; i++)
        data[i] = logii2s_read32(port->port_base_virt, LOGII2S_FIFO);
}

/**
*
* This function writes count of the data words to the FIFO register.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    data is a pointer to 32-bit value data for writing to the FIFO register.
* @param    count is number of data words.
*
* @return   None.
*
* @note     None.
*
******************************************************************************/
void logii2s_port_write_fifo(struct logii2s_port *port,
    unsigned long *data, unsigned long count)
{
    int i;

    for (i = 0; i < count; i++)
        logii2s_write32(port->port_base_virt, LOGII2S_FIFO, data[i]);
}

/**
*
* This function performs data transfer to or from FIFO register.
*
* @param    port is a pointer to the logii2s_port structure.
* @param    data is a pointer to 32-bit value data for writing or reading the
*           FIFO register.
*
* @return   Number of transfered bytes.
*
* @note     None.
*
******************************************************************************/
unsigned long logii2s_port_transfer_data(struct logii2s_port *port,
    unsigned long *data, unsigned long size)
{
    unsigned int direction;
    unsigned int samples;

    direction = logii2s_port_direction(port);

    if (direction == LOGII2S_TX_INSTANCE) {
        if (logii2s_port_get_isr(port) & LOGII2S_INT_FE) {
            samples = port->fifo_size;
        } else if (logii2s_port_get_isr(port) & LOGII2S_INT_FAE) {
            samples = port->fifo_size - port->almost_empty;
        } else {
            samples = 0;
        }
        if (size != 0) {
            if ((samples != 0) && (size < samples)) {
                samples = size;
            }
        }
        logii2s_port_write_fifo(port, data, samples);
        logii2s_port_clear_isr(port, (LOGII2S_INT_FE | LOGII2S_INT_FAE));
    } else if (direction == LOGII2S_RX_INSTANCE) {
        if (logii2s_port_get_isr(port) & LOGII2S_INT_FF) {
            samples = port->fifo_size;
        } else if (logii2s_port_get_isr(port) & LOGII2S_INT_FAF) {
            samples = port->almost_full;
        } else {
            samples = 0;
        }
        if (size != 0) {
            if ((samples != 0) && (size < samples)) {
                samples = size;
            }
        }
        logii2s_port_read_fifo(port, data, samples);
        logii2s_port_clear_isr(port, (LOGII2S_INT_FF | LOGII2S_INT_FAF));
    } else {
        return 0;
    }

    return samples * 4;
}
