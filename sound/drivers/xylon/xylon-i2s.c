/*
 * xylon-i2s.c sound driver for logiI2S FPGA IP core
 *
 * Driver supports following functionality:
 * - playing PCM stream samples received from ALSA sound application
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <sound/initval.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "logi-i2s.h"

#ifdef DEBUG
#define driver_devel(f, x...) \
	do { \
		printk(KERN_INFO "%s: " f, __func__,## x); \
	} while (0)
#else
#define driver_devel(format, ...)
#endif

#define DRIVER_NAME "xylon-i2s"
#define DEVICE_NAME "logii2s"

#define PERIOD_BYTES_MIN (1 * 4)
#define PERIODS_MIN 32
#define PERIODS_MAX 64
#define BUFFER_SIZE (PERIODS_MIN * LOGII2S_FIFO_SIZE_MAX * 4)
#define MAX_BUFFER_SIZE (PERIODS_MAX * LOGII2S_FIFO_SIZE_MAX * 4)

struct snd_i2s_pcm_port {
	struct snd_pcm_substream *substream;
	struct logii2s_port *i2s_port;
	spinlock_t lock;
	unsigned int buf_size;
	unsigned int buf_pos;
	unsigned char xfer_dir;
};

extern void audio_codec_set_sample_rate(int);
extern void audio_codec_init(void);

/*-----------------------------------------------------------------*/

static irqreturn_t i2s_irq_handler(int irq, void *priv)
{
	struct logii2s_device *logii2s_dev = priv;
	struct logii2s_port *port;
	struct snd_i2s_pcm_port *i2s_pcm;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned long ir, data_xfer, size;
	int i;

	driver_devel("\n");

	ir = logii2s_get_device_ir(logii2s_dev);

	for (i = 0; i < logii2s_dev->instances; i++) {
		if (ir & (1 << i)) {
			port = logii2s_dev->port[i];
			i2s_pcm = port->private;

			substream = i2s_pcm->substream;
			if (substream) {
				runtime = substream->runtime;
				if (runtime) {
					if (runtime->dma_area) {
						if ((i2s_pcm->buf_pos + (port->fifo_size * 4)) < i2s_pcm->buf_size) {
							size = 0;
						} else {
							size = (i2s_pcm->buf_size - i2s_pcm->buf_pos) / 4;
						}
						data_xfer = logii2s_port_transfer_data(port,
							(unsigned long *)(runtime->dma_area + i2s_pcm->buf_pos),
							size);
						i2s_pcm->buf_pos += data_xfer;
						if (i2s_pcm->buf_pos >= (i2s_pcm->buf_size - 1))
							i2s_pcm->buf_pos = 0;
						snd_pcm_period_elapsed(substream);
					}
				}
			}
		}
	}

	return IRQ_HANDLED;
}


static struct snd_pcm_hardware xylon_i2s_playback_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = (LOGII2S_FIFO_SIZE_MAX * 4),
	.periods_min = PERIODS_MIN,
	.periods_max = PERIODS_MAX,
	.fifo_size = 0,
};

static struct snd_pcm_hardware xylon_i2s_capture_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = (LOGII2S_FIFO_SIZE_MAX * 4),
	.periods_min = PERIODS_MIN,
	.periods_max = PERIODS_MAX,
	.fifo_size = 0,
};


static void xylon_i2s_start(struct snd_i2s_pcm_port *i2s_pcm)
{
	driver_devel("\n");

	if (i2s_pcm->xfer_dir == LOGII2S_TX_INSTANCE) {
		logii2s_port_unmask_int(i2s_pcm->i2s_port, LOGII2S_INT_FAE);
	} else if (i2s_pcm->xfer_dir == LOGII2S_RX_INSTANCE) {
		logii2s_port_unmask_int(i2s_pcm->i2s_port, LOGII2S_INT_FAF);
	}
	logii2s_port_enable_xfer(i2s_pcm->i2s_port);
}

static void xylon_i2s_stop(struct snd_i2s_pcm_port *i2s_pcm)
{
	driver_devel("\n");

	logii2s_port_disable_xfer(i2s_pcm->i2s_port);

	if (i2s_pcm->xfer_dir == LOGII2S_TX_INSTANCE) {
		logii2s_port_mask_int(i2s_pcm->i2s_port, LOGII2S_INT_MASK_ALL);
		logii2s_port_reset(i2s_pcm->i2s_port);
	} else if (i2s_pcm->xfer_dir == LOGII2S_RX_INSTANCE) {
		logii2s_port_mask_int(i2s_pcm->i2s_port, LOGII2S_INT_MASK_ALL);
		logii2s_port_reset(i2s_pcm->i2s_port);
	}
}

static int xylon_i2s_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_i2s_pcm_port *i2s_pcm = substream->private_data;
	struct logii2s_device *logii2s_dev = i2s_pcm->i2s_port->device;

	driver_devel("\n");

	if ((i2s_pcm->i2s_port->port_id > logii2s_dev->instances) ||
		(i2s_pcm->i2s_port->port_id < 0)) {
		pr_err("%s - Error invalid port index\n", __func__);
		return -ENODEV;
	}

	xylon_i2s_playback_hardware.fifo_size = i2s_pcm->i2s_port->fifo_size;

	runtime->hw = xylon_i2s_playback_hardware;

	logii2s_port_reset(i2s_pcm->i2s_port);
	i2s_pcm->xfer_dir = LOGII2S_TX_INSTANCE;
	i2s_pcm->substream = substream;

	runtime->private_data = i2s_pcm;

	return 0;
}

static int xylon_i2s_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_i2s_pcm_port *i2s_pcm = substream->private_data;
	struct logii2s_device *logii2s_dev = i2s_pcm->i2s_port->device;

	driver_devel("\n");

	if ((i2s_pcm->i2s_port->port_id > logii2s_dev->instances) ||
		(i2s_pcm->i2s_port->port_id < 0)) {
		pr_err("%s - Error invalid port index\n", __func__);
		return -ENODEV;
	}

	xylon_i2s_capture_hardware.fifo_size = i2s_pcm->i2s_port->fifo_size;

	runtime->hw = xylon_i2s_capture_hardware;

	logii2s_port_reset(i2s_pcm->i2s_port);
	i2s_pcm->xfer_dir = LOGII2S_RX_INSTANCE;
	i2s_pcm->substream = substream;

	runtime->private_data = i2s_pcm;

	return 0;
}

static int xylon_i2s_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_i2s_pcm_port *i2s_pcm = runtime->private_data;

	driver_devel("\n");

	logii2s_port_reset(i2s_pcm->i2s_port);

	return 0;
}

static int xylon_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	int ret;

	driver_devel("\n");

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));

	driver_devel("rate %d, format %d\n",
		params_rate(hw_params), params_format(hw_params));

	return ret;
}

static int xylon_i2s_hw_free(struct snd_pcm_substream *substream)
{
	driver_devel("\n");

	return snd_pcm_lib_free_pages(substream);
}

static int xylon_i2s_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_i2s_pcm_port *i2s_pcm = runtime->private_data;

	driver_devel("\n");

	if (runtime->dma_area == NULL) {
		driver_devel("memory not available\n");
		return -EINVAL;
	}

	audio_codec_set_sample_rate(runtime->rate);
	i2s_pcm->buf_size = snd_pcm_lib_buffer_bytes(substream);
	logii2s_port_init_clock(i2s_pcm->i2s_port, runtime->rate);

	return 0;
}

static int xylon_i2s_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_i2s_pcm_port *i2s_pcm = runtime->private_data;

	driver_devel("\n");

	spin_lock(&i2s_pcm->lock);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
			driver_devel("START / RESUME %d\n", i2s_pcm->i2s_port->port_id);
			i2s_pcm->buf_pos = 0;
			xylon_i2s_start(i2s_pcm);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			driver_devel("STOP / SUSPEND %d\n", i2s_pcm->i2s_port->port_id);
			xylon_i2s_stop(i2s_pcm);
			break;

		default:
			return -EINVAL;
	}

	spin_unlock(&i2s_pcm->lock);

	return 0;
}

static snd_pcm_uframes_t xylon_i2s_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_i2s_pcm_port *i2s_pcm = runtime->private_data;
	snd_pcm_uframes_t p;

	driver_devel("\n");

	spin_lock(&i2s_pcm->lock);

	p = bytes_to_frames(runtime, i2s_pcm->buf_pos);

	if (p >= runtime->buffer_size)
		p = 0;

	spin_unlock(&i2s_pcm->lock);

	return p;
}

static struct snd_pcm_ops xylon_i2s_playback_ops = {
	.open = xylon_i2s_playback_open,
	.close = xylon_i2s_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = xylon_i2s_hw_params,
	.hw_free = xylon_i2s_hw_free,
	.prepare = xylon_i2s_pcm_prepare,
	.trigger = xylon_i2s_pcm_trigger,
	.pointer = xylon_i2s_pcm_pointer,
};

static struct snd_pcm_ops xylon_i2s_capture_ops = {
	.open = xylon_i2s_capture_open,
	.close = xylon_i2s_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = xylon_i2s_hw_params,
	.hw_free = xylon_i2s_hw_free,
	.prepare = xylon_i2s_pcm_prepare,
	.trigger = xylon_i2s_pcm_trigger,
	.pointer = xylon_i2s_pcm_pointer,
};

static void xylon_i2s_private_free(struct snd_pcm *pcm)
{
	struct snd_i2s_pcm_port *i2s_pcm = pcm->private_data;

	driver_devel("\n");

	kfree(i2s_pcm);
}

static int __init xylon_i2s_pcm_new(struct snd_i2s_pcm_port *i2s_pcm,
	struct snd_card *card, int id)
{
	struct snd_pcm *pcm;
	int playback_new, capture_new;
	int err, tx;
	char card_id[25];

	driver_devel("\n");

	tx = logii2s_port_direction(i2s_pcm->i2s_port);

	if (tx) {
		playback_new = 1;
		capture_new = 0;
		sprintf(card_id, DEVICE_NAME"-tx-%d", id);
	} else {
		playback_new = 0;
		capture_new = 1;
		sprintf(card_id, DEVICE_NAME"-rx-%d", id);
	}

	err = snd_pcm_new(card, card_id, id, playback_new, capture_new, &pcm);
	if (err < 0) {
		pr_err("%s - Error new PCM\n", __func__);
		return err;
	}

	if (tx) {
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &xylon_i2s_playback_ops);
		sprintf(pcm->name, DEVICE_NAME"-tx-%d PCM", id);
	} else {
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &xylon_i2s_capture_ops);
		sprintf(pcm->name, DEVICE_NAME"-rx-%d PCM", id);
	}

	pcm->private_data = i2s_pcm;
	pcm->private_free = xylon_i2s_private_free;
	pcm->info_flags = 0;
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
		snd_dma_continuous_data(GFP_KERNEL), BUFFER_SIZE, MAX_BUFFER_SIZE);

	return 0;
}

static int __init xylon_i2s_get_device_parameters(
	struct platform_device *pdev, struct logii2s_device *logii2s_dev)
{
#ifdef CONFIG_OF
	const unsigned int *prop;
	int size, id;
	char instance[20+1];

	driver_devel("\n");

	id = 0;
	sprintf(instance, "instance_%d", id);
	while (of_find_node_by_name(pdev->dev.of_node, instance)) {
		logii2s_dev->instances++;
		id++;
		sprintf(instance, "instance_%d", id);
	}

	prop = of_get_property(pdev->dev.of_node, "system-clock-frequency", &size);
	if (prop) {
		logii2s_dev->system_clock = be32_to_cpup(prop);
	} else {
		logii2s_dev->system_clock = LOGII2S_SYSTEM_CLOCK;
	}
#else
	struct xylon_i2s_platform_data *pdata;

	driver_devel("\n");

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err("%s - Error no platform data\n", __func__);
		return -ENODEV;
	}

	logii2s_dev->instances = pdata->instances;
	logii2s_dev->system_clock = pdata->system_clock;
#endif

	return 0;
}

static void __init xylon_i2s_get_port_parameters(
	struct platform_device *pdev, struct logii2s_port *port, int id)
{
#ifdef CONFIG_OF
	struct device_node *instance_dn;
	const unsigned int *prop;
	int size;
	char instance[20+1];

	driver_devel("\n");

	sprintf(instance, "instance_%d", id);
	instance_dn = of_find_node_by_name(pdev->dev.of_node, instance);
	if (!instance_dn)
		return;

	prop = of_get_property(instance_dn, "i2s-clock-frequency", &size);
	if (prop) {
		port->i2s_clock = be32_to_cpup(prop);
	} else {
		port->i2s_clock = LOGII2S_BUS_CLOCK;
	}
	prop = of_get_property(instance_dn, "fifo-size", &size);
	if (prop) {
		port->fifo_size = be32_to_cpup(prop);
	} else {
		port->fifo_size = LOGII2S_FIFO_SIZE;
	}
	prop = of_get_property(instance_dn, "almost-full-level", &size);
	if (prop) {
		port->almost_full = be32_to_cpup(prop);
	} else {
		port->almost_full = LOGII2S_ALMOST_FULL_SIZE;
	}
	prop = of_get_property(instance_dn, "almost-empty-level", &size);
	if (prop) {
		port->almost_empty = be32_to_cpup(prop);
	} else {
		port->almost_empty = LOGII2S_ALMOST_EMPTY_SIZE;
	}
#else
	struct xylon_i2s_platform_data *pdata = pdev->dev.platform_data;

	driver_devel("\n");

	port->i2s_clock = pdata->instance_pdata[id].i2s_clock;
	port->fifo_size = pdata->instance_pdata[id].fifo_size;
	port->almost_full = pdata->instance_pdata[id].almost_full;
	port->almost_empty = pdata->instance_pdata[id].almost_empty;
#endif
}

static int xylon_i2s_probe(struct platform_device *pdev)
{
	struct resource *iomem;
	struct logii2s_device *logii2s_dev;
	struct snd_card *card;
	struct snd_i2s_pcm_port *i2s_pcm;
	struct snd_i2s_pcm_port *i2s_pcm_ptr[LOGII2S_MAX_INSTANCES] = {0};
	int i, err, irq;

	driver_devel("\n");

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		pr_err(KERN_ERR "%s - Error platform resource\n", __func__);
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("%s - Error platform IRQ\n", __func__);
		return -ENODEV;
	}

	logii2s_dev = kzalloc(sizeof(struct logii2s_device), GFP_KERNEL);
	if (!logii2s_dev) {
		pr_err("%s - Error internal data allocation\n", __func__);
		err = -ENOMEM;
		goto err_mem;
	}

	logii2s_dev->regs_base_phys = iomem->start;
	logii2s_dev->regs_size =
		resource_size(platform_get_resource(pdev, IORESOURCE_MEM, 0));
	logii2s_dev->regs_base_virt =
		ioremap_nocache(logii2s_dev->regs_base_phys, logii2s_dev->regs_size);
	if (!logii2s_dev->regs_base_virt) {
		pr_err("%s - Error ioremap\n", __func__);
		err = -ENOMEM;
		goto clean_res;
	}

	err = xylon_i2s_get_device_parameters(pdev, logii2s_dev);
	if (err) {
		pr_err("%s - Error device parameters\n", __func__);
		goto clean_res;
	}

	logii2s_dev->port =
		kzalloc((sizeof(struct logii2s_port *) * logii2s_dev->instances),
		GFP_KERNEL);
	if (!logii2s_dev->port) {
		pr_err("%s - Error no memory\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < logii2s_dev->instances; i++) {
		logii2s_dev->port[i] =
			kzalloc(sizeof(struct logii2s_port), GFP_KERNEL);
		if (!logii2s_dev->port[i]) {
			pr_err("%s - Error no memory\n", __func__);
			return -ENOMEM;
		}
	}

	err = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			THIS_MODULE, 0, &card);
	if (err < 0) {
		pr_err("%s - Error sound card creation\n", __func__);
		goto clean_res;
	}

	err = request_irq(irq, i2s_irq_handler, 0, DRIVER_NAME,
		(void *)logii2s_dev);
	if (err != 0) {
		pr_err("%s - Error irq\n", __func__);
		goto clean_card;
	}
	logii2s_dev->irq = irq;

	for (i = 0; i < logii2s_dev->instances; i++) {
		xylon_i2s_get_port_parameters(pdev, logii2s_dev->port[i], i);

		logii2s_port_init(logii2s_dev, i);
		logii2s_port_reset(logii2s_dev->port[i]);

		i2s_pcm = kzalloc(sizeof(*i2s_pcm), GFP_KERNEL);
		if (!i2s_pcm) {
			pr_err("Error I2S PCM allocation\n");
			goto clean_pcm_ports;
		}

		i2s_pcm_ptr[i] = i2s_pcm;
		i2s_pcm->i2s_port = logii2s_dev->port[i];
		i2s_pcm->i2s_port->private = i2s_pcm;

		spin_lock_init(&i2s_pcm->lock);

		err = xylon_i2s_pcm_new(i2s_pcm, card, i);
		if (err < 0) {
			pr_err("%s - Error PCM creation\n", __func__);
			goto clean_pcm_ports;
		}
	}

	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, DRIVER_NAME);
	sprintf(card->longname, DRIVER_NAME"-"DEVICE_NAME" %i", pdev->id + 1);

	card->private_data = (void *)logii2s_dev;

	snd_card_set_dev(card, &pdev->dev);

	err = snd_card_register(card);
	if (err != 0) {
		pr_err("%s - Error card register\n", __func__);
		goto clean_pcm_ports;
	}

	platform_set_drvdata(pdev, card);

	audio_codec_init();

	return 0;

clean_pcm_ports:
	for (i = 0; i < logii2s_dev->instances; i++) {
		if (i2s_pcm_ptr[i])
			kfree(i2s_pcm_ptr[i]);
	}

	free_irq(irq, logii2s_dev);

clean_card:
	snd_card_free(card);

clean_res:
	if (logii2s_dev->port) {
		for (i = 0; i < logii2s_dev->instances; i++) {
			if (logii2s_dev->port[i])
				kfree(logii2s_dev->port[i]);
		}
		kfree(logii2s_dev->port);
	}
	kfree(logii2s_dev);

err_mem:
	return err;
}

static int __exit xylon_i2s_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct logii2s_device *logii2s_dev = card->private_data;
	int i;

	driver_devel("\n");

	free_irq(logii2s_dev->irq, logii2s_dev);

	for (i = 0; i < logii2s_dev->instances; i++) {
		if (logii2s_dev->port[i])
			kfree(logii2s_dev->port[i]);
	}
	kfree(logii2s_dev->port);
	kfree(logii2s_dev);

	snd_card_free(card);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id xylon_i2s_of_match[] = {
	{ .compatible = "xylon,logii2s-2.00.a" },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, xylon_i2s_of_match);
#endif

static struct platform_driver xylon_i2s_platform_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = xylon_i2s_of_match,
#endif
	},
	.probe = xylon_i2s_probe,
	.remove = __exit_p(xylon_i2s_remove),
};

static int __init xylon_i2s_init(void)
{
	return platform_driver_register(&xylon_i2s_platform_driver);
}

static void __exit xylon_i2s_exit(void)
{
	platform_driver_unregister(&xylon_i2s_platform_driver);
}

module_init(xylon_i2s_init);
module_exit(xylon_i2s_exit);

MODULE_DESCRIPTION("logiI2S driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xylon d.o.o.");
MODULE_ALIAS("platform:"DRIVER_NAME);
