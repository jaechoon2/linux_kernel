/*
 * lw.c FPGA logiWIN Video Input driver
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

#include <linux/module.h>
#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>

#include "lw.h"
#include "logiwin.h"

#define dbg(...) //printk(KERN_DEBUG __VA_ARGS__)

#define MAX_OUT_RES_X 2048
#define MAX_OUT_RES_Y 2048

#define LOGIWIN_NAME "logiWIN driver"
#define LOGIWIN_VERSION_CODE 0x02
#define MAX_DMA_FRAMEBUFFERS 3
#define LOGIWIN_FRAMEBUFFERS MAX_DMA_FRAMEBUFFERS
#define DRIVER_NAME "logiwin"

#define FLAG_DEVICE_IN_USE      0x01
#define FLAG_BUFFERS_AVAILABLE  0x02
#define FLAG_BUFFER_IN_SYNC     0x04
#define FLAG_DO_UPDATE          0x10
#define FLAG_DEINTERLACE        0x20


enum logiwin_stream_state {
	STREAM_OFF = 0,
	STREAM_ON
};

enum logiwin_frame_state {
	F_UNUSED = 0,
	F_QUEUED,
	F_GRABBING,
	F_DONE,
	F_ERROR
};

struct logiwin_frame {
	struct v4l2_buffer buf;
	struct list_head frame;
	enum logiwin_frame_state state;
	unsigned long vma_use_count;
	u32 buf_addr;
};

struct logiwin_video_norm {
	v4l2_std_id std;
	char *name;
	u16 width;
	u16 height;
};

struct logiwin_dma_addr {
	dma_addr_t pa;
	void *va;
};

struct logiwin_dma_handle {
	struct logiwin_dma_addr *dma_vbuff_addr;
	unsigned int dma_vbuff_size;
	unsigned char dma_vbuff_id;
};

struct logiwin_params {
	dma_addr_t regs_base_phys;
	dma_addr_t vmem_base_phys;
	void *regs_base_virt;
	void *vmem_base_virt;

	unsigned long regs_size;
	unsigned long vmem_buff_size;

	unsigned long output_format;
	unsigned short irq;
	unsigned char output_bpp;
};

struct logiwin {
	struct logiwin_params lw_params;
	struct logiwin_priv lw;

	struct video_device *video_dev;

	struct logiwin_dma_handle dma_handle;
	struct logiwin_frame *frame;
	struct list_head inqueue;
	struct list_head outqueue;

	struct v4l2_cropcap cropcap;
	struct v4l2_pix_format pix_format;
	struct v4l2_rect crop_rect;
	struct logiwin_video_norm video_norm;

	struct mutex fops_lock;
	spinlock_t queue_lock;

	struct tasklet_struct tasklet;
	wait_queue_head_t wait;

	enum logiwin_stream_state stream_state;

	unsigned int frames_num;

	unsigned char flags;
};

/*---------------------------------------------------------------------------*/

static void logiwin_set_video_norm(struct logiwin_video_norm *video_norm,
	int width, int height)
{
	video_norm->std = V4L2_STD_UNKNOWN;
	video_norm->name = "CUSTOM";
	video_norm->width = width;
	video_norm->height = height;
}

static int logiwin_update(struct logiwin *win)
{
	unsigned char weave_deinterlace = 0;
	unsigned long start_y;

	if (win->flags & FLAG_BUFFERS_AVAILABLE)
		logiwin_stop(&win->lw);

	logiwin_update_registers(&win->lw);

	if (weave_deinterlace) {
		start_y = 0;
		logiwin_enable_weave_deinterlace(&win->lw, weave_deinterlace);
	} else {
		start_y = 32;
	}

	logiwin_set_interpolation_points(&win->lw, 0, start_y);
	logiwin_set_pixel_alpha(&win->lw, 0xFF);

	if (win->flags & FLAG_BUFFERS_AVAILABLE)
		logiwin_start(&win->lw);

	return 0;
}

static void logiwin_empty_framequeues(struct logiwin *win)
{
	int i;

	spin_lock_bh(&win->queue_lock);

	INIT_LIST_HEAD(&win->inqueue);
	INIT_LIST_HEAD(&win->outqueue);

	for (i = 0; i < win->frames_num; i++) {
		win->frame[i].state = F_UNUSED;
		win->frame[i].buf.bytesused = 0;
	}

	spin_unlock_bh(&win->queue_lock);
}

static void logiwin_reinit_framequeues(struct logiwin *win)
{
	int i;

	INIT_LIST_HEAD(&win->inqueue);
	INIT_LIST_HEAD(&win->outqueue);

	for (i = 0; i < win->frames_num; i++) {
		list_add_tail(&win->frame[i].frame, &win->inqueue);
		win->frame[i].state = F_UNUSED;
		win->frame[i].buf.bytesused = 0;
	}
}

static unsigned long logiwin_request_buffers(struct logiwin *win,
	unsigned long count)
{
	int i;

	if (count > MAX_DMA_FRAMEBUFFERS) {
		printk(KERN_INFO "%s - buffer count to big, setting to %d\n",
			__func__, MAX_DMA_FRAMEBUFFERS);
		count = MAX_DMA_FRAMEBUFFERS;
	}

	win->dma_handle.dma_vbuff_addr = 
		kzalloc((sizeof(struct logiwin_dma_addr) * count), GFP_KERNEL);
	if (win->dma_handle.dma_vbuff_addr == NULL)
		return 0;

	win->dma_handle.dma_vbuff_id = 0;
	win->frames_num = 0;

	if (win->lw_params.vmem_base_phys) {
		for (i = 0; i < count; i++) {
			win->dma_handle.dma_vbuff_addr[i].pa =
				win->lw_params.vmem_base_phys + (i * win->dma_handle.dma_vbuff_size);
			win->dma_handle.dma_vbuff_addr[i].va =
				win->lw_params.vmem_base_virt + (i * win->dma_handle.dma_vbuff_size);
			win->frames_num++;
		}
	} else {
		/* try to create "count" buffers, or try to create at least one buffer */
		for (i = 0; i < count; i++) {
			win->dma_handle.dma_vbuff_addr[i].va = 
				dma_alloc_coherent(NULL,
					win->dma_handle.dma_vbuff_size,
					&win->dma_handle.dma_vbuff_addr[i].pa,
					GFP_KERNEL);
			dbg("%s - PA 0x%X : VA 0x%X\n", __func__,
				win->dma_handle.dma_vbuff_addr[i].pa,
				win->dma_handle.dma_vbuff_addr[i].va);
			if (win->dma_handle.dma_vbuff_addr[i].va == NULL)
				break;
			win->frames_num++;
		}
		dbg("%s - DMA frame buffers created %d\n", __func__, win->frames_num);
		if (win->frames_num == 0) {
			/* failed to create any buffer! free and go out */
			kfree(win->dma_handle.dma_vbuff_addr);
			return 0;
		}
	}

	/* create and fill frame buffer handles and parameters */
	win->frame = kzalloc((sizeof(struct logiwin_frame) * win->frames_num),
		GFP_KERNEL);
	if (win->frame == NULL)
		goto error_frame;
	/* initialize all frame buffers */
	for (i = 0; i < win->frames_num; i++) {
		win->frame[i].state = F_UNUSED;
		win->frame[i].buf_addr = win->dma_handle.dma_vbuff_addr[i].pa;
		win->frame[i].buf.index = i;
		win->frame[i].buf.m.offset = i * win->dma_handle.dma_vbuff_size;
		win->frame[i].buf.length = win->dma_handle.dma_vbuff_size;
		win->frame[i].buf.bytesused = 0;
		win->frame[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		win->frame[i].buf.sequence = 0;
		win->frame[i].buf.field = V4L2_FIELD_NONE;
		win->frame[i].buf.memory = V4L2_MEMORY_MMAP;
		win->frame[i].buf.flags = 0;
	}

	win->flags |= FLAG_BUFFERS_AVAILABLE;

	return win->frames_num;

error_frame:
	if (!win->lw_params.vmem_base_phys)
		for (i = win->frames_num-1; i >= 0; i--) {
			dma_free_coherent(NULL,
				win->dma_handle.dma_vbuff_size,
				win->dma_handle.dma_vbuff_addr[i].va,
				win->dma_handle.dma_vbuff_addr[i].pa);
		}
	kfree(win->dma_handle.dma_vbuff_addr);

	return -ENOMEM;
}

static void logiwin_release_buffers(struct logiwin *win)
{
	int i;

	kfree(win->frame);
	if (!win->lw_params.vmem_base_phys)
		for (i = win->frames_num-1; i >= 0; i--)
			dma_free_coherent(NULL,
				win->dma_handle.dma_vbuff_size,
				win->dma_handle.dma_vbuff_addr[i].va,
				win->dma_handle.dma_vbuff_addr[i].pa);
	kfree(win->dma_handle.dma_vbuff_addr);

	win->frames_num = 0;
	win->flags &= ~FLAG_BUFFERS_AVAILABLE;
}

/*---------------------------------------------------------------------------*/

static int logiwin_querycap(struct logiwin *win, struct v4l2_capability *cap)
{
	memset(cap, 0, sizeof(*cap));
	strncpy((char *)cap->card, "logiWIN", sizeof(cap->card)-1);
	strncpy((char *)cap->driver, DRIVER_NAME, sizeof(cap->card)-1);
	cap->version = LOGIWIN_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int logiwin_querystd(struct logiwin *win, v4l2_std_id *std)
{
	return -EINVAL;
}

static int logiwin_enum_fmt(struct logiwin *win, struct v4l2_fmtdesc *fmt)
{
	int id;
	/* this is on purpose... */
	char fmt_desc[][21] = {
		{"5:6:5, packed, RGB"},
		{"8:8:8:8, packed, RGB"},
		{"4:2:2, packed, UYVY"}
	};

	if (fmt->index != 0)
		return -EINVAL;

	switch (win->lw_params.output_format) {
		case V4L2_PIX_FMT_RGB565:
			id = 0;
			break;
		case V4L2_PIX_FMT_RGB32:
			id = 1;
			break;
		case V4L2_PIX_FMT_YUYV:
			id = 2;
			break;
		default:
			return -EINVAL;
	}

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = 0;
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strncpy((char *)fmt->description, &fmt_desc[id][0],
		sizeof(fmt->description)-1);
	fmt->pixelformat = win->pix_format.pixelformat;
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	return 0;
}

static int logiwin_g_fmt(struct logiwin *win, struct v4l2_format *format)
{
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memcpy(&(format->fmt.pix), &(win->pix_format),
		sizeof(struct v4l2_pix_format));

	return 0;
}

static int logiwin_s_fmt(struct logiwin *win, struct v4l2_format *format)
{
	struct v4l2_pix_format *pix = &(format->fmt.pix);
	unsigned long dummy;

	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (format->fmt.pix.pixelformat != win->lw_params.output_format)
		return -EINVAL;

	if ((format->fmt.pix.field == V4L2_FIELD_NONE) ||
		(format->fmt.pix.field == V4L2_FIELD_ANY)) {
		win->flags &= ~FLAG_DEINTERLACE;
	} else if (format->fmt.pix.field == V4L2_FIELD_INTERLACED) {
		win->flags |= FLAG_DEINTERLACE;
	} else {
		return -EINVAL;
	}

	pix->bytesperline = pix->width * (win->lw_params.output_bpp / 8);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = win->pix_format.colorspace;

	logiwin_set_rect_parameters(&win->lw,
		0, 0, pix->width, pix->height, OUTPUT_RECTANGLE);
	logiwin_set_scale(&win->lw);
	win->flags |= FLAG_DO_UPDATE;
	dummy = 0;
	logiwin_get_rect_parameters(&win->lw, &dummy, &dummy, 
		(unsigned long *)&pix->width, (unsigned long *)&pix->height,
		OUTPUT_RECTANGLE);
	memcpy(&(win->pix_format), pix, sizeof(struct v4l2_pix_format));

	return 0;
}

static int logiwin_enumstd(struct logiwin *win, struct v4l2_standard *std)
{
	if (std->index != 0)
		return -EINVAL;

	memset(std, 0, sizeof(*std));
	std->index = 0;
	std->id = win->video_norm.std;
	strncpy((char *)std->name,
		v4l2_norm_to_name(win->video_norm.std), sizeof(std->name)-1);

	return 0;
}

static int logiwin_g_std(struct logiwin *win, v4l2_std_id *std)
{
	*std = win->video_norm.std;

	return 0;
}

static int logiwin_s_std(struct logiwin *win, v4l2_std_id *std)
{
	if (!(*std & win->video_norm.std))
		return -EINVAL;

	return 0;
}

static int logiwin_enuminput(struct logiwin *win, struct v4l2_input *inp)
{
	if (inp->index != 0)
		return -EINVAL;

	memset(inp, 0, sizeof(*inp));
	inp->index = 0;

	strncpy((char *)inp->name, "logiWIN Input", sizeof(inp->name) - 1);
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = V4L2_STD_ALL;

	return 0;
}

static int logiwin_g_input(struct logiwin *win, int *input)
{
	*input = 0;

	return 0;
}

static int logiwin_s_input(struct logiwin *win, int *input)
{
	if (*input != 0)
		return -EINVAL;

	return 0;
}

static int logiwin_enum_framesizes(struct logiwin *win,
	struct v4l2_frmsizeenum *fsize)
{
	if ((fsize->index != 0) ||
		(fsize->pixel_format != win->pix_format.pixelformat))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = win->video_norm.width;
	fsize->discrete.height = win->video_norm.height;

	return 0;
}

static int logiwin_g_parm(struct logiwin *win, struct v4l2_streamparm *sp)
{
	if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	sp->parm.capture.extendedmode = 0;
	sp->parm.capture.readbuffers = win->frames_num;
	return 0;
}

static int logiwin_reqbufs(struct logiwin *win, struct v4l2_requestbuffers *rb)
{
	if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		rb->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	logiwin_empty_framequeues(win);

	if (rb->count && !(win->flags & FLAG_BUFFERS_AVAILABLE))
		rb->count = logiwin_request_buffers(win, rb->count);

	if (rb->count == 0)
		return -ENOMEM;

	return 0;
}

static int logiwin_querybuf(struct logiwin *win, struct v4l2_buffer *b)
{
	if (!(win->flags & FLAG_BUFFERS_AVAILABLE))
		return -ENOMEM;

	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || b->index >= win->frames_num)
		return -EINVAL;

	memcpy(b, &win->frame[b->index].buf, sizeof(*b));

	if (win->frame[b->index].vma_use_count)
		b->flags |= V4L2_BUF_FLAG_MAPPED;

	if (win->frame[b->index].state == F_DONE)
		b->flags |= V4L2_BUF_FLAG_DONE;
	else if (win->frame[b->index].state != F_UNUSED)
		b->flags |= V4L2_BUF_FLAG_QUEUED;

	return 0;
}

static int logiwin_qbuf(struct logiwin *win, struct v4l2_buffer *b)
{
	if (!(win->flags & FLAG_BUFFERS_AVAILABLE))
		return -ENOMEM;

	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || b->index >= win->frames_num)
		return -EINVAL;

	if (win->frame[b->index].state != F_UNUSED)
		return -EAGAIN;

	if (b->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	win->frame[b->index].state = F_QUEUED;

	spin_lock_bh(&win->queue_lock);
	list_add_tail(&win->frame[b->index].frame, &win->inqueue);
	spin_unlock_bh(&win->queue_lock);

	return 0;
}

static int logiwin_dqbuf(struct logiwin *win, struct file *fp,
	struct v4l2_buffer *b)
{
	struct logiwin_frame *frame;
	int ret = 0;

	if (!(win->flags & FLAG_BUFFERS_AVAILABLE))
		return -ENOMEM;

	if (win->stream_state == STREAM_OFF ||
		b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		b->index >= win->frames_num)
		return -EINVAL;

	if (list_empty(&win->outqueue)) {
		if (win->flags & FLAG_BUFFER_IN_SYNC) {
			if (fp->f_flags & O_NONBLOCK)
				return -EAGAIN;
			ret = wait_event_timeout(win->wait,
				!list_empty(&win->outqueue), HZ/5);
		} else {
			if (win->stream_state == STREAM_ON)
				ret = wait_event_timeout(win->wait,
					(win->flags & FLAG_BUFFER_IN_SYNC), HZ);
		}
		if (ret == 0)
			ret = -ETIMEDOUT;
		if (win->stream_state == STREAM_OFF)
			ret = -EPERM;
		if (ret < 0)
			return ret;
	}

	spin_lock_bh(&win->queue_lock);
	frame = list_entry(win->outqueue.next, struct logiwin_frame, frame);
	list_del(win->outqueue.next);
	spin_unlock_bh(&win->queue_lock);

	frame->state = F_UNUSED;
	memcpy(b, &frame->buf, sizeof(*b));

	if (frame->vma_use_count)
		b->flags |= V4L2_BUF_FLAG_MAPPED;

	return 0;
}

static int logiwin_streamon(struct logiwin *win, int *type)
{
	if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (!(win->flags & FLAG_BUFFERS_AVAILABLE))
		return -ENOMEM;

	if (win->stream_state == STREAM_OFF) {
		if (!list_empty(&win->outqueue))
			INIT_LIST_HEAD(&win->outqueue);

		/* write video buffer address where logiWIN will stream data */
		logiwin_set_memory_offset(&win->lw,
			win->dma_handle.dma_vbuff_addr[win->dma_handle.dma_vbuff_id].pa,
			win->dma_handle.dma_vbuff_addr[win->dma_handle.dma_vbuff_id].pa);

		/* force resyncing at every startup */
		//win->flags |= FLAG_BUFFER_IN_SYNC;
		win->stream_state = STREAM_ON;

		logiwin_clear_int_stat(&win->lw, LOGIWIN_FRAME_START_INT);
		logiwin_unmask_int(&win->lw, LOGIWIN_FRAME_START_INT);
		logiwin_start(&win->lw);
	}

	return 0;
}

static int logiwin_streamoff(struct logiwin *win, int *type)
{
	if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (win->stream_state == STREAM_ON) {
		logiwin_stop(&win->lw);
		logiwin_mask_int(&win->lw, LOGIWIN_FRAME_START_INT);
		logiwin_clear_int_stat(&win->lw, LOGIWIN_FRAME_START_INT);

		win->stream_state = STREAM_OFF;
		win->flags &= ~FLAG_BUFFER_IN_SYNC;

		wake_up(&win->wait);
	}

	return 0;
}

static int logiwin_cropcap(struct logiwin *win, struct v4l2_cropcap *cropcap)
{
	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cropcap, 0, sizeof(cropcap));

	/* area which can be sampled */
	memcpy(&(cropcap->bounds), &(win->cropcap.bounds),
		sizeof(struct v4l2_rect));

	dbg("%s - bounds = { .left=%i, .top=%i, .width=%i, .height=%i }\n",
		__func__,
		cropcap->bounds.left, cropcap->bounds.top,
		cropcap->bounds.width, cropcap->bounds.height);

	/* default source rectangle */
	memcpy(&(cropcap->defrect), &(win->cropcap.defrect),
		sizeof(struct v4l2_rect));

	dbg("%s - defrect = { .left=%i, .top=%i, .width=%i, .height=%i }\n",
		__func__,
		cropcap->defrect.left, cropcap->defrect.top,
		cropcap->defrect.width, cropcap->defrect.height);

	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;

	return 0;
}

static int logiwin_g_crop(struct logiwin *win, struct v4l2_crop *crop)
{
	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memcpy(&(crop->c), &(win->crop_rect), sizeof(struct v4l2_rect));

	return 0;
}

static int logiwin_s_crop(struct logiwin *win, struct v4l2_crop *crop)
{
	struct v4l2_rect *rect = &(crop->c);

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	logiwin_set_rect_parameters(&win->lw,
		rect->left, rect->top, rect->width, rect->height, CROPPING_RECTANGLE);
	logiwin_set_scale(&win->lw);
	win->flags |= FLAG_DO_UPDATE;
	logiwin_get_rect_parameters(&win->lw,
		(unsigned long *)&rect->left, (unsigned long *)&rect->top,
		(unsigned long *)&rect->width, (unsigned long *)&rect->height,
		CROPPING_RECTANGLE);
	memcpy(&(win->crop_rect), rect, sizeof(struct v4l2_rect));

	return 0;
}

static int logiwin_g_fbuf(struct logiwin *win, struct v4l2_framebuffer *fb)
{
	fb->capability = V4L2_FBUF_CAP_LIST_CLIPPING;
	fb->flags = (V4L2_FBUF_FLAG_PRIMARY | V4L2_FBUF_FLAG_OVERLAY);
	fb->base = (void *)win->dma_handle.dma_vbuff_addr[0].pa;

	fb->fmt = win->pix_format;

	return 0;
}

static int logiwin_s_fbuf(struct logiwin *win, struct v4l2_framebuffer *fb)
{
	int i;

	if (!capable(CAP_SYS_ADMIN) && !capable(CAP_SYS_RAWIO))
		return -EPERM;

	if (fb->capability != V4L2_FBUF_CAP_LIST_CLIPPING)
		return -EINVAL;

	for (i = 0; i < win->frames_num; i++)
		win->dma_handle.dma_vbuff_addr[i].pa =
			(dma_addr_t)(fb->base + (i * win->dma_handle.dma_vbuff_size));

	logiwin_g_fbuf(win, fb);

	return 0;
}

/*---------------------------------------------------------------------------*/

static int logiwin_open(struct file *fp)
{
	struct video_device *vdev = video_devdata(fp);
	struct logiwin *win = video_get_drvdata(vdev);

	if (win->flags & FLAG_DEVICE_IN_USE)
		return -EBUSY;

	mutex_lock(&win->fops_lock);

	win->flags &= ~FLAG_DEINTERLACE;
//	if (!(win->flags & FLAG_DEINTERLACE)) {
//		//bob
//		width_max = win->video_norm.width;
//		height_max = win->video_norm.height/2;
//	} else {
//		//weave
//		width_max = win->video_norm.width;
//		height_max = win->video_norm.height;
//	}

	/* possible cropping area */
	win->cropcap.bounds.left = 0;
	win->cropcap.bounds.top = 0;
	win->cropcap.bounds.width = win->video_norm.width;
	win->cropcap.bounds.height = win->video_norm.height;

	/* default cropping rectangle */
	win->cropcap.defrect.left = 0;
	win->cropcap.defrect.top = 0;
	win->cropcap.defrect.width = win->video_norm.width;
	win->cropcap.defrect.height = win->video_norm.height;

	/* initial pix format data */
	win->pix_format.width = win->video_norm.width;
	win->pix_format.height = win->video_norm.height;
	win->pix_format.pixelformat = win->lw_params.output_format;
	win->pix_format.field = V4L2_FIELD_NONE;
	win->pix_format.bytesperline =
		win->pix_format.width * (win->lw_params.output_bpp / 8);
	win->pix_format.sizeimage = win->dma_handle.dma_vbuff_size;
	win->pix_format.colorspace = V4L2_COLORSPACE_SRGB;
	win->pix_format.priv = 0;

	/* initial cropping rectangle */
	win->crop_rect.left = 0;
	win->crop_rect.top = 0;
	win->crop_rect.width = win->video_norm.width;
	win->crop_rect.height = win->video_norm.height;

	fp->private_data = win;

	win->flags &= ~FLAG_DO_UPDATE;
	win->stream_state = STREAM_OFF;
	win->flags |= FLAG_DEVICE_IN_USE;

	logiwin_update(win);

	mutex_unlock(&win->fops_lock);

	return 0;
}

static int logiwin_close(struct file *fp)
{
	struct logiwin *win = fp->private_data;

	mutex_lock(&win->fops_lock);

	logiwin_stop(&win->lw);
	logiwin_mask_int(&win->lw, LOGIWIN_FRAME_START_INT);
	logiwin_clear_int_stat(&win->lw, LOGIWIN_FRAME_START_INT);

	win->flags &= ~FLAG_DEVICE_IN_USE;
	if (win->flags & FLAG_BUFFERS_AVAILABLE)
		logiwin_release_buffers(win);

	mutex_unlock(&win->fops_lock);

	return 0;
}

static ssize_t logiwin_read(struct file *fp, char __user *data,
	size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static void logiwin_vm_open(struct vm_area_struct *vma)
{
	struct logiwin_frame *frame = vma->vm_private_data;
	frame->vma_use_count++;
}

static void logiwin_vm_close(struct vm_area_struct *vma)
{
	struct logiwin_frame *frame = vma->vm_private_data;
	frame->vma_use_count--;
}

static struct vm_operations_struct logiwin_vm_ops = {
	.open = logiwin_vm_open,
	.close = logiwin_vm_close,
};

static int logiwin_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct logiwin *win = fp->private_data;
	unsigned long size;
	int i;
	int ret = -EINVAL;

	if (!(win->flags & FLAG_BUFFERS_AVAILABLE))
		return -ENOMEM;

	if (mutex_lock_interruptible(&win->fops_lock))
		return -ERESTARTSYS;

	size = vma->vm_end - vma->vm_start;
	if (size != PAGE_ALIGN(win->frame[0].buf.length)) {
		printk(KERN_ERR "%s - size page align error\n", __func__);
		goto error_unlock;
	}

	for (i = 0; i < win->frames_num; i++)
		if ((win->frame[i].buf.m.offset >> PAGE_SHIFT) == vma->vm_pgoff)
			break;

	if (i == win->frames_num) {
		printk(KERN_ERR "%s - mapping address out of range\n", __func__);
		goto error_unlock;
	}

	if ((vma->vm_flags & (VM_WRITE | VM_SHARED)) != 
		(VM_WRITE | VM_SHARED)) {
		printk(KERN_ERR "%s - mapping protection error\n", __func__);
		goto error_unlock;
	}

	ret = remap_pfn_range(vma, vma->vm_start,
		(win->frame[i].buf_addr >> PAGE_SHIFT), size, vma->vm_page_prot);
	if (ret) {
		printk(KERN_ERR "%s - mapping address failed\n", __func__);
		goto error_unlock;
	}

	vma->vm_ops = &logiwin_vm_ops;
	vma->vm_private_data = &win->frame[i];

	logiwin_vm_open(vma);

error_unlock:
	mutex_unlock(&win->fops_lock);

	return ret;
}

static long logiwin_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct logiwin *win = fp->private_data;

	switch (cmd) {

		case VIDIOC_QUERYCAP:
			dbg("%s - VIDIOC_QUERYCAP\n", __func__);
			return logiwin_querycap(win, (struct v4l2_capability *)arg);

		case VIDIOC_QUERYSTD:
			dbg("%s - VIDIOC_QUERYSTD\n", __func__);
			return logiwin_querystd(win, (v4l2_std_id *)arg);

		case VIDIOC_ENUM_FMT:
			dbg("%s - VIDIOC_ENUM_FMT\n", __func__);
			return logiwin_enum_fmt(win, (struct v4l2_fmtdesc *)arg);

		case VIDIOC_G_FMT:
			dbg("%s - VIDIOC_G_FMT\n", __func__);
			return logiwin_g_fmt(win, (struct v4l2_format *) arg);

		case VIDIOC_TRY_FMT:
		case VIDIOC_S_FMT:
			dbg("%s - VIDIOC_S_FMT\n", __func__);
			return logiwin_s_fmt(win, (struct v4l2_format *)arg);

		case VIDIOC_ENUMSTD:
			dbg("%s - VIDIOC_ENUMSTD\n", __func__);
			return logiwin_enumstd(win, (struct v4l2_standard *)arg);

		case VIDIOC_G_STD:
			dbg("%s - VIDIOC_G_STD\n", __func__);
			return logiwin_g_std(win, (v4l2_std_id *)arg);

		case VIDIOC_S_STD:
			dbg("%s - VIDIOC_S_STD\n", __func__);
			return logiwin_s_std(win, (v4l2_std_id *)arg);

		case VIDIOC_ENUMINPUT:
			dbg("%s - VIDIOC_ENUMINPUT\n", __func__);
			return logiwin_enuminput(win, (struct v4l2_input *)arg);

		case VIDIOC_G_INPUT:
			dbg("%s - VIDIOC_G_INPUT\n", __func__);
			return logiwin_g_input(win, (int *)arg);

		case VIDIOC_S_INPUT:
			dbg("%s - VIDIOC_S_INPUT\n", __func__);
			return logiwin_s_input(win, (int *)arg);

		case VIDIOC_ENUM_FRAMESIZES:
			dbg("%s - VIDIOC_ENUM_FRAMESIZES\n", __func__);
			return logiwin_enum_framesizes(win, (struct v4l2_frmsizeenum *)arg);

		case VIDIOC_G_PARM:
			dbg("%s - VIDIOC_G_PARM\n", __func__);
			return logiwin_g_parm(win, (struct v4l2_streamparm *)arg);

		case VIDIOC_REQBUFS:
			dbg("%s - VIDIOC_REQBUFS\n", __func__);
			return logiwin_reqbufs(win, (struct v4l2_requestbuffers *)arg);

		case VIDIOC_QUERYBUF:
			dbg("%s - VIDIOC_QUERYBUF\n", __func__);
			return logiwin_querybuf(win, (struct v4l2_buffer *)arg);

		case VIDIOC_QBUF:
			//dbg("%s - VIDIOC_QBUF\n", __func__);
			return logiwin_qbuf(win, (struct v4l2_buffer *)arg);

		case VIDIOC_DQBUF:
			//dbg("%s - VIDIOC_DQBUF\n", __func__);
			return logiwin_dqbuf(win, fp, (struct v4l2_buffer *)arg);

		case VIDIOC_STREAMON:
			dbg("%s - VIDIOC_STREAMON\n", __func__);
			return logiwin_streamon(win, (int *)arg);

		case VIDIOC_STREAMOFF:
			dbg("%s - VIDIOC_STREAMOFF\n", __func__);
			return logiwin_streamoff(win, (int *)arg);

		case VIDIOC_CROPCAP :
			dbg("%s - VIDIOC_CROPCAP\n", __func__);
			return logiwin_cropcap(win, (struct v4l2_cropcap *)arg);

		case VIDIOC_G_CROP :
			dbg("%s - VIDIOC_G_CROP\n", __func__);
			return logiwin_g_crop(win, (struct v4l2_crop *)arg);

		case VIDIOC_S_CROP :
			dbg("%s - VIDIOC_S_CROP\n", __func__);
			return logiwin_s_crop(win, (struct v4l2_crop *)arg);

		case VIDIOC_G_FBUF:
			dbg("%s - VIDIOC_G_FBUF\n", __func__);
			return logiwin_g_fbuf(win, (struct v4l2_framebuffer *)arg);

		case VIDIOC_S_FBUF:
			dbg("%s - VIDIOC_S_FBUF\n", __func__);
			return logiwin_s_fbuf(win, (struct v4l2_framebuffer *)arg);

		default:
			dbg("%s Unknown command 0x%x: dir: %x, type: %x, nr: %x, size: %x\n",
				__func__, cmd,
				_IOC_DIR(cmd),
				_IOC_TYPE(cmd),
				_IOC_NR(cmd),
				_IOC_SIZE(cmd));
	}

	return -EINVAL;
}

void logiwin_vdev_release(struct video_device *vdev)
{
	kfree(vdev);
}

static const struct v4l2_file_operations logiwin_fops = {
	.owner		= THIS_MODULE,
	.open		= logiwin_open,
	.release	= logiwin_close,
	.ioctl		= logiwin_ioctl,
	.mmap		= logiwin_mmap,
	.read		= logiwin_read,
};

static const struct video_device logiwin_template = {
	.name		= LOGIWIN_NAME,
	.fops		= &logiwin_fops,
	.release	= &logiwin_vdev_release,
	.minor		= -1
};

/*---------------------------------------------------------------------------*/

static void logiwin_handleframe(unsigned long arg)
{
	struct logiwin *win = (struct logiwin *)arg;
	struct logiwin_frame *frame;

	spin_lock_bh(&win->queue_lock);

	if ((win->flags & FLAG_BUFFER_IN_SYNC) && !list_empty(&win->inqueue)) {
		/* get the first frame from the inqueue */
		frame = list_entry(win->inqueue.next, struct logiwin_frame, frame);
		do_gettimeofday(&frame->buf.timestamp);
		frame->buf.sequence++;
		frame->state = F_DONE;
		frame->buf.bytesused = frame->buf.length;
		/* remove the entry from the inqueue and put it in the outqueue */
		list_move_tail(&frame->frame, &win->outqueue);
		wake_up(&win->wait);
	} else if (!(win->flags & FLAG_BUFFER_IN_SYNC) &&
			list_empty(&win->outqueue)) {
		if (win->dma_handle.dma_vbuff_id == 0) {
			logiwin_reinit_framequeues(win);
			win->flags |= FLAG_BUFFER_IN_SYNC;
		}
	} else if (win->flags & FLAG_BUFFER_IN_SYNC) {
		win->flags &= ~FLAG_BUFFER_IN_SYNC;
	}

	spin_unlock_bh(&win->queue_lock);

	if (win->flags & FLAG_DO_UPDATE) {
		logiwin_update(win);
		win->flags &= ~FLAG_DO_UPDATE;
	}
}

static irqreturn_t logiwin_isr(int irq, void *pdev)
{
	struct logiwin *win = (struct logiwin *)pdev;

	logiwin_clear_int_stat(&win->lw, LOGIWIN_FRAME_START_INT);

	if (win->frames_num > 1) {
		/* set video buffer address index */
		if (win->dma_handle.dma_vbuff_id < (win->frames_num - 1))
			win->dma_handle.dma_vbuff_id++;
		else
			win->dma_handle.dma_vbuff_id = 0;
		/* write video buffer address where logiWIN will stream data */
		if (win->dma_handle.dma_vbuff_addr) {
			logiwin_set_memory_offset(&win->lw,
				win->dma_handle.dma_vbuff_addr[win->dma_handle.dma_vbuff_id].pa,
				win->dma_handle.dma_vbuff_addr[win->dma_handle.dma_vbuff_id].pa);
		}
	}

	tasklet_schedule(&win->tasklet);

	return IRQ_HANDLED;
}

/*---------------------------------------------------------------------------*/

static void logiwin_init_params(struct logiwin *win,
	struct logiwin_platform_data *lw_pdata, unsigned short id)
{
	win->lw.reg_base = win->lw_params.regs_base_virt;

	win->lw.in_bounds_rect.x = 0;
	win->lw.in_bounds_rect.y = 0;
	win->lw.in_bounds_rect.width = lw_pdata->input_res_x;
	win->lw.in_bounds_rect.height = lw_pdata->input_res_y;

	logiwin_set_video_norm(&win->video_norm,
		lw_pdata->input_res_x, lw_pdata->input_res_y);

	win->lw.in_crop_rect = win->lw.in_bounds_rect;

	win->lw.out_rect = win->lw.in_bounds_rect;

	win->lw.max_out_hres = lw_pdata->vmem_width;
	win->lw.max_out_vres = lw_pdata->vmem_height / LOGIWIN_FRAMEBUFFERS;

	win->lw.scale_step_x = SCALE_STEP_1;
	win->lw.scale_step_y = SCALE_STEP_1;

	win->lw.scale_shift_bits = 16 - lw_pdata->scale_fraction_bits;

	win->lw.out_byte_align = lw_pdata->out_byte_align;
	win->lw.out_byte_align_mask = ~(win->lw.out_byte_align - 1);

	win->lw.int_mask = 0xFFFF;

	win->lw.device_id = id;

	win->lw.input_format = lw_pdata->input_format;
//	if (win->lw.input_format == LOGIWIN_ITU) {
//		win->lw.weave_deinterlace = 1;
//		win->lw.ctrl = LOGIWIN_CR_WEAVE_DEINT_MSK;
//	} else {
//		win->lw.weave_deinterlace = 0;
//		win->lw.ctrl = 0;
//	}
	win->lw.brightness = 0;
	win->lw.contrast = 0;
	win->lw.saturation = 0;
	win->lw.hue = 0;
}

#ifdef CONFIG_OF
static int logiwin_get_params(struct platform_device *pdev, int id,
	struct logiwin_platform_data *lw_pdata)
{
	struct device_node *dn;
	const unsigned int *prop;
	int size;
	char device[20+1];

	sprintf(device, "logiwin_%d", id);
	dn = of_find_node_by_name(pdev->dev.of_node, device);
	if (!dn) {
		printk(KERN_ERR "%s - no device %d\n", __func__, id);
		return -ENODEV;
	}

	prop = of_get_property(dn, "vmem-address", &size);
	if (prop) {
		lw_pdata->vmem_base_addr = be32_to_cpup(prop);
		prop++;
		lw_pdata->vmem_high_addr = lw_pdata->vmem_base_addr + be32_to_cpup(prop);
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "vmem-buffer", &size);
	if (prop) {
		lw_pdata->vmem_width = be32_to_cpup(prop);
		prop++;
		lw_pdata->vmem_height = be32_to_cpup(prop);
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "input-num", &size);
	if (prop) {
		lw_pdata->input_num = be32_to_cpup(prop);
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "input-resolution", &size);
	if (prop) {
		lw_pdata->input_res_x = be32_to_cpup(prop);
		prop++;
		lw_pdata->input_res_y = be32_to_cpup(prop);
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "input-format", &size);
	if (prop) {
		lw_pdata->input_format = be32_to_cpup(prop);
		switch (lw_pdata->input_format) {
		case 0:
			lw_pdata->input_format = LOGIWIN_DVI;
			break;
		case 1:
			lw_pdata->input_format = LOGIWIN_ITU;
			break;
		case 2:
			lw_pdata->input_format = LOGIWIN_RGB;
			break;
		}
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "output-format", &size);
	if (prop) {
		lw_pdata->output_format = be32_to_cpup(prop);
		switch (lw_pdata->output_format) {
		case 0:
			lw_pdata->output_format = V4L2_PIX_FMT_RGB565;
			break;
		case 1:
			lw_pdata->output_format = V4L2_PIX_FMT_RGB24;
			break;
		case 2:
		case 3:
			lw_pdata->output_format = V4L2_PIX_FMT_RGB32;
			break;
		case 4:
			lw_pdata->output_format = V4L2_PIX_FMT_YUYV;
			break;
		}
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "out-byte-align", &size);
	if (prop) {
		lw_pdata->out_byte_align = be32_to_cpup(prop);
	} else {
		goto logiwin_get_params_error;
	}
	prop = of_get_property(dn, "scale-fraction-bits", &size);
	if (prop) {
		lw_pdata->scale_fraction_bits = be32_to_cpup(prop);
	} else {
		goto logiwin_get_params_error;
	}

	return 0;

logiwin_get_params_error:
	return -EFAULT;
}
#endif

#define FMC1_GPIO_I2C_MUX_RST 54
#define FMC1_I2C_ADAPTER_ID 9
#define FMC1_I2C_ADDR_MUX 0xE0
#define FMC1_I2C_IO_EXP_ADDR 0x40
#define FMC1_I2C_ADDRESS_HDMI_INPUT 0x98

static int set_device_reg(struct i2c_adapter *fmc, struct i2c_msg *msg, int len)
{
	int ret;

	msg->flags &= ~I2C_M_RD;
	msg->len = len;

	ret = i2c_transfer(fmc, msg, 1);

	return ret;
}

static int get_device_reg(struct i2c_adapter *fmc, struct i2c_msg *msg, int len)
{
	int ret;

	msg->flags &= ~I2C_M_RD;
	msg->len = len;
	ret = i2c_transfer(fmc, msg, 1);
	mdelay(1);
	msg->flags |= I2C_M_RD;
	ret = i2c_transfer(fmc, msg, 1);

	return ret;
}

static int set_hdmi_input_mapping(
	struct i2c_adapter *adapter, struct i2c_msg *msg)
{
#define IIC_ADV7611_CEC_ADDR       0x80
#define IIC_ADV7611_INFOFRAME_ADDR 0x6A
#define IIC_ADV7611_DPLL_ADDR      0x4C
#define IIC_ADV7611_KSV_ADDR       0x64
#define IIC_ADV7611_EDID_ADDR      0x6C
#define IIC_ADV7611_HDMI_ADDR      0x68
#define IIC_ADV7611_CP_ADDR        0x44
#define IIC_HDMI_IN_MAPPING_LEN    7
	u8 iic_hdmi_in_mapping[IIC_HDMI_IN_MAPPING_LEN][3] = {
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xF4, IIC_ADV7611_CEC_ADDR},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xF5, IIC_ADV7611_INFOFRAME_ADDR},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xF8, IIC_ADV7611_DPLL_ADDR},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xF9, IIC_ADV7611_KSV_ADDR},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xFA, IIC_ADV7611_EDID_ADDR},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xFB, IIC_ADV7611_HDMI_ADDR},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0xFD, IIC_ADV7611_CP_ADDR}
	};
	int i;

	for (i = 0; i < IIC_HDMI_IN_MAPPING_LEN; i++) {
		msg->addr = iic_hdmi_in_mapping[i][0] >> 1;
		msg->buf[0] = iic_hdmi_in_mapping[i][1];
		msg->buf[1] = iic_hdmi_in_mapping[i][2];
		if (set_device_reg(adapter, msg, 2) < 0) {
			printk(KERN_ERR "Error setting FMC1 I2C HDMI input mapping\n");
			return -EIO;
		}
	}

	return 0;
}

static void get_hdmi_input_lock(
	struct i2c_adapter *adapter, struct i2c_msg *msg)
{
	u8 lock;
	int i, count;

	for (i = 0, count = 10; i < count; i++) {
		msg->addr = IIC_ADV7611_HDMI_ADDR >> 1;
		msg->buf[0] = 0x07;
		get_device_reg(adapter, msg, 1);

		lock = ((msg->buf[0] & 0xA0) == 0xA0) ? 1 : 0;
		if (lock) {
			printk(KERN_INFO "ADV7611 locked\n");
			break;
		}

		mdelay(100);
	}
}

static int set_hdmi_input_config(
	struct i2c_adapter *adapter, struct i2c_msg *msg)
{
#define IIC_HDMI_IN_CONFIG_LEN 41
	u8 iic_hdmi_in_config[IIC_HDMI_IN_CONFIG_LEN][3] = {
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x01, 0x06},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x02, 0xF5},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x03, 0x80},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x04, 0x62},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x05, 0x2C},
		{IIC_ADV7611_CP_ADDR, 0x7B, 0x05},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x0B, 0x44},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x0C, 0x42},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x14, 0x7F},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x15, 0x80},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x19, 0x83},
		{FMC1_I2C_ADDRESS_HDMI_INPUT, 0x33, 0x40},
		{IIC_ADV7611_CP_ADDR, 0xBA, 0x01},
		{IIC_ADV7611_KSV_ADDR, 0x40, 0x81},
		{IIC_ADV7611_HDMI_ADDR, 0x9B, 0x03},
		{IIC_ADV7611_HDMI_ADDR, 0xC1, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC2, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC3, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC4, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC5, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC6, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC7, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC8, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xC9, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xCA, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xCB, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0xCC, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0x00, 0x08},
		{IIC_ADV7611_HDMI_ADDR, 0x02, 0x03},
		{IIC_ADV7611_HDMI_ADDR, 0x83, 0xFC},
		{IIC_ADV7611_HDMI_ADDR, 0x6F, 0x0C},
		{IIC_ADV7611_HDMI_ADDR, 0x85, 0x1F},
		{IIC_ADV7611_HDMI_ADDR, 0x87, 0x70},
		{IIC_ADV7611_HDMI_ADDR, 0x8D, 0x04},
		{IIC_ADV7611_HDMI_ADDR, 0x8E, 0x1E},
		{IIC_ADV7611_HDMI_ADDR, 0x1A, 0x8A},
		{IIC_ADV7611_HDMI_ADDR, 0x57, 0xDA},
		{IIC_ADV7611_HDMI_ADDR, 0x58, 0x01},
		{IIC_ADV7611_HDMI_ADDR, 0x75, 0x10},
		{IIC_ADV7611_HDMI_ADDR, 0x90, 0x04},
		{IIC_ADV7611_HDMI_ADDR, 0x91, 0x1E}
	};
#define IIC_HDMI_IN_SPDIF_CONFIG_LEN  2
	u8 iic_hdmi_in_spdif_config[IIC_HDMI_IN_SPDIF_CONFIG_LEN][3] = {
		{IIC_ADV7611_HDMI_ADDR, 0x03, 0x78},
		{IIC_ADV7611_HDMI_ADDR, 0x6E, 0x0C}
	};
	int i;

	for (i = 0; i < IIC_HDMI_IN_CONFIG_LEN; i++) {
		msg->addr = iic_hdmi_in_config[i][0] >> 1;
		msg->buf[0] = iic_hdmi_in_config[i][1];
		msg->buf[1] = iic_hdmi_in_config[i][2];
		if (set_device_reg(adapter, msg, 2) < 0) {
			printk(KERN_ERR "Error initializing HDMI input config at %d\n", i);
			return -EIO;
		}
	}

	for (i = 0; i < IIC_HDMI_IN_SPDIF_CONFIG_LEN; i++) {
		msg->addr = iic_hdmi_in_spdif_config[i][0] >> 1;
		msg->buf[0] = iic_hdmi_in_spdif_config[i][1];
		msg->buf[1] = iic_hdmi_in_spdif_config[i][2];
		if (set_device_reg(adapter, msg, 2) < 0) {
			printk(KERN_ERR "Error initializing HDMI input SPDIF at %d\n", i);
			return -EIO;
		}
	}

	mdelay(10);

	return 0;
}

static int set_hdmi_input_edid(
	struct i2c_adapter *adapter, struct i2c_msg *msg)
{
#define IIC_HDMI_IN_EDID_PRE_LEN 1
	u8 iic_hdmi_in_edid_pre[IIC_HDMI_IN_EDID_PRE_LEN][3] = {
		{IIC_ADV7611_KSV_ADDR, 0x77, 0x00}
	};
#define IIC_HDMI_IN_EDID_POST_LEN 5
	u8 iic_hdmi_in_edid_post[IIC_HDMI_IN_EDID_POST_LEN][3] = {
		{IIC_ADV7611_KSV_ADDR, 0x77, 0x00},
		{IIC_ADV7611_KSV_ADDR, 0x52, 0x20},
		{IIC_ADV7611_KSV_ADDR, 0x53, 0x00},
		{IIC_ADV7611_KSV_ADDR, 0x70, 0x9E},
		{IIC_ADV7611_KSV_ADDR, 0x74, 0x03}
	};
	u8 fmc_imageon_hdmii_edid_content[] = {
		0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
		0x06, 0x8F, 0x07, 0x11, 0x01, 0x00, 0x00, 0x00,
		0x17, 0x11, 0x01, 0x03, 0x80, 0x0C, 0x09, 0x78,
		0x0A, 0x1E, 0xAC, 0x98, 0x59, 0x56, 0x85, 0x28,
		0x29, 0x52, 0x57, 0x00, 0x00, 0x00, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A,
		0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x58,
		0x45, 0x00, 0x80, 0x38, 0x74, 0x00, 0x00, 0x18,
		0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20,
		0x6E, 0x50, 0x55, 0x00, 0x00, 0xD0, 0x52, 0x00,
		0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x56,
		0x41, 0x2D, 0x31, 0x38, 0x30, 0x39, 0x41, 0x0A,
		0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
		0x00, 0x17, 0x3D, 0x0D, 0x2E, 0x11, 0x00, 0x0A,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x51,
		0x02, 0x03, 0x34, 0x71, 0x4D, 0x82, 0x05, 0x04,
		0x01, 0x10, 0x11, 0x14, 0x13, 0x1F, 0x06, 0x15,
		0x03, 0x12, 0x35, 0x0F, 0x7F, 0x07, 0x17, 0x1F,
		0x38, 0x1F, 0x07, 0x30, 0x2F, 0x07, 0x72, 0x3F,
		0x7F, 0x72, 0x57, 0x7F, 0x00, 0x37, 0x7F, 0x72,
		0x83, 0x4F, 0x00, 0x00, 0x67, 0x03, 0x0C, 0x00,
		0x10, 0x00, 0x88, 0x2D, 0x00, 0x00, 0x00, 0xFF,
		0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00,
		0x00, 0xFF, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		0x00, 0x00, 0x00, 0xFF, 0x00, 0x0A, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA
	};
	int i;

	for (i = 0; i < IIC_HDMI_IN_EDID_PRE_LEN; i++) {
		msg->addr = iic_hdmi_in_edid_pre[i][0] >> 1;
		msg->buf[0] = iic_hdmi_in_edid_pre[i][1];
		msg->buf[1] = iic_hdmi_in_edid_pre[i][2];
		if (set_device_reg(adapter, msg, 2) < 0) {
			printk(KERN_ERR "Error initializing EDID (PRE) at %d\n", i);
			return -EIO;
		}
	}

	for (i = 0; i < 256; i++) {
		msg->addr = IIC_ADV7611_EDID_ADDR >> 1;
		msg->buf[0] = i;
		msg->buf[1] = fmc_imageon_hdmii_edid_content[i];
		if (set_device_reg(adapter, msg, 2) < 0) {
			printk(KERN_ERR "Error initializing EDID at %d\n", i);
			return -EIO;
		}
	}

	for (i = 0; i < IIC_HDMI_IN_EDID_POST_LEN; i++) {
		msg->addr = iic_hdmi_in_edid_post[i][0] >> 1;
		msg->buf[0] = iic_hdmi_in_edid_post[i][1];
		msg->buf[1] = iic_hdmi_in_edid_post[i][2];
		if (set_device_reg(adapter, msg, 2) < 0) {
			printk(KERN_ERR "Error initializing EDID (POST) at %d\n", i);
			return -EIO;
		}
	}

	return 0;
}

static int set_i2c_hdmi_reset(
	struct i2c_adapter *adapter, struct i2c_msg *msg, bool active)
{
	msg->addr = FMC1_I2C_IO_EXP_ADDR >> 1;
	msg->buf[0] = 0x01;
	if (get_device_reg(adapter, msg, 1) < 0) {
		printk(KERN_ERR "Error getting FMC1 I2C HDMI reset\n");
		return -EIO;
	}
	msg->buf[1] = msg->buf[0];
	msg->buf[0] = 0x01;
	if (active)
		msg->buf[1] &= ~0x01;
	else
		msg->buf[1] |= 0x01;
	if (set_device_reg(adapter, msg, 2) < 0) {
		printk(KERN_ERR "Error setting FMC1 I2C HDMI reset\n");
		return -EIO;
	}
	mdelay(10);

	return 0;
}

static int set_i2c_hdmi_hpd(
	struct i2c_adapter *adapter, struct i2c_msg *msg, bool enabled)
{
	msg->addr = FMC1_I2C_IO_EXP_ADDR >> 1;
	msg->buf[0] = 0x01;
	if (get_device_reg(adapter, msg, 1) < 0) {
		printk(KERN_ERR "Error getting FMC1 I2C HDMI HPD\n");
		return -EIO;
	}
	msg->buf[1] = msg->buf[0];
	msg->buf[0] = 0x01;
	if (enabled)
		msg->buf[1] |= 0x04;
	else
		msg->buf[1] &= ~0x04;
	if (set_device_reg(adapter, msg, 2) < 0) {
		printk(KERN_ERR "Error setting FMC1 I2C HDMI HPD\n");
		return -EIO;
	}
	mdelay(10);

	return 0;
}

static int set_i2c_gpio_outputs(
	struct i2c_adapter *adapter, struct i2c_msg *msg)
{
	msg->addr = FMC1_I2C_IO_EXP_ADDR >> 1;
	msg->buf[0] = 0x03;
	msg->buf[1] = 0xEA;
	if (set_device_reg(adapter, msg, 2) < 0) {
		printk(KERN_ERR "Error setting FMC1 I2C GPIO direction\n");
	}
	msg->buf[0] = 0x01;
	msg->buf[1] = 0x01;
	if (set_device_reg(adapter, msg, 2) < 0) {
		printk(KERN_ERR "Error setting FMC1 I2C GPIO value\n");
		return -EIO;
	}

	return 0;
}

static int select_i2c_mux_output(
	struct i2c_adapter *adapter, struct i2c_msg *msg, u8 data)
{
	msg->addr = FMC1_I2C_ADDR_MUX >> 1;
	msg->buf[0] = data;
	if (set_device_reg(adapter, msg, 1) < 0) {
		printk(KERN_ERR "Error setting FMC1 I2C MUX\n");
		return -EIO;
	}

	return 0;
}

static int video_in_init(void)
{
	struct i2c_msg msg;
	struct i2c_adapter *adapter;
	int ret;
	u8 data_buff[256];

	msg.addr = 0;
	msg.flags = 0;
	msg.len = 0;
	msg.buf = data_buff;
	ret = 0;

	/* set I2C MUX out of reset */
	ret = gpio_request(FMC1_GPIO_I2C_MUX_RST, "FMC1 I2C MUX reset");
	if (ret < 0) {
		printk(KERN_ERR "Error requesting GPIO %d", ret);
		goto video_in_init_error;
	}
	ret = gpio_direction_output(FMC1_GPIO_I2C_MUX_RST, 1);
	if (ret) {
		printk(KERN_ERR "Error setting GPIO \"out\" %d", ret);
		goto video_in_init_error;
	}
	gpio_set_value(FMC1_GPIO_I2C_MUX_RST, 1);
	gpio_free(FMC1_GPIO_I2C_MUX_RST);
	mdelay(10);

	adapter = i2c_get_adapter(FMC1_I2C_ADAPTER_ID);
	if (!adapter) {
		printk(KERN_ERR "%s - no I2C device\n", __func__);
		return -ENODEV;
	}

	/* Select I2C GPIO */
	ret = select_i2c_mux_output(adapter, &msg, 0x08);
	if (ret < 0)
		goto video_in_init_error;
	/* Set I2C GPIO direction and values */
	ret = set_i2c_gpio_outputs(adapter, &msg);
	if (ret < 0)
		goto video_in_init_error;
	/* disable HDMI input HPD */
	ret = set_i2c_hdmi_hpd(adapter, &msg, 0);
	if (ret < 0)
		goto video_in_init_error;
	/* remove reset for HDMI input */
	ret = set_i2c_hdmi_reset(adapter, &msg, 0);
	if (ret < 0)
		goto video_in_init_error;
	/* Select I2C HDMI input */
	ret = select_i2c_mux_output(adapter, &msg, 0x04);
	if (ret < 0)
		goto video_in_init_error;
	ret = set_hdmi_input_mapping(adapter, &msg);
	if (ret < 0)
		goto video_in_init_error;
	ret = set_hdmi_input_edid(adapter, &msg);
	if (ret < 0)
		goto video_in_init_error;
	/* Select I2C GPIO */
	ret = select_i2c_mux_output(adapter, &msg, 0x08);
	if (ret < 0)
		goto video_in_init_error;
	/* enable HDMI input HPD */
	ret = set_i2c_hdmi_hpd(adapter, &msg, 1);
	if (ret < 0)
		goto video_in_init_error;
	/* Select I2C HDMI input */
	ret = select_i2c_mux_output(adapter, &msg, 0x04);
	if (ret < 0)
		goto video_in_init_error;
	ret = set_hdmi_input_config(adapter, &msg);
	if (ret < 0)
		goto video_in_init_error;
	get_hdmi_input_lock(adapter, &msg);

	i2c_put_adapter(adapter);

video_in_init_error:
	printk(KERN_INFO "Video input card %sinitialized!", ret < 0 ? "not " : "");
	return ret;
}

static int __devinit logiwin_probe(struct platform_device *pdev)
{
	struct logiwin **win;
	struct logiwin_platform_data *lw_pdata;
	struct resource *iomem;
	int num_of_devices;
	int err, i;
	char *vdev_registered = NULL;

	win = NULL;
	i = -1;

	num_of_devices = pdev->num_resources / 2;

#ifdef CONFIG_OF
	lw_pdata = kzalloc((sizeof(*lw_pdata) * num_of_devices), GFP_KERNEL);
	if (!lw_pdata) {
		printk(KERN_ERR "%s - no memory\n", __func__);
		return -ENOMEM;
	}
	pdev->dev.platform_data = (void *)lw_pdata;
#else
	lw_pdata = (struct logiwin_platform_data *)pdev->dev.platform_data;
	if (!lw_pdata) {
		printk(KERN_ERR "%s - no device\n", __func__);
		return -ENODEV;
	}
#endif

	/* Initialize video input and output */
	err = video_in_init();
	if (err)
		goto err_handle;

	win = kzalloc((sizeof(*win) * num_of_devices), GFP_KERNEL);
	if (!win) {
		err = -EINVAL;
		printk(KERN_ERR "%s - Failed to allocate win resources\n", __func__);
		goto err_handle;
	}

	vdev_registered = kzalloc((sizeof(char) * num_of_devices), GFP_KERNEL);
	if (!vdev_registered) {
		err = -EINVAL;
		printk(KERN_ERR "%s - Failed to allocate internal resources\n",
			__func__);
		goto err_handle;
	}

	for (i = 0; i < num_of_devices; i++) {
		dbg("%s - logiWIN %d\n", __func__, i);

		win[i] = kzalloc(sizeof(struct logiwin), GFP_KERNEL);
		if (!win[i]) {
			err = -EINVAL;
			goto err_handle;
		}

		iomem = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!iomem) {
			err = -EINVAL;
			goto err_handle;
		}
		win[i]->lw_params.regs_base_phys = iomem->start;
		win[i]->lw_params.regs_size =
			resource_size(platform_get_resource(pdev, IORESOURCE_MEM, i));
		win[i]->lw_params.regs_base_virt =
			ioremap_nocache(win[i]->lw_params.regs_base_phys,
			win[i]->lw_params.regs_size);
		if (!win[i]->lw_params.regs_base_virt) {
			printk(KERN_ERR "%s - Failed ioremap REGS 0x%X\n",
				__func__,
				(unsigned int)win[i]->lw_params.regs_base_virt);
			err = -ENOMEM;
			goto err_handle;
		}

#ifdef CONFIG_OF
		logiwin_get_params(pdev, i, &lw_pdata[i]);
#endif

		win[i]->lw_params.output_format = lw_pdata[i].output_format;
		switch (win[i]->lw_params.output_format) {
			case V4L2_PIX_FMT_RGB565:
			case V4L2_PIX_FMT_YUYV:
				win[i]->lw_params.output_bpp = 16;
				break;
			case V4L2_PIX_FMT_RGB24:
			case V4L2_PIX_FMT_RGB32:
				win[i]->lw_params.output_bpp = 32;
				break;
			default:
				printk(KERN_ERR "%s - Invalid output format\n",
					__func__);
				break;
		}

		if (lw_pdata[i].vmem_base_addr) {
			win[i]->lw_params.vmem_base_phys = lw_pdata[i].vmem_base_addr;
			win[i]->lw_params.vmem_buff_size = lw_pdata[i].vmem_width *
				(win[i]->lw_params.output_bpp/4) * lw_pdata[i].vmem_height;
			if ((win[i]->lw_params.vmem_buff_size) >
				(lw_pdata[i].vmem_base_addr - lw_pdata[i].vmem_high_addr)) {
					printk(KERN_ERR "%s - VMEM size out of range\n", __func__);
					win[i]->lw_params.vmem_base_virt = NULL;
			} else {
				win[i]->lw_params.vmem_base_virt =
					ioremap(win[i]->lw_params.vmem_base_phys,
						(win[i]->lw_params.vmem_buff_size));
			}
			if (!win[i]->lw_params.vmem_base_virt) {
				printk(KERN_ERR "%s - Failed ioremap VMEM 0x%X\n",
					__func__,
					(unsigned int)win[i]->lw_params.vmem_base_virt);
				err = -ENOMEM;
				goto err_handle;
			}

			win[i]->dma_handle.dma_vbuff_size = lw_pdata[i].vmem_width *
				(win[i]->lw_params.output_bpp/4) * lw_pdata[i].input_res_y;
		}

		win[i]->video_dev = video_device_alloc();
		if (!win[i]->video_dev) {
			err = -ENOMEM;
			printk(KERN_ERR "%s - Failed to allocate video device\n",
				__func__);
			goto err_handle;
		}
		*win[i]->video_dev = logiwin_template;

		err = video_register_device(win[i]->video_dev, VFL_TYPE_GRABBER, i);
		if (err) {
			printk(KERN_ERR "%s - Failed to register video device\n",
				__func__);
			goto err_handle;
		} else {
			printk(KERN_INFO "/dev/video%d registered (logiWIN %d)\n", i, i);
		}

		tasklet_init(&win[i]->tasklet, logiwin_handleframe,
			(unsigned long)win[i]);
		mutex_init(&win[i]->fops_lock);
		spin_lock_init(&win[i]->queue_lock);
		init_waitqueue_head(&win[i]->wait);

		win[i]->lw_params.irq = platform_get_irq(pdev, i);
		if (win[i]->lw_params.irq < 0) {
			printk(KERN_ERR "%s - Failed get IRQ\n", __func__);
			err = -EINVAL;
			goto err_handle;
		} else {
			err = request_irq(win[i]->lw_params.irq, logiwin_isr, IRQF_TRIGGER_HIGH,
				DRIVER_NAME, win[i]);
			if (err) {
				printk(KERN_ERR "%s - Failed request IRQ\n", __func__);
				err = -EINVAL;
				goto err_handle;
			}
		}

		logiwin_init_params(win[i], &lw_pdata[i], i);

		video_set_drvdata(win[i]->video_dev, win[i]);
	}

	platform_set_drvdata(pdev, win);

	return 0;

err_handle:
	for(; i >= 0; i--) {
		if (win[i]->lw_params.irq > 0)
			free_irq(win[i]->lw_params.irq, win[i]);
		if (vdev_registered[i])
			video_unregister_device(win[i]->video_dev);
		if (win[i]->video_dev)
			video_device_release(win[i]->video_dev); //???
		if (win[i]->lw_params.vmem_base_virt) {
			iounmap(win[i]->lw_params.vmem_base_virt);
			win[i]->lw_params.vmem_base_virt = NULL;
		}
		if (win[i]->lw_params.regs_base_virt) {
			iounmap(win[i]->lw_params.regs_base_virt);
			win[i]->lw_params.regs_base_virt = NULL;
		}
		if (win[i])
			kfree(win[i]);
	}

	if (vdev_registered)
		kfree(vdev_registered);
	if (win)
		kfree(win);
#ifdef CONFIG_OF
	if (lw_pdata)
		kfree(lw_pdata);
#endif

	return err;
}

static int __devexit logiwin_remove(struct platform_device *pdev)
{
	struct logiwin **win = platform_get_drvdata(pdev);
	int num_of_devices = pdev->num_resources / 2;
	int i;

	for (i = 0; i < num_of_devices; i++) {
		dbg("%s - logiWIN %d\n", __func__, i);
		tasklet_kill(&win[i]->tasklet);
		free_irq(win[i]->lw_params.irq, win[i]);
		video_unregister_device(win[i]->video_dev);
		video_device_release(win[i]->video_dev); //???
		if (win[i]->lw_params.vmem_base_virt)
			iounmap(win[i]->lw_params.vmem_base_virt);
		iounmap(win[i]->lw_params.regs_base_virt);
		kfree(win[i]);
	}
	kfree(win);

	return 0;
}

/*---------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static struct of_device_id logiwin_of_match[] __devinitdata = {
	{ .compatible = "xylon,logiwin-3.02.g" },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, logiwin_of_match);
#endif

static struct platform_driver logiwin_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = logiwin_of_match,
#endif
	},
	.probe = logiwin_probe,
	.remove = __devexit_p(logiwin_remove),
};


#ifndef CONFIG_OF
static struct logiwin_platform_data logiwin_pdata = {
	.vmem_base_addr = 0x30000000,
	.vmem_high_addr = 0x37FFFFFF,
	.vmem_width = 2048,
	.vmem_height = 3240,
	.input_res_x = 1600,
	.input_res_y = 1200,
	.input_num = 1,
	.input_format = LOGIWIN_ITU,
	.output_format = V4L2_PIX_FMT_RGB32,
	.out_byte_align = 1;
	.scale_fraction_bits = 6,
};

static struct resource logiwin_resource[] = {
	{
		.start = 0x400E0000,
		.end = 0x400E1000,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 90,
		.end = 90,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device logiwin_device = {
	.name = DRIVER_NAME,
	.id = 0,
	.dev = {
		.platform_data = &logiwin_pdata,
	},
	.resource = logiwin_resource,
	.num_resources = ARRAY_SIZE(logiwin_resource),
};
#endif


static int __init logiwin_init(void)
{
	int ret;

#ifndef CONFIG_OF
	ret = platform_device_register(&logiwin_device);
	if (ret) {
		printk(KERN_ERR "Error registering logiwin device\n");
		return ret;
	}
#endif
	ret = platform_driver_register(&logiwin_driver);
	if (ret) {
		printk(KERN_ERR "Error registering logiwin driver\n");
		return ret;
	}

	return 0;
}

static void __exit logiwin_exit(void)
{
	platform_driver_unregister(&logiwin_driver);
#ifndef CONFIG_OF
	platform_device_unregister(&logiwin_device);
#endif
}

module_init(logiwin_init);
module_exit(logiwin_exit);

MODULE_DESCRIPTION(LOGIWIN_NAME);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:logiwin");
