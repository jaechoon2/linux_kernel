/*
 * Xylon logiWIN frame grabber driver header file
 *
 * Author: Xylon d.o.o.
 * e-mail: davor.joja@logicbricks.com
 *
 * 2012 Xylon d.o.o.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/* Driver information
 * - before configuring logiWIN registers, stop logiWIN using
 *   logiWIN_stop() function;
 * - choose video input channel using logiWIN_select_input_ch() function
 * - (optional) if logiWIN video input is in ITU656 format set deinterlace mode
 *   using logiWIN_weave_deinterlace() function;
 *   default mode is "weave" deinterlace
 * - (optional) set logiWIN frame rate using logiWIN_frame_rate function; it is
 *   set to full frame rate by default
 * - set output resolution using logiWIN_set_output_res() function
 * - call logiWIN_set_scale() function to set corresponding scaling steps
 * - (optional) set upper-left coordinates for positioning using
 *   logiWIN_set_ul_coords() function; default values are (0,0)
 * - set video memory pointer for the output image using
 *   logiWIN_set_memory_offset() function;
 *   Both of the registers are used only if weave deinterlace is enabled
 *   (ctrl0_reg(4) = 1). Otherwise, value in MEM_OFFSET_ODD register is ignored
 *   and value in MEM_OFFSET_EVEN represents the pointer to block of video
 *   memory where the image is stored.
 * - start logiWIN using logiWIN_start() function;
 */

#ifndef __LOGIWIN_H__
#define __LOGIWIN_H__


#include <asm/io.h>


#define LOGIWIN_DRIVER_VER 3.4


/* logiWIN register offsets */
#define LOGIWIN_REG_DIST_USED 8

/* Down-right X Coordinate. */
#define LOGIWIN_DR_X_ROFF            ( 0  * LOGIWIN_REG_DIST_USED )
/* Down-right Y Coordinate. */
#define LOGIWIN_DR_Y_ROFF            ( 1  * LOGIWIN_REG_DIST_USED )
/* Upper-left X Coordinate. */
#define LOGIWIN_UL_X_ROFF            ( 2  * LOGIWIN_REG_DIST_USED )
/* Upper-left Y Coordinate. */
#define LOGIWIN_UL_Y_ROFF            ( 3  * LOGIWIN_REG_DIST_USED )
/* Horizontal scaling step. */
#define LOGIWIN_SCALE_X_ROFF         ( 4  * LOGIWIN_REG_DIST_USED )
/* Vertical scaling step. */
#define LOGIWIN_SCALE_Y_ROFF         ( 5  * LOGIWIN_REG_DIST_USED )
/* Control register. */
#define LOGIWIN_CTRL0_ROFF           ( 6  * LOGIWIN_REG_DIST_USED )
/* Represents first interpolated pixel’s horizontal distance in respect to the
   first original pixel. */
#define LOGIWIN_START_X_ROFF         ( 7  * LOGIWIN_REG_DIST_USED )
/* Represents first interpolated pixel’s vertical distance in respect to the
   first original pixel. */
#define LOGIWIN_START_Y_ROFF         ( 8  * LOGIWIN_REG_DIST_USED )
/* Number of pixel columns cropped from left side of input video stream. */
#define LOGIWIN_CROP_X_ROFF          ( 9  * LOGIWIN_REG_DIST_USED )
/* Number of lines cropped from upper side of input video stream. */
#define LOGIWIN_CROP_Y_ROFF          ( 10 * LOGIWIN_REG_DIST_USED )
/* Video memory pointer for VGA video input or ITU656 in "bob" deinterlace mode
   only LOGIWIN_MEM_OFFSET_EVEN_ROFF is used.
   If "weave" deinterlace is enabled it defines pointer to the block of video
   memory where even field are stored. */
#define LOGIWIN_MEM_OFFSET_EVEN_ROFF ( 11 * LOGIWIN_REG_DIST_USED )
/* If "weave" deinterlace is enabled it defines pointer to the block of video
   memory where odd fields are stored. If logiWIN is in "bob" deinterlace mode
   or there is VGA video input this register is not used. */
#define LOGIWIN_MEM_OFFSET_ODD_ROFF  ( 12 * LOGIWIN_REG_DIST_USED )
/* Pixel alpha value. */
#define LOGIWIN_PIX_ALPHA_ROFF       ( 13 * LOGIWIN_REG_DIST_USED )
/* Pointer to the block of video memory where data extracted from Vertical
   Blanking Interval is stored. */
#define LOGIWIN_VBI_PTR_ROFF         ( 14 * LOGIWIN_REG_DIST_USED )
/* Contrast enhancement. */
#define LOGIWIN_CONTRAST_ROFF        ( 15 * LOGIWIN_REG_DIST_USED )
/* Saturation enhancement. */
#define LOGIWIN_SATURATION_ROFF      ( 16 * LOGIWIN_REG_DIST_USED )
/* Brightness enhancement. */
#define LOGIWIN_BRIGHTNESS_ROFF      ( 17 * LOGIWIN_REG_DIST_USED )
/* Hue enhancement. */
#define LOGIWIN_COS_HUE_ROFF         ( 18 * LOGIWIN_REG_DIST_USED )
/* Hue enhancement. */
#define LOGIWIN_SIN_HUE_ROFF         ( 19 * LOGIWIN_REG_DIST_USED )
/* 3rd bit is used for field detection (0 - even or bottom, 1 - odd or top). */
#define LOGIWIN_VBUFF_SWITCH_ROFF    ( 20 * LOGIWIN_REG_DIST_USED )
/* Interupt status. */
#define LOGIWIN_INT_STAT_ROFF        ( 22 * LOGIWIN_REG_DIST_USED )
/* Interupt mask. */
#define LOGIWIN_INT_MASK_ROFF        ( 23 * LOGIWIN_REG_DIST_USED )

/* Position of stencil mask BRAM. */
#define LOGIWIN_MASK_BRAM_OFFSET ( 0x200 * LOGIWIN_REG_DIST_USED )

/* Control Register Bit Positions */
/* Start (1)/ stop (0) logiWIN */
#define LOGIWIN_CR_ENABLE_MSK          0x01
/* Enable/disable logiWIN VBI mode. */
#define LOGIWIN_CR_VBI_ENABLE_MSK      0x02
/* Switch Buffer mask. */
#define LOGIWIN_CR_SWBUFF_EVEN_FLD_MSK 0x04
/* Deinterlace mode mask. */
#define LOGIWIN_CR_WEAVE_DEINT_MSK     0x08
/* LogiWIN input channel selection. */
#define LOGIWIN_CR_IN_CH_SEL_MSK       0x10
/* Enable/disable output buffer masking. */
#define LOGIWIN_CR_MASK_ENABLE_MSK     0x20
/* Full frame rate selection mask. */
#define LOGIWIN_CR_FULL_FR_MSK         0x00
/* 75% frame rate selection mask. */
#define LOGIWIN_CR_75_FR_MSK           0x40
/* 50% frame rate selection mask. */
#define LOGIWIN_CR_50_FR_MSK           0x80
/* 25% frame rate selection mask. */
#define LOGIWIN_CR_25_FR_MSK           0xC0
/* HSYNC polarity mask for channel 1 */
#define LOGIWIN_HSYNC_POL_CH_1         0x1000
/* VSYNC polarity mask for channel 1 */
#define LOGIWIN_VSYNC_POL_CH_1         0x2000
/* HSYNC polarity mask for channel 2 */
#define LOGIWIN_HSYNC_POL_CH_2         0x4000
/* VSYNC polarity mask for channel 2 */
#define LOGIWIN_VSYNC_POL_CH_2         0x8000

/* Vbuff Switch Register Bit Positions */
/* Bit 3: 0-even, 1-odd */
#define LOGIWIN_VBUFF_FIELD_MSK 0x08

/* Interrupt Register Bit Positions */
#define LOGIWIN_FRAME_START_INT 0x01

/* Scale step constants */
#define SCALE_STEP_1   (1<<16)
#define SCALE_STEP_MAX ((1<<20)-1)
#define SCALE_FRAC_MSK (SCALE_STEP_1-1)

/* Interpolation starting points */
#define START_X_0   0
#define START_X_0_5 (SCALE_STEP_1/2)
#define START_Y_0   0
#define START_Y_0_5 (SCALE_STEP_1/2)

/* Video memory address where data extracted from
   Vertical Blanking Interval is stored */
#define VBI_PTR_NOT_USED 0xFFFFFFFF
#define VBI_PTR_ADDR VBI_PTR_NOT_USED


/**
 * The logiWIN rectangle.
 */
struct logiwin_rect {
    unsigned long x;      /* x upper left coordinate of the rectangle */
    unsigned long y;      /* y upper left coordinate of the rectangle */
    unsigned long width;  /* width of the rectangle (in pixels) */
    unsigned long height; /* height of the rectangle (in pixels) */
};

/* logiWIN private parameter structure.
* This structure carries pointer to the bus access module structure
* and pointer to the private device driver parameter structure.
* \n Private device driver structure can not be accessed outside of this driver.
*/
struct logiwin_priv {
    /* logiWIN Register base address */
    void *reg_base;
    /* Maximal input rectangle */
    struct logiwin_rect in_bounds_rect;
    /* Input cropping rectangle */
    struct logiwin_rect in_crop_rect;
    /* Output rectangle */
    struct logiwin_rect out_rect;
    /* Calculated output horizontal resolution after scaling or cropping */
    unsigned long max_out_hres;
    /* Calculated output vertical resolution after scaling or cropping */
    unsigned long max_out_vres;
    /* Horizontal scaling step */
    unsigned long scale_step_x;
    /* Vertical scaling step */
    unsigned long scale_step_y;
    /* Number of bits to shift scale/start values before writing to register */
    unsigned long scale_shift_bits;
    /* Output byte alignment mask
       (UL_X and DR_X should be aligned to that number) */
    unsigned long out_byte_align;
    /* Output byte alignment mask */
    unsigned long out_byte_align_mask;
    /* Control register value */
    unsigned long ctrl;
    /* Interrupt mask value */
    unsigned long int_mask;
    /* Defines output image brightness in range 0 - 100 (percent) */
    long brightness;
    /* Defines output image contrast in range 0 - 100 (percent) */
    long contrast;
    /* Defines output image color saturation in range 0 - 100 (percent) */
    long saturation;
    /* Defines output image hue in range -30 - 30 (degrees) */
    long hue;
    /* Device ID */
    unsigned char device_id;
    /* DVI, ITU, RGB */
    unsigned char input_format;
    /* 0 - "bob" deinterlace, 1 - "weave" deinterlace */
    unsigned char weave_deinterlace;
};


/* logiWIN supported Input Format types */
enum logiwin_video_format {
   LOGIWIN_DVI = 0,
   LOGIWIN_ITU,
   LOGIWIN_RGB
};

/* logiWIN supported Frame Rate types */
enum logiwin_frame_rate {
   FRAME_RATE_FULL = 0,
   FRAME_RATE_75,
   FRAME_RATE_50,
   FRAME_RATE_25
};

/* logiWIN supported Rectangle types */
enum logiwin_rectangle {
    CROPPING_RECTANGLE = 0,
    OUTPUT_RECTANGLE,
    INBOUNDS_RECTANGLE,
};

/***************** Macros (Inline Functions) Definitions ********************/

#define logiwin_read32(base_virt, offset) \
    readl(base_virt + offset)
#define logiwin_write32(base_virt, offset, val) \
    writel(val, (base_virt + offset))


/************************** Function Prototypes *****************************/

void logiwin_start(struct logiwin_priv *lw);
void logiwin_stop(struct logiwin_priv *lw);

unsigned long logiwin_get_int_mask(struct logiwin_priv *lw);
void logiwin_unmask_int(struct logiwin_priv *lw, unsigned long mask);
void logiwin_mask_int(struct logiwin_priv *lw, unsigned long mask);
unsigned long logiwin_get_int_stat(struct logiwin_priv *lw);
void logiwin_clear_int_stat(struct logiwin_priv *lw, unsigned long mask);

void logiwin_select_input_ch(struct logiwin_priv *lw, unsigned char select);

void logiwin_sw_sync_polarity(struct logiwin_priv *lw, unsigned char channel,
    unsigned char polarity, unsigned char hsync, unsigned char vsync);

void logiwin_enable_image_masking(struct logiwin_priv *lw,
    unsigned char enable);
void logiwin_enable_buff_sw_even_field(struct logiwin_priv *lw,
    unsigned char enable);
void logiwin_enable_weave_deinterlace(struct logiwin_priv *lw,
    unsigned char weave_deinterlace);

void logiwin_get_rect_parameters(struct logiwin_priv *lw,
    unsigned long *x, unsigned long *y,
    unsigned long *w, unsigned long *h,
    enum logiwin_rectangle rect_type);
void logiwin_set_rect_parameters(struct logiwin_priv *lw,
    unsigned long x, unsigned long y,
    unsigned long w, unsigned long h,
    enum logiwin_rectangle rect_type);

void logiwin_set_frame_rate(struct logiwin_priv *lw,
    enum logiwin_frame_rate frame_rate_sel);
void logiwin_set_memory_offset(struct logiwin_priv *lw,
    unsigned long even_ptr, unsigned long odd_ptr);
void logiwin_set_interpolation_points(struct logiwin_priv *lw,
    unsigned long start_x, unsigned long start_y);
void logiwin_set_pixel_alpha(struct logiwin_priv *lw,
    unsigned long pixel_alpha);
void logiwin_set_vbi_offset(struct logiwin_priv *lw, unsigned char enable);

void logiwin_get_scale_steps(struct logiwin_priv *lw,
    unsigned long *scale_x, unsigned long *scale_y);
void logiwin_set_scale_steps(struct logiwin_priv *lw,
    unsigned long scale_x, unsigned long scale_y);
void logiwin_set_scale(struct logiwin_priv *lw);
void logiwin_set_start_scale(struct logiwin_priv *lw);

void logiwin_set_brightness(struct logiwin_priv *lw, long brightness);
void logiwin_set_contrast(struct logiwin_priv *lw, long contrast);
void logiwin_set_saturation(struct logiwin_priv *lw, long saturation);
void logiwin_set_hue(struct logiwin_priv *lw, long hue);

void logiwin_update_registers(struct logiwin_priv *lw);

void logiwin_write_mask_stencil(struct logiwin_priv *lw,
    unsigned short *mask_buffer, unsigned long offset, unsigned long length);

#endif /* __LOGWIN_H__ */
