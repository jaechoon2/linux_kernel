/*
 * Xylon logiWIN frame grabber driver file
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


#include "logiwin.h"


/**
*
* Starts logiWIN operation.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   None.
*
* @note     None.
*
*
*****************************************************************************/
void logiwin_start(struct logiwin_priv *lw)
{
    lw->ctrl |= (LOGIWIN_CR_ENABLE_MSK);
    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* Stops logiWIN operation.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_stop(struct logiwin_priv *lw)
{
    lw->ctrl &= ~(LOGIWIN_CR_ENABLE_MSK);
    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* Reads logiWIN interrupt mask register.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   Interrupt mask register value.
*
* @note     None.
*
*****************************************************************************/
unsigned long logiwin_get_int_mask(struct logiwin_priv *lw)
{
	return lw->int_mask;
}

/**
*
* Enable interrupt in logiWIN interrupt mask register.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    mask is a interrupt bit mask.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_unmask_int(struct logiwin_priv *lw, unsigned long mask)
{
    lw->int_mask &= ~mask;
    logiwin_write32(lw->reg_base, LOGIWIN_INT_MASK_ROFF, lw->int_mask);
}

/**
*
* Disable interrupt in logiWIN interrupt mask register.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    mask is a interrupt bit mask.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_mask_int(struct logiwin_priv *lw, unsigned long mask)
{
    lw->int_mask |= mask;
    logiwin_write32(lw->reg_base, LOGIWIN_INT_MASK_ROFF, lw->int_mask);
}

/**
*
* Reads logiWIN interrupt status register.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   Interrupt status register value.
*
* @note     None.
*
*****************************************************************************/
unsigned long logiwin_get_int_stat(struct logiwin_priv *lw)
{
    return logiwin_read32(lw->reg_base, LOGIWIN_INT_STAT_ROFF);
}

/**
*
* Clears logiWIN interrupt status register.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    mask is a interrupt bit mask.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_clear_int_stat(struct logiwin_priv *lw, unsigned long mask)
{
    logiwin_write32(lw->reg_base, LOGIWIN_INT_STAT_ROFF, mask);
}

/**
*
* Input Video Channel Selection.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    select is a flag for choosing between two video input streams.
*
* @return   None.
*
* @note     0 - logiWIN input channel 0 selected.\n
*           1 - logiWIN input channel 1 selected.
*
*****************************************************************************/
void logiwin_select_input_ch(struct logiwin_priv *lw, unsigned char select)
{
    if (select == 1)
        lw->ctrl |= LOGIWIN_CR_IN_CH_SEL_MSK;
    else
        lw->ctrl &= (~LOGIWIN_CR_IN_CH_SEL_MSK);

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* Enables/disables logiWIN output image masking operation.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    enable is an enable/disable flag.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_enable_image_masking(struct logiwin_priv *lw, unsigned char enable)
{
    if (enable)
        lw->ctrl |= LOGIWIN_CR_MASK_ENABLE_MSK;
    else
        lw->ctrl &= (~LOGIWIN_CR_MASK_ENABLE_MSK);

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* This function is valid only when logiWIN input is configured for
* ITU(interlaced) video stream with double buffering in use.
* It enables video buffer switching after both fields are stored to memory
* i.e. on every even field.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    enable is a flag for enabling switching video buffer on every
*           even field.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_logiwin_enable_buff_sw_even_field(struct logiwin_priv *lw,
    unsigned char enable)
{
    if (enable)
        lw->ctrl |= LOGIWIN_CR_SWBUFF_EVEN_FLD_MSK;
    else
        lw->ctrl &= (~LOGIWIN_CR_SWBUFF_EVEN_FLD_MSK);

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* Enables or disables logiWIN deinterlacing operation.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    weave_deinterlace is a flag for enabling "weave" deinterlace.
*
* @return   None.
*
* @note
*           - 0 - "Bob"   deinterlace.
*           - 1 - "Weave" deinterlace.
*
*****************************************************************************/
void logiwin_enable_weave_deinterlace(struct logiwin_priv *lw,
    unsigned char weave_deinterlace)
{
    if (lw->input_format != LOGIWIN_ITU)
        return;

    if (!lw->weave_deinterlace == !weave_deinterlace)
        return;

    if (weave_deinterlace) {
        lw->ctrl |= LOGIWIN_CR_WEAVE_DEINT_MSK;
        lw->out_rect.y /= 2;
        lw->out_rect.height /= 2;
        lw->scale_step_y *= 2;
    } else {
        lw->ctrl &= (~LOGIWIN_CR_WEAVE_DEINT_MSK);
        lw->out_rect.y *= 2;
        lw->out_rect.height *= 2;
        lw->scale_step_y =
            (lw->scale_step_y / 2) & ((~0) << lw->scale_shift_bits);
    }
    lw->weave_deinterlace = weave_deinterlace;

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* Updates logiWIN max output resolution
*
* @param    rect rectangle to be cropped.
* @param    crop_rect cropping rectangle.
*
* @return   None.
*
* @note     First rectangle is cropped by the other one
*
*****************************************************************************/
static void crop_rect(struct logiwin_rect *rect,
    const struct logiwin_rect *crop_rect)
{
    unsigned long end = rect->x + rect->width;
    unsigned long end_crop = crop_rect->x + crop_rect->width;
    long diff;

    if (rect->x < crop_rect->x)
        rect->x = crop_rect->x;

    if (end > end_crop)
        end = end_crop;

    diff = end - rect->x;

    rect->width = (diff <= 0) ? 0 : diff;

    end = rect->y + rect->height;
    end_crop = crop_rect->y + crop_rect->height;

    if (rect->y < crop_rect->y)
        rect->y = crop_rect->y;

    if (end > end_crop)
        end = end_crop;

    diff = end - rect->y;

    rect->height = (diff <= 0) ? 0 : diff;
}

/**
*
* Gets logiWIN cropping/output rectangle.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    x is a pointer to the upper left coordinate of the cropping/output
*           rectangle.
* @param    y is a pointer to the upper right coordinate of the cropping/output
*           rectangle.
* @param    w is a pointer to the width of the cropping/output rectangle.
* @param    h is a pointer to the height of the cropping/output rectangle.
* @param    rect_type a flag for choosing between cropping and output rectangle.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_get_rect_parameters(struct logiwin_priv *lw,
    unsigned long *x, unsigned long *y,
    unsigned long *w, unsigned long *h,
    enum logiwin_rectangle rect_type)
{
    struct logiwin_rect *rect;
    int weave = 0;

    switch (rect_type) {
    case CROPPING_RECTANGLE:
        rect = &lw->in_crop_rect;
        break;

    case OUTPUT_RECTANGLE:
        rect = &lw->out_rect;
        if (lw->weave_deinterlace)
            weave = 1;
        break;

    case INBOUNDS_RECTANGLE:
        rect = &lw->in_bounds_rect;
        break;

    default:
        return;
    }

    if (x)
        *x = rect->x;
    if (y)
        *y = rect->y << weave;
    if (w)
        *w = rect->width;
    if (h)
        *h = rect->height << weave;
}

/**
*
* Sets logiWIN desired rectangle.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    x is a upper left coordinate of the cropping/output rectangle.
* @param    y is a upper right coordinate of the cropping/output rectangle.
* @param    w is a width of the cropping/output rectangle.
* @param    h is a height of the cropping/output rectangle.
* @param    rect_type a flag for choosing between cropping and output rectangle.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_rect_parameters(struct logiwin_priv *lw,
    unsigned long x, unsigned long y,
    unsigned long w, unsigned long h,
    enum logiwin_rectangle rect_type)
{
    struct logiwin_rect out_rect;

    if ((x % lw->out_byte_align))
        x = x & lw->out_byte_align_mask;
    if ((w % lw->out_byte_align))
        w = w & lw->out_byte_align_mask;

    switch (rect_type) {
    case CROPPING_RECTANGLE:
        lw->in_crop_rect.x = x;
        lw->in_crop_rect.y = y;
        lw->in_crop_rect.width = w;
        lw->in_crop_rect.height = h;

        crop_rect(&lw->in_crop_rect, &lw->in_bounds_rect);

        break;

    case OUTPUT_RECTANGLE:
        lw->out_rect.x = x & lw->out_byte_align_mask;
        lw->out_rect.y = y;
        lw->out_rect.width = w & lw->out_byte_align_mask;
        lw->out_rect.height = h;

        out_rect.x = 0;
        out_rect.y = 0;
        out_rect.width = lw->max_out_hres;
        out_rect.height = lw->max_out_vres;
        crop_rect(&lw->out_rect, &out_rect);

        if (lw->weave_deinterlace) {
            lw->out_rect.y /= 2;
            lw->out_rect.height /= 2;
        }
        break;

    case INBOUNDS_RECTANGLE:
        lw->in_bounds_rect.x = x;
        lw->in_bounds_rect.y = y;
        lw->in_bounds_rect.width = w;
        lw->in_bounds_rect.height = h;

        crop_rect(&lw->in_crop_rect, &lw->in_bounds_rect);

        break;
    }
}

/**
*
* Sets logiWIN frame rate.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    frame_rate_sel is a frame rate selector.
*
* @return   None.
*
* @note
*           frame_rate_sel :
*           - FRAME_RATE_FULL - full frame rate
*           - FRAME_RATE_75   - 75% frame is stored
*           - FRAME_RATE_50   - 50% frame is stored
*           - FRAME_RATE_25   - 25% frame is stored
*
*****************************************************************************/
void logiwin_set_frame_rate(struct logiwin_priv *lw,
    enum logiwin_frame_rate frame_rate_sel)
{
    lw->ctrl &= 0x3F;

    switch(frame_rate_sel) {
    case FRAME_RATE_FULL:
        lw->ctrl |= LOGIWIN_CR_FULL_FR_MSK;
        break;
    case FRAME_RATE_75:
        lw->ctrl |= LOGIWIN_CR_75_FR_MSK;
        break;
    case FRAME_RATE_50:
        lw->ctrl |= LOGIWIN_CR_50_FR_MSK;
        break;
    case FRAME_RATE_25:
        lw->ctrl |= LOGIWIN_CR_25_FR_MSK;
        break;
    }

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}

/**
*
* Sets logiWIN memory offset values.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    even_ptr is a memory pointer for the even field of the processed
*           Video input stream.
* @param    odd_ptr is a memory pointer for the odd field of the processed Video
*           input stream.
*
* @return   None.
*
* @note     \n For VGA video input or ITU656 in "bob" deinterlace mode only
*              even_ptr is used.
*           \n odd_ptr is used only when logiWIN has ITU656 as video input and
*              is working in "weave" deinterlace mode.
*           \n even_ptr defines pointer to the block of video memory where even
*              field are stored.
*           \n odd_ptr  defines pointer to the block of video memory where odd
*              field are stored.
*
*****************************************************************************/
void logiwin_set_memory_offset(struct logiwin_priv *lw,
    unsigned long even_ptr, unsigned long odd_ptr)
{
    logiwin_write32(lw->reg_base, LOGIWIN_MEM_OFFSET_EVEN_ROFF, even_ptr);
    logiwin_write32(lw->reg_base, LOGIWIN_MEM_OFFSET_ODD_ROFF, odd_ptr);
}

/**
*
* Sets logiWIN interpolation starting points.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    start_x is a horizontal distance of the first interpolated pixel
*           from the first original pixel.
* @param    start_y is a vertical distance of the first interpolated pixel from
*           the first original pixel.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_interpolation_points(struct logiwin_priv *lw,
    unsigned long start_x, unsigned long start_y)
{
    logiwin_write32(lw->reg_base,
        LOGIWIN_START_X_ROFF, start_x >> lw->scale_shift_bits);
    logiwin_write32(lw->reg_base,
        LOGIWIN_START_Y_ROFF, start_y >> lw->scale_shift_bits);
}

/**
*
* Sets logiWIN pixel alpha blending value.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    pixelAlpha is a value of pixel alpha.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_pixel_alpha(struct logiwin_priv *lw, unsigned long pixelAlpha)
{
    logiwin_write32(lw->reg_base, LOGIWIN_PIX_ALPHA_ROFF, pixelAlpha);
}

/**
*
* Sets logiWIN VBI memory offset.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    enable is a VBI Mode enable/disable flag.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_vbi_offset(struct logiwin_priv *lw, unsigned char enable)
{
    if (VBI_PTR_ADDR == VBI_PTR_NOT_USED)
        return;

    if (enable)
        lw->ctrl |= LOGIWIN_CR_VBI_ENABLE_MSK;
    else
        lw->ctrl &= (~LOGIWIN_CR_VBI_ENABLE_MSK);

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
    logiwin_write32(lw->reg_base, LOGIWIN_VBI_PTR_ROFF, VBI_PTR_ADDR);
}

/**
*
* Gets logiWIN scaling factors.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    scale_x is a pointer to the horizontal scaling step.
* @param    scale_y is a pointer to the vertical scaling step.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_get_scale_steps(struct logiwin_priv *lw,
    unsigned long *scale_x, unsigned long *scale_y)
{
    *scale_x = lw->scale_step_x;
    *scale_y = lw->scale_step_y;
    if (lw->weave_deinterlace)
        *scale_y /= 2;
}

/**
*
* Sets horizontal and vertical scaling steps.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    scale_x is a horizontal scaling step.
* @param    scale_y is a vertical scaling step.
*
* @return   None.
*
* @note     This function is used to set up desired scaling steps directly.
*           When using this function, function logiwin_set_scale is not used.
*
*           Scaling steps are formated in 4.6 fraction format.
*           Maximum downscale factor 16 (scale step equals 16).
*           Maximum upscale factor 64 (scale step equals 1/64 = 0.015625).
*
*****************************************************************************/
void logiwin_set_scale_steps(struct logiwin_priv *lw,
    unsigned long scale_x, unsigned long scale_y)
{
    unsigned long scale_prec_mask = (~0) << lw->scale_shift_bits;
    unsigned long scale_step_min = 1 << lw->scale_shift_bits;

    if (lw->weave_deinterlace)
        scale_y *= 2;

    scale_x &= scale_prec_mask;
    scale_y &= scale_prec_mask;

    /* Horizontal scale step */
    if (scale_x < scale_step_min)
        scale_x = scale_step_min;
    else if (scale_x > SCALE_STEP_MAX)
        scale_x = SCALE_STEP_MAX;

    /* Vertical scale step */
    if (scale_y < scale_step_min)
        scale_y = scale_step_min;
    else if (scale_y > SCALE_STEP_MAX)
        scale_y = SCALE_STEP_MAX;

    /* Scale steps in 4.6 fraction format */
    lw->scale_step_x = scale_x;
    lw->scale_step_y = scale_y;

    /* Set interpolation point */
    logiwin_set_start_scale(lw);
}

/**
*
* Sets logiWIN scaling factors.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   None.
*
* @note     Calculate new scaling factors based on input and output resolution.
*
*****************************************************************************/
void logiwin_set_scale(struct logiwin_priv *lw)
{
    unsigned long scale_step_x, scale_step_y;

    scale_step_x =
        (SCALE_STEP_1 * lw->in_crop_rect.width) / lw->out_rect.width;
    scale_step_y =
        (SCALE_STEP_1 * lw->in_crop_rect.height) / lw->out_rect.height;

    if (lw->weave_deinterlace)
        scale_step_y /= 2;

    logiwin_set_scale_steps(lw, scale_step_x, scale_step_y);
}

/**
*
* Sets starting points for interpolation depending on scaling step.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_start_scale(struct logiwin_priv *lw)
{
    unsigned long scale_step_x, scale_step_y;
    unsigned long start_x, start_y;

    scale_step_x = lw->scale_step_x;
    scale_step_y = lw->scale_step_y;

    /* Horizontal distance of the first interpolated pixel from the first
       original pixel */
    if (scale_step_x <= SCALE_STEP_1)
        start_x = START_X_0;
    else if ((scale_step_x & SCALE_FRAC_MSK) == 0)
        start_x = START_X_0_5;
    else
        start_x = (scale_step_x & SCALE_FRAC_MSK) / 2;

    /* Vertical distance of the first interpolated pixel from the first
       original pixel */
    if (scale_step_y <= SCALE_STEP_1)
        start_y = START_Y_0;
    else if ((scale_step_y & SCALE_FRAC_MSK) == 0)
        start_y = START_Y_0_5;
    else
        start_y = (scale_step_y & SCALE_FRAC_MSK) / 2;

    /* Write interpolation starting points into registers */
    logiwin_write32(lw->reg_base,
        LOGIWIN_START_X_ROFF, start_x >> lw->scale_shift_bits);
    logiwin_write32(lw->reg_base,
        LOGIWIN_START_Y_ROFF, start_y >> lw->scale_shift_bits);
}

/**
*
* Sets output image brightness.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    brightness defines image brightness, integer values range -50 - 50.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_brightness(struct logiwin_priv *lw, long brightness)
{
    unsigned long tmp;

    if (brightness < -50)
        brightness = -50;
    else if (brightness > 50)
        brightness = 50;

    lw->brightness = brightness;

    tmp = 32 + ((63 * brightness) / 100);
    logiwin_write32(lw->reg_base, LOGIWIN_BRIGHTNESS_ROFF, tmp);
}

/**
*
* Sets output image contrast.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    contrast defines image contrast, integer values range -50 - 50.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_contrast(struct logiwin_priv *lw, long contrast)
{
    unsigned long tmp;
    //unsigned long sat_tmp;

    if (contrast < -50)
        contrast = -50;
    else if (contrast > 50)
        contrast = 50;

    lw->contrast = contrast;

    tmp = 1992 * (contrast + 50) * 2048 / 100000;
    logiwin_write32(lw->reg_base, LOGIWIN_CONTRAST_ROFF, tmp);

    //sat_tmp = 1992 * (lw->saturation + 50) / 100;
    //tmp = tmp * sat_tmp / 1000;
    //logiwin_write32(lw->reg_base, LOGIWIN_SATURATION_ROFF, tmp);
}

/**
*
* Sets output image color saturation.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    saturation defines image saturation, integer values range -50 - 50.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_saturation(struct logiwin_priv *lw, long saturation)
{
    unsigned long tmp;
    //unsigned long con_tmp;

    if (saturation < -50)
        saturation = -50;
    else if (saturation > 50)
        saturation = 50;

    lw->saturation = saturation;

    tmp = 1992 * (saturation + 50) * 2048 / 100000;
    //con_tmp = 1992 * (lw->contrast + 50) / 100;
    //tmp = tmp * con_tmp / 1000;

    logiwin_write32(lw->reg_base, LOGIWIN_SATURATION_ROFF, tmp);
}

/**
*
* Sets output image hue.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    hue defines image hue, integer values range -30 - 30 (degrees).
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_set_hue(struct logiwin_priv *lw, long hue)
{
    static const unsigned short cos_table[31] = {
        2048, 2047, 2046, 2045, 2043, 2040, 2036, 2032, 2028, 2022,
        2016, 2010, 2003, 1995, 1987, 1978, 1968, 1958, 1947, 1936,
        1924, 1911, 1898, 1885, 1870, 1856, 1840, 1824, 1808, 1791, 1773
    };
    static const unsigned short sin_table[31] = {
        0, 35, 71, 107, 142, 178, 214, 249, 285, 320,
        355, 390, 425, 460, 495, 530, 564, 598, 632, 666,
        700, 733, 767, 800, 832, 865, 897, 929, 961, 992, 1024
    };
    unsigned short cos_reg, sin_reg;

    if (hue < -30)
        hue = -30;
    else if (hue > 30)
        hue = 30;

    lw->hue = hue;

    sin_reg = (hue < 0) ? -sin_table[-hue] : sin_table[hue];
    cos_reg = (hue < 0) ?  cos_table[-hue] : cos_table[hue];

    logiwin_write32(lw->reg_base, LOGIWIN_COS_HUE_ROFF, cos_reg);
    logiwin_write32(lw->reg_base, LOGIWIN_SIN_HUE_ROFF, sin_reg);
}

/**
*
* Sets logiWIN registers: CROP, SCALE, UL and DR.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void logiwin_update_registers(struct logiwin_priv *lw)
{
    unsigned long dr_x;
    unsigned long dr_y;
    unsigned long out_hres_scale =
        lw->in_bounds_rect.x + lw->in_bounds_rect.width - lw->in_crop_rect.x;
    unsigned long out_vres_scale =
        lw->in_bounds_rect.y + lw->in_bounds_rect.height - lw->in_crop_rect.y;
    unsigned long tmp_width = lw->out_rect.width;
    unsigned long tmp_height = lw->out_rect.height;

    out_hres_scale = out_hres_scale * SCALE_STEP_1 / lw->scale_step_x;
    out_vres_scale = out_vres_scale * SCALE_STEP_1 / lw->scale_step_y;

    if (lw->out_rect.width > out_hres_scale)
        tmp_width = out_hres_scale;
    if (lw->out_rect.height > out_vres_scale)
        tmp_height = out_vres_scale;

    dr_x = lw->out_rect.x + tmp_width - 1;
    dr_y = lw->out_rect.y + tmp_height - 1;

    if (dr_x % lw->out_byte_align)
        dr_x &= lw->out_byte_align_mask;

    logiwin_write32(lw->reg_base, LOGIWIN_CROP_X_ROFF, lw->in_crop_rect.x);
    logiwin_write32(lw->reg_base, LOGIWIN_CROP_Y_ROFF, lw->in_crop_rect.y);

    logiwin_write32(lw->reg_base,
        LOGIWIN_SCALE_X_ROFF, lw->scale_step_x >> lw->scale_shift_bits);
    logiwin_write32(lw->reg_base,
        LOGIWIN_SCALE_Y_ROFF, lw->scale_step_y >> lw->scale_shift_bits);

    logiwin_write32(lw->reg_base, LOGIWIN_UL_X_ROFF, lw->out_rect.x);
    logiwin_write32(lw->reg_base, LOGIWIN_UL_Y_ROFF, lw->out_rect.y);

    logiwin_write32(lw->reg_base, LOGIWIN_DR_X_ROFF, dr_x);
    logiwin_write32(lw->reg_base, LOGIWIN_DR_Y_ROFF, dr_y);
}

/**
*
* Writes masking stencil to dedicated BRAM registers.
*
* @param    lw is a pointer to the logiWIN device parameter structure.
* @param    mask_buffer is a pointer to input mask data.
* @param    offset is offset in mask .
* @param    length is length of input mask data.
*
* @return   None.
*
* @note     Offset and length must be even values.
*
*****************************************************************************/
void logiwin_write_mask_stencil(struct logiwin_priv *lw,
    unsigned short *mask_buffer, unsigned long offset, unsigned long length)
{
    if ((offset >= 2048) || (offset + length > 2048)) {
        return;
    } else {
        unsigned long pos = LOGIWIN_MASK_BRAM_OFFSET + offset * 2;
        unsigned long end = pos + length * 2;
        unsigned long *bufferPtr = (unsigned long *)mask_buffer;

        for (; pos<end; pos+=4, bufferPtr++)
            logiwin_write32(lw->reg_base, pos, *bufferPtr);
    }
}

/**
*
* Switches HSYNC and VSYNC polarity.
*
* @param[in]    logiWinPtr is a pointer to the logiWIN instance to be worked on.
* @param[in]    Channel: 0 - select logiWIN channel 0, 1 - select logiWIN channel 1
* @param[in]    Polarity: 0 - sync signals active low, 1 - sync signals active high
* @param[in]    Hsync: 0 - preserve HSYNC polarity, 1 - switch HSYNC polarity
* @param[in]    Vsync: 0 - preserve VSYNC polarity, 1 - switch VSYNC polarity
*
* @return   XST_SUCCESS
*
* @note     None
*
*****************************************************************************/
void logiwin_sw_sync_polarity(struct logiwin_priv *lw, unsigned char channel,
    unsigned char polarity, unsigned char hsync, unsigned char vsync)
{
    if (channel == 0) {
        // Selected logiWIN channel 1.
        // Switch HSYNC polarity.
        if (hsync == 1) {
            if (!polarity) {
                // Sync signal active low.
                lw->ctrl |= LOGIWIN_HSYNC_POL_CH_1;
            } else {
                // Sync signal active high
                lw->ctrl &= (~LOGIWIN_HSYNC_POL_CH_1);
            }
        }
        // Switch VSYNC polarity.
        if (vsync == 1) {
            if (!polarity) {
                // Sync signal active low.
                lw->ctrl |= LOGIWIN_VSYNC_POL_CH_1;
            } else {
                // Sync signal active high.
                lw->ctrl &= (~LOGIWIN_VSYNC_POL_CH_1);
            }
        }
    } else if (channel == 1) {
        // Selected logiWIN channel 0.
        if (hsync == 1) {
            // Switch HSYNC polarity.
            if (!polarity) {
                // Sync signal active low.
                lw->ctrl |= LOGIWIN_HSYNC_POL_CH_2;
            } else {
                // Sync signal active high.
                lw->ctrl &= (~LOGIWIN_HSYNC_POL_CH_2);
            }
        }
        if (vsync == 1) {
            // Switch VSYNC polarity.
            if (!polarity) {
                // Sync signal active low.
                lw->ctrl |= LOGIWIN_VSYNC_POL_CH_2;
            } else {
                // Sync signal active high.
                lw->ctrl &= (~LOGIWIN_VSYNC_POL_CH_2);
            }
        }
    }

    logiwin_write32(lw->reg_base, LOGIWIN_CTRL0_ROFF, lw->ctrl);
}
