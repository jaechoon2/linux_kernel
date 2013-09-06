#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>


//#define SSM2603_CODEC
#define VT1603_CODEC


struct audio_codec_configuration {
	u8 address;
	u8 data;
};

static int set_device_reg(struct i2c_adapter *fmc, struct i2c_msg *msg, int len)
{
	int ret;

	msg->flags &= ~I2C_M_RD;
	msg->len = len;

	ret = i2c_transfer(fmc, msg, 1);

	return ret;
}

//static int get_device_reg(struct i2c_adapter *fmc, struct i2c_msg *msg, int len)
//{
//	int ret;
//
//	msg->flags &= ~I2C_M_RD;
//	msg->len = len;
//	ret = i2c_transfer(fmc, msg, 1);
//	mdelay(1);
//	msg->flags |= I2C_M_RD;
//	ret = i2c_transfer(fmc, msg, 1);
//
//	return ret;
//}


#ifdef SSM2603_CODEC

#define FMC2_GPIO_AC_MUTE 55
#define FMC2_I2C_ADAPTER_ID 7
#define FMC2_I2C_SSM2603_ADDR 0x1A

static void ssm2603_set_sample_rate(int sample_rate)
{
	struct i2c_adapter *adapter;
	struct i2c_msg msg;
	u8 data_buff[2];

	adapter = i2c_get_adapter(FMC2_I2C_ADAPTER_ID);
	if (!adapter) {
		pr_err("%s - no I2C device\n", __func__);
		return;
	}

	msg.addr = FMC2_I2C_SSM2603_ADDR;
	msg.flags = 0;
	msg.len = 0;
	msg.buf = data_buff;

	msg.buf[0] = (0x06<<1);
	// power down, DAC, ADC, MIC, LINEIN
	msg.buf[1] = 0x70;
	if (set_device_reg(adapter, &msg, 2) < 0) {
		goto ssm2603_set_sample_rate_error;
	}
	// sampling rate ADC and DAC, USB mode for 12MHz system clock
	msg.buf[0] = (0x08<<1);
	switch (sample_rate) {
	case 8000:
		msg.buf[1] = 0x0D; //8kHz : 8kHz
		break;
	case 11025:
		msg.buf[1] = 0x33; //11.025kHz : 11.025kHz
		break;
	case 16000:
		msg.buf[1] = 0x29; //16kHz : 16kHz
		break;
	case 22050:
		msg.buf[1] = 0x37; //22.050kHz : 22.050kHz
		break;
	case 44100:
		msg.buf[1] = 0x23; //44.1kHz : 44.1kHz
		break;
	case 48000:
		msg.buf[1] = 0x01; //48kHz : 48kHz
		break;
	case 96000:
		msg.buf[1] = 0x1D; //96kHz : 96kHz
		break;
	default:
		pr_err("Sample rate %d not supported\n", sample_rate);
		pr_info("Set to default 48kHz\n");
		msg.buf[1] = 0x01; //48kHz : 48kHz
		break;
	}

	if (set_device_reg(adapter, &msg, 2) < 0) {
		goto ssm2603_set_sample_rate_error;
	}
	// wait at least 73ms (10.1uF * 25000/3.5)
	mdelay(100);
	// enable DAC output path
	msg.buf[0] = (0x06<<1);
	msg.buf[1] = 0x60;
	if (set_device_reg(adapter, &msg, 2) < 0) {
		goto ssm2603_set_sample_rate_error;
	}

	return;

ssm2603_set_sample_rate_error:
	pr_err("SSM2603 set sample rate error at A 0x%X D 0x%X\n",
		msg.buf[0], msg.buf[1]);
}

static void ssm2603_init(void)
{
	struct audio_codec_configuration ssm2603_config[] = {
		{0x06, 0xFF}, // power down all
		{0x06, 0x70}, // power up DEVICE, DAC, ADC, MIC, LINEIN
		{0x00, 0x17}, // ADC unmute LINEIN L+R
		{0x01, 0x17},
		{0x02, 0x6F}, // DAC volume L+R
		{0x03, 0x6F},
		{0x04, 0x13}, // use DACSEL, disable bypass
		{0x05, 0x06}, // unmute DAC, set de-emphasis rate
		//{0x07, 0x42}, // master and i2s mode - PCB FMC-HMI rev A
		{0x07, 0x02}, // slave and i2s mode - PCB FMC-HMI rev B
		{0x08, 0x01}, // sampling rate ADC and DAC, USB mode
		{0x09, 0x01}, // activate digital core
		{0x06, 0x60}, // enable DAC output path
	};
	struct i2c_adapter *adapter;
	struct i2c_msg msg;
	int i, ret;
	u8 data_buff[2];

	adapter = i2c_get_adapter(FMC2_I2C_ADAPTER_ID);
	if (!adapter) {
		pr_err("%s - no I2C device\n", __func__);
		return;
	}

	msg.addr = FMC2_I2C_SSM2603_ADDR;
	msg.flags = 0;
	msg.len = 0;
	msg.buf = data_buff;

	for(i = 0; i < ARRAY_SIZE(ssm2603_config); i++) {
		msg.buf[0] = (ssm2603_config[i].address<<1);
		msg.buf[1] = ssm2603_config[i].data;
		if (set_device_reg(adapter, &msg, 2) < 0)
			goto ssm2603_init_error;
	}
	// check register values
//	for (i = 0; i < 9; i++) {
//		msg.buf[0] = i<<1;
//		if (get_device_reg(adapter, &msg, 1))
//			printk(KERN_ERR "Error SSM2603 readback\n");
//		pr_info("read A 0x%X D 0x%X\n\r", i, msg.buf[0]);
//	}

	ret = gpio_request(FMC2_GPIO_AC_MUTE, "FMC2 AC mute");
	if (ret < 0)
		printk(KERN_ERR "Error requesting GPIO %d", ret);
	ret = gpio_direction_output(FMC2_GPIO_AC_MUTE, 1);
	if (ret)
		printk(KERN_ERR "Error setting GPIO \"out\" %d", ret);
	gpio_set_value(FMC2_GPIO_AC_MUTE, 1);
	gpio_free(FMC2_GPIO_AC_MUTE);
	mdelay(10);

	return;

ssm2603_init_error:
	pr_err("SSM2603 error at A 0x%X D 0x%X\n", msg.buf[0], msg.buf[1]);
}

#endif //#ifdef SSM2603_CODEC

#ifdef VT1603_CODEC

#define I2C_VT1603_ADDR 0x1A
#define CONFIG_NUM 5

static void vt1603_set_sample_rate(int sample_rate)
{
	struct audio_codec_configuration vt1603_config[][CONFIG_NUM] = {
		{
			{0x03, 0x44},
			{0x05, 0xC4},
			{0x40, 0xF3},
			{0x41, 0x0B},
			{0x42, 0x0C},
		},
		{
			{0x03, 0x04},
			{0x05, 0xC0},
			{0x40, 0x70},
			{0x41, 0x02},
			{0x42, 0x07},
		}
	};
	int adapter_id = 0;
	struct i2c_adapter *adapter;
	struct i2c_msg msg;
	int i;
	int config_index;

	u8 data_buff[2];
	if(sample_rate == 48000) 
		config_index = 1;
	else
		config_index = 0;

	for(adapter_id = 0 ; adapter_id  <= 1 ; adapter_id++) {
		adapter = i2c_get_adapter(adapter_id);
		if (!adapter) {
			pr_err("%s - no I2C device\n", __func__);
			return;
		}

		msg.addr = I2C_VT1603_ADDR;
		msg.flags = 0;
		msg.len = 0;
		msg.buf = data_buff;
		for(i = 0; i < CONFIG_NUM; i++) {
			msg.buf[0] = vt1603_config[config_index][i].address;
			msg.buf[1] = vt1603_config[config_index][i].data;
			if (set_device_reg(adapter, &msg, 2) < 0)
				goto vt1603_init_error;
		}
	}
	return;

vt1603_init_error:
	pr_err("VT1603 error at A 0x%X D 0x%X\n", msg.buf[0], msg.buf[1]);
}

static void vt1603_init(unsigned int i2c_adapter_id)
{
	struct audio_codec_configuration vt1603_config[] = {
		{0xc2, 0x01},
		{0x15, 0xff},  // Software reset for codec part
		{0x15, 0x00},
		{0x60, 0x04},  // Reset codec analog part
		{0x19, 0x2a},  //master mode 24bit  I2S
		{0x07, 0xc0},  //L 0dB
		{0x08, 0xc0},  //R 0dB
		{0x05, 0xc0},  //gain, update, pu aow, sample rate
		{0x0a, 0x41},  //hpf en(parameterized)
		{0x0b, 0x40},  //L2L, R2R, unmute
		{0x0c, 0x00},  //volume  -36dB
		{0x0f, 0x93},  //DRC disable,  DRC 48k SR
		{0x28, 0x00},  //EQ disable
		{0x40, 0x70},  //clk enable
		{0x41, 0x02},  //DAC_DIV=clk_sys/2
		{0x42, 0x07},  //bclk=clk_sys/8
		{0x62, 0xF4},  // Enable and un-mute DAC
		{0x68, 0x4c},  //100% HP local current, hp output enable and hp unmute
		//{0x6e, 0x34},  //NCP setting
		{0x69, 0x93},  // driver input select
		{0x7a, 0x18},
		{0x00, 0xd0},  //DC-Remove, 0dB shift gain
		{0x01, 0x57},  //unmute, LADC gain
		{0x02, 0x57},  //unmute, RADC gain
		{0x03, 0x04},  //48k SR, L2L, R2R
		{0x04, 0x00},  //default
		{0x60, 0xcc},  //vmid: 50k div, vref enable, micbias enable, mic_det enable
		{0x61, 0xf9},  // Enable VREF_SC_DA  4X/4 bias current
		{0x63, 0xe4},  //L/R ADC enable, 30u AAF local current, 50u SDM current
		{0x64, 0x17},  //Lpga gain=0dB, zero cross
		{0x65, 0x17},  //Rpga gain=0dB, zero cross
		{0x66, 0x1E},  //pga not mute, L/R analog in channel enable
		{0x8e, 0xCf},  //L/R input form micin enable, timeout enable
		{0x92, 0x0c},  //bandgap on(default), micbias=90%AVDD
		{0x88, 0x28},
		{0x19, 0x2A},  //full loop
		{0x93, 0x20},  //add offset
		{0x67, 0xf6},
		{0x25, 0x3c}
	};
	struct i2c_adapter *adapter;
	struct i2c_msg msg;
	int i;
	u8 data_buff[2];

	adapter = i2c_get_adapter(i2c_adapter_id);
	if (!adapter) {
		pr_err("%s - no I2C device\n", __func__);
		return;
	}

	msg.addr = I2C_VT1603_ADDR;
	msg.flags = 0;
	msg.len = 0;
	msg.buf = data_buff;

	for(i = 0; i < ARRAY_SIZE(vt1603_config); i++) {
		msg.buf[0] = vt1603_config[i].address;
		msg.buf[1] = vt1603_config[i].data;
		if (set_device_reg(adapter, &msg, 2) < 0)
			goto vt1603_init_error;
	}
	// check register values
//	for (i = 0; i < 197; i++) {
//		msg.buf[0] = i;
//		if (get_device_reg(adapter, &msg, 1))
//			printk(KERN_ERR "Error VT1603 readback\n");
//		pr_info("read A 0x%X D 0x%X\n\r", i, msg.buf[0]);
//	}

	return;

vt1603_init_error:
	pr_err("VT1603 error at A 0x%X D 0x%X\n", msg.buf[0], msg.buf[1]);
}

#endif //#ifdef VT1603_CODEC


void audio_codec_set_sample_rate(int sample_rate)
{
#ifdef SSM2603_CODEC
	ssm2603_set_sample_rate(sample_rate);
#endif
#ifdef VT1603_CODEC
	vt1603_set_sample_rate(sample_rate);
#endif
}

void audio_codec_init(void)
{
#ifdef SSM2603_CODEC
	ssm2603_init();
#endif
#ifdef VT1603_CODEC
	vt1603_init(1);
	vt1603_init(0);
#endif
}
