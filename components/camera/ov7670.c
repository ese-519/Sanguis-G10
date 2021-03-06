#include "ov7670.h"
#include "camera.h"

static const uint8_t default_regs[][2] =
{
//	{COM7, COM7_RESET},
	{TSLB,  0x04},	/* OV */
	{COM7, 0},	/* VGA */
	{CLKRC, 0x01},
	/*
	 * Set the hardware window.  These values from OV don't entirely
	 * make sense - hstop is less than hstart.  But they work...
	 */
	{HSTART, 0x13},	{HSTOP, 0x01},
	{HREF, 0xb6},	{VSTART, 0x02},
	{VSTOP, 0x7a},	{VREF, 0x0a},

	{COM3, 0},	{COM14, 0},
	/* Mystery scaling numbers */
	{SCALING_XSC, 0x3a},		{SCALING_YSC, 0x35},
	{SCALING_DCWCTR, 0x11},		{SCALING_PCLK_DIV, 0xf0},
	{SCALING_PCLK_DELAY,/* 0x02 changed to 1*/1},
	{COM10, COM10_VSYNC_NEG},
	/* Gamma curve values */
	{0x7a, 0x20},		{0x7b, 0x10},
	{0x7c, 0x1e},		{0x7d, 0x35},
	{0x7e, 0x5a},		{0x7f, 0x69},
	{0x80, 0x76},		{0x81, 0x80},
	{0x82, 0x88},		{0x83, 0x8f},
	{0x84, 0x96},		{0x85, 0xa3},
	{0x86, 0xaf},		{0x87, 0xc4},
	{0x88, 0xd7},		{0x89, 0xe8},
	/* AGC and AEC parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
	{COM8, COM8_FAST_AUTO | COM8_STEP_UNLIMIT},
	{GAIN, 0},	{AECH, 0},
	{COM4, 0x40}, /* magic reserved bit */
//	TEKKER MOD
//	{COM9, 0x18}, /* 4x gain + magic rsvd bit */
	{COM9, 0x6A }, /* 128x gain ceiling; 0x8 is reserved bit */
	{BD50MAX, 0x05},	{BD60MAX, 0x07},
	{AEW, 0x95},	{AEB, 0x33},
	{VPT, 0xe3},	{HAECC1, 0x78},
	{HAECC2, 0x68},	{0xa1, 0x03}, /* magic */
	{HAECC3, 0xd8},	{HAECC4, 0xd8},
	{HAECC5, 0xf0},	{HAECC6, 0x90},
	{HAECC7, 0x94},
	{COM8, COM8_FAST_AUTO|COM8_STEP_UNLIMIT|COM8_AGC_EN|COM8_AEC_EN},
	{0x30,0},{0x31,0},//disable some delays
	/* Almost all of these are magic "reserved" values.  */
	{COM5, 0x61},	{COM6, 0x4b},
	{0x16, 0x02},		{MVFP, 0x07},
	{0x21, 0x02},		{0x22, 0x91},
	{0x29, 0x07},		{0x33, 0x0b},
	{0x35, 0x0b},		{0x37, 0x1d},
	{0x38, 0x71},		{0x39, 0x2a},
	{COM12, 0x78},	{0x4d, 0x40},
	{0x4e, 0x20},		{GFIX, 0},
	/*{0x6b, 0x4a},*/		{0x74,0x10},
	{0x8d, 0x4f},		{0x8e, 0},
	{0x8f, 0},		{0x90, 0},
	{0x91, 0},		{0x96, 0},
	{0x9a, 0},		{0xb0, 0x84},
	{0xb1, 0x0c},		{0xb2, 0x0e},
	{0xb3, 0x82},		{0xb8, 0x0a},

	/* More reserved magic, some of which tweaks white balance */
	{0x43, 0x0a},		{0x44, 0xf0},
	{0x45, 0x34},		{0x46, 0x58},
	{0x47, 0x28},		{0x48, 0x3a},
	{0x59, 0x88},		{0x5a, 0x88},
	{0x5b, 0x44},		{0x5c, 0x67},
	{0x5d, 0x49},		{0x5e, 0x0e},
	{0x6c, 0x0a},		{0x6d, 0x55},
	{0x6e, 0x11},		{0x6f, 0x9e}, /* it was 0x9F "9e for advance AWB" */
	{0x6a, 0x40},		{BLUE, 0x40},
	{RED, 0x60},
	{COM8, COM8_FAST_AUTO|COM8_STEP_UNLIMIT|COM8_AGC_EN|COM8_AEC_EN|COM8_AWB_EN},

	/* Matrix coefficients */
	{0x4f, 0x80},		{0x50, 0x80},
	{0x51, 0},		{0x52, 0x22},
	{0x53, 0x5e},		{0x54, 0x80},
	{0x58, 0x9e},

	{COM16, COM16_AWB},	{EDGE, 0},
	{0x75, 0x05},		{REG76, 0xe1},
	{0x4c, 0x00},		{0x77, 0x01},
	{COM13, /*0xc3*/0x48},	{0x4b, 0x09},
	{0xc9, 0x60},		/*{COM16, 0x38},*/
	{0x56, 0x40},

	{0x34, 0x11},		{COM11, COM11_EXP|COM11_HZAUTO},
	{0xa4, 0x82/*Was 0x88*/},		{0x96, 0},
	{0x97, 0x30},		{0x98, 0x20},
	{0x99, 0x30},		{0x9a, 0x84},
	{0x9b, 0x29},		{0x9c, 0x03},
	{0x9d, 0x4c},		{0x9e, 0x3f},
	{0x78, 0x04},

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{0x79, 0x01},		{0xc8, 0xf0},
	{0x79, 0x0f},		{0xc8, 0x00},
	{0x79, 0x10},		{0xc8, 0x7e},
	{0x79, 0x0a},		{0xc8, 0x80},
	{0x79, 0x0b},		{0xc8, 0x01},
	{0x79, 0x0c},		{0xc8, 0x0f},
	{0x79, 0x0d},		{0xc8, 0x20},
	{0x79, 0x09},		{0xc8, 0x80},
	{0x79, 0x02},		{0xc8, 0xc0},
	{0x79, 0x03},		{0xc8, 0x40},
	{0x79, 0x05},		{0xc8, 0x30},
	{0x79, 0x26},

	{UNDOC_COLOR_CORRECTION, 0x8C}, // Undocumented color correction, set earlier to 0x84 but test this...
	{0xff, 0xff},	/* END MARKER */
};

void ov7670_detect(ovcam_info *info)
{
	sccb_read(OV7670_ADDR, MIDL, (uint8_t *)&info->mid);
	sccb_read(OV7670_ADDR, MIDH, ((uint8_t *)&info->mid)+1);
	sccb_read(OV7670_ADDR, PID, &info->pid);
	sccb_read(OV7670_ADDR, VER, &info->ver);
}

void ov7670_init()
{
	const uint8_t (*ptr)[2]=default_regs;
	while((*ptr)[0]!=0xff)
	{
		sccb_write(OV7670_ADDR, (*ptr)[0], (*ptr)[1]);
		ptr++;
	}
	camera_skip();
}
