#include "stdint.h"

#ifndef __REG_REGS_H__
#define __REG_REGS_H__

#define GAIN                    0x00 /* AGC – Gain control gain setting  */
#define BLUE                    0x01 /* AWB – Blue channel gain setting  */
#define RED                     0x02 /* AWB – Red channel gain setting   */

#define VREF                    0x03 /* VREF – Vertical Frame Control */
#define VREF_AGCH				0xC0 /* AGC[9:8] */
#define VREF_EL					0x0C /* VREF end low 2 bits */
#define VREF_SL					0x03 /* VREF start low 2 bits */

#define COM1                    0x04 /* Common Control 1 */
#define COM1_CCIR               0x40 /* CCIR656 Format */
#define COM1_AECL               0x03 /* Exposure Value – AEC LSBs */

#define BAVe                    0x05 /* U/B Average Level   */
#define GbAVE                   0x06 /* Y/Gb Average Level  */
#define AECHH                   0x07 /* Exposure Value – AEC MSBs */
#define RAVE                    0x08 /* V/R Average Level   */

#define COM2                    0x09 /* Common Control 2 */
#define COM2_SOFT_SLEEP         0x10 /* Soft sleep mode  */
#define COM2_OUT_DRIVE_1x       0x00 /* Output drive capability 1x */
#define COM2_OUT_DRIVE_2x       0x01 /* Output drive capability 2x */
#define COM2_OUT_DRIVE_3x       0x02 /* Output drive capability 3x */
#define COM2_OUT_DRIVE_4x       0x03 /* Output drive capability 4x */

#define PID     	            0x0A /* Product ID Number MSB */
#define VER	                    0x0B /* Product ID Number LSB */

#define COM3                    0x0C /* Common Control 3                                        */
#define COM3_SWAP_MSB           0x40 /* Swap output MSB/LSB                                     */
#define COM3_TRI_CLOCK          0x20 /* Tri-state option for output clock at power-down period  */
#define COM3_TRI_DATA           0x10 /* Tri-state option for output data at power-down period   */
#define COM3_SCALE              0x08 /* Scale enable */
#define COM3_DCW                0x04 /* DCW enable */

#define COM4                    0x0D /* Common Control 4         */
#define COM4_AEC_FULL           0x00 /* AEC evaluate full window */
#define COM4_AEC_1_2            0x10 /* AEC evaluate 1/2 window  */
#define COM4_AEC_1_4            0x20 /* AEC evaluate 1/4 window  */
#define COM4_AEC_2_3            0x30 /* AEC evaluate 2/3 window  */

#define COM5                    0x0E /* Common Control 5 */

#define COM6                    0x0F /* Common Control 6 */
#define COM6_HREF_BLACK         0x80 /* Enable/Disable HREF at optical black */
#define COM6_BLC_INPUT          0x40 /* Black-Level Calibration input selection */
#define COM6_BLC_ENABLE         0x20 /* Digital Black-Level Calibration Enable */
#define COM6_RST_FMTCHG         0x02 /* Reset all timing when format changes */

#define AECH                    0x10 /* AEC[9:2] (see register AECH for AEC[15:10], COM1 for AEC[1:0]) */

#define CLKRC                   0x11 /* Internal Clock */
#define CLKRC_DOUBLE            0x80 /* Digital PLL option */
#define CLKRC_EXTERN            0x40 /* Use external clock directly */
#define CLKRC_INTERN            0x3F /* Internal clock prescaler */

#define COM7                    0x12 /* Common Control 7         */
#define COM7_RESET              0x80 /* SCCB Register Reset      */
#define COM7_OF_CIF             0x20 /* Output format CIF selection   */
#define COM7_OF_QVGA	        0x10 /* Output format QVGA selection */
#define COM7_OF_QCIF            0x08 /* Output format QCIF selection */
#define COM7_OF_RGB_YUV         0x00 /* YUV */
#define COM7_OF_RGB_RGB			0x04 /* RGB */
#define COM7_OF_RGB_BRGB		0x01 /* Bayer RAW */
#define COM7_OF_RGB_PBRGB		0x05 /* Processed Bayer RAW */
#define COM7_CB_EN		        0x02 /* Color Bar Enable */

#define COM8                    0x13 /* Common Control 8                */
#define COM8_FAST_AUTO          0x80 /* Enable fast AGC/AEC algorithm   */
#define COM8_STEP_VSYNC         0x00 /* AEC - Step size limited to vertical blank */
#define COM8_STEP_UNLIMIT       0x40 /* AEC - Step size unlimited step size       */
#define COM8_BANDF_OFF          0x20 /* Banding filter OFF */
#define COM8_AGC_EN             0x04 /* AGC Enable */
#define COM8_AWB_EN             0x02 /* AWB Enable */
#define COM8_AEC_EN             0x01 /* AEC Enable */

#define COM9                    0x14 /* Common Control 9 */
#define COM9_AGC_GAIN_2x        0x00 /* Automatic Gain Ceiling 2x  */
#define COM9_AGC_GAIN_4x        0x10 /* Automatic Gain Ceiling 4x  */
#define COM9_AGC_GAIN_8x        0x20 /* Automatic Gain Ceiling 8x  */
#define COM9_AGC_GAIN_16x       0x30 /* Automatic Gain Ceiling 16x */
#define COM9_AGC_GAIN_32x       0x40 /* Automatic Gain Ceiling 32x */
#define COM9_AGC_GAIN_64x		0x50 /* Automatic Gain Ceiling 64x */
#define COM9_AGC_GAIN_128x		0x60 /* Automatic Gain Ceiling 128x */
#define COM9_FREZE_AGC          0x01 /* Freeze AGC/AEC */

#define COM10                   0x15 /* Common Control 10 */
#define COM10_HSYNC_EN          0x40 /* HREF changes to HSYNC */
#define COM10_PCLK_FREE         0x00 /* PCLK output option: free running PCLK */
#define COM10_PCLK_MASK         0x20 /* PCLK output option: PCLK does not toggle during horizonal blank */
#define COM10_PCLK_REV          0x10 /* PCLK reverse */
#define COM10_HREF_REV          0x08 /* HREF reverse */
#define COM10_VSYNC_FALLING     0x00 /* VSYNC changes on falling edge of PCLK */
#define COM10_VSYNC_RISING      0x04 /* VSYNC changes on rising edge of PCLK */
#define COM10_VSYNC_NEG         0x02 /* VSYNC negative */
#define COM10_HSYNC_NEG	        0x01 /* HSYNC negative */

//#define RSVD                	0x16 /* Register 16 */
#define HSTART                  0x17 /* Horizontal Frame (HREF column) start high 8-bit (low 3 bts are at HREF[2:0]) */
#define HSTOP                   0x18 /* Horizontal Frame (HREF column) end high 8-bit (low 3 bts are at HREF[5:3])*/
#define VSTART                  0x19 /* Vertical Frame (row) start high 8-bit (low 2 bts are at VREF[1:0]) */
#define VSTOP                   0x1A /* Vertical Frame (row) end high 8-bit (low 2 bts are at VREF[3:2]) */
#define PSHFT                   0x1B /* Data Format - Pixel Delay Select */
#define MIDH               	    0x1C /* Manufacturer ID Byte – High */
#define MIDL                    0x1D /* Manufacturer ID Byte – Low */

#define MVFP					0x1E /* Mirror/VFlip Enable */
#define MVFP_MI					0x20 /* Mirror Image */
#define MVFP_VE					0x10 /* VFlip enable Vertically flip image */
#define MVFP_BSE				0x02 /* Black sun enable */

#define LAEC                    0x1F /* Fine AEC Value - defines exposure value less than one row period */

#define ADCCTR0                 0x20 /* Common Control 11 */
#define ADCCTR0_RANGE_1x     	0x00 /* ADC range adjustment 1x range */
#define ADCCTR0_RANGE_1_5x     	0x08 /* ADC range adjustment 1.5x range */
#define ADCCTR0_REF_0_8x		0x00 /* ADC reference adjustment 0.8x */
#define ADCCTR0_REF_1x			0x04 /* ADC reference adjustment 1x */
#define ADCCTR0_REF_1_2x		0x07 /* ADC reference adjustment 1.2x */

#define ADCCTR1                 0x21
#define ADCCTR2                 0x22 /* Banding Filter Minimum AEC Value */
#define ADCCTR3                 0x23 /* Banding Filter Maximum Step */
#define AEW                     0x24 /* AGC/AEC - Stable Operating Region (Upper Limit) */
#define AEB                     0x25 /* AGC/AEC - Stable Operating Region (Lower Limit) */

#define VPT                     0x26 /* AGC/AEC Fast Mode Operating Region */
#define VPT_UPPER				0xF0 /* High nibble of upper limit of fast mode control zone */
#define VPT_LOWER				0x0F /* High nibble of lower limit of fast mode control zone */

#define BBIAS                   0x27 /* B channel signal output bias */
#define BBIAS_AS_ADD			0x00 /* Bias adjustment sign Add bias */
#define BBIAS_AS_SUB			0x80 /* Bias adjustment sign Subtract bias */
#define BBIAS_BV				0x7F /* Bias value of 10-bit range */

#define GbBIAS                  0x28 /* Gb Channel Signal Output Bias */
#define GbBIAS_AS_ADD			0x00 /* Bias adjustment sign Add bias */
#define GbBIAS_AS_SUB			0x80 /* Bias adjustment sign Subtract bias */
#define GbBIAS_BV				0x7F /* Bias value of 10-bit range */

//#define RSVD                    0x29 
#define EXHCH                   0x2A /* Dummy Pixel Insert MSB */
#define EXHCH_HD				0xF0 /* 4 MSB for dummy pixel insert in horizontal direction */
#define EXHCH_FALLING			0x0C /* HSYNC falling edge delay 2 MSB */ 
#define EXHCH_RISING			0x03 /* HSYNC rising edge delay 2 MSB */

#define EXHCL                   0x2B /* Dummy Pixel Insert LSB */

#define RBIAS                   0x2C /* R Channel Signal Output Bias */
#define RBIAS_AS_ADD			0x00 /* Bias adjustment sign Add bias */
#define RBIAS_AS_SUB			0x80 /* Bias adjustment sign Subtract bias */
#define RBIAS_BV				0x7F /* Bias value of 10-bit range */			

#define ADVFL                   0x2D /* LSB of Insert Dummy Rows in Vertical Sync (1 bit equals 1 row)  */
#define ADVFH                   0x2E /* MSB of Insert Dummy Rows in Vertical Sync */
#define YAVE                    0x2F /* Y/G Channel Average Value */
#define HSYST                   0x30 /* HSYNC Rising Edge Delay (low 8 bits) */
#define HSYEN                   0x31 /* HSYNC Falling Edge Delay (low 8 bits) */

#define HREF                    0x32 /* HREF Control */
#define HREF_EO					0xC0 /* HREF edge offset to date output */
#define HREF_EL					0x38 /* HREF end 3 LSB */
#define HREF_SL					0x07 /* HREF start 3 LSB */

#define CHLF                    0x33 /* Array Current Control */
#define ARBLM                   0x34 /* Array Reference Control */
//#define RSVD                    0x35
//#define RSVD                    0x36 
#define ADC                     0x37 /* ADC Control */
#define ACOM                    0x38 /* ADC and Analog Common Mode Control */
#define OFON                    0x39 /* ADC Offset Control  */

#define TSLB                    0x3A /* Line Buffer Test Option */
#define TSLB_NI					0x20 /* Negative image enable */
#define TSLB_UV_OV				0x10 /* Use fixed UV value set in registers MANU and MANV as UV output instead of chip output */
#define TSLB_OS_YUYV_YVYU		0x00 /* Output sequence (use with register COM13[1]) TSLB[3], COM13[1]: YUYV & YVYU */
#define TSLB_OS_UYVY_VYUY		0X40 /* Output sequence (use with register COM13[1]) TSLB[3], COM13[1]: UYVY & VYUY */

#define COM11                   0x3B /* Common Control 11 */
#define COM11_NM				0x80 /* Night mode enable */
#define COM11_MFR_N				0x00 /* Same as normal mode frame rate */
#define COM11_MFR_1_2_N			0x20 /* 1/2 of normal mode frame rate */
#define COM11_MFR_1_4_N			0x40 /* 1/4 of normal mode frame rate */
#define COM11_MFR_1_8_N			0x60 /* 1/8 of normal mode frame rate */
#define COM11_HZAUTO				0x10 /* D56_Auto: Enabel 50/60 Hz auto detection */
#define COM11_FVS_E				0x00 /* Banding filter value select: Select BD60ST[7:0] (0x9E) as Banding filter value */
#define COM11_FVS_D				0x08 /* Banding filter value select: Select BD60ST[7:0] (0x9D) as Banding filter value */
#define COM11_EXP				0x02 /* Exposure timing can be less than limit of banding filter when light is too strong */

#define COM12                   0x3C /* Common Control 12 */

#define COM13                   0x3D /* Common Control 13 */
#define COM13_GE				0x80 /* Gamma enable */
#define COM13_UV_SL				0x40 /* UV saturation level - UV auto adjustment */
#define COM13_UVSW_YUYV_UYVY	0x00 /* UV swap (use with register TSLB[3] (0x3A)) TSLB[3], COM13[1]: YUYV & UYVY */
#define COM13_UVSW_YVYU_VYUY	0x00 /* UV swap (use with register TSLB[3] (0x3A)) TSLB[3], COM13[1]: YVYU & VYUY */

#define COM14                   0x3E /* Common Control 14 */
#define COM14_DCW_EN            0x10 /* DCW and scaling PCLK enable */
#define COM13_MS_EN             0x08 /* Manual scaling enable for pre-defined resolution modes such as CIF, QCIF, and QVGA */
#define COM13_PD_1       	    0x00 /* PCLK divided by 1 (only when COM14[4] = 1) */
#define COM13_PD_2       	    0x01 /* PCLK divided by 1 (only when COM14[4] = 2) */
#define COM13_PD_4       	    0x02 /* PCLK divided by 1 (only when COM14[4] = 4) */
#define COM13_PD_8       	    0x03 /* PCLK divided by 1 (only when COM14[4] = 8) */
#define COM13_PD_16       	    0x04 /* PCLK divided by 1 (only when COM14[4] = 16) */

#define EDGE                    0x3F /* Edge Enhancement Adjustment */

#define COM15                   0x40 /* Common Control 15 */
#define COM15_OR_10_F0			0x00 /* Date format - output full range enable: [10] to [F0] */
#define COM15_OR_01_FE			0x80 /* Date format - output full range enable: [01] to [FE] */
#define COM15_OR_00_FF			0xC0 /* Date format - output full range enable: [00] to [FF] */
#define COM15_RGB565			0X10 /* RGB 565 option (must set COM7[2] = 1 and COM7[0] = 0) */
#define COM15_RGB555			0X30 /* RGB 555 option (must set COM7[2] = 1 and COM7[0] = 0) */

#define COM16                   0x41 /* Common Control 16 */
#define COM16_EE				0x20 /* Enable edge enhancement threshold auto-adjustment for YUV output */
#define COM16_DTA				0x10 /* De-noise threshold auto-adjustment */
#define COM16_AWB				0x08 /* AWB gain enable */
#define COM16_DM				0x02 /* Color matrix coefficient double option */

#define COM17                   0x42 /* Common Control 17 */
#define COM17_ACE_N				0x00 /* AEC window must be the same value as COM4[5:4]: Normal */
#define COM17_ACE_1_2			0x40 /* AEC window must be the same value as COM4[5:4]: 1/2 */
#define COM17_ACE_1_4			0x80 /* AEC window must be the same value as COM4[5:4]: 1/4 */
#define COM17_ACE_1_8			0xC0 /* AEC window must be the same value as COM4[5:4]: 1/8 */
#define COM17_DSP_CB			0x08 /* DSP color bar enable */

#define AWBC1                   0x43 /* Reserved */
#define AWBC2                   0x44 /* Reserved */
#define AWBC3                   0x45 /* Reserved */
#define AWBC4                   0x46 /* Reserved */
#define AWBC5                   0x47 /* Reserved */
#define AWBC6                   0x48 /* Reserved */
//#define RSVD                    0x49
//#define RSVD                    0x4A
#define REG4B                   0x4B /* Register 4B */
#define DNSTH                   0x4C /* De-noise Threshold */
//#define RSVD                    0x4D
//#define RSVD                    0x4E
#define MTX1                    0x4F /* Matrix Coefficient 1 */
#define MTX2                    0x50 /* Matrix Coefficient 2 */
#define MTX3                    0x51 /* Matrix Coefficient 3 */
#define MTX4                    0x52 /* Matrix Coefficient 4 */
#define MTX5                    0x53 /* Matrix Coefficient 5 */
#define MTX6                    0x54 /* Matrix Coefficient 6 */
#define BRIGHT                  0x55 /* Brightness Control */
#define CONTRAS       			0X56 /* Contrast Control */
#define CONTRAS_CENTER       	0X57 /* Contrast Center */

#define	MTXS					0x58 /* Matrix Coefficient Sign for coefficient 5 to 0 */
#define	MTXS_AC					0x80 /* Auto contrast center enable */
#define	MTXS_MC_PLUS			0x00 /* Matrix coefficient sign: Plus */
#define	MTXS_MC_MINUS			0x3F /* Matrix coefficient sign: Minus */

//#define RSVD					0X59 /* AWB Control */
//#define RSVD                    0x60 /* AWB Control */
//#define RSVD                    0x61 /* AWB Control */
#define LCC1	                0x62 /* Lens Correction Option 1 */
#define LCC2                    0x63 /* Lens Correction Option 2 */
#define LCC3                    0x64 /* Lens Correction Option 3 */
#define LCC4                    0x65 /* Lens Correction Option 4 */
#define LCC5                    0x66 /* Lens Correction Option 5 */
#define MANU	                0x67 /* Manual U Value (effective only when register TSLB[4] is high) */
#define MANV                    0x68 /* Manual V Value (effective only when register TSLB[4] is high) */

#define GFIX	                0x69 /* Fix Gain Control */
#define GFIX_GR_1x				0x00 /* Fix gain for Gr channel */
#define GFIX_GR_1_25x			0x40 /* Fix gain for Gr channel */
#define GFIX_GR_1_5x			0x80 /* Fix gain for Gr channel */
#define GFIX_GR_1_75x			0xC0 /* Fix gain for Gr channel */
#define GFIX_GB_1x				0x00 /* Fix gain for Gb channel */
#define GFIX_GB_1_25x			0x10 /* Fix gain for Gb channel */
#define GFIX_GB_1_5x			0x20 /* Fix gain for Gb channel */
#define GFIX_GB_1_75x			0x30 /* Fix gain for Gb channel */
#define GFIX_R_1x				0x00 /* Fix gain for R channel */
#define GFIX_R_1_25x			0x04 /* Fix gain for R channel */
#define GFIX_R_1_5x				0x08 /* Fix gain for R channel */
#define GFIX_R_1_75x			0x0C /* Fix gain for R channel */
#define GFIX_B_1x				0x00 /* Fix gain for B channel */
#define GFIX_B_1_25x			0x01 /* Fix gain for B channel */
#define GFIX_B_1_5x				0x02 /* Fix gain for B channel */
#define GFIX_B_1_75x			0x03 /* Fix gain for B channel */

#define GGAIN	                0x6A /* G Channel AWB Gain */

#define DBLV	                0x6B /* DBLV */
#define DBLV_PC_BP				0x00 /* PLL control: Bypass PLL */
#define DBLV_PC_x4				0x40 /* PLL control: Input clock x4 */
#define DBLV_PC_x8				0x80 /* PLL control: Input clock x8 */
#define DBLV_PC_x16				0xC0 /* PLL control: Input clock x16 */

#define AWBCTR3	                0x6C /* AWB Control 3  */
#define AWBCTR2                 0x6D /* AWB Control 2  */
#define AWBCTR1                 0x6E /* AWB Control 1  */
#define AWBCTR0                 0x6F /* AWB Control 0  */

#define SCALING_XSC             0x70 /* SCALING_XSC */
#define SCALING_XSC_TP_S1		0x00 /* Shifting "1" (Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7])) */
#define SCALING_XSC_TP_8B		0x80 /* 8-bar color bar (Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7]) */
#define SCALING_XSC_TP_FG		0x80 /* Fade to gray color bar (Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7]) */
#define SCALING_XSC_HS			0x7F /* Horizontal scale factor */

#define SCALING_YSC             0x71 /* SCALING_YSC */
#define SCALING_YSC_TP_S1		0x80 /* Shifting "1" (Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7])) */
#define SCALING_YSC_TP_8B		0x00 /* 8-bar color bar (Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7]) */
#define SCALING_YSC_TP_FG		0x80 /* Fade to gray color bar (Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7]) */
#define SCALING_YSC_HS			0x7F /* Horizontal scale factor */

#define SCALING_DCWCTR          0x72 /* DCW Control */

#define SCALING_PCLK_DIV        0x73 /* Clock divider control for DSP scale control */
#define SCALING_PCLK_DIV_1		0x00 /* Divided by 1 */
#define SCALING_PCLK_DIV_2		0x01 /* Divided by 2 */
#define SCALING_PCLK_DIV_4		0x02 /* Divided by 4 */
#define SCALING_PCLK_DIV_8		0x03 /* Divided by 8 */
#define SCALING_PCLK_DIV_16		0x04 /* Divided by 16 */

#define REG74                   0x74 /* REG74 */
#define REG74_DG				0x10 /* DG_Manu */
#define REG74_DGMC_BP			0x00 /* Digital gain manual control: Bypass */
#define REG74_DGMC_1x			0x00 /* Digital gain manual control: 1x */
#define REG74_DGMC_2x			0x00 /* Digital gain manual control: 2x */
#define REG74_DGMC_4x			0x00 /* Digital gain manual control: 4x */

#define REG75                   0x75 /* Register 75 */

#define REG76                   0x76 /* Register 76 */
#define REG76_WE				0x40 /* White pixel correction enable */
#define REG76_BE				0x20 /* Black pixel correction enable */
#define REG76_HL				0x1F /* Edge enhancement higher limit */

#define REG77                   0x77 /* Register 77 */
//#define RSVD                    0x78
//#define RSVD                    0x79
#define SLOP					0x7A /* Gamma Curve Highest Segment Slope - calculated as follows: SLOP[7:0] = (0x100 - GAM15[7:0]*4/3) */
#define GAM1                    0x7B /* Gamma Curve 1st Segment Input End Point 0x04 Output Value */
#define GAM2                    0x7C /* Gamma Curve 2nd Segment Input End Point 0x08 Output Value */
#define GAM3                    0x7D /* Gamma Curve 3rd Segment Input End Point 0x10 Output Value */
#define GAM4                    0x7E /* Gamma Curve 4th Segment Input End Point 0x20 Output Value */
#define GAM5                    0x7F /* Gamma Curve 5th Segment Input End Point 0x28 Output Value */
#define GAM6                    0x80 /* Gamma Curve 6th Segment Input End Point 0x30 Output Value */
#define GAM7                    0x81 /* Gamma Curve 7th Segment Input End Point 0x38 Output Value */
#define GAM8                    0x82 /* Gamma Curve 8th Segment Input End Point 0x40 Output Value */
#define GAM9                    0x83 /* Gamma Curve 9th Segment Input End Point 0x48 Output Value */
#define GAM10                   0x84 /* Gamma Curve 10th Segment Input End Point 0x50 Output Value */
#define GAM11                   0x85 /* Gamma Curve 11th Segment Input End Point 0x60 Output Value */
#define GAM12                   0x86 /* Gamma Curve 12th Segment Input End Point 0x70 Output Value */
#define GAM13                   0x87 /* Gamma Curve 13th Segment Input End Point 0x90 Output Value */
#define GAM14                   0x88 /* Gamma Curve 14th Segment Input End Point 0xB0 Output Value */
#define GAM15                   0x89 /* Gamma Curve 15th Segment Input End Point 0xE5 Output Value */
//#define RSVD                    0x8A
//#define RSVD                    0x8B

#define RGB444                  0x8C /* RGB444 */
#define RGB444_EN				0x02 /* RGB444 enable, effective only when COM15[4] is high */
#define RGB444_WF_xRGB			0x00 /* RGB444 word format: xRGB */
#define RGB444_WF_RGBx			0x01 /* RGB444 word format: RGBx */

//#define RSVD                    0x8D
//#define RSVD                    0x8E
//#define RSVD                    0x8F
//#define RSVD                    0x90
//#define RSVD                    0x91
#define DM_LNL                  0x92 /* Dummy Line low 8 bits */
#define DM_LNH                  0x93 /* Dummy Line high 8 bits */
#define LCC6                    0x94 /* Lens Correction Option 6 (effective only when LCC5[2] IS HIGH) */
#define LCC7                    0x95 /* Lens Correction Option 7 (effective only when LCC5[2] IS HIGH) */
//#define RSVD                    0x96
//#define RSVD                    0x97
//#define RSVD                    0x98
//#define RSVD                    0x99
//#define RSVD                    0x9A
//#define RSVD                    0x9B
//#define RSVD                    0x9C
#define BD50ST                  0X9D /* 50Hz Banding Filter Value (effective only when COM8[5] is high and COM11[3] is high) */
#define BD60ST                  0x9E /* 60Hz Banding Filter Value (effective only when COM8[5] is high and COM11[3] is low) */
#define HAECC1                  0x9F /* Histogram-based AEC/AGC Control 1 */
#define HAECC2                  0xA0 /* Histogram-based AEC/AGC Control 2 */
//#define RSVD                   0xA1
#define SCALING_PCLK_DELAY      0xA2 /* Pixel Clock Delay */
//#define RSVD                	 0xA3

#define NT_CTRL                 0xA4 /* NT_CTRL */
#define NT_CTRL_AC_2x			0x00 /* Auto frame rate adjustment control: Double exposure time */
#define NT_CTRL_AC				0x08 /* Auto frame rate adjustment control: Reduce frame rate by half */
#define NT_CTRL_ASP_2x			0x00 /* Auto frame rate adjustment switch point: Insert dummy row at 2x gain */
#define NT_CTRL_ASP_4x			0x01 /* Auto frame rate adjustment switch point: Insert dummy row at 4x gain */
#define NT_CTRL_ASP_8x			0x02 /* Auto frame rate adjustment switch point: Insert dummy row at 8x gain */

#define BD50MAX                 0xA5 /* 50Hz Banding Step Limit */
#define HAECC3                  0xA6 /* Histogram-based AEC/AGC Control 3 */
#define HAECC4                  0xA7 /* Histogram-based AEC/AGC Control 4 */
#define HAECC5                  0xA8 /* Histogram-based AEC/AGC Control 5 */
#define HAECC6                  0xA9 /* Histogram-based AEC/AGC Control 6 */
#define HAECC7                  0xAA /* Histogram-based AEC/AGC Control 7 */

#define BD60MAX                 0xAB /* 60Hz Banding Step Limit */

#define STR_OPT                 0xAC /* Register AC */
#define STR_OPT_SE				0x80 /* Strobe enable */
#define STR_OPT_RGB_G			0x40 /* R/G/B gain control by STR_R (0XAD) / STR_G (0xAE) / STR_B (0xAF) for LED output frame */
#define STR_OPT_XM_1R			0x00 /* Xenon mode option: 1 row */
#define STR_OPT_XM_2R			0x10 /* Xenon mode option: 2 row */
#define STR_OPT_XM_3R			0x20 /* Xenon mode option: 3 row */
#define STR_OPT_XM_4R			0x30 /* Xenon mode option: 4 row */
#define STR_OPT_MS_X			0x00 /* Mode select: Xenon */
#define STR_OPT_MS_L1			0x01 /* Mode select: LED 1 */
#define STR_OPT_MS_L2			0x02 /* Mode select: LED 2 */

#define STR_R					0xAD /* R Gain for LED Output Frame */
#define STR_G					0xAE /* G Gain for LED Output Frame */
#define STR_B					0xAF /* B Gain for LED Output Frame */
#define UNDOC_COLOR_CORRECTION	0xB0
#define ABLC1					0xB1 /* ABLC enable */
//#define RSVD					 0xB2
#define THL_ST					0xB3 /* ABLC Target */
//#define RSVD					 0xB4
#define THL_DLT					0xB5 /*ABLC Stable Range */
//#define RSVD					 0xB6
//#define RSVD					 0xB7
//#define RSVD					 0xB8
//#define RSVD					 0xB9
//#define RSVD					 0xBA
//#define RSVD					 0xBB
//#define RSVD					 0xBC
//#define RSVD					 0xBD

#define AD_CHB					0xBE /* Blue Channel Black Level Compensation */
#define AD_CHB_SB				0x40 /* Sign bit */
#define AD_CHB_BC				0x3F /* Blue Channel Black Level Compensation */

#define AD_CHR					0xBF /* Red Channel Black Level Compensation */
#define AD_CHR_SB				0x40 /* Sign bit */
#define AD_CHR_RC				0x3F /* Red Channel Black Level Compensation */ 

#define AD_CHGb					0xC0 /* Gb Channel Black Level Compensation */
#define AD_CHGb_SB				0x40 /* Sign bit */
#define AD_CHGb_GbC				0x3F /* Gb Channel Black Level Compensation */ 

#define AD_CHGr					0xBF /* Gr Channel Black Level Compensation */
#define AD_CHGr_SB				0x40 /* Sign bit */
#define AD_CHGr_GrC				0x3F /* Gr Channel Black Level Compensation */ 

//#define RSVD					 0xC2
//#define RSVD					 0xC3
//#define RSVD					 0xC4
//#define RSVD					 0xC5
//#define RSVD					 0xC6
//#define RSVD					 0xC7
//#define RSVD					 0xC8

#define SATCTR					0xC9 /* Saturation Control */
#define SATCTR_CM				0xF0 /* UV saturation control min */
#define SATCTR_CR				0x0F /* UV saturation control result */

#define OV7670_ADDR		0x42

typedef struct {
	uint16_t mid;
	uint8_t pid;
	uint8_t ver;
} ovcam_info;

void ov7670_detect(ovcam_info *info);
void ov7670_init();

#endif //__REG_REGS_H__
