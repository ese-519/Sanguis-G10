#ifndef ILI9341_H
#define ILI9341_H

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE		0x0A
#define ILI9341_RDMADCTL	0x0B
#define ILI9341_RDPIXFMT	0x0C
#define ILI9341_RDIMGFMT	0x0D
#define ILI9341_RDSELFDIAG	0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR    0x30
#define ILI9341_MADCTL   0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT   0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6
#define ILI9341_ETMOD	0xB7

#define ILI9341_BKLCTR1 0xB8
#define ILI9341_BKLCTR2 0xB9
#define ILI9341_BKLCTR3 0xBA
#define ILI9341_BKLCTR4	0xBB
#define ILI9341_BKLCTR5 0xBC
//#define ILI9341_BKLCTR6 0xBD
#define ILI9341_BKLCTR7 0xBE
#define ILI9341_BKLCTR8 0xBF

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_PWCTRA  0xCB
#define ILI9341_PWCTRB  0xCF

#define ILI9341_NVMWR	0xD0
#define ILI9341_NVMPK	0xD1
#define ILI9341_NVMSRD	0xD2
#define ILI9341_RDID4	0xD3

#define ILI9341_GMCTRP1 	0xE0
#define ILI9341_GMCTRN1 	0xE1
#define ILI9341_GMCTRD1 	0xE2
#define ILI9341_GMCTRD2 	0xE3

#define ILI9341_DTCTRA		0xE8
#define ILI9341_DTCTRAE		0xE9
#define ILI9341_DTCTRB		0xEA
#define ILI9341_PWRONSEQ	0xED

#define ILI9341_EN3G	0xF2
#define ILI9341_IFCTR	0xF6
#define ILI9341_PRCTR	0xF7

#ifdef __cplusplus
extern "C" {
#endif

void ili9341_init(spi_host_device_t host, int cs, int dc, int rst);
void ili9341_update(int x, int y, int width, int height, uint8_t *buffer);
void ili9341_update_await();

#ifdef __cplusplus
}
#endif

#endif
