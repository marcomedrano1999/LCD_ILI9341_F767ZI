/*
 * ILI9341.h
 *
 *  Created on: Apr 10, 2023
 *      Author: Medrano
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_


#include "stm32f767xx.h"


/*
 * GPIO pin and port definitions
 *
 */
#define LCD_RESX_PIN			7
#define LCD_CSX_PIN				8
#define LCD_DCX_PIN				9
#define LCD_SCL_PIN				10
#define LCD_SDI_PIN				11	// MISO
#define LCD_SDO_PIN				12	// MOSI

#define LCD_RESX_PORT			GPIOC
#define LCD_CSX_PORT			GPIOC
#define LCD_DCX_PORT			GPIOC
#define LCD_SCL_PORT			GPIOC
#define LCD_SDI_PORT			GPIOC
#define LCD_SDO_PORT			GPIOC






/*
 *  Commands definitions
 */
#define LCD_NO_OPERATION					0x00
#define LCD_SW_RESET						0x01
#define LCD_READ_DISPLAY_ID_INFO			0x04
#define LCD_READ_DISPLAY_STATUS				0x09
#define LCD_READ_DISPLAY_PWR_MODE			0x0A
#define LCD_READ_DISPLAY_MADCTL				0x0B
#define LCD_READ_DISPLAY_PIXEL_FORMAT		0x0C
#define LCD_READ_DISPLAY_IMAGE_FORMAT		0x0D
#define LCD_READ_DISPLAY_SIGNAL_FORMAT		0x0E
#define LCD_READ_DISPLAY_SELF_DIAG_RESULT	0x0F
#define LCD_ENTER_SLEEP_MODE				0x10
#define LCD_SLEEP_OUT						0x11
#define LCD_PARTIAL_MODE_ON					0x12
#define LCD_NORMAL_DISPLAY_MODE_ON			0x13
#define LCD_DISPLAY_INV_OFF					0x20
#define LCD_DISPLAY_INV_ON					0x21
#define LCD_GAMMA_SET						0x26
#define LCD_DISPLAY_OFF						0x28
#define LCD_DISPLAY_ON						0x29
#define LCD_COLUMN_ADDR_SET					0x2A
#define LCD_PAGE_ADDR_SET					0x2B
#define LCD_MEM_WRITE						0x2C
#define LCD_COLOR_SET						0x2D
#define LCD_MEM_READ						0x2E
#define LCD_PARTIAL_AREA					0x30
#define LCD_VERTICAL_SCROLLING_DEF			0x33
#define LCD_TEARING_EFFECT_LINE_OFF			0x34
#define LCD_TEARING_EFFECT_LINE_ON			0x35
#define LCD_MEM_ACCESS_CTRL					0x36
#define LCD_VERTIAL_SCROLLING_START_ADDR	0x37
#define LCD_IDLE_MODE_OFF					0x38
#define LCD_IDLE_MODE_ON					0x39
#define LCD_PIXEL_FORMAT_SET				0x3A
#define LCD_WRITE_MEM_CONTINUE				0x3C
#define LCD_READ_MEM_CONTINUE				0x3E
#define LCD_SET_TEAR_SCANLINE				0x44
#define LCD_GET_SCANLINE					0x45
#define LCD_WRITE_DISPLAY_BRIGHTNESS		0x51
#define LCD_READ_DISPLAY_BRIGHTNESS			0x52
#define LCD_WRITE_CTRL_DISPLAY				0x53
#define LCD_READ_CTRL_DISPLAY				0x54
#define LCD_WRITE_CONT_ADAPT_BRIGHT_CTRL	0x55
#define LCD_READ_CONT_ADAPT_BRIGHT_CTRL		0x56
#define LCD_WRITE_CABC_MIN_BRIGHTNESS		0x5E
#define LCD_READ_CABC_MIN_BRIGHTNESS		0x5F
#define LCD_READ_ID1						0xDA
#define LCD_READ_ID2						0xDB
#define LCD_READ_ID3						0xDC


// Extended command set
#define LCD_RGB_INTERFACE_SIG_CTRL			0xB0
#define LCD_FRAME_CTRL_NORMAL_MODE			0xB1
#define LCD_FRAME_CTRL_IDLE_MODE			0xB2
#define LCD_FRAME_CTRL_PARTIAL_MODE			0xB3
#define LCD_DISPLAY_INV_CTRL				0xB4
#define LCD_BLANKING_PORCH_CTRL				0xB5
#define LCD_DISPLAY_FUNC_CTRL				0xB6
#define LCD_ENTRY_MODE_SET					0xB7
#define LCD_BACKLIGHT_CTRL1					0xB8
#define LCD_BACKLIGHT_CTRL2					0xB9
#define LCD_BACKLIGHT_CTRL3					0xBA
#define LCD_BACKLIGHT_CTRL4					0xBB
#define LCD_BACKLIGHT_CTRL5					0xBC
#define LCD_BACKLIGHT_CTRL7					0xBE
#define LCD_BACKLIGHT_CTRL8					0xBF
#define LCD_PWR_CTRL1						0xC0
#define LCD_PWR_CTRL2						0xC1
#define LCD_VCOM_CTRL1						0xC5
#define LCD_VCOM_CTRL2						0xC7
#define LCD_NV_MEM_WRITE					0xD0
#define LCD_NV_MEM_PROTECT_KEY				0xD1
#define LCD_NV_MEM_STATUS_READ				0xD2
#define LCD_READ_ID4						0xD3
#define LCD_POSITIVE_GAMMA_CORRECTION		0xE0
#define LCD_NEGATIVE_GAMMA_CORRECTION		0xE1
#define LCD_DIGITAL_GAMMA_CTRL1				0xE2
#define LCD_DIGITAL_GAMMA_CTRL2				0xE3
#define LCD_INTERFACE_CTRL					0xF6

// More commands
#define LCD_PWR_CTRL_A						0xCB
#define LCD_PWR_CTRL_B						0xCF
#define LCD_DRIVER_TIMING_CTRL_A			0xE8
#define LCD_DRIVER_TIMING_CTRL_A2			0xE9
#define LCD_DRIVER_TIMING_CTRL_B			0xEA
#define LCD_PWR_ON_SEQUENCE_CTRL			0xED
#define LCD_ENABLE_3G						0xF2
#define LCD_PUMP_RATIO_CTRL					0xF7




/*
 *  Display macros
 */
#define MADCTL_MY				0x80		// Bottom to top
#define MADCTL_MX				0x40		// Right to left
#define MADCTL_MV				0x20		// Reverse mode
#define MADCTL_ML				0x10		// LCD refresh bottom to top
#define MADCTL_RGB				0x00		// Red-Green-Blue pixel order
#define MADCTL_BGR				0x08		// Blue-Green-Red pixel order
#define MADCTL_MH				0x04		// LCD refresh right to left


/*
 * Auxiliary functions
 */
#define LCD_RESX_LOW() 		(LCD_RESX_PORT->ODR &= ~(1 << LCD_RESX_PIN))
#define LCD_RESX_HIGH() 	(LCD_RESX_PORT->ODR |= (1 << LCD_RESX_PIN))
#define LCD_CSX_LOW() 		(LCD_CSX_PORT->ODR &= ~(1 << LCD_CSX_PIN))
#define LCD_CSX_HIGH() 		(LCD_CSX_PORT->ODR |= (1 << LCD_CSX_PIN))
#define LCD_DCX_LOW() 		(LCD_DCX_PORT->ODR &= ~(1 << LCD_DCX_PIN))
#define LCD_DCX_HIGH() 		(LCD_DCX_PORT->ODR |= (1 << LCD_DCX_PIN))


/*
 * Function declarations
 */
void LCD_GPIO_Init(void);
void LCD_SPI_Init(void);
void LCD_SPI_Enable(void);
void LCD_Reset();
void LCD_Config(void);

void LCD_Write_Cmd(uint8_t cmd);
void LCD_Write_Data(uint8_t *data, uint32_t len);
void LCD_ILI9341_Init(void);









#endif /* INC_ILI9341_H_ */
