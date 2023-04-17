/*
 * ILI9341.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Medrano
 */


#include "ILI9341.h"
#include "stm32f7xx.h"

bsp_lcd_t lcd_handle;
bsp_lcd_t *hlcd = &lcd_handle;

uint8_t bsp_db[DB_SIZE];
uint8_t bsp_wb[DB_SIZE];


void LCD_ILI9341_Init(void)
{
	//Initialize the pins
	LCD_GPIO_Init();

	// Initialize SPI peripheral
	LCD_SPI_Init();

	// Enable SPI peripheral
	LCD_SPI_Enable();

	// Reset the LCD
	LCD_Reset();

	// Set display format data
	hlcd->orientation = BSP_LCD_ORIENTATION;
	hlcd->pixel_format = BSP_LCD_PIXEL_FMT;
	hlcd->area.x1 = 0;
	hlcd->area.x2 = BSP_LCD_ACTIVE_WIDTH-1;
	hlcd->area.y1 = 0;
	hlcd->area.y2 = BSP_LCD_ACTIVE_HEIGHT-1;

	// Configure the LCD
	LCD_Config();

	// Configure the display
	lcd_set_display_area(&hlcd->area);
	lcd_set_orientation(hlcd->orientation);
	lcd_buffer_init(hlcd);
}



void LCD_GPIO_Init(void)
{
	RCC_TypeDef *pRCC = RCC;
	GPIO_TypeDef *pGPIOC = GPIOC;

	// Initialize peripheral clock
	pRCC->AHB1ENR |= (1 << 2);

	// Set pins mode
	pGPIOC->MODER |= (1 << (LCD_RESX_PIN*2)); // RESX in general purpose mode
	pGPIOC->MODER |= (1 << (LCD_CSX_PIN*2)); // CSX in general purpose mode
	pGPIOC->MODER |= (1 << (LCD_DCX_PIN*2)); // DCX in general purpose mode
	pGPIOC->MODER |= (2 << (LCD_SCL_PIN*2)); // SCL in alternate function mode
	pGPIOC->MODER |= (2 << (LCD_SDI_PIN*2)); // SDI in alternate function mode
	pGPIOC->MODER |= (2 << (LCD_SDO_PIN*2)); // SDO in alternate function mode

	// Set pins output type
	pGPIOC->OTYPER &= ~(1 << LCD_RESX_PIN); // RESX in output push-pull
	pGPIOC->OTYPER &= ~(1 << LCD_CSX_PIN); // CSX in output push-pull
	pGPIOC->OTYPER &= ~(1 << LCD_DCX_PIN); // DCX in output push-pull
	pGPIOC->OTYPER &= ~(1 << LCD_SCL_PIN); // SCL in output push-pull
	pGPIOC->OTYPER &= ~(1 << LCD_SDI_PIN); // SDI in output push-pull
	pGPIOC->OTYPER &= ~(1 << LCD_SDO_PIN); // SDO in output push-pull


	// set pins output speed register
	pGPIOC->OSPEEDR |= (2 << (LCD_RESX_PIN*2)); // RESX in high speed
	pGPIOC->OSPEEDR |= (2 << (LCD_CSX_PIN*2));  // CSX in high speed
	pGPIOC->OSPEEDR |= (2 << (LCD_DCX_PIN*2));  // DCX in high speed
	pGPIOC->OSPEEDR |= (2 << (LCD_SCL_PIN*2));  // SCL in high speed
	pGPIOC->OSPEEDR |= (2 << (LCD_SDI_PIN*2));  // SDI in high speed
	pGPIOC->OSPEEDR |= (2 << (LCD_SDO_PIN*2));  // SDO in high speed


	// set pins alternate function
	pGPIOC->AFR[1] |= (6 << ((LCD_SCL_PIN*4) - 32)); // SCL in AF6
	pGPIOC->AFR[1] |= (6 << ((LCD_SDI_PIN*4) - 32)); // SDI in AF6
	pGPIOC->AFR[1] |= (6 << ((LCD_SDO_PIN*4) - 32)); // SDO in AF6


	// set output value for CSX, DCX and RESX
	pGPIOC->ODR |= (1 << LCD_RESX_PIN);		// CSX output to 1
	pGPIOC->ODR |= (1 << LCD_CSX_PIN);		// CSX output to 1
	pGPIOC->ODR |= (1 << LCD_DCX_PIN);		// CSX output to 1


}


void LCD_SPI_Init(void)
{
	RCC_TypeDef *pRCC = RCC;
	SPI_TypeDef *pSPI = SPI3;

	// Initialize peripheral clock
	pRCC->APB1ENR |= (1 << 15);

	// Set SPI mode to full duplex
	pSPI->CR1 &= ~(1 << 15);

	// Set data frame format
	pSPI->CR1 &= ~(1 << 11);

	// Set software slave management
	pSPI->CR1 |= (1 << 9);

	// Set internal slave select
	pSPI->CR1 |= (1 << 8);

	// Set the baudrate ctrl -> 48 / 16 = 3 MHZ
	pSPI->CR1 |= (4 << 3);

	// Set configuration to Master
	pSPI->CR1 |= (1 << 2);

	// Set clock polarity to 0
	pSPI->CR1 &= ~(1 << 1);

	// set clock phase to 0
	pSPI->CR1 &= ~(1 << 0);

	// Set frame format
	pSPI->CR2 &= ~(1 << 4);
}


void LCD_SPI_Enable(void)
{
	SPI_TypeDef *pSPI = SPI3;

	// Enable the peripheral
	pSPI->CR1 |= (1 << 6);
}


void LCD_Reset()
{
	LCD_RESX_LOW();
	HAL_Delay(50);
	LCD_RESX_HIGH();
	HAL_Delay(50);
}


void LCD_Write_Cmd(uint8_t cmd)
{
	SPI_TypeDef *pSPI = SPI3;

	// Set CSX and DCX to low for command transmission
	LCD_CSX_LOW();
	LCD_DCX_LOW();

	// Wait till the transfer buffer is empty
	while(!(pSPI->SR & (1 << 1)));

	// Load the command into the peripheral data register
	pSPI->DR = cmd;

	// Make sure the command is sent
	while(!(pSPI->SR & (1 << 1)));
	while((pSPI->SR & (1 << 7)));

	LCD_CSX_HIGH();
	LCD_DCX_HIGH();

}

void LCD_Read_data(uint8_t *data)
{
	uint8_t i = 0;
	while((SPI3->SR & (1 << 0)))
	{
		if(i<255)
		{
			data[i] = SPI3->DR;
			i++;
		}
	}
}


void LCD_Write_Data(uint8_t *data, uint32_t len)
{
	SPI_TypeDef *pSPI = SPI3;

	// Set CSX to low for data transmission
	LCD_CSX_LOW();

	for(uint32_t i=0;i<len;i++)
	{
		// Wait till the transfer buffer is empty
		while(!(pSPI->SR & (1 << 1)));

		// Load the command into the peripheral data register
		pSPI->DR = data[i];
	}
	// Make sure the command is sent
	while(!(pSPI->SR & (1 << 1)));
	while((pSPI->SR & (1 << 7)));

	// Reset pins
	LCD_CSX_HIGH();
}


void LCD_Config(void)
{
	uint8_t params[15];
	uint8_t data[255];
	LCD_Write_Cmd(LCD_SW_RESET);

	LCD_Write_Cmd(LCD_PWR_CTRL_B);
	LCD_Read_data(data);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	LCD_Write_Data(params, 3);

	LCD_Write_Cmd(LCD_PWR_ON_SEQUENCE_CTRL);
	params[0] = 0x64;
	params[1] = 0x03;
	params[2] = 0x12;
	params[3] = 0x81;
	LCD_Write_Data(params, 4);

	LCD_Write_Cmd(LCD_DRIVER_TIMING_CTRL_A);
	params[0] = 0x85;
	params[1] = 0x10;
	params[2] = 0x7A;
	LCD_Write_Data(params, 3);

	LCD_Write_Cmd(LCD_PWR_CTRL_A);
	params[0] = 0x39;
	params[1] = 0x2C;
	params[2] = 0x00;
	params[3] = 0x34;
	params[4] = 0x02;
	LCD_Write_Data(params, 5);

	LCD_Write_Cmd(LCD_PUMP_RATIO_CTRL);
	params[0] = 0x20;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_DRIVER_TIMING_CTRL_B);
	params[0] = 0x00;
	params[1] = 0x00;
	LCD_Write_Data(params, 2);

	LCD_Write_Cmd(LCD_PWR_CTRL1);
	params[0] = 0x1B;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_PWR_CTRL2);
	params[0] = 0x12;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_VCOM_CTRL1);
	params[0] = 0x08;
	params[1] = 0x26;
	LCD_Write_Data(params, 2);

	LCD_Write_Cmd(LCD_VCOM_CTRL2);
	params[0] = 0xB7;
	LCD_Write_Data(params, 1);

	uint8_t m;
	m = MADCTL_MV | MADCTL_MY | MADCTL_BGR;

	LCD_Write_Cmd(LCD_MEM_ACCESS_CTRL);
	params[0] = m;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_PIXEL_FORMAT_SET);
	params[0] = 0x55;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_FRAME_CTRL_NORMAL_MODE);
	params[0] = 0x00;
	params[1] = 0x1B;
	LCD_Write_Data(params, 2);

	LCD_Write_Cmd(LCD_DISPLAY_FUNC_CTRL);
	params[0] = 0x0A;
	params[1] = 0xA2;
	LCD_Write_Data(params, 2);

	LCD_Write_Cmd(LCD_ENABLE_3G);
	params[0] = 0x02;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_GAMMA_SET);
	params[0] = 0x01;
	LCD_Write_Data(params, 1);

	LCD_Write_Cmd(LCD_POSITIVE_GAMMA_CORRECTION);
	params[0] = 0x0F;
	params[1] = 0x1D;
	params[2] = 0x1A;
	params[3] = 0x0A;
	params[4] = 0x0D;
	params[5] = 0x07;
	params[6] = 0x49;
	params[7] = 0x66;
	params[8] = 0x3B;
	params[9] = 0x07;
	params[10] = 0x11;
	params[11] = 0x01;
	params[12] = 0x09;
	params[13] = 0x05;
	params[14] = 0x04;
	LCD_Write_Data(params, 15);

	LCD_Write_Cmd(LCD_NEGATIVE_GAMMA_CORRECTION);
	params[0] = 0x00;
	params[1] = 0x18;
	params[2] = 0x1D;
	params[3] = 0x02;
	params[4] = 0x0F;
	params[5] = 0x04;
	params[6] = 0x36;
	params[7] = 0x13;
	params[8] = 0x4C;
	params[9] = 0x07;
	params[10] = 0x13;
	params[11] = 0x0F;
	params[12] = 0x2E;
	params[13] = 0x2F;
	params[14] = 0x05;
	LCD_Write_Data(params, 15);

	LCD_Write_Cmd(LCD_PAGE_ADDR_SET);
	params[0] = 0x00;
	params[1] = 0x00;
	params[2] = 0x00;
	params[3] = 0xF0;
	LCD_Write_Data(params, 4);

	LCD_Write_Cmd(LCD_COLUMN_ADDR_SET);
	params[0] = 0x00;
	params[1] = 0x00;
	params[2] = 0x01;
	params[3] = 0x40;
	LCD_Write_Data(params, 4);

	LCD_Write_Cmd(LCD_SLEEP_OUT);
	HAL_Delay(100);
	LCD_Write_Cmd(LCD_DISPLAY_ON);
}

void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
	lcd_area_t area;
	area.x1 = x1;
	area.x2 = x2;
	area.y1 = y1;
	area.y2 = y2;
	lcd_set_display_area(&area);
}


void lcd_set_display_area(lcd_area_t *area)
{
	uint8_t params[4];
	// Column address set(2Ah)
	params[0] = HIGH_16(area->x1);
	params[1] = LOW_16(area->x1);
	params[2] = HIGH_16(area->x2);
	params[3] = LOW_16(area->x2);
	LCD_Write_Cmd(LCD_COLUMN_ADDR_SET);
	LCD_Write_Data(params,4);

	params[0] = HIGH_16(area->y1);
	params[1] = LOW_16(area->y1);
	params[2] = HIGH_16(area->y2);
	params[3] = LOW_16(area->y2);
	LCD_Write_Cmd(LCD_PAGE_ADDR_SET);
	LCD_Write_Data(params,4);
}
void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes)
{
	SPI_TypeDef *pSPI = SPI3;
	uint16_t *buff_ptr;

	__disable_spi();
	__spi_set_dff_16bit();
	__enable_spi();

	LCD_CSX_LOW();

	buff_ptr = (uint16_t*)buffer;
	while(nbytes)
	{
	 while(!(pSPI->SR & (1 << 1)));
	 pSPI->DR = *buff_ptr;
	 ++buff_ptr;
	 nbytes -= 2;
	}

	__disable_spi();
	LCD_CSX_HIGH();
	__spi_set_dff_8bit();
	__enable_spi();
}


void lcd_set_orientation(uint8_t orientation)
{
	uint8_t param;

	if(orientation == LANDSCAPE)
		param = MADCTL_MV | MADCTL_MY | MADCTL_BGR; // Memory access control <Landscape setting>
	else if(orientation == PORTRAIT)
		param = MADCTL_MY | MADCTL_MX | MADCTL_BGR; // Memory access control <portrait setting>

	LCD_Write_Cmd(LCD_MEM_ACCESS_CTRL);
	LCD_Write_Data(&param, 1);
}


void lcd_buffer_init(bsp_lcd_t *lcd)
{
	lcd->draw_buffer1 = bsp_db;
	lcd->draw_buffer2 = bsp_wb;
	lcd->buff_to_draw = NULL;
	lcd->buff_to_flush = NULL;
}


void bsp_lcd_set_backgrounf_color(uint32_t rgb888)
{
	bsp_lcd_fill_rect(rgb888,0,(BSP_LCD_ACTIVE_WIDTH),0,(BSP_LCD_ACTIVE_HEIGHT));
}


uint16_t bsp_lcd_convert_rgb888_to_rgb565(uint32_t rgb888)
{
	uint16_t r = (rgb888 >> 19) & 0x1FU;
	uint16_t g = (rgb888 >> 10) & 0x3FU;
	uint16_t b = (rgb888 >> 3)  & 0x1FU;

	return (uint16_t)((r<<11) | (g << 5) | b);
}



void bsp_lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
{
	uint32_t total_bytes_to_write = 0;
	uint32_t bytes_sent_so_far = 0;
	uint32_t remaining_bytes = 0;
	uint32_t npix;
	uint32_t pixels_sent = 0;
	uint32_t x1,y1;
	uint32_t pixel_per_line = x_width;

	if((x_start+x_width) > BSP_LCD_ACTIVE_WIDTH) return;
	if((y_start+y_height) > BSP_LCD_ACTIVE_HEIGHT) return;


	//1. Calculate total number of bytes written into DB
	total_bytes_to_write = get_total_bytes(hlcd, x_width, y_height);
	remaining_bytes = total_bytes_to_write;

	while(remaining_bytes)
	{
		x1 = x_start + (pixels_sent % pixel_per_line);
		y1 = y_start + (pixels_sent / pixel_per_line);

		make_area(&hlcd->area,x1,x_width,y1,y_height);

		if(x1 != x_start)
			npix = x_start + x_width - x1;
		else
			npix = bytes_to_pixels(remaining_bytes, hlcd->pixel_format);

		bytes_sent_so_far += copy_to_draw_buffer(hlcd,pixels_to_bytes(npix,hlcd->pixel_format),rgb888);
		pixels_sent = bytes_to_pixels(bytes_sent_so_far, hlcd->pixel_format);
		remaining_bytes = total_bytes_to_write - bytes_sent_so_far;
	}
}


uint32_t get_total_bytes(bsp_lcd_t *hlcd, uint32_t width, uint32_t height)
{
	uint8_t bytes_per_pixel = 2;
	if(hlcd->pixel_format == BSP_LCD_PIXEL_FMT_RGB565)
		bytes_per_pixel=2;
	return (width*height*bytes_per_pixel);
}

void make_area(lcd_area_t *area, uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
{
	uint16_t lcd_total_width = BSP_LCD_ACTIVE_WIDTH-1;
	uint16_t lcd_total_height = BSP_LCD_ACTIVE_HEIGHT-1;

	area->x1 = x_start;
	area->x2 = x_start + x_width - 1;
	area->y1 = y_start;
	area->y2 = y_start + y_height - 1;

	area->x2 = (area->x2 > lcd_total_width) ? lcd_total_width : area->x2;
	area->y2 = (area->y2 > lcd_total_height) ? lcd_total_height : area->y2;
}

uint32_t bytes_to_pixels(uint32_t nbytes, uint8_t pixel_format)
{
	(void)pixel_format;
	return nbytes/2;
}

uint32_t copy_to_draw_buffer(bsp_lcd_t *hlcd, uint32_t nbytes, uint32_t rgb888)
{
	uint16_t *fb_ptr = NULL;
	uint32_t npixels;
	hlcd->buff_to_draw = get_buff(hlcd);
	fb_ptr = (uint16_t*)hlcd->buff_to_draw;
	nbytes = ((nbytes > DB_SIZE)? DB_SIZE : nbytes);
	npixels = bytes_to_pixels(nbytes, hlcd->pixel_format);

	if(hlcd->buff_to_draw != NULL)
	{
		for(uint32_t i=0;i < npixels; i++)
		{
			*fb_ptr = bsp_lcd_convert_rgb888_to_rgb565(rgb888);
			fb_ptr++;
		}

		hlcd->write_lenght = pixels_to_bytes(npixels,hlcd->pixel_format);
		while(!is_lcd_write_allowed(hlcd));
		hlcd->buff_to_flush = hlcd->buff_to_draw;
		hlcd->buff_to_draw = NULL;
		hlcd_flush(hlcd);

		return pixels_to_bytes(npixels,hlcd->pixel_format);
	}

	return 0;
}

uint8_t *get_buff(bsp_lcd_t *hlcd)
{
	uint32_t buf1 = (uint32_t)hlcd->draw_buffer1;
	uint32_t buf2 = (uint32_t)hlcd->draw_buffer2;

	//__disable_irq();

	if(hlcd->buff_to_draw == NULL && hlcd->buff_to_flush == NULL)
		return hlcd->draw_buffer1;
	else if((uint32_t)hlcd->buff_to_flush == buf1 && hlcd->buff_to_draw == NULL)
		return hlcd->draw_buffer2;
	else if((uint32_t)hlcd->buff_to_flush==buf2 && hlcd->buff_to_draw==NULL)
		return hlcd->draw_buffer1;

	//__enable_irq()

	return NULL;
}


uint8_t is_lcd_write_allowed(bsp_lcd_t *hlcd)
{
	//__disable_irq();
	if(!hlcd->buff_to_flush)
		return 1;

	//__enable_irq();

	return 0;
}

void hlcd_flush(bsp_lcd_t *hlcd)
{
	lcd_set_display_area(&hlcd->area);

	LCD_Write_Cmd(LCD_MEM_WRITE);
	bsp_lcd_write(hlcd->buff_to_flush,hlcd->write_lenght);
	hlcd->buff_to_flush = NULL;
}

uint32_t pixels_to_bytes(uint32_t pixels,uint8_t pixel_format)
{
	(void)pixel_format;
	return pixels * 2UL;
}
