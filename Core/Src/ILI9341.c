/*
 * ILI9341.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Medrano
 */


#include "ILI9341.h"
#include "stm32f7xx.h"


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

	// Configure the LCD
	LCD_Config();
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
	pSPI->CR1 |= (3 << 3);

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


void LCD_Write_Data(uint8_t *data, uint32_t len)
{
	SPI_TypeDef *pSPI = SPI3;

	for(uint32_t i=0;i<len;i++)
	{
		// Set CSX to low for data transmission
		LCD_CSX_LOW();

		// Wait till the transfer buffer is empty
		while(!(pSPI->SR & (1 << 1)));

		// Load the command into the peripheral data register
		pSPI->DR = data[i];

		// Make sure the command is sent
		while(!(pSPI->SR & (1 << 1)));
		while((pSPI->SR & (1 << 7)));

		// Reset pins
		LCD_CSX_HIGH();
	}
}


void LCD_Config(void)
{
	uint8_t params[15];

	LCD_Write_Cmd(LCD_SW_RESET);

	LCD_Write_Cmd(LCD_PWR_CTRL_B);
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





