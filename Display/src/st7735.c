/* vim: set ai et ts=4 sw=4: */
#include "stm32f4xx_hal.h"
#include "st7735.h"


#define DELAY 0x80
#ifndef M_PI_2
#define M_PI_2 	3.14159265/2
#endif

static uint16_t bufScreen [ST7735_WIDTH * ST7735_HEIGHT];

// based on Adafruit ST7735 library
static const uint8_t  init_cmds1[] = {    // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


DMA_HandleTypeDef DMA_ST7735;
SPI_HandleTypeDef ST7735_SPI_PORT;


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	ST7735_Unselect();
}


static void ST7735_Select()
{
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}


void ST7735_Unselect()
{
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}


static void ST7735_Reset()
{
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
}


static void ST7735_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}



static void ST7735_WriteData(uint8_t* buff, size_t buff_size)
{
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
}



static void ST7735_WriteDataDMA(uint8_t* buff, size_t buff_size)
{
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

    HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, (uint8_t *)bufScreen, buff_size);

}



static void ST7735_ExecuteCommandList(const uint8_t *addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            ST7735_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}



static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    // column address set
    ST7735_WriteCommand(ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    ST7735_WriteData(data, sizeof(data));

    // row address set
    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;
    ST7735_WriteData(data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(ST7735_RAMWR);
}



void ST7735_Init()
{
    /* SPI1 parameter configuration*/
    ST7735_SPI_PORT.Instance = SPI3;
    ST7735_SPI_PORT.Init.Mode = SPI_MODE_MASTER;
    ST7735_SPI_PORT.Init.Direction = SPI_DIRECTION_2LINES;
    ST7735_SPI_PORT.Init.DataSize = SPI_DATASIZE_8BIT;
    ST7735_SPI_PORT.Init.CLKPolarity = SPI_POLARITY_LOW;
    ST7735_SPI_PORT.Init.CLKPhase = SPI_PHASE_1EDGE;
    ST7735_SPI_PORT.Init.NSS = SPI_NSS_SOFT;
    ST7735_SPI_PORT.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    ST7735_SPI_PORT.Init.FirstBit = SPI_FIRSTBIT_MSB;
    ST7735_SPI_PORT.Init.TIMode = SPI_TIMODE_DISABLE;
    ST7735_SPI_PORT.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    ST7735_SPI_PORT.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&ST7735_SPI_PORT);
    
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    DMA_ST7735.Instance = DMA1_Stream5;
    DMA_ST7735.Init.Channel = DMA_CHANNEL_0;
    DMA_ST7735.Init.Direction = DMA_MEMORY_TO_PERIPH;
    DMA_ST7735.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_ST7735.Init.MemInc = DMA_MINC_ENABLE;
    DMA_ST7735.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DMA_ST7735.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DMA_ST7735.Init.Mode = DMA_NORMAL;
    DMA_ST7735.Init.Priority = DMA_PRIORITY_MEDIUM;
    DMA_ST7735.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    //DMA_ST7735.Init.FIFOThreshold
    //DMA_ST7735.Init.MemBurst
    //DMA_ST7735.Init.PeriphBurst
    
    HAL_DMA_Init(&DMA_ST7735);
    
    __HAL_LINKDMA(&ST7735_SPI_PORT, hdmatx, DMA_ST7735);
    
    
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 4, 4);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);


    ST7735_Select();
    ST7735_Reset();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    ST7735_WriteCommand(0x44);

    ST7735_Unselect();

    //ST7735_FillScreen(ST7735_WHITE);
}



void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;

    *(bufScreen + y * 160 + x) = color;
}

uint16_t ST7735_GetPixel(uint16_t x, uint16_t y)
{
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return 0;

    return *(bufScreen + y * 160 + x);
}



static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color,
		uint16_t bgcolor)
{
    uint32_t i, b, j;

    for(i = 0; i < font.height; i++)
    {
        b = font.data[(ch - 32) * font.height + i];

        for(j = 0; j < font.width; j++)
        {
            if((b << j) & 0x8000)
            {

                *(bufScreen + x + j + (y + i) * 160) = color;

            }
        }
    }
}



void ST7735_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font,
		uint16_t color,	uint16_t bgcolor)
{

    while(*str) {
        if(x + font.width >= ST7735_WIDTH)
        {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7735_HEIGHT)
            {
                break;
            }

            if(*str == ' ')
            {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

}



void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    for(int i = y; i < y + h; i++)
    {
        for(int j = x; j < x + w; j++)
        {
        	uint8_t temp = color >> 8;
        	color = (color << 8) | temp;
            *(bufScreen + j + i * 160) = color;
        }
    }
}



void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}



void ST7735_DrawScreen(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) return;
    if((y + h - 1) >= ST7735_HEIGHT) return;

    if(DMA_ST7735.State != HAL_DMA_STATE_READY)
    {
    	return;
    }

    if(ST7735_SPI_PORT.State != HAL_SPI_STATE_READY)
    	return;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
    ST7735_WriteDataDMA((uint8_t*)bufScreen, w*h*2);
    //ST7735_Unselect();
}


void ST7735_DrawImageMemRot(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data,
		double rotation, uint16_t xo, uint16_t yo)
{
	for(unsigned short int i = 0; i < h; i++)
	{
		for(unsigned short int j = 0; j < w; j++)
		{

			if (*data != 0xffff)
			{
				double io, jo, l, phi;
				int offset = 0;

				int jxo = (int)j - (int)xo;
				int iyo = (int)i - (int)yo;

				l = sqrt((jxo)*(jxo) + (iyo)*(iyo));	//����� �� ����� �� ������ ��������

				phi = rotation * 0.01745329; // - getAngle(jxo, iyo);

				if(i >= yo)
				{
					l = - l;
				}

				phi = - phi - M_PI_2;

				io = l * sin(phi);
				jo = l * cos(phi);

				offset = (int)(jo + x + xo) + (int)(io + y + yo) * 160;

				if(((int)jo + x + xo) >= 0 && ((int)jo + x + xo) < 160 &&
						((int)io + y + yo) >= 0 && ((int)io + y + yo) < 128)
					*(bufScreen + offset) = *data;
			}
			data++;
		}
	}
}


void ST7735_DrawImageMem(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{

	int offset = 0;
	for(unsigned short int i = y; i < y+h; i++)
	{
		for(unsigned short int j = x; j < x+w; j++)
		{

			offset = j + i * 160;

			if (*data != 0xffff)
				*(bufScreen + offset) = *data;

			data++;
		}
	}

}

void ST7735_InvertColors(int invert)
{
    ST7735_Select();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect();
}

void ST7735_Update(void)
{
	ST7735_DrawScreen(0, 0, ST7735_WIDTH, ST7735_HEIGHT);//, bufScreen);
}

void ST7735_print(const char* str)//, uint16_t* fon)
{
	ST7735_WriteString(0, 0, str, Font_7x10, ST7735_WHITE, ST7735_BLACK);

}







void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /*Configure GPIO pin : PC7 */
    GPIO_InitStruct.Pin = ST7735_RES_Pin;  //ST7735_Reset
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PC9 */
    GPIO_InitStruct.Pin = ST7735_DC_Pin;  //ST7735_DC
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PC6 */
    GPIO_InitStruct.Pin = ST7735_CS_Pin;  //ST7735_CS
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
    /**SPI1 GPIO Configuration    
    PA10     ------> SPI1_SCK
    PA12     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }

}



