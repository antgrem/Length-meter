//*****************************************************************************


#include "lcd_lib.h"
#include <stdint.h>                      /*!< standard types definitions                      */

extern void HAL_Delay(__IO uint32_t nTime);


void LCDinit(void)//Initializes LCD
{
	STM_EVAL_Control_LCD_Init();
	HAL_Delay(5);
	STM_EVAL_DataLine_Init();
	HAL_Delay(5);
	
	SET_E_OFF;
	HAL_Delay(5);
	SET_RS_OFF;
	HAL_Delay(5);
	SET_RW_OFF;
	
	HAL_Delay(1000);
	//---------one------
	HAL_GPIO_WritePin(LDPORT, (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7), GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LDPORT, (LCD_D4 | LCD_D5), GPIO_PIN_SET);//4 bit mode
	SET_E_ON;
	HAL_Delay(5);
	SET_E_OFF;
	HAL_Delay(5);
	
//	_delay_ms(0xFFFF);
//---------------------------
	//-----------two-----------
	LCDsendCommand((0x02<<4) | (1<<LCD_FUNCTION_2LINES) | (1<<LCD_FUNCTION_10DOTS));//4 bit mode
		HAL_Delay(5);
	SET_E_ON;
	HAL_Delay(5);
	SET_E_OFF;
	HAL_Delay(5);
	//-------three-------------
	LCDsendCommand((0x02<<4) | (1<<LCD_FUNCTION_2LINES) | (1<<LCD_FUNCTION_10DOTS));//4 bit mode
		HAL_Delay(5);
	SET_E_ON;
	HAL_Delay(5);
	SET_E_OFF;
	HAL_Delay(5);
	//--------4 bit--dual line---------------
	LCDsendCommand(0x28);
   //-----increment address, cursor shift------
	LCDsendCommand(0xE);
}			


void	_delay_ms(uint16_t nCount)
{
	for (; nCount != 0; nCount--);
}

void STM_EVAL_Control_LCD_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.Pin = LCD_E | LCD_RS |LCD_RW;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCPORT, &GPIO_InitStructure);
	
}


void STM_EVAL_DataLine_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.Pin = (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7);
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LDPORT, &GPIO_InitStructure);
}


void LCDsendChar(uint8_t ch)		//Sends Char to LCD
{ uint16_t temp;
	
	HAL_GPIO_WritePin(LDPORT, (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7), GPIO_PIN_RESET);
	_delay_ms(1000);
	//GPIO_SetBits(LDPORT, (ch&0xF0)>>4);
	
	temp = 0;
	temp |= ((((ch&0xF0)>>4)&0x01)==0) ? 0 : LCD_D4;
	temp |= ((((ch&0xF0)>>4)&0x02)==0) ? 0 : LCD_D5;
	temp |= ((((ch&0xF0)>>4)&0x04)==0) ? 0 : LCD_D6;
	temp |= ((((ch&0xF0)>>4)&0x08)==0) ? 0 : LCD_D7;
	HAL_GPIO_WritePin(LDPORT, temp, GPIO_PIN_SET);
	
//	LDP=(ch&0b11110000);
	SET_RS_ON;
	SET_E_ON;		
	_delay_ms(1000);
	SET_E_OFF;	
	SET_RS_OFF;
	_delay_ms(1000);
	HAL_GPIO_WritePin(LDPORT, (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7), GPIO_PIN_RESET);
	_delay_ms(1000);
	
	temp = 0;
	temp |= (((ch&0x0F)&0x01)==0) ? 0 : LCD_D4;
	temp |= (((ch&0x0F)&0x02)==0) ? 0 : LCD_D5;
	temp |= (((ch&0x0F)&0x04)==0) ? 0 : LCD_D6;
	temp |= (((ch&0x0F)&0x08)==0) ? 0 : LCD_D7;
	HAL_GPIO_WritePin(LDPORT, temp, GPIO_PIN_SET);
	
	SET_RS_ON;
	SET_E_ON;		
	_delay_ms(1000);
	SET_E_OFF;	
	SET_RS_OFF;
	_delay_ms(1000);
}

void LCDsendCommand(uint8_t cmd)	//Sends Command to LCD
{ uint16_t temp;
	
	SET_RS_OFF;
		HAL_Delay(5);
	
	temp = 0;
	HAL_GPIO_WritePin(LDPORT, (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7), GPIO_PIN_RESET);
	_delay_ms(1000);
	HAL_Delay(5);
	//GPIO_SetBits(LDPORT, (cmd&0xF0)>>4);
	
	temp |= ((((cmd&0xF0)>>4)&0x01)==0) ? 0 : LCD_D4;
	temp |= ((((cmd&0xF0)>>4)&0x02)==0) ? 0 : LCD_D5;
	temp |= ((((cmd&0xF0)>>4)&0x04)==0) ? 0 : LCD_D6;
	temp |= ((((cmd&0xF0)>>4)&0x08)==0) ? 0 : LCD_D7;
	HAL_GPIO_WritePin(LDPORT, temp, GPIO_PIN_SET);
	_delay_ms(1000);	
	HAL_Delay(5);
	
	SET_E_ON;		
	_delay_ms(1000);
	HAL_Delay(5);
	SET_E_OFF;
	_delay_ms(1000);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LDPORT, (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7), GPIO_PIN_RESET);
	_delay_ms(1000);
		HAL_Delay(5);
	//GPIO_SetBits(LDPORT, cmd&0x0F);
	
	temp = 0;
	temp |= (((cmd&0x0F)&0x01)==0) ? 0 : LCD_D4;
	temp |= (((cmd&0x0F)&0x02)==0) ? 0 : LCD_D5;
	temp |= (((cmd&0x0F)&0x04)==0) ? 0 : LCD_D6;
	temp |= (((cmd&0x0F)&0x08)==0) ? 0 : LCD_D7;
	HAL_GPIO_WritePin(LDPORT, temp, GPIO_PIN_SET);
		HAL_Delay(5);
	
	SET_E_ON;		
	_delay_ms(1000);
	HAL_Delay(5);
	SET_E_OFF;
	_delay_ms(1000);
	HAL_Delay(5);
		SET_RS_ON;
			HAL_Delay(5);
}

void LCDclr(void)				//Clears LCD
{
	LCDsendCommand(0x01);
}

void LCDhome(void)			//LCD cursor home
{
	LCDsendCommand(0x02);
}

void LCDstring(uint8_t* data, uint8_t nBytes)	//Outputs string to LCD
{
register uint8_t i;

	// check to make sure we have a good pointer
	if (!data) return;

	// print data
	for(i=0; i<nBytes; i++)
	{
		LCDsendChar(data[i]);
	}
}
void LCDGotoXY(uint8_t x, uint8_t y)	//Cursor to X Y position
{
	register uint8_t DDRAMAddr;
	// remap lines into proper order
	switch(y)
	{
	case 0: DDRAMAddr = LCD_LINE0_DDRAMADDR+x; break;
	case 1: DDRAMAddr = LCD_LINE1_DDRAMADDR+x; break;
	case 2: DDRAMAddr = LCD_LINE2_DDRAMADDR+x; break;
	case 3: DDRAMAddr = LCD_LINE3_DDRAMADDR+x; break;
	default: DDRAMAddr = LCD_LINE0_DDRAMADDR+x;
	}
	// set data address
	LCDsendCommand(1<<LCD_DDRAM | DDRAMAddr);
	
}



void LCDshiftLeft(uint8_t n)	//Scrol n of characters Right
{
	uint8_t  i;
	for (i=0;i<n;i++)
	{
		LCDsendCommand(0x1E);
	}
}
void LCDshiftRight(uint8_t n)	//Scrol n of characters Left
{
	uint8_t  i;
	for (i=0;i<n;i++)
	{
		LCDsendCommand(0x18);
	}
}
void LCDcursorOn(void) //displays LCD cursor
{
	LCDsendCommand(0x0E);
}
void LCDcursorOnBlink(void)	//displays LCD blinking cursor
{
	LCDsendCommand(0x0F);
}
void LCDcursorOFF(void)	//turns OFF cursor
{
	LCDsendCommand(0x0C);
}
void LCDblank(void)		//blanks LCD
{
	LCDsendCommand(0x08);
}
void LCDvisible(void)		//Shows LCD
{
	LCDsendCommand(0x0C);
}
void LCDcursorLeft(uint8_t n)	//Moves cursor by n poisitions left
{
	uint8_t  i;
	for (i=0;i<n;i++)
	{
		LCDsendCommand(0x10);
	}
}
void LCDcursorRight(uint8_t n)	//Moves cursor by n poisitions left
{
	uint8_t  i;
	for (i=0;i<n;i++)
	{
		LCDsendCommand(0x14);
	}
}
