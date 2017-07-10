//*****************************************************************************
//
// File Name	: 'lcd_lib.h'
// Title		: 4 bit LCd interface header file
// Author		: Scienceprog.com - Copyright (C) 2007
// Created		: 2007-06-18
// Revised		: 2007-06-18
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************
//remake for STM32

#ifndef LCD_LIB
#define LCD_LIB

//#include <inttypes.h>
#include <stdint.h>                      /*!< standard types definitions                      */
#include "stm32f1xx_hal.h"

#define LCD_RS	GPIO_PIN_8 	//define MCU pin connected to LCD RS
#define LCD_RW	GPIO_PIN_7 	//define MCU pin connected to LCD R/W
#define LCD_E		GPIO_PIN_6		//define MCU pin connected to LCD E
#define LCD_D4	GPIO_PIN_5	//define MCU pin connected to LCD D3
#define LCD_D5	GPIO_PIN_4	//define MCU pin connected to LCD D4
#define LCD_D6	GPIO_PIN_3	//define MCU pin connected to LCD D5
#define LCD_D7	GPIO_PIN_2	//define MCU pin connected to LCD D6
#define LDP RCC_AHB1Periph_GPIOB	//define MCU port connected to LCD data pins
#define LCP RCC_AHB1Periph_GPIOB	//define MCU port connected to LCD control pins
#define LCPORT GPIOB // define port configuration
#define LDPORT GPIOB // 

#define SET_RS_ON 	HAL_GPIO_WritePin(LCPORT, LCD_RS, GPIO_PIN_SET)
#define SET_RS_OFF 	HAL_GPIO_WritePin(LCPORT, LCD_RS, GPIO_PIN_RESET)
#define SET_RW_ON 	HAL_GPIO_WritePin(LCPORT, LCD_RW, GPIO_PIN_SET)
#define SET_RW_OFF 	HAL_GPIO_WritePin(LCPORT, LCD_RW, GPIO_PIN_RESET)
#define SET_E_ON 		HAL_GPIO_WritePin(LCPORT, LCD_E, GPIO_PIN_SET)
#define SET_E_OFF 	HAL_GPIO_WritePin(LCPORT, LCD_E, GPIO_PIN_RESET)

#define LCD_CLR             0	//DB0: clear display
#define LCD_HOME            1	//DB1: return to home position
#define LCD_ENTRY_MODE      2	//DB2: set entry mode
#define LCD_ENTRY_INC       1	//DB1: increment
#define LCD_ENTRY_SHIFT     0	//DB2: shift
#define LCD_ON_CTRL         3	//DB3: turn lcd/cursor on
#define LCD_ON_DISPLAY      2	//DB2: turn display on
#define LCD_ON_CURSOR       1	//DB1: turn cursor on
#define LCD_ON_BLINK        0	//DB0: blinking cursor
#define LCD_MOVE            4	//DB4: move cursor/display
#define LCD_MOVE_DISP       3	//DB3: move display (0-> move cursor)
#define LCD_MOVE_RIGHT      2	//DB2: move right (0-> left)
#define LCD_FUNCTION        5	//DB5: function set
#define LCD_FUNCTION_8BIT   4	//DB4: set 8BIT mode (0->4BIT mode)
#define LCD_FUNCTION_2LINES 3	//DB3: two lines (0->one line)
#define LCD_FUNCTION_10DOTS 2	//DB2: 5x10 font (0->5x7 font)
#define LCD_CGRAM           6	//DB6: set CG RAM address
#define LCD_DDRAM           7	//DB7: set DD RAM address
// reading:
#define LCD_BUSY            7	//DB7: LCD is busy
#define LCD_LINES			2	//visible lines
#define LCD_LINE_LENGTH		16	//line length (in characters)
// cursor position to DDRAM mapping
#define LCD_LINE0_DDRAMADDR		0x00
#define LCD_LINE1_DDRAMADDR		0x40
#define LCD_LINE2_DDRAMADDR		0x14
#define LCD_LINE3_DDRAMADDR		0x54

void LCDsendChar(uint8_t);		//forms data ready to send to 74HC164
void LCDsendCommand(uint8_t);	//forms data ready to send to 74HC164
void LCDinit(void);			//Initializes LCD
void LCDclr(void);				//Clears LCD
void LCDhome(void);			//LCD cursor home
void LCDstring(uint8_t*, uint8_t);	//Outputs string to LCD
void LCDGotoXY(uint8_t, uint8_t);	//Cursor to X Y position
void LCDshiftRight(uint8_t);	//shift by n characters Right
void LCDshiftLeft(uint8_t);	//shift by n characters Left
void LCDcursorOn(void);		//Underline cursor ON
void LCDcursorOnBlink(void);	//Underline blinking cursor ON
void LCDcursorOFF(void);		//Cursor OFF
void LCDblank(void);			//LCD blank but not cleared
void LCDvisible(void);			//LCD visible
void LCDcursorLeft(uint8_t);	//Shift cursor left by n
void LCDcursorRight(uint8_t);	//shif cursor right by n

//void	_delay_ms(uint16_t);
void STM_EVAL_Control_LCD_Init(void);
void STM_EVAL_DataLine_Init(void);

#endif

