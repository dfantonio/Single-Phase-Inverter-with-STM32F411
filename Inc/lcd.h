/*
 * lcd.h
 *
 *  Created on: 15/06/2016
 *      Author: PedroHenrique
 */

#ifndef LCD_H_
#define LCD_H_

typedef struct {
    GPIO_TypeDef *porta;
    uint16_t enable;
    uint16_t rs;
    uint16_t pinos[4];
} LCD_Init;

FILE __stdout;

extern LCD_Init LCD;

void init_display(void);
void write_nibble(uint8_t value, uint8_t ctrl);
void write_byte(uint8_t value, uint8_t ctrl);
int __io_putchar(int ch);
void print(const char *string);
void gotoxy(uint8_t x, uint8_t y);
void clear();

#endif /* LCD_H_ */
