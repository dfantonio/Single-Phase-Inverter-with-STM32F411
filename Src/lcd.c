/*
 * lcd.c
 *
 *  Created on: 15/06/2016
 *      Author: PedroHenrique
 */

#include "stdio.h"

#include "stm32f4xx.h"
#include "lcd.h"

// isso é uma gambiarra pq o openstm32 não suporta c++
// a vida não é bela
LCD_Init LCD;


void init_display(void) {

	// precisamos fazer isso para que os bytes saiam assim que enviados
	setvbuf(stdout, NULL, _IONBF, 0);

	// esperar a tensão estabilizar
	HAL_Delay(20);

	GPIO_InitTypeDef GPIO_init; // Declara uma estrutura para popularmos
	GPIO_init.Pin = LCD.enable | LCD.rs | LCD.pinos[0] | LCD.pinos[1] | LCD.pinos[2] | LCD.pinos[3];
	GPIO_init.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull
	GPIO_init.Pull = GPIO_NOPULL; // Desativa os resistores de pull-up/down
	GPIO_init.Speed = GPIO_SPEED_FAST; // Clock no modo rápido
	HAL_GPIO_Init(LCD.porta, &GPIO_init);

	// toc toc, vc está aí?
	write_nibble(0x03, 0);
	HAL_Delay(10);
	write_nibble(0x03, 0);
	HAL_Delay(3);
	write_nibble(0x03, 0);
	HAL_Delay(3);

	// vamos usar a interface no modo de 4 bits pq eu tenho poucos jumpers em casa
	write_nibble(0x02, 0);

	// cursor vai incrementar sozinho :o
	write_byte(0x06, 0);

	// liga display com cursor, sem piscar
	write_byte(0x0E, 0);
}

void write_nibble(uint8_t value, uint8_t ctrl) {

	//GPIOA->ODR = (value & 0xF) << 5; // ignore extra bits, shift 5 to the left bc we're using PA5-PA8 for data
	uint8_t i;


	for(i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(LCD.porta, LCD.pinos[i], ((value >> i) & 1));
	}
	HAL_GPIO_WritePin(LCD.porta, LCD.rs, ctrl); // controle ou dados? mistério.
	//rs = pin1
	// enable = pin0
	// pulsa o enable
	HAL_GPIO_WritePin(LCD.porta, LCD.enable, 0);
	HAL_GPIO_WritePin(LCD.porta, LCD.enable, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD.porta, LCD.enable, 0);
}

void write_byte(uint8_t value, uint8_t ctrl) {
	write_nibble(value >> 4, ctrl); // enviamos primeiro os mais significativos
	write_nibble(value, ctrl); // logo depois os menos significativos
	HAL_Delay(1); // delays nunca fazem mal
}

int __io_putchar(int ch) {
	if(ch < 32 || ch > 127) return -1; // ignoramos caracteres de controle
	write_byte(ch, 1);
	return ch;
}

void gotoxy(uint8_t x, uint8_t y) {
	//		0x28 colunas por linhas + coluna
	write_byte(0x80 | (0x28*y + x), 0); // calcula o endereço da memória da posição
}

void clear() {
	write_byte(0x01, 0);
	HAL_Delay(10); // essa operação é meio lenta, então damos um delayzinho
}

