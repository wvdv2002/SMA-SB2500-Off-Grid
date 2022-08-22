/*
 * defines.h
 *
 *  Created on: 22 aug. 2017
 *      Author: wvdv2
 */

#ifndef DEFINES_H_
#define DEFINES_H_

/* Project definitions for TM libraries if necessary */
#define DMA2_STREAM0_DISABLE_IRQHANDLER
#define DMA2_STREAM1_DISABLE_IRQHANDLER
#define DMA2_STREAM2_DISABLE_IRQHANDLER
#define DMA2_STREAM3_DISABLE_IRQHANDLER
#define DMA2_STREAM4_DISABLE_IRQHANDLER
#define DMA2_STREAM5_DISABLE_IRQHANDLER
#define DMA2_STREAM6_DISABLE_IRQHANDLER
#define DMA2_STREAM7_DISABLE_IRQHANDLER
#define USART1_DISABLE_IRQHANDLER

//RS - Register select pin
#define HD44780_RS_PORT     GPIOB
#define HD44780_RS_PIN      GPIO_PIN_4
//E - Enable pin
#define HD44780_E_PORT      GPIOB
#define HD44780_E_PIN       GPIO_PIN_5
//D4 - Data 4 pin
#define HD44780_D4_PORT     GPIOC
#define HD44780_D4_PIN      GPIO_PIN_12
//D5 - Data 5 pin
#define HD44780_D5_PORT     GPIOB
#define HD44780_D5_PIN      GPIO_PIN_8
//D6 - Data 6 pin
#define HD44780_D6_PORT     GPIOC
#define HD44780_D6_PIN      GPIO_PIN_11
//D7 - Data 7 pin
#define HD44780_D7_PORT     GPIOC
#define HD44780_D7_PIN      GPIO_PIN_10




#endif /* DEFINES_H_ */
