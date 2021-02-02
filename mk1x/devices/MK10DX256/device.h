#include "MK10D10.h"                  // Device header

#ifndef DEVICE_H
#define DEVICE_H

//===SIM DEFENITION===//
/*const uint32_t SCGC1_OFFSET = 0x1028;
const uint32_t SCGC2_OFFSET = 0x102C;
const uint32_t SCGC3_OFFSET = 0x1030;
const uint32_t SCGC4_OFFSET = 0x1034;
const uint32_t SCGC5_OFFSET = 0x1038;
const uint32_t SCGC6_OFFSET = 0x103C;
const uint32_t SCGC7_OFFSET = 0x1040;
*/
//===GPIO DEFENITION===//
enum class  port {A , B , C , D , E};
enum class mux {Alt0, Gpio, Alt2 , Alt3 , Alt4 , Alt5 , Alt6 , Alt7};
const uint32_t portAddress [5]={PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE};
const uint32_t gpioAddress [5]={GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE};

const IRQn pinInt [5] = {PORTA_IRQn, PORTB_IRQn, PORTC_IRQn, PORTD_IRQn, PORTE_IRQn};
/*
//===MODE STATE DEFENITION===//
enum class state {FEI, FEE, FBI, FBE, BLPE, BLPI, PBE, PEE};
enum class lptmrSource {osc32, lpo=3};
enum class periphSource {fllClock, pllClock};
enum class fllSource {external, internal};
enum class clockSourceSelect {fllPll, internal, external};
enum class fllDivider {div1_32, div2_64, div4_128, div8_256, div16_512, 
	div32_1024, div64_1280, div128_1536};
enum class clockDivider {div1=1, div2, div3, div4, div5, div6, div7, div8, div9, div10,
                                div11, div12, div13, div14, div15, div16};
enum class dividers {coreClock = 28, busClock = 24, flexBusClock = 20, flashClock = 16};
const uint32_t extFllDividerArr[2][8] = {{32, 64, 128, 256, 512, 1024, 1280, 1536},
                                           {1, 2, 4, 8, 16, 32, 64, 128}};



//===SPI DEFENITION===//
enum class numberSpi {SPI_0, SPI_1, SPI_2};
const uint32_t spiAddress [3]={SPI0_BASE, SPI1_BASE, SPI2_BASE};
const uint32_t spiClockShift [1]={12};

//===DMA DEFENITION===//
enum class dmaMux {uart0Rx = 2, uart0Tx, uart1Rx, uart1Tx, spi0Rx = 14, 
	spi0Tx, i2c=18, ftm0ch0=20, ftm0ch1, ftm0ch2, ftm0ch3, ftm0ch4, ftm0ch5,
	ftm1ch0 = 28, ftm1ch1, ftm2ch0, ftm2ch1, adc = 40, cmp0 = 42, cmp1 ,dac = 45, 
	pdb = 48, pta = 49, ptb, ptc, ptd, pte,	dma0 = 60, dma1, dma2, dma3};

enum class dmaChannel {ch0, ch1, ch2 , ch3, ch4, ch5, ch6 , ch7, ch8, ch9, ch10 , ch11, ch12, ch13, ch14 , ch15};
const IRQn dmaInt [16] = {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn, DMA4_IRQn, DMA5_IRQn, DMA6_IRQn,
                          DMA7_IRQn, DMA8_IRQn, DMA9_IRQn, DMA10_IRQn, DMA11_IRQn, DMA12_IRQn, DMA13_IRQn, 
                          DMA14_IRQn, DMA15_IRQn};

//===CAN DEFENITION===//
enum class numberCan {CAN_0, CAN_1};
const uint32_t canBaseAddress [2]={CAN0_BASE, CAN1_BASE};
const uint8_t FLEXCAN_MSG_BUFADDR_OFFSET = 0x80;
const uint8_t msgBuffSize = 16; 
enum class canMb {mb0, mb1, mb2 , mb3, mb4, mb5, mb6 , mb7, mb8, mb9, mb10 , mb11, mb12, mb13, mb14 , mb15};

//===PIT DEFENITION===//
enum class pitChannel {ch0, ch1, ch2, ch3};
const IRQn pitInt [4] = {PIT0_IRQn, PIT1_IRQn, PIT2_IRQn, PIT3_IRQn};



//===UART DEFENITION===//

enum class numberUart {UART_0, UART_1, UART_2, UART_3, UART_4, UART_5};
const uint32_t uartAddress [6]={UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE, UART4_BASE, UART5_BASE};
const uint32_t uartClock [6][2]={{SIM_BASE+SCGC4_OFFSET, SIM_SCGC4_UART0_MASK},
                                 {SIM_BASE+SCGC4_OFFSET, SIM_SCGC4_UART1_MASK},    
                                 {SIM_BASE+SCGC4_OFFSET, SIM_SCGC4_UART2_MASK},
                                 {SIM_BASE+SCGC4_OFFSET, SIM_SCGC4_UART3_MASK},
                                 {SIM_BASE+SCGC1_OFFSET, SIM_SCGC1_UART4_MASK},
                                 {SIM_BASE+SCGC1_OFFSET, SIM_SCGC1_UART5_MASK},
                                 };
*/
#endif
