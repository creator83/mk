#include "MK02F12810.h"                 // Device header


#ifndef DEVICE_H
#define DEVICE_H

//===GPIO DEFENITION===//
enum class port {A , B , C , D , E};
const uint32_t portAddress [5]={PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE};
const uint32_t gpioAddress [5]={PTA_BASE, PTB_BASE, PTC_BASE, PTD_BASE, PTE_BASE};
//===SPI DEFENITION===//
enum class numberSpi {SPI_0=0};
const uint32_t spiAddress [1]={SPI0_BASE};
const uint32_t spiClockShift [1]={12};

//===DMA DEFENITION===//
enum class dmaMux {uart0Rx = 2, uart0Tx, uart1Rx, uart1Tx, spi0Rx = 14, 
	spi0Tx, i2c=18, ftm0ch0=20, ftm0ch1, ftm0ch2, ftm0ch3, ftm0ch4, ftm0ch5,
	ftm1ch0 = 28, ftm1ch1, ftm2ch0, ftm2ch1, adc = 40, cmp0 = 42, cmp1 ,dac = 45, 
	pdb = 48, pta = 49, ptb, ptc, ptd, pte,	dma0 = 60, dma1, dma2, dma3};

enum class dmaChannel {ch0, ch1, ch2 , ch3};
//IRQn dmaInt [4] = {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn};
#endif
