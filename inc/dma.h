#include "device.h"                  // Device header

#ifndef DMA_H
#define DMA_H
/*
namespace dmaMux
{
	const uint8_t uart0Rx = 2;
	const uint8_t uart0Tx = 3;
	const uint8_t uart1Rx = 4;
	const uint8_t uart1Tx = 5;
	const uint8_t uart2Rx = 6;
	const uint8_t uart2Tx = 7;
	const uint8_t i2sRx   = 14;
	const uint8_t i2sTx   = 15;
	const uint8_t spi0Rx  = 16;
	const uint8_t spi0Tx  = 17;
	const uint8_t spi1Rx  = 18;
	const uint8_t spi1Tx  = 19;
	const uint8_t i2c0    = 22;
	const uint8_t i2c1    = 23;
	const uint8_t tpm0ch0 = 24;
	const uint8_t tpm0ch1 = 25;
	const uint8_t tpm0ch2 = 26;
	const uint8_t tpm0ch3 = 27;
	const uint8_t tpm0ch4 = 28;
	const uint8_t tpm0ch5 = 29;
	const uint8_t tpm1ch0 = 32;
	const uint8_t tpm1ch1 = 33;
	const uint8_t tpm2ch0 = 34;
	const uint8_t tpm2ch1 = 35;
	const uint8_t adc     = 40;
	const uint8_t cmp     = 42;
	const uint8_t dac     = 45;
	const uint8_t pta     = 49;
	const uint8_t ptc     = 51;
	const uint8_t ptd     = 52;
	const uint8_t tpm0Ovf = 54;
	const uint8_t tpm1Ovf = 55;
	const uint8_t tpm2Ovf = 56;
	const uint8_t tsi     = 57;
}
*/
class dma;

typedef void (dma::*PTR_DMA)() ;

class Dma
{
	//variables
public:
	enum class dmaChannel {ch0, ch1, ch2 , ch3};
	enum class dmaMux {uart0Rx = 2, uart0Tx, uart1Rx, uart1Tx,
	uart2Rx, uart2Tx, i2sRx = 14, i2sTx, spi0Rx, spi0Tx, spi1Rx, spi1Tx,
	i2c0 = 22, i2c1, tpm0ch0, tpm0ch1, tpm0ch2, tpm0ch3, tpm0ch4, tpm0ch5,
	tpm1ch0 = 32, tpm1ch1, tpm2ch0, tpm2ch1, adc = 40, cmp = 42, dac = 45, pta = 49,
	ptc = 51, ptd, tpm0Ovf = 54, tpm1Ovf, tpm2Ovf, tsi,
	dma0 = 60, dma1, dma2, dma3};
	enum class size {bit32, bit8, bit16};

private:
	uint8_t ch;
	static IRQn dmaInt [4];
	//functions
public:	
	Dma ();
	Dma (dmaChannel ch);
	void enableDmaMux (dmaMux);
	void setChannel (dmaChannel ch);
	void setSource (uint32_t  ptr);
	void setDestination (uint32_t ptr);
	void setLength (uint32_t length);
	void setSizes (size m, size p);
	void setSsize (size);
	void setDsize (size);
	void setIncDestination (bool state);
	void setIncSource (bool state);
	void enableInterrupt ();
	void disableInterrupt ();
	void enablePeriph ();
	void disablePeriph ();
	void start ();
	void clearFlags ();
	bool flagDone();
	uint8_t & getChannel ();
};

#endif
