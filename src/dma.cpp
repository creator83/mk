#include "dma.h"

IRQn Dma::dmaInt [4] = {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn};

Dma::Dma ()
{
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
}

Dma::Dma (dmaChannel ch_)
{
	ch = static_cast<uint8_t>(ch_);
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
}

void Dma::enableDmaMux (dmaMux m)
{
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
}

void Dma::setChannel (dmaChannel ch_)
{
	ch = (uint8_t)ch_;
}

void Dma::setSource (uint32_t ptr)
{
	DMA0->TCD[ch].SADDR = ptr;
}

void Dma::setDestination (uint32_t  ptr)
{
	DMA0->TCD[ch].DADDR = ptr;
}

void Dma::setMinorLoop (uint16_t n){
   
}
void setMajorLoop (uint16_t){
    
}

void Dma::setSizes (size d, size s)
{
	DMA0->TCD[ch].ATTR &=~ DMA_ATTR_DSIZE(0x07);
	DMA0->TCD[ch].ATTR |= DMA_ATTR_DSIZE(d);
	DMA0->TCD[ch].ATTR &=~ DMA_ATTR_SSIZE(0x07);
	DMA0->TCD[ch].ATTR |= DMA_ATTR_SSIZE(s);
}

void Dma::setSsize (size s)
{
	DMA0->TCD[ch].ATTR &=~ DMA_ATTR_SSIZE(0x07);
	DMA0->TCD[ch].ATTR |= DMA_ATTR_SSIZE(s);
}

void Dma::setDsize (size d)
{
	DMA0->TCD[ch].ATTR &=~ DMA_ATTR_DSIZE(0x07);
	DMA0->TCD[ch].ATTR |= DMA_ATTR_DSIZE(d);
}

void Dma::setIncDestination (bool state)
{
}

void Dma::setIncSource (bool state)
{
}

void Dma::enableInterrupt ()
{
	DMA0->DMA[ch].DCR |= DMA_DCR_EINT_MASK;
	NVIC_EnableIRQ(dmaInt[ch]);
}

void Dma::disableInterrupt ()
{
	DMA0->DMA[ch].DCR &= ~ DMA_DCR_EINT_MASK;
	NVIC_DisableIRQ (dmaInt[ch]);
}

void Dma::enablePeriph ()
{
	DMA0->DMA[ch].DCR |= DMA_DCR_ERQ_MASK;
}

void Dma::disablePeriph ()
{
	DMA0->DMA[ch].DCR |= DMA_DCR_D_REQ_MASK| DMA_DCR_ERQ_MASK;
}

void Dma::start ()
{
	DMA0->DMA[ch].DCR |= DMA_DCR_START_MASK;
}

void Dma::clearFlags ()
{
	DMA0->DMA[ch].DSR_BCR |= DMA_DSR_BCR_DONE_MASK;
}

bool Dma::flagDone()
{
	return DMA0->DMA[ch].DSR_BCR&DMA_DSR_BCR_DONE_MASK;
}

uint8_t & Dma::getChannel ()
{
	return ch;
}
