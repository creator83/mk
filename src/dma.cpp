#include "dma.h"

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
   DMA0->TCD[ch].NBYTES_MLNO = n;
}
void setMajorLoop (uint16_t n){
    
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

void Dma::setOffsetDestination (uint32_t val)
{
	DMA0->TCD[ch].DOFF = val;
}

void Dma::setOffsetSource (uint32_t val)
{
	DMA0->TCD[ch].SOFF = val;
}

void Dma::setSLast (uint32_t val){
	DMA0->TCD[ch].SLAST = val;
}
void Dma::setDLast (uint32_t val){
	DMA0->TCD[ch].DLAST_SGA = val;
}
void Dma::enableInterrupt ()
{

	NVIC_EnableIRQ(dmaInt[ch]);
}

void Dma::disableInterrupt ()
{

	NVIC_DisableIRQ (dmaInt[ch]);
}

void Dma::enablePeriph ()
{

}

void Dma::disablePeriph ()
{
}

void Dma::start ()
{
}

void Dma::clearFlags ()
{
}

bool Dma::flagDone()
{
}

uint8_t & Dma::getChannel ()
{
}
