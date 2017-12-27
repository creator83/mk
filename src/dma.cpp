#include "dma.h"

Dma::Dma ()
{
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
}

Dma::Dma (dmaChannel ch_)
{
	ch = static_cast<uint8_t>(ch_);
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
	DMA0->ERQ |= 1 << ch;
}

void Dma::enableDmaMux (dmaMux m)
{
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
}

void Dma::setChannel (dmaChannel ch_)
{
	ch = (uint8_t)ch_;
	DMA0->ERQ |= 1 << ch;
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

	//NVIC_EnableIRQ(dmaInt[ch]);
}

void Dma::disableInterrupt ()
{

	//NVIC_DisableIRQ (dmaInt[ch]);
}

void Dma::enablePeriph ()
{

}

void Dma::disablePeriph ()
{
}

void Dma::start ()
{
	DMA0->TCD[ch].CSR |= DMA_CSR_START_MASK;
}

void Dma::clearFlags ()
{
}

bool Dma::flagDone()
{
	return DMA0->TCD[ch].CSR&DMA_CSR_DONE_MASK;
}

uint8_t & Dma::getChannel ()
{
	return ch;
}
