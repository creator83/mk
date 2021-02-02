#include "dma.h"

Dma::Dma ()
{
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
}

Dma::Dma (dmaChannel ch_)
{
	ch = static_cast<uint8_t>(ch_);
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
	//DMA0->ERQ |= 1 << ch;
}

void Dma::enableDmaMux (dmaMux m)
{
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
    DMAMUX->CHCFG[ch] |= DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(m);
}

void Dma::setChannel (dmaChannel ch_){
	ch = (uint8_t)ch_;
}

void Dma::setSource (uint32_t ptr){
    while (DMA0->TCD[ch].SADDR!= ptr){
        DMA0->CR |= DMA_CR_HALT_MASK;
        while (DMA0->CR& (1 << 31));
        DMA0->TCD[ch].SADDR = ptr;
        DMA0->CR &=~ DMA_CR_HALT_MASK;
    }
}

void Dma::setDestination (uint32_t  ptr){
    while (DMA0->TCD[ch].DADDR!= ptr){
        DMA0->CR |= DMA_CR_HALT_MASK;
        while (DMA0->CR& (1 << 31));
        DMA0->TCD[ch].DADDR = ptr;
        DMA0->CR &=~ DMA_CR_HALT_MASK;
    }
}

void Dma::setCountMnrLoop (uint32_t n){
    while (DMA0->TCD[ch].NBYTES_MLNO!= n){
        DMA0->CR |= DMA_CR_HALT_MASK;
        while (DMA0->CR& (1 << 31));
        DMA0->TCD[ch].NBYTES_MLNO = n;
        DMA0->CR &=~ DMA_CR_HALT_MASK;
    }
}
void Dma::setCountMjrLoop (uint16_t n){
    DMA0->TCD[ch].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(n);
	DMA0->TCD[ch].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(n);
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

void Dma::setSLast (int32_t val){
	DMA0->TCD[ch].SLAST = val;
}
void Dma::setDLast (int32_t val){
	DMA0->TCD[ch].DLAST_SGA = val;
}

void Dma::setMjrLink(dmaChannel linkCh){
    DMA0->TCD[ch].CSR |= DMA_CSR_MAJORELINK_MASK|DMA_CSR_MAJORLINKCH(linkCh);
}

void Dma::setMjrLink(Dma &d){
    DMA0->TCD[ch].CSR |= DMA_CSR_MAJORELINK_MASK|DMA_CSR_MAJORLINKCH(d.getChannel());
}

void Dma::triggerEnable(){
    DMAMUX->CHCFG[ch] |= DMAMUX_CHCFG_TRIG_MASK;
}

void Dma::triggerDisable(){
    DMAMUX->CHCFG[ch] &=~ DMAMUX_CHCFG_TRIG_MASK;
}

void Dma::enableInterruptMajor ()
{
	DMA0->TCD[ch].CSR |= DMA_CSR_INTMAJOR_MASK;
	NVIC_EnableIRQ(DMA1_IRQn);
}

void Dma::disableInterruptMajor ()
{
	DMA0->TCD[ch].CSR &=~ DMA_CSR_INTMAJOR_MASK;
	NVIC_DisableIRQ (DMA1_IRQn);
}

void Dma::startRequest (){
    DMA0->SERQ = DMA_SERQ_SERQ(ch);
}

void Dma::start ()
{
    DMA0->SSRT |= DMA_SSRT_SSRT(ch);
}

void Dma::clearFlags ()
{
}
void Dma::setDREQ(){
    DMA0->TCD[ch].CSR |= DMA_CSR_DREQ_MASK;
}
bool Dma::flagDone()
{
	return DMA0->TCD[ch].CSR&DMA_CSR_DONE_MASK;
}

uint8_t & Dma::getChannel ()
{
	return ch;
}
