#include "device.h"
#include "state.h"
#include "pin.h"
#include "dma.h"
#include "spi.h"
#include "ili9341.h"
#include "pit.h"
//#define SIMPLE
extern "C"{
	void DMA1_IRQHandler();
    void SPI0_IRQHandler();
    void PIT1_IRQHandler ();
    void PIT0_IRQHandler ();
}

uint8_t source[10]={0,1,2,3,4,5,6,7,8,9};
uint8_t dest[10] = {1};
uint16_t color = 0x07E0;
uint32_t buffer[1024];
uint32_t buffer0[20];
uint32_t currentColor = 0x10010000|color;

void memtomem (uint32_t , uint32_t );
void memSample(uint32_t , uint32_t );

void memtoSpi (uint32_t );
void initSpi ();
void pinSpiInit ();
void fillBuffer();
void sendDMA();
Dma m2m (dmaChannel::ch0);
Dma m2spi (dmaChannel::ch1);
Dma mem2mem (dmaChannel::ch2);
Spi spi0 (numberSpi::SPI_0, Spi::ctarNumber::ctar1, Spi::csNumber::cs0);
Spi spiDma (numberSpi::SPI_0, Spi::ctarNumber::ctar0, Spi::csNumber::cs0);
Pit trg (pitChannel::ch1, 1, Pit::mode::ms);
Pit trgmem (pitChannel::ch0, 1, Pit::mode::ms);

void SPI0_IRQHandler(){
    static uint8_t i=0;
    SPI0->SR |= SPI_SR_EOQF_MASK| SPI_SR_TCF_MASK;
    DMA0->SERQ = DMA_SERQ_SERQ(1);
    /*if (i<5){
        DMA0->SERQ = DMA_SERQ_SERQ(1);
        ++i;
    }*/
}
void PIT1_IRQHandler (){
    trg.clear_flag();
    
}
void PIT0_IRQHandler (){
    trgmem.clear_flag();
    
}

int main(){
    State & frq = State::getInstance();
	//tact dma
    trg.interrupt_enable();
    trgmem.interrupt_enable();
    initSpi ();
    SPI0->RSER |= SPI_RSER_EOQF_RE_MASK;
    NVIC_EnableIRQ (SPI0_IRQn);
    buffer [1023] = 0x18010000|color;
    //fillBuffer();
    memtomem ((uint32_t)&currentColor, (uint32_t)buffer);
    
    //memSample ((uint32_t)source, (uint32_t)dest);
    //memSample ((uint32_t)&currentColor, (uint32_t)buffer);
    memtoSpi ((uint32_t)buffer);
    trg.start();
    //trg.start();
    //
	
    
    uint16_t t = 254;
	while (1){        
	}
}

void memtomem (uint32_t s, uint32_t d){
    //DMA0->CR |= DMA_CR_EMLM_MASK;
#ifdef SIMPLE
    DMA0->TCD[0].SADDR = s;
	DMA0->TCD[0].DADDR = d;
    DMA0->TCD[0].ATTR = DMA_ATTR_DSIZE(0x02)|DMA_ATTR_SSIZE(0x02);
    DMA0->TCD[0].SOFF = DMA_SOFF_SOFF(0x04);
    DMA0->TCD[0].DOFF = DMA_DOFF_DOFF(0x04);
    DMA0->TCD[0].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(24);
    DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(1);
	DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(1);
    DMA0->TCD[0].CSR = 0;
    DMA0->TCD[0].CSR |= DMA_CSR_START_MASK;
#else
    m2m.enableDmaMux(dmaMux::dma0);
    //m2m.triggerEnable();
    m2m.setSource (s);
    m2m.setDestination(d);
    m2m.setDsize (Dma::size::bit32);
    m2m.setSsize (Dma::size::bit32);
    m2m.setOffsetSource (0);
    m2m.setOffsetDestination (4);
    m2m.setCountMnrLoop (4);
    /*DMA0->TCD[0].CITER_ELINKYES = DMA_CITER_ELINKYES_ELINK_MASK|DMA_CITER_ELINKYES_LINKCH(1)|DMA_CITER_ELINKYES_CITER(20);
	DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKYES_ELINK_MASK|DMA_BITER_ELINKYES_LINKCH(1)|DMA_BITER_ELINKYES_BITER(20);
    */
    m2m.setCountMjrLoop (1023);
    /*m2m.setSLast (-80);
    m2m.setDLast (-80);
    */
    //DMA0->TCD[0].CSR |= DMA_CSR_MAJORELINK_MASK|DMA_CSR_MAJORLINKCH(1);
    //m2m.start();
    m2m.setDREQ();
    m2m.startRequest();
#endif    
   
}
void memSample(uint32_t s, uint32_t d){

    mem2mem.setSource (s);
    mem2mem.setDestination(d);
    mem2mem.setDsize (Dma::size::bit32);
    mem2mem.setSsize (Dma::size::bit32);
    mem2mem.setOffsetSource (0);
    mem2mem.setOffsetDestination (4);
    mem2mem.setCountMnrLoop (4096);
    mem2mem.setCountMjrLoop (1);
    //mem2mem.setMjrLink (m2spi);
    mem2mem.setSLast (-4096);
    mem2mem.start();
    /*mem2mem.start();
    mem2mem.start();*/
}

void memtoSpi (uint32_t s){
    pinSpiInit ();
  
    spiDma.moduleDisable();
    spiDma.txFifoEnable();
    spiDma.clearTcf();
    spiDma.clearEOQF();
    spiDma.clearTfuf();
    spiDma.clearTfff();
    spiDma.moduleEnable();
    spiDma.enableTransmiteDmaRequest();
    spiDma.setFrameSize(Spi::fSize::bit_8);
    //spiDma.setBaudrate(Spi::division::div4);
    
    m2spi.enableDmaMux(dmaMux::spi0Tx);
    m2spi.triggerEnable();
    m2spi.setSource (s);
    m2spi.setOffsetSource(4);
    m2spi.setDestination ((uint32_t)&SPI0->PUSHR); 
    m2spi.setOffsetDestination (0);
    m2spi.setDsize (Dma::size::bit32);
    m2spi.setSsize(Dma::size::bit32);
    m2spi.setDREQ();
    m2spi.setCountMnrLoop(4);
    m2spi.setCountMjrLoop(1);
    m2spi.setSLast (-4);
    
    //m2spi.setCountMjrLoop(1024);
    //m2spi.setSLast(-4096);
    m2spi.startRequest();
}

void initSpi (){
    pinSpiInit ();
    spi0.moduleDisable();
    spi0.clearTxFifo();
    spi0.setCtar(Spi::ctarNumber::ctar1);
    spi0.setBaudrate(Spi::scaler::div4);
    spi0.clearTcf();
    spi0.clearEOQF();
    spi0.moduleEnable();
    uint16_t cw = spi0.getSpiCommandWord();
    for(uint8_t i=0;i<10;++i){
        spi0.transmit(i);
    }
    
}
void pinSpiInit (){
	//set pin
    Pin cs (port::E, 16, Gpio::mux::Alt2);
    Pin sck (port::E, 17, Gpio::mux::Alt2);
    Pin mosi (port::E, 18, Gpio::mux::Alt2);
    Pin miso (port::E, 19, Gpio::mux::Alt2);
}
void fillBuffer(){
	for (uint8_t i=0;i<20;++i){
		buffer[i] = SPI_PUSHR_CTAS(0)|SPI_PUSHR_PCS(1)|SPI_PUSHR_TXDATA(i)
        //| SPI_PUSHR_CTCNT_MASK
        ;

	}
    buffer[19] = SPI_PUSHR_CTAS(0)|SPI_PUSHR_PCS(1)|SPI_PUSHR_TXDATA(30)|SPI_PUSHR_EOQ_MASK
    //| SPI_PUSHR_CTCNT_MASK
    ;
	
}
void sendDMA () 
{	
    //DMA0->SERQ = DMA_SERQ_SERQ(1);	
    m2spi.startRequest();
    
	while(!(SPI0->SR & SPI_SR_EOQF_MASK));//wait till the last entry has been transmitted

	SPI0->SR  |= SPI_SR_EOQF_MASK; //clear EOQF flag	
}
