#include "spi.h"

Spi::Spi(numberSpi nSpi, ctarNumber n, csNumber cs, role r){
	command[0].bits.ctas = 0;
	command[1].bits.ctas = 1;
	uint8_t nSpi_ = static_cast<uint8_t>(nSpi);
	spiPtr = ((SPI_Type *)spiAddress[nSpi_]);
	//Turn on tacting
	SIM->SCGC6 |= 1 << spiClockShift[nSpi_];
	setPcs(static_cast<uint8_t>(cs), n);
	txFifoDisable();
	rxFifoDisable();
	//set ctar number
	//setCtar(n);
	//Settings role
	spiPtr->MCR |= (static_cast<uint8_t>(r) << SPI_MCR_MSTR_SHIFT)|SPI_MCR_DIS_TXF_MASK|SPI_MCR_DIS_RXF_MASK|SPI_MCR_PCSIS(1<<static_cast<uint8_t>(cs));
	//Start
	spiPtr->SR |= SPI_SR_EOQF_MASK|SPI_SR_TCF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TFUF_MASK | SPI_SR_TFFF_MASK | SPI_SR_RFOF_MASK | SPI_SR_RFDF_MASK | SPI_SR_TXCTR(0) | SPI_SR_TXNXTPTR(0) | SPI_SR_RXCTR(0) | SPI_SR_POPNXTPTR(0);	
	spiPtr->MCR &= ~(SPI_MCR_HALT_MASK|SPI_MCR_FRZ_MASK);
	moduleEnable();
}

void Spi::setCpol (ctarNumber n, cpol c){
    spiPtr->CTAR[static_cast<uint8_t>(n)] &= ~ SPI_CTAR_CPOL_MASK;
	spiPtr->CTAR[static_cast<uint8_t>(n)] |= (static_cast<uint8_t>(c)) << SPI_CTAR_CPOL_SHIFT;
}

void Spi::setCpha (ctarNumber n, cpha c)
{
    spiPtr->CTAR[static_cast<uint8_t>(n)] &= ~SPI_CTAR_CPHA_MASK;
    spiPtr->CTAR[static_cast<uint8_t>(n)] |= (static_cast<uint8_t>(c)) << SPI_CTAR_CPHA_SHIFT;
}

void Spi::setFrameSize (ctarNumber n, fSize f)
{
    spiPtr->CTAR[static_cast<uint8_t>(n)] &= ~SPI_CTAR_FMSZ_MASK;
	spiPtr->CTAR[static_cast<uint8_t>(n)] |= (static_cast<uint8_t>(f)) << SPI_CTAR_FMSZ_SHIFT;
}

void Spi::setBaudrate (ctarNumber n, scaler s, prescaler p){
    spiPtr->CTAR[static_cast<uint8_t>(n)] &= ~(SPI_CTAR_BR_MASK|SPI_CTAR_PBR_MASK);
	spiPtr->CTAR[static_cast<uint8_t>(n)] |= SPI_CTAR_BR(static_cast<uint8_t>(s))|SPI_CTAR_PBR(static_cast<uint8_t>(p));
}
void Spi::doubleBrEnable (ctarNumber n){
    spiPtr->CTAR[static_cast<uint8_t>(n)] |= SPI_CTAR_DBR_MASK;
}
void Spi::doubleBrDisable (ctarNumber n){
    spiPtr->CTAR[static_cast<uint8_t>(n)] &= ~SPI_CTAR_DBR_MASK;
}
void Spi::setDelayAfterTransfer(scaler s, prescaler p, ctarNumber n){
    spiPtr->CTAR[static_cast<uint8_t>(n)] &= ~(SPI_CTAR_PDT_MASK|SPI_CTAR_DT_MASK);
    spiPtr->CTAR[static_cast<uint8_t>(n)] |= SPI_CTAR_DT(static_cast<uint8_t>(s))|SPI_CTAR_PDT(static_cast<uint8_t>(p));
}


void Spi::txFifoEnable(){
	moduleDisable();
	spiPtr->MCR &=~ SPI_MCR_DIS_TXF_MASK;
	moduleEnable();
}
void Spi::txFifoDisable(){
	moduleDisable();
	spiPtr->MCR |= SPI_MCR_DIS_TXF_MASK;
	moduleEnable();
}
void Spi::clearTxFifo(){
    spiPtr->MCR |= SPI_MCR_CLR_TXF_MASK;
}
void Spi::rxFifoEnable(){
	moduleDisable();
	spiPtr->MCR &=~ SPI_MCR_DIS_RXF_MASK;
	moduleEnable();
}

void Spi::rxFifoDisable(){
	moduleDisable();
	spiPtr->MCR |= SPI_MCR_DIS_RXF_MASK;
	moduleEnable();
}
void Spi::clearRxFifo(){
    spiPtr->MCR |= SPI_MCR_CLR_RXF_MASK;
}
void Spi::moduleDisable(){
    spiPtr->MCR |= SPI_MCR_HALT_MASK;
    SPI0->MCR &=~ SPI_MCR_MDIS_MASK;
}
void Spi::moduleEnable(){
    spiPtr->MCR &=~ SPI_MCR_HALT_MASK;
    //SPI0->MCR |= SPI_MCR_MDIS_MASK;
}

void Spi::transmit (uint16_t data, ctarNumber n){
    while (!flagTfff());
	spiPtr->PUSHR = (command[static_cast<uint8_t>(n)].word<<16)|data;
	while (!flagTcf ()){
		asm("NOP");
	};
    clearTcf();
}

void Spi::transmit (const uint16_t * data, uint32_t n, ctarNumber num){
	uint16_t comm = command[static_cast<uint8_t>(num)].word;
    for (uint8_t i=0;i<n;++i){
        while (!flagTfff());
        spiPtr->PUSHR = (comm<<16)|*data;
        while (!flagTcf ());
        clearTcf();
    }
}

uint16_t Spi::receive (){
	while (!flagRfof());
	return spiPtr->POPR;	
}

uint16_t Spi::exchange (uint16_t data, ctarNumber num){
    transmit(data, num);
	return receive();
}

uint16_t Spi::get_data (){
	return SPI0->POPR;
}

void Spi::enableTransmiteDmaRequest(){
    SPI0->RSER |= SPI_RSER_TFFF_RE_MASK|SPI_RSER_TFFF_DIRS_MASK;
}

void Spi::disableTransmiteDmaRequest(){
    SPI0->RSER &= ~(SPI_RSER_TFFF_RE_MASK|SPI_RSER_TFFF_DIRS_MASK);
}
void Spi::enableReceiveDmaRequest(){
    SPI0->RSER |= SPI_RSER_RFDF_RE_MASK|SPI_RSER_RFDF_DIRS_MASK;
}

void Spi::disableReceiveDmaRequest(){
    SPI0->RSER &= ~(SPI_RSER_RFDF_RE_MASK|SPI_RSER_RFDF_DIRS_MASK);
}

void Spi::setContinuousMode(bool state, ctarNumber num){
    command[static_cast<uint8_t>(num)].bits.cont = state;
}
void Spi::EOFEnable(){
    
}

void Spi::EOFDisable(){
    
}

void Spi::setPcs (uint8_t n, ctarNumber num){
    spiPtr->MCR &= ~ SPI_MCR_PCSIS_MASK;
    spiPtr->MCR |= SPI_MCR_PCSIS(1 << n);
    command[static_cast<uint8_t>(num)].bits.pcs = 1 << n;
}

bool Spi::flagTcf (){
	return spiPtr->SR&SPI_SR_TCF_MASK;
}

bool Spi::flagTfff (){
	return spiPtr->SR&SPI_SR_TFFF_MASK;
}

bool Spi::flagTfuf (){
	return spiPtr->SR&SPI_SR_TFUF_MASK;
}

bool Spi::flagTxctr (){
	return spiPtr->SR&SPI_SR_TXCTR_MASK;
}

bool Spi::flagRfof (){
	return spiPtr->SR&SPI_SR_RFOF_MASK;
}

bool Spi::flagRfdf (){
	return spiPtr->SR&SPI_SR_RFDF_MASK;
}

bool Spi::flagEOQ (){
    return spiPtr->SR&SPI_SR_EOQF_MASK;
}

void Spi::clearTcf(){
    spiPtr->SR |= SPI_SR_TCF_MASK;
}

void Spi::clearTfuf(){
    spiPtr->SR |= SPI_SR_TFUF_MASK;
}

void Spi::clearTfff(){ 
    spiPtr->SR |= SPI_SR_TFFF_MASK;
}
void Spi::clearEOQF(){
    spiPtr->SR |= SPI_SR_EOQF_MASK;
}
void Spi::clearRfof(){
    spiPtr->SR |= SPI_SR_RFOF_MASK;
}

void Spi::clearRfdf(){
    spiPtr->SR |= SPI_SR_RFDF_MASK;
}
void Spi::clearAllFlag(){
    spiPtr->SR |= SPI_SR_RFDF_MASK|SPI_SR_RFOF_MASK|SPI_SR_EOQF_MASK|SPI_SR_TFFF_MASK|SPI_SR_TFUF_MASK|SPI_SR_TCF_MASK;
}
uint16_t Spi::getSpiCommandWord(ctarNumber n){
    return command[static_cast<uint8_t>(n)].word;
}
