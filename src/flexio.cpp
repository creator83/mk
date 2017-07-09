#include "flexio.h"

Flexio::Flexio(interface i, nBuffer b) {

	SIM->SOPT2 |= SIM_SOPT2_FLEXIOSRC(0x01);
	SIM->SCGC5 |= SIM_SCGC5_FLEXIO_MASK;

	//setInterface (i, b);
	setSpi(b);
}

void Flexio::setInterface (interface i, nBuffer b) {

}

void Flexio::putData (uint32_t data) {
	FLEXIO->SHIFTBUF [sBuffer] = data;
}

uint32_t Flexio::getData () {
	return FLEXIO->SHIFTBUF [sBuffer+1];
}

uint32_t Flexio::transmite (uint32_t data) {
	putData(data);
	//while (!statusFlag(sBuffer));
	/*while (!statusFlag(sBuffer+1));
	return getData();*/
}

uint32_t Flexio::receive () {

}

uint32_t Flexio::exchange (uint32_t data) {

}

bool Flexio::statusFlag (uint8_t s) {
	return FLEXIO->SHIFTSTAT&(s << FLEXIO_SHIFTSTAT_SSF_SHIFT);
}

void Flexio::setSpi(nBuffer b) {
	sBuffer = static_cast <uint8_t>(b);
	//settings mosi
	FLEXIO->SHIFTCFG[sBuffer] &= ~(FLEXIO_SHIFTCFG_SSTART_MASK|FLEXIO_SHIFTCFG_SSTOP_MASK|FLEXIO_SHIFTCFG_INSRC_MASK);
	FLEXIO->SHIFTCTL [sBuffer] = FLEXIO_SHIFTCTL_PINSEL(0)|FLEXIO_SHIFTCTL_TIMPOL_MASK|FLEXIO_SHIFTCTL_PINCFG_MASK|FLEXIO_SHIFTCTL_SMOD(2);

	//settings miso
	FLEXIO->SHIFTCFG[sBuffer+1] = 0;
	FLEXIO->SHIFTCTL [sBuffer+1] = FLEXIO_SHIFTCTL_PINSEL(1)| FLEXIO_SHIFTCTL_SMOD(1);

	//settings baudrate
	FLEXIO->TIMCMP[sBuffer] = 0x0F3F;
	FLEXIO->TIMCFG[sBuffer] = FLEXIO_TIMCFG_TIMOUT(1)|FLEXIO_TIMCFG_TIMDIS(2)|FLEXIO_TIMCFG_TIMENA(2)|FLEXIO_TIMCFG_TSTOP(2)|FLEXIO_TIMCFG_TSTART_MASK;

	//settings sck
	FLEXIO->TIMCTL[sBuffer] = FLEXIO_TIMCTL_TRGSEL(1)|FLEXIO_TIMCTL_TRGPOL_MASK|FLEXIO_TIMCTL_TRGSRC_MASK|FLEXIO_TIMCTL_PINCFG(3)|FLEXIO_TIMCTL_PINSEL(2)|FLEXIO_TIMCTL_TIMOD(1);
	FLEXIO->TIMCMP[sBuffer+1] = 0xFFFF;
	FLEXIO->TIMCFG[sBuffer+1] = FLEXIO_TIMCFG_TIMDIS(1)|FLEXIO_TIMCFG_TIMENA(1);

	//settings cs
	FLEXIO->TIMCTL[sBuffer+1] = FLEXIO_TIMCTL_PINCFG(3)|FLEXIO_TIMCTL_PINSEL(3)|FLEXIO_TIMCTL_PINPOL_MASK|FLEXIO_TIMCTL_TIMOD(3);
}

void Flexio::setI2c (nBuffer b) {

}
