#include "i8080.h"



I8080::I8080()
//settings port
:high (i8080def::highDataPort, i8080def::highDataPins), low (i8080def::lowDataPort, i8080def::lowDataPins),
 rs (i8080def::rsPort, i8080def::rsPin),
 rst (i8080def::rstPort, i8080def::rstPin),
 wr (i8080def::wrPort, i8080def::wrPin),
 rd (i8080def::rdPort, i8080def::rdPin),
 cs (i8080def::csPort, i8080def::csPin)
{
	rd.set();
	cs.set();
	wr.set();
	rst.clear();
}


void I8080::index(uint16_t val)
{
	cs.clear();
	//command
	rs.clear();
	low.clear();
	high.clear();
	low.set(val&0x00FF);
	high.set((val&0xFF00)<<8);
	wr.clear();
	wr.set();
	cs.set();
}

void I8080::data(uint16_t val)
{
	cs.clear();
	//data
	rs.set();
	low.clear();
	high.clear();
	low.set(val&0x00FF);
	high.set((val&0xFF00)<<8);
	wr.clear();
	wr.set();
	cs.set();
}

void I8080::putData (uint16_t val)
{
	low.clear();
	high.clear();
	low.set(val&0x00FF);
	high.set((val&0xFF00)<<8);
	wr.clear();
	wr.set();
}

void I8080::wReg (uint16_t indx , uint16_t dta)
{
	cs.clear();
	//command
	rs.clear();
	low.clear();
	high.clear();
	low.set(indx&0x00FF);
	high.set((indx&0xFF00)<<8);
	wr.clear();
	wr.set();
	//data
	rs.set();
	low.clear();
	high.clear();
	low.set(dta&0x00FF);
	high.set((dta&0xFF00)<<8);
	wr.clear();
	wr.set();
	cs.set();
}

void I8080::strob ()
{
	wr.clear();
	wr.set();
}





