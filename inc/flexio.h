#include "device.h"

#ifndef FLEXIO_H_
#define FLEXIO_H_

class Flexio {
public:
	enum class interface {spi, i2c};
	enum class nBuffer {buffer0, buffer1, buffer2, buffer3};
	enum class nPin {fxIo0, fxIo1, fxIo2, fxIo3, fxIo4, fxIo5, fxIo6, fxIo7};
	enum class spiDivider {div4=4, div8=8, div16=16, div32=32, div64=64, div128=128};
	enum class clockSource {off, irc48m, oscerclk, mcgirclk};
private:
	uint8_t sBuffer;
public:
	Flexio(interface i, nBuffer b);
	void setInterface (interface i, nBuffer b);
	void putData (uint32_t data);
	uint32_t getData ();
	uint32_t transmite (uint32_t data);
	uint32_t receive ();
	uint32_t exchange (uint32_t data);
	bool statusFlag (uint8_t);
private:
	void setSpi(nBuffer b);
	void setI2c (nBuffer b);
};

#endif /* FLEXIO_H_ */
