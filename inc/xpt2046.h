#include "device.h"
#include "pin.h"
#include "spi.h"
#include "intrpt.h"
//#include "delay.h"


#ifndef XPT2046_H
#define XPT2046_H

const uint8_t channelY = 0x90;
const uint8_t channelX = 0xD0;


class Xpt2046
{
private:
    Spi * spiDriver;
    Intrpt * irq;
	uint16_t x, y;
	uint16_t Xmin, Ymin, dX, dY;
	uint8_t ptrF;

public:

    Xpt2046 (Spi &, Intrpt & irq_);
    void getData ();
    void getDataSpi ();
    void getDataSoft ();
    uint16_t & getX ();
    uint16_t & getY ();
    uint16_t & getdX ();
    uint16_t & getdY ();

    void clearFlag ();
private:
    void init ();

};


#endif
