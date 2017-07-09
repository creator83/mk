#include "i2c.h"

I2C_Type * I2c::i2cAdr [2] = {I2C0, I2C1};

I2c::I2c (nI2c n)
{
	numberI2c = static_cast <uint8_t> (n);
	SIM->SCGC4 |= 1 << (SIM_SCGC4_I2C0_SHIFT + numberI2c);
	uint8_t dummy = getData();
	//400kHz
	//i2cAdr [numberI2c]->F = 0x13;
	//100kHz
	i2cAdr [numberI2c]->F = 0x1F;
	i2cAdr [numberI2c]->C2 &= ~ (I2C_C2_HDRS_MASK|I2C_C2_ADEXT_MASK);
	i2cAdr [numberI2c]->C1 =  I2C_C1_IICEN_MASK;
}

void I2c::start ()
{
	i2cAdr [numberI2c]->C1 |= I2C_C1_TX_MASK;
	i2cAdr [numberI2c]->C1 |= I2C_C1_MST_MASK;
}

void I2c::restart ()
{
	i2cAdr [numberI2c]->C1 |= I2C_C1_RSTA_MASK;
}

void I2c::stop ()
{
	i2cAdr [numberI2c]->C1 &= ~I2C_C1_MST_MASK;
	i2cAdr [numberI2c]->C1 &= ~I2C_C1_TX_MASK;
}

void I2c::putData (uint8_t val)
{
	i2cAdr [numberI2c]->D = val;
}

uint8_t I2c::getData ()
{
	return i2cAdr [numberI2c]->D;
}

void I2c::setAddress (uint8_t addr, directionBit d)
{
	if ((uint8_t) d) putData(addr|0x01);
	else putData(addr&0xFE);
}

void I2c::write (uint8_t val)
{
	while (!flagTcf());
	i2cAdr [numberI2c]->C1 |= I2C_C1_TX_MASK;
	putData(val);
	while (!flagIicif());
	i2cAdr [numberI2c]->S |= I2C_S_IICIF_MASK;
}

uint8_t I2c::read (bool ack)
{
	while (!flagTcf());
	i2cAdr [numberI2c]->C1 &= ~(I2C_C1_TX_MASK| I2C_C1_TXAK_MASK);
	i2cAdr [numberI2c]->C1 |= ack << I2C_C1_TXAK_SHIFT;
	uint8_t data = getData();
	while (!flagIicif());
	i2cAdr [numberI2c]->S |= I2C_S_IICIF_MASK;
	return data;
}

void I2c::read (uint8_t *data, uint8_t addr, uint8_t reg, uint16_t n)
{
	uint8_t count = n-1;
	start ();
	while (!flagBusy());
	setAddress (addr, directionBit::write);
	waitAck();
	putData(reg);
	waitAck();
	restart();
	setAddress (addr, directionBit::read);
	waitAck();
	setMode (mode::receiver);
	generateAck();
	while (count--)
	{
		*data++ = getData();
		waitAck();
	}
	generateNack ();
	*data = getData();
	waitAck();
	stop ();
}

bool I2c::flagBusy ()
{
	return i2cAdr [numberI2c]->S & I2C_S_BUSY_MASK ;
}

bool I2c::flagTcf ()
{
	return i2cAdr [numberI2c]->S & I2C_S_TCF_MASK;
}

bool I2c::flagIicif ()
{
	return i2cAdr [numberI2c]->S & I2C_S_IICIF_MASK;
}

bool I2c::flagRxak ()
{
	return i2cAdr [numberI2c]->S & I2C_S_RXAK_MASK;
}

void I2c::clearIicif ()
{
	i2cAdr [numberI2c]->S |= I2C_S_IICIF_MASK;
}

void I2c::wByte (uint8_t addr, uint8_t reg, uint8_t d)
{
	start ();
	while (!flagBusy());
	//address
	setAddress (addr, directionBit::write);
	waitAck();
	putData(reg);
	waitAck();
	putData(d);
	waitAck();
	stop ();
	while (flagBusy());
}

uint8_t I2c::rByte (uint8_t addr, uint8_t reg)
{
	start ();
	while (!flagBusy());
	setAddress (addr, directionBit::write);
	waitAck();
	putData(reg);
	waitAck();
	restart();
	setAddress (addr, directionBit::read);
	waitAck();
	setMode (mode::receiver);
	uint8_t dummy = getData();
	waitAck();
	generateNack ();
	stop ();
	uint8_t result = getData();
	while (flagBusy());
	return result;

}


void I2c::generateAck ()
{
	i2cAdr [numberI2c]->C1 &= ~I2C_C1_TXAK_MASK ;
}

void I2c::generateNack ()
{
	i2cAdr [numberI2c]->C1 |= I2C_C1_TXAK_MASK ;
}

bool I2c::waitAck ()
{
	bool ret= false;
	while (!flagIicif())
	{
		if (flagRxak()) ret = true;
	}
	clearIicif ();
	return ret;
}

void I2c::setMode (mode m)
{
	i2cAdr [numberI2c]->C1 &= ~I2C_C1_TX_MASK;
	i2cAdr [numberI2c]->C1 |= static_cast <uint8_t>(m) << I2C_C1_TX_SHIFT ;
}

