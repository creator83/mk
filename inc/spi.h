#include "device.h"
#include "pin.h"

#ifndef SPI_H
#define SPI_H


//typedef (Ctar_set*) Spi::setptr;



class Spi
{


//variables
public:
    enum class scaler: uint8_t {div2, div4, div8, div16, div32, div64, div128, div256, div512,
                                div1024, div2048, div4096, div8192, div16384, div32768, div65536};
    enum class prescaler: uint8_t {div1, div3, div5, div7};   
    enum class division :uint8_t {div2 , div4 , div8 , div16 , div32 , div64 , div128 , div256, div512};
    enum class role :uint8_t {slave , master};
    enum class cpol : uint8_t {neg, pos};
    enum class cpha : uint8_t {first, second};
    enum class csNumber : uint8_t {cs0, cs1, cs2, cs3, cs4};
    enum class ctarNumber : uint8_t {ctar0, ctar1};
	enum class ctasNumber : uint8_t {ctas0, ctas1};
    enum class state : bool {off, on};
    enum class fSize {bit_4=3, bit_5, bit_6, bit_7, bit_8, bit_9, bit_10, bit_11, bit_12, bit_13, bit_14, bit_15, bit_16};

protected:
  SPI_Type * spiPtr;
  uint8_t nCtar;
  union commandWord{
	  uint16_t word;
	  struct bitField{
	  unsigned pcs: 6;
	  unsigned dump: 4;
	  unsigned ctcnt : 1;
	  unsigned eoq : 1;
	  unsigned ctas : 3;
	  unsigned cont : 1;
	  }bits;
  }command[2];
//functions
public:
	//constructor for Spi0
	Spi(numberSpi nSpi, ctarNumber, csNumber cs, role r=role::master);

	//constructor for all module

	void setCpol (ctarNumber, cpol c = cpol::neg);
	void setCpha (ctarNumber, cpha c = cpha::first );
	void setFrameSize (ctarNumber, fSize f = fSize::bit_8);
    void setBaudrate (ctarNumber, scaler, prescaler = prescaler::div1);
    void doubleBrEnable (ctarNumber);
    void doubleBrDisable (ctarNumber);
	void setCtas (ctarNumber, ctasNumber);
    void setDelayAfterTransfer(scaler, prescaler, ctarNumber);
    
	void txFifoEnable();
	void txFifoDisable();
    void clearTxFifo();
	void rxFifoEnable();
	void rxFifoDisable();
    void clearRxFifo();
	void moduleDisable();
	void moduleEnable();


	void transmit (uint16_t data, ctarNumber);
	void transmit (const uint16_t * data, uint32_t n, ctarNumber);
	uint16_t receive ();
	uint16_t exchange (uint16_t data, ctarNumber num);
    uint16_t get_data ();
    void enableTransmiteDmaRequest();
    void disableTransmiteDmaRequest();
    void enableReceiveDmaRequest();
    void disableReceiveDmaRequest();
    void setContinuousMode(bool state,ctarNumber num);
    void EOFEnable();
    void EOFDisable();

	bool flagTcf ();
	bool flagTfff ();
	bool flagTfuf ();
	bool flagTxctr ();
	bool flagRfof ();
	bool flagRfdf ();
    bool flagEOQ ();
	void clearTcf();
	void clearTfuf();
    void clearTfff();
    void clearEOQF();
	void clearRfof();
	void clearRfdf();
    void clearAllFlag();
    uint16_t getSpiCommandWord(ctarNumber);
    void setContinuousMode(bool state);
    void setEOQMode(bool state);
	//void putData (uint16_t data, CS_number, CTAR_number, State cont = State::off);
protected:
    void setPcs (uint8_t, ctarNumber);

};

typedef uint16_t(Spi::*PotMemFn)() ;
typedef uint16_t(Spi::*ptr_ex)(uint16_t) ;

typedef void(Spi::*PotMemF)(uint16_t) ;

#endif

