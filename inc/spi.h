#include "device.h"
#include "pin.h"
#include "spi_dev.h"

#ifndef SPI_H
#define SPI_H


//typedef (Ctar_set*) Spi::setptr;



class Spi
{


//variables
public:
  enum class division :uint8_t {div2 , div4 , div8 , div16 , div32 , div64 , div128 , div256, div512};
  enum class role :uint8_t {slave , master};
  enum class cpol : uint8_t {neg, pos};
  enum class cpha : uint8_t {first, second};
  enum class csNumber : uint8_t {cs0, cs1, cs2, cs3, cs4};
  enum class ctarNumber : uint8_t {ctar0, ctar1};
  enum class state : bool {off, on};
  enum class fSize {bit_4=3, bit_5, bit_6, bit_7, bit_8, bit_9, bit_10, bit_11, bit_12, bit_13, bit_14, bit_15, bit_16};

private:
protected:
  uint8_t ctar_;
  SPI_Type * spiPtr;
union ctarDef
  {
   uint32_t set;
   struct {
    unsigned br:4;
    unsigned dt:4;
    unsigned asc:4;
    unsigned cssk:4;
    unsigned pbr:2;
    unsigned pdt:2;
    unsigned pasc:2;
    unsigned pcssck:2;
    unsigned lsbfe:1;
    unsigned cpha:1;
    unsigned cpol:1;
    unsigned fmsz:4;
    unsigned dbr:1;
   }ctarBits;
  }ctar;
  union command {
   uint16_t set;
   struct {
    unsigned pcs:6;
    unsigned dummy:4;
    unsigned ctcnt:1;
    unsigned eoq:1;
    unsigned ctas:3;
    unsigned cont:1;
   }txCommandBits;
  }txCommand;
  uint8_t nCtar;
//functions
public:
	//constructor for Spi0
Spi(numberSpi nSpi, ctarNumber nCtar, csNumber cs, role r=role::master);

	//constructor for all module

  void setCpol (cpol c = cpol::neg);
  void setCpha (cpha c = cpha::first);
  void setFrameSize (fSize f = fSize::bit_8);
  void setBaudrate (division d);
  void setCtar (ctarNumber);
  void updateCtar ();

  void settings ();
  void transmit (uint16_t data);
  uint8_t receive ();
  uint16_t exchange (uint16_t data);

  //void putData (uint16_t data, CS_number, CTAR_number, State cont = State::off);

  uint16_t get_data ();
  bool flag_tcf ();
  bool flag_tfff ();
  bool flag_tfuf ();
  bool flag_txctr ();
  bool flag_rfof ();
  bool flag_rfdf ();
  void clear_flag_tcf();
  void clear_flag_tfuf();
  void clear_flag_rfof();
  void clear_flag_rfdf();


};

typedef uint16_t(Spi::*PotMemFn)() ;
typedef uint16_t(Spi::*ptr_ex)(uint16_t) ;

typedef void(Spi::*PotMemF)(uint16_t) ;

#endif

