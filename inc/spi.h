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
  enum class Division :uint8_t {div2 , div4 , div8 , div16 , div32 , div64 , div128 , div256, div512};
  enum class Role :uint8_t {slave , master};
  enum class Cpol : uint8_t {neg, pos};
  enum class Cpha : uint8_t {first, second};
  enum class CS_number : uint8_t {CS0, CS1, CS2, CS3, CS4};
  enum class CTAR_number : uint8_t {CTAR0, CTAR1};
  enum class State : bool {off, on};
  enum class Fsize {bit_4=3, bit_5, bit_6, bit_7, bit_8, bit_9, bit_10, bit_11, bit_12, bit_13, bit_14, bit_15, bit_16};

private:
protected:
  uint8_t ctar_;

static struct Ctar_set
  {
	  uint8_t cpol;
	  uint8_t cpha;
	  uint8_t f_size;
	  uint8_t lsbfe;
	  uint8_t dbr;
	  uint8_t pcssck;
	  uint8_t pask;
	  uint8_t pdt;
	  uint8_t pbr;
	  uint8_t cssck;
	  uint8_t asc;
	  uint8_t dt;
	  uint8_t br;
  }C0, C1;
  using ctarPtr = Spi::Ctar_set*;
  Pin cs, sck, mosi, miso;
  static ctarPtr s_ctar [2];

//functions
public:
	//constructor for Spi0
  Spi(Role r=Role::master);

	//constructor for all module
	Spi();

  void set_cpol (Cpol c = Cpol::neg);
  void set_cpha (Cpha c = Cpha::first);
  void set_f_size (Fsize f = Fsize::bit_8);
  void set_baudrate (Division d);
  void set_ctar (CTAR_number);
  void update_ctar ();

  static void set_cpol (Spi &, Cpol c);
  static void set_cpha (Spi &, Cpha c);
  static void set_ctar (Spi &, CTAR_number c);
  static void set_baudrate (Spi &, Division d);
  static void set_f_size (Spi &, Fsize f = Fsize::bit_8);

  void settings ();
  void transmit (uint16_t data);
  uint8_t receive ();
  uint8_t exchange (uint8_t data);

  void put_data (uint16_t data, CS_number, CTAR_number, State cont = State::off);
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

