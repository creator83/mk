#include "device.h"                // Device header

#ifndef TACT_H
#define TACT_H


class Tact
{
  //variables
public:
  enum mode {LIRC_2, LIRC_8, HIRC, EXT};
  enum divider {div1, div2, div4, div8, div16, div32, div64, div128};

 private:
  static uint16_t cpuClock;
  static uint16_t busClock;
  static uint16_t mcgirClock;
  uint8_t src;
  //functions
public:
  Tact ();
  Tact (mode m, uint8_t frq);
  static uint16_t & getFrqCpu (){return cpuClock;};
  static uint16_t & getFrqBus (){return busClock;};
  static uint16_t & getFrqMcgir (){return mcgirClock;};
  void get_LIRC_8 (divider div_1=div1, divider div_2=div1);

  void get_LIRC_2 (divider div_1=div1, divider div_2=div1);

  void set_LIRC_div (mode m, divider div_1, divider div_2);

private:

  void initHIRC ();

  //===ext_OSC===//

  void get_EXT (uint8_t i);

  void Set_frq (uint8_t frq);

};

 
#endif

