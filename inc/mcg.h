#include "device.h"             // Device header

#ifndef TACT_H
#define TACT_H


class Tact
{
  //variables
public:
  enum divider {div1, div2, div4, div8, div16, div32, div64, div128};

 private:
  static uint16_t cpuClock;
  static uint16_t busClock;
  static uint16_t mcgirClock;
  static uint16_t mcgpllClock;
  static uint16_t mcgfllClock;
  uint8_t src;
  //functions
public:
  Tact ();
  static uint16_t & getFrqCpu (){return cpuClock;};
  static uint16_t & getFrqBus (){return busClock;};
  static uint16_t & getFrqMcgir (){return mcgirClock;};
  static uint16_t & getFrqMcgpll (){return mcgirClock;};
  static uint16_t & getFrqMcgfll (){return mcgirClock;};

private:

  void initFei ();
  void initFee ();
  void initFbi ();
  void initFbe ();
  void initPee ();

  //===ext_OSC===//

};

 
#endif

