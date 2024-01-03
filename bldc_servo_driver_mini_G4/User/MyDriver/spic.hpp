#ifndef SPIC_HPP_
#define SPIC_HPP_

#include "main.h"
#include "timcnt.hpp"

class SPIC {
public:
  SPIC(SPI_TypeDef *_spi) : spi_(_spi) {};

  void init();

  void send_bytes(uint16_t *_tbuf, uint16_t *_rbuf, uint16_t _len);

protected:
  virtual void select()   = 0;
  virtual void deselect() = 0;

  const SPI_TypeDef *spi_;
};


class SoftSPIC{
public:
  struct PinName{
    GPIO_TypeDef* _gpio;
    uint32_t      _pin;
  };

  struct Config {
    PinName _mosi;
    PinName _miso;
    PinName _sck;
    PinName _cs;
    int _freq_hz;
    int _width;
    TIMUSCNT* _us_cnt;
  };

  SoftSPIC(const Config& _conf) : c_(_conf) {
    
  }

  void init(){
    LL_GPIO_SetOutputPin(c_._cs._gpio,   c_._cs._pin);
    LL_GPIO_ResetOutputPin(c_._sck._gpio,  c_._sck._pin);
    LL_GPIO_ResetOutputPin(c_._mosi._gpio, c_._mosi._pin);
    us_delay_ = 1000000 / (c_._freq_hz * 2);
    us_delay_ = (us_delay_ <= 0) ? 1 : us_delay_;
  }

  uint16_t transmit(uint16_t _w_d){
    LL_GPIO_ResetOutputPin(c_._cs._gpio,   c_._cs._pin);
    c_._us_cnt->wait_us(us_delay_);

    uint16_t res = 0;

    for(int i=c_._width; i>0; i--){
      // MOSIのbit設定
      if(_w_d & (1 << (i-1))){
        LL_GPIO_SetOutputPin(c_._mosi._gpio, c_._mosi._pin);
      } else {
        LL_GPIO_ResetOutputPin(c_._mosi._gpio, c_._mosi._pin);
      }
      // SCKをHighに
      LL_GPIO_SetOutputPin(c_._sck._gpio, c_._sck._pin);

      c_._us_cnt->wait_us(us_delay_);

      // SCKをLowに
      LL_GPIO_ResetOutputPin(c_._sck._gpio, c_._sck._pin);

      res <<= 1;
      res |= LL_GPIO_IsInputPinSet(c_._miso._gpio, c_._miso._pin);
      
      c_._us_cnt->wait_us(us_delay_);
    }

    LL_GPIO_ResetOutputPin(c_._mosi._gpio, c_._mosi._pin);
    LL_GPIO_SetOutputPin(c_._cs._gpio,   c_._cs._pin);
    c_._us_cnt->wait_us(us_delay_);

    return res;
  }

protected:
  Config c_;

  uint16_t us_delay_;

};

#endif