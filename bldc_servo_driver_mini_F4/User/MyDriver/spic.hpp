#ifndef SPIC_HPP_
#define SPIC_HPP_

#include "main.h"

class SPIC {
public:
  SPIC(SPI_TypeDef *_spi) : spi_(_spi) {};

  void init();

  void send_bytes(uint8_t *_tbuf, uint8_t *_rbuf, uint16_t _len);

protected:
  virtual void select()   = 0;
  virtual void deselect() = 0;

  const SPI_TypeDef *spi_;
};

#endif