#include "spic.hpp"

void SPIC::init() {
    deselect();
    LL_SPI_Enable((SPI_TypeDef *)spi_);
}

void SPIC::send_bytes(uint8_t *_tbuf, uint8_t *_rbuf, uint16_t _len) {
  select();
  for(int i = 0; i < _len; i++) {
    LL_SPI_TransmitData8((SPI_TypeDef *)spi_, _tbuf[i]);
    while(LL_SPI_IsActiveFlag_TXE((SPI_TypeDef *)spi_) == RESET) {
      ;
    }
    while(LL_SPI_IsActiveFlag_RXNE((SPI_TypeDef *)spi_) == RESET) {
      ;
    }
    _rbuf[i] = LL_SPI_ReceiveData8((SPI_TypeDef *)spi_);
  }
  deselect();
}