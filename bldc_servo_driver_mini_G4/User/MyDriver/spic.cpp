#include "spic.hpp"

void SPIC::init() {
    deselect();
    LL_SPI_Enable((SPI_TypeDef *)spi_);
}

void SPIC::send_bytes(uint16_t *_tbuf, uint16_t *_rbuf, uint16_t _len) {
  select();
  for(int i = 0; i < _len; i++) {
    LL_SPI_TransmitData16((SPI_TypeDef *)spi_, _tbuf[i]);
    while(LL_SPI_IsActiveFlag_TXE((SPI_TypeDef *)spi_) == RESET) {
      ;
    }
    while(LL_SPI_IsActiveFlag_RXNE((SPI_TypeDef *)spi_) == RESET) {
      ;
    }
    _rbuf[i] = LL_SPI_ReceiveData16((SPI_TypeDef *)spi_);
  }
  deselect();
}