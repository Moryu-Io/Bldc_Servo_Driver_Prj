#include "i2cc.hpp"

I2CC::I2CC(I2C_TypeDef *_i2cx, DMA_TypeDef *_dmax, uint32_t _rx_dma_ch, uint32_t _tx_dma_ch) : i2cx_(_i2cx), dmax_(_dmax), rx_dma_ch_(_rx_dma_ch), tx_dma_ch_(_tx_dma_ch) {
}

void I2CC::init() {
  LL_DMA_ConfigAddresses(dmax_, tx_dma_ch_,
                         (uint32_t)(this->txbuffer_), (uint32_t)(&(i2cx_->TXDR)),
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_ConfigAddresses(dmax_, rx_dma_ch_,
                         (uint32_t)(&(i2cx_->RXDR)), (uint32_t)(this->rxbuffer_),
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_EnableIT_TC(dmax_, tx_dma_ch_);
  LL_DMA_EnableIT_TC(dmax_, rx_dma_ch_);
  LL_I2C_EnableDMAReq_RX(i2cx_);
  LL_I2C_EnableDMAReq_TX(i2cx_);
  LL_I2C_Enable(i2cx_);
}

void I2CC::writes_with_dma(uint32_t _dnum) {
    volatile DMA_Channel_TypeDef* p_ch = DMA1_Channel2;
  _dnum = (_dnum > BUFFER_LENGTH) ? BUFFER_LENGTH : _dnum;

  LL_DMA_SetDataLength(dmax_, tx_dma_ch_, _dnum);

  LL_DMA_EnableChannel(dmax_, tx_dma_ch_);

  LL_I2C_HandleTransfer(i2cx_, u8_device_ic2_addr, LL_I2C_ADDRSLAVE_7BIT, _dnum, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  while(!is_txDMA_completed)
    ;
  while(!LL_I2C_IsActiveFlag_STOP(i2cx_))
    ;
  LL_I2C_ClearFlag_STOP(i2cx_);
  LL_DMA_DisableChannel(dmax_, tx_dma_ch_);
  is_txDMA_completed = false;
}

void I2CC::reads_with_dma(uint32_t _dnum) {
  _dnum = (_dnum > BUFFER_LENGTH) ? BUFFER_LENGTH : _dnum;

  LL_DMA_SetDataLength(dmax_, rx_dma_ch_, _dnum);

  LL_DMA_EnableChannel(dmax_, rx_dma_ch_);
  LL_I2C_Enable(i2cx_);

  LL_I2C_HandleTransfer(i2cx_, u8_device_ic2_addr | 0x01, LL_I2C_ADDRSLAVE_7BIT, _dnum, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
  while(!is_rxDMA_completed)
    ;
  while(!LL_I2C_IsActiveFlag_STOP(i2cx_))
    ;

  LL_I2C_ClearFlag_STOP(I2C2);
  LL_DMA_DisableChannel(dmax_, rx_dma_ch_);
  is_rxDMA_completed = false;
}

void I2CC::rx_dma_TC() {
  is_rxDMA_completed = true;
}

void I2CC::tx_dma_TC() {
  is_txDMA_completed = true;
}

void I2CC::writes(uint32_t _dnum){
  uint8_t _d_header = 0;
  _dnum = (_dnum > BUFFER_LENGTH) ? BUFFER_LENGTH : _dnum;
  LL_I2C_HandleTransfer(i2cx_, u8_device_ic2_addr, LL_I2C_ADDRSLAVE_7BIT, _dnum, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  while (!LL_I2C_IsActiveFlag_STOP(i2cx_))
  {
    /* Check TXIS flag value in ISR register */
    if (LL_I2C_IsActiveFlag_TXIS(i2cx_))
    {
      /* Write data in Transmit Data register.
      TXIS flag is cleared by writing data in TXDR register */
      LL_I2C_TransmitData8(i2cx_, txbuffer_[_d_header]);
      _d_header++;
    }
  }
  LL_I2C_ClearFlag_STOP(i2cx_);
}

void I2CC::reads(uint8_t _reg, uint32_t _dnum){
  _dnum = (_dnum > BUFFER_LENGTH) ? BUFFER_LENGTH : _dnum;
  LL_I2C_HandleTransfer(i2cx_, u8_device_ic2_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  while(!LL_I2C_IsActiveFlag_TXIS(i2cx_));
  LL_I2C_TransmitData8(i2cx_,_reg);
  while(!LL_I2C_IsActiveFlag_STOP(i2cx_));
  LL_I2C_ClearFlag_STOP(i2cx_);
  
  while(LL_I2C_IsActiveFlag_BUSY(i2cx_) == SET);
  
  LL_I2C_HandleTransfer(i2cx_, u8_device_ic2_addr, LL_I2C_ADDRSLAVE_7BIT, _dnum, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
  
  for(int i=0;i<_dnum;i++){
    while(!LL_I2C_IsActiveFlag_RXNE(i2cx_));
    rxbuffer_[i] = LL_I2C_ReceiveData8(i2cx_);
  }

  while(!LL_I2C_IsActiveFlag_STOP(i2cx_));

  LL_I2C_ClearFlag_STOP(i2cx_);
}
