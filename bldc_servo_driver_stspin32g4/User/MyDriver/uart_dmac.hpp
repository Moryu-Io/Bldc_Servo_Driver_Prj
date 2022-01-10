#ifndef UART_DMAC_HPP_
#define UART_DMAC_HPP_

#include "usart.h"

#include "com_base.hpp"

/**
 * @brief STM32のUARTをDMAで制御するクラス
 * @note  Bufferの長さは2の乗数で規定する必要有
 * 
 */
class UART_DMAC : public COM_BASE {
public:
  UART_DMAC(const USART_TypeDef *_u, const DMA_TypeDef *_d,
            const uint32_t _dch_r, const uint32_t _dch_t)
      : COM_BASE(), usart_(_u), dma_(_d), dma_ch_rx_(_dch_r), dma_ch_tx_(_dch_t){};

  void init_constparam(uint8_t *_rbuf, uint16_t _rbuf_len,
                       uint8_t *_tbuf, uint16_t _tbuf_len) {
    pu8_rxBuf_     = _rbuf;
    u16_rxBuf_len_ = _rbuf_len;
    pu8_txBuf_     = _tbuf;
    u16_txBuf_len_ = _tbuf_len;
  };

  void init_rxtx() {
    init_rx();
    init_tx();
  };

  uint16_t get_rxBuf_head();
  uint16_t get_rxBuf_datasize();
  bool     is_rxBuf_empty();

  /* 通信用処理 */
  virtual uint16_t get_rxbyte(uint8_t &_d);
  virtual uint16_t get_rxbytes(uint8_t *_arr, uint16_t _size);
  virtual void     set_txbytes(uint8_t *_arr, uint16_t _size);

  /* 割り込み配置用 */
  virtual void tx_callback();
  virtual void rx_callback();

protected:
  virtual void init_rx();
  virtual void init_tx();

  /* RS485用 */
  virtual void enable_tx(){};
  virtual void disable_tx(){};

  uint16_t u16_rxBuf_tail_ = 0;
  bool     is_tx_comp      = false;

  /* Buffer関連 */
  uint8_t *pu8_rxBuf_ = NULL;
  uint8_t *pu8_txBuf_ = NULL;

  uint16_t u16_rxBuf_len_ = 0;
  uint16_t u16_txBuf_len_ = 0;

  /* ペリフェラル関連 */
  const USART_TypeDef *usart_;
  const DMA_TypeDef *  dma_;

  const uint32_t dma_ch_rx_;
  const uint32_t dma_ch_tx_;
};

#endif
