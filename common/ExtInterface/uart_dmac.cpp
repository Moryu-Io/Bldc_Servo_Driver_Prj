
#include "uart_dmac.hpp"
#include <string.h>

/**
 * @brief 受信側ペリフェラル初期化
 * 
 */
void UART_DMAC::init_rx() {
  // 設定中はDMAを無効化
  LL_DMA_DisableChannel((DMA_TypeDef *)dma_, (uint32_t)dma_ch_rx_);

  // 受信バッファの初期化
  memset(pu8_rxBuf_, 0, u16_rxBuf_len_);
  u16_rxBuf_tail_ = 0;

  // UART受信 DMAアドレスの設定
  LL_DMA_ConfigAddresses((DMA_TypeDef *)dma_, (uint32_t)dma_ch_rx_,
                         LL_USART_DMA_GetRegAddr((USART_TypeDef *)usart_, LL_USART_DMA_REG_DATA_RECEIVE),
                         (uint32_t)pu8_rxBuf_, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  // UART受信 データアドレスの長さを設定
  LL_DMA_SetDataLength((DMA_TypeDef *)dma_, (uint32_t)dma_ch_rx_, u16_rxBuf_len_);

  // DMAの有効化
  LL_DMA_EnableChannel((DMA_TypeDef *)dma_, (uint32_t)dma_ch_rx_);

  // DMAリクエストの有効化
  LL_USART_EnableDMAReq_RX((USART_TypeDef *)usart_);
}

/**
 * @brief 送信側ペリフェラル初期化
 * 
 */
void UART_DMAC::init_tx() {
  LL_USART_EnableIT_TC((USART_TypeDef *)usart_);
  is_tx_comp = true;
  disable_tx();
}

/**
 * @brief 受信データの先頭位置取得
 * 
 * @return uint16_t 
 */
uint16_t UART_DMAC::get_rxBuf_head() {
  DMA_Channel_TypeDef *_dma_ch = (DMA_Channel_TypeDef *)((uint32_t)dma_ + 0x08 + 0x14 * dma_ch_rx_);
  return ((u16_rxBuf_len_ - (uint16_t)_dma_ch->CNDTR)) & (u16_rxBuf_len_ - 1);
}

/**
 * @brief 受信データの利用可能長の取得
 * 
 * @return uint16_t 
 */
uint16_t UART_DMAC::get_rxBuf_datasize() {
  uint16_t _head = get_rxBuf_head();
  return ((_head - u16_rxBuf_tail_) & (u16_rxBuf_len_ - 1));
}

/**
 * @brief 受信データが空かどうか
 * 
 * @return true : 空
 * @return false : データ有
 */
bool UART_DMAC::is_rxBuf_empty() {
  return (u16_rxBuf_tail_ == get_rxBuf_head());
};

/**
 * @brief リングバッファから1byte取り出し
 * 
 * @param _d : データ格納用変数(参照渡し)
 * @return uint16_t : 取り出したデータ数(データがない場合は0になる)
 */
uint16_t UART_DMAC::get_rxbyte(uint8_t &_d) {
  return get_rxbytes(&_d, 1);
}

/**
 * @brief リングバッファから指定byte取り出し
 * 
 * @param _arr : データ格納用配列
 * @param _size : 取り出しbyte数
 * @return uint16_t : 取り出したデータ数
 */
uint16_t UART_DMAC::get_rxbytes(uint8_t *_arr, uint16_t _size) {
  if(is_rxBuf_empty()) {
    return 0;
  }
  // DMAを無効化
  LL_DMA_DisableChannel((DMA_TypeDef *)dma_, (uint32_t)dma_ch_rx_);

  /* サイズチェックとデータコピー */
  uint16_t _rxbuf_size = get_rxBuf_datasize();
  if(_rxbuf_size < _size) _size = _rxbuf_size;

  if(u16_rxBuf_tail_ + _size <= u16_rxBuf_len_) {
    memcpy(_arr, &(pu8_rxBuf_[u16_rxBuf_tail_]), _size);
  } else {
    // 取得データが循環部分をまたぐ場合は分けてメモリコピー
    uint16_t _hlen = u16_rxBuf_len_ - u16_rxBuf_tail_;
    memcpy(_arr, &(pu8_rxBuf_[u16_rxBuf_tail_]), _hlen);
    memcpy(&_arr[_hlen], pu8_rxBuf_, _size - _hlen);
  }

  u16_rxBuf_tail_ = (u16_rxBuf_tail_ + _size) & (u16_rxBuf_len_ - 1);

  // DMAの有効化
  LL_DMA_EnableChannel((DMA_TypeDef *)dma_, (uint32_t)dma_ch_rx_);

  return _size;
}

/**
 * @brief DMAを使った送信
 * @note 送信データは内部でバッファにコピーされるので呼び出し元では保持不要
 * 
 * @param _arr : 送信データ
 * @param _size : 送信データ数
 */
void UART_DMAC::set_txbytes(uint8_t *_arr, uint16_t _size) {
  /* 送信完了待ち */
  while(!is_tx_comp)
    ;

  /* サイズチェックとデータコピー */
  if(u16_txBuf_len_ < _size) _size = u16_txBuf_len_;
  memcpy(pu8_txBuf_, _arr, _size);
  is_tx_comp = false;

  // DMAを無効化
  LL_DMA_DisableChannel((DMA_TypeDef *)dma_, (uint32_t)dma_ch_tx_);

  // UART送信 DMAアドレスの設定
  LL_DMA_ConfigAddresses((DMA_TypeDef *)dma_, (uint32_t)dma_ch_tx_,
                         (uint32_t)pu8_txBuf_,
                         LL_USART_DMA_GetRegAddr((USART_TypeDef *)usart_, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  // UART送信 データアドレスの長さを設定
  LL_DMA_SetDataLength((DMA_TypeDef *)dma_, (uint32_t)dma_ch_tx_, _size);

  // Tx有効化(RS485用)
  enable_tx();

  // DMAの有効化
  LL_DMA_EnableChannel((DMA_TypeDef *)dma_, (uint32_t)dma_ch_tx_);

  // DMAリクエストの有効化
  LL_USART_EnableDMAReq_TX((USART_TypeDef *)usart_);
}

/**
 * @brief 送信完了割り込み内で呼ぶコールバック関数
 * 
 */
void UART_DMAC::tx_callback() {
  is_tx_comp = true;
  disable_tx();
}

/**
 * @brief 受信完了割り込み内で呼ぶコールバック関数
 * @note 現在は使用していない
 */
void UART_DMAC::rx_callback() {
}
