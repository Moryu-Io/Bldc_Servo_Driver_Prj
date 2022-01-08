#ifndef COM_BASE_HPP_
#define COM_BASE_HPP_

#include "main.h"

/**
 * @brief 通信管理クラスの基底となるIFクラス
 * 
 */
class COM_BASE {
public:
  COM_BASE(){};

  /* 通信用処理 */
  virtual uint16_t get_rxbyte(uint8_t &_d)                    = 0;
  virtual uint16_t get_rxbytes(uint8_t *_arr, uint16_t _size) = 0;
  virtual void     set_txbytes(uint8_t *_arr, uint16_t _size) = 0;

  /* 割り込み配置用 */
  virtual void tx_callback() = 0;
  virtual void rx_callback() = 0;
};

#endif
