#ifndef COM_BASE_HPP_
#define COM_BASE_HPP_

#include "main.h"
#include <string.h>

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

  virtual uint16_t get_rxBuf_datasize(){ return 0;}
  virtual bool     is_rxBuf_empty(){ return true;}

  /* 割り込み配置用 */
  virtual void tx_callback() = 0;
  virtual void rx_callback() = 0;
};

class EXT_COM_BASE {
public:
  EXT_COM_BASE(){};

  /* 通信用処理 */
  virtual bool transmit(uint32_t _cmd_id, uint8_t *_txd)                = 0;
  virtual bool receive(uint32_t &_cmd_id, uint8_t &_dlc, uint8_t *_rxd) = 0;

  /* フラグ管理 */
  virtual uint32_t getFreeLevelTxMailboxes() = 0;
  virtual uint32_t getFillLevelRxMailboxes() = 0;
};

class EXT_DEBUG_CAN_COM : public EXT_COM_BASE{
public:
  EXT_DEBUG_CAN_COM(COM_BASE* _p_debug_com)
    : EXT_COM_BASE(), p_debug_com_(_p_debug_com) {};

  bool transmit(uint32_t _cmd_id, uint8_t *_txd) override {
    struct txstruct{
      uint32_t u32_cmd_id;
      uint8_t  u8_txd[8];
    }txd;
    txd.u32_cmd_id = _cmd_id;
    memcpy(txd.u8_txd, _txd, 8);
    p_debug_com_->set_txbytes((uint8_t*)&txd, sizeof(txstruct));

    return true;
  };

  bool receive(uint32_t &_cmd_id, uint8_t &_dlc, uint8_t *_rxd) override {
    _cmd_id = stRcvData_.u32_cmd_id;
    _dlc    = stRcvData_.u8_dlc;
    memcpy(_rxd, stRcvData_.u8_d, 8);
    u32_RcvDataNum_ = 0;
    return true;
  };
  
  uint32_t getFreeLevelTxMailboxes() override {
    return 1;
  }

  uint32_t getFillLevelRxMailboxes() override {
    return u32_RcvDataNum_;
  }

  struct RcvData{
    uint32_t u32_cmd_id;
    uint8_t  u8_dlc;
    uint8_t  u8_d[8];
  };

  void setReceiveData(RcvData* _d){ 
    stRcvData_    = *_d;
    u32_RcvDataNum_ = 1;
  };

private:
  COM_BASE* p_debug_com_;
  volatile uint32_t u32_RcvDataNum_ = 0;
  RcvData stRcvData_ = {};
};

#endif
