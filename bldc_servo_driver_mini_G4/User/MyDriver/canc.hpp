#ifndef CANC_HPP_
#define CANC_HPP_

#include "com_base.hpp"
#include "main.h"

class CANC : public EXT_COM_BASE {
public:
  CANC(FDCAN_HandleTypeDef *_p_hcan, uint16_t _u16_id)
      : EXT_COM_BASE(), p_hcan_(_p_hcan), u16_CanDeviceId_(_u16_id){};

  void init(void) { this->init(u16_CanDeviceId_); };
  void init(uint16_t _id);

  // void rx_fifo0_callback();

  uint32_t getFreeLevelTxMailboxes() override { return HAL_FDCAN_GetTxFifoFreeLevel(p_hcan_); };
  uint32_t getFillLevelRxMailboxes() override { return HAL_FDCAN_GetRxFifoFillLevel(p_hcan_, FDCAN_RX_FIFO0); };

  bool transmit(uint32_t _cmd_id, uint8_t *_txd) override;
  bool receive(uint32_t &_cmd_id, uint8_t &_dlc, uint8_t *_rxd) override;

protected:
  FDCAN_HandleTypeDef *p_hcan_;

  uint16_t u16_CanDeviceId_; // 11bitの標準ID
};

#endif