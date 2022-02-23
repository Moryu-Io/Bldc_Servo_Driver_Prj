#ifndef CANC_HPP_
#define CANC_HPP_

#include "main.h"

class CANC {
private:
  void enable() { LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13); }
  void disable() { LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13); }

public:
  CANC(CAN_HandleTypeDef *_p_hcan, uint16_t _u16_id)
      : p_hcan_(_p_hcan), u16_CanDeviceId_(_u16_id){};

  void init(void) { this->init(u16_CanDeviceId_); };
  void init(uint16_t _id);

  // void rx_fifo0_callback();

  uint32_t getFreeLevelTxMailboxes() { return HAL_CAN_GetTxMailboxesFreeLevel(p_hcan_); };
  uint32_t getFillLevelRxMailboxes() { return HAL_CAN_GetRxFifoFillLevel(p_hcan_, CAN_RX_FIFO0); };

  bool transmit(uint32_t _cmd_id, uint8_t *_txd);
  bool receive(uint32_t &_cmd_id, uint8_t &_dlc, uint8_t *_rxd);

protected:
  CAN_HandleTypeDef *p_hcan_;

  uint16_t u16_CanDeviceId_; // 11bitの標準ID
};

#endif