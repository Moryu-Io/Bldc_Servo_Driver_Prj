#include <string.h>

#include "canc.hpp"

void CANC::init(uint16_t _id) {
  // デバイスとしてのIDは11bitで標準IDと同じ扱い
  // 拡張分の18bitは指示コマンドを載せる

  // filter設定1
  uint32_t _u32_fId   = (_id << 21) | 0b100; // IDEbit:1(拡張ID)
  uint32_t _u32_fmask = (0x7FF << 21) | 0x00000004; // IDEbit:1

  CAN_FilterTypeDef _filter;
  _filter.FilterIdHigh         = _u32_fId >> 16;
  _filter.FilterIdLow          = _u32_fId & 0x00000FFFF;
  _filter.FilterMaskIdHigh     = _u32_fmask >> 16;
  _filter.FilterMaskIdLow      = _u32_fmask & 0x0000FFFF;
  _filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  _filter.FilterBank           = 0;
  _filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  _filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  _filter.FilterActivation     = ENABLE;
  _filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(p_hcan_, &_filter);

  // filter設定2
  // ALL指示用(ID:0x7FF)
  _u32_fId                     = (0x7FF << 21) | 0b100; // IDEbit:1(拡張ID)
  _filter.FilterIdHigh         = _u32_fId >> 16;
  _filter.FilterIdLow          = _u32_fId & 0x00000FFFF;
  _filter.FilterMaskIdHigh     = _u32_fmask >> 16;
  _filter.FilterMaskIdLow      = _u32_fmask & 0x0000FFFF;
  _filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  _filter.FilterBank           = 1;
  _filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  _filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  _filter.FilterActivation     = ENABLE;
  _filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(p_hcan_, &_filter);

  // deviceIDの保存
  u16_CanDeviceId_ = _id;

  HAL_CAN_Start(p_hcan_);
  //HAL_CAN_ActivateNotification(p_hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
}

bool CANC::transmit(uint32_t _cmd_id, uint8_t *_txd) {
  CAN_TxHeaderTypeDef _txHeader;
  uint32_t            _u32_mailbox;

  // txHeader_.StdId = u16_CanDeviceId_;
  _txHeader.ExtId              = (u16_CanDeviceId_ << 18) | _cmd_id ;
  _txHeader.IDE                = CAN_ID_EXT;
  _txHeader.RTR                = CAN_RTR_DATA;
  _txHeader.DLC                = 8;
  _txHeader.TransmitGlobalTime = DISABLE;

  return (HAL_CAN_AddTxMessage(p_hcan_, &_txHeader, _txd, &_u32_mailbox) == HAL_OK) ? true : false;
}

bool CANC::receive(uint32_t &_cmd_id, uint8_t &_dlc, uint8_t *_rxd) {
  bool ret = false;

  CAN_RxHeaderTypeDef _rxHeader;

  if(HAL_CAN_GetRxMessage(p_hcan_, CAN_RX_FIFO0, &_rxHeader, _rxd) == HAL_OK) {
    if(_rxHeader.IDE != CAN_ID_EXT) {
      ret = false;
    } else {
      _dlc    = _rxHeader.DLC;
      _cmd_id = _rxHeader.ExtId & 0x3FFFF;
      ret     = true;
    }
  }

  return ret;
}