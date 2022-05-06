#include <string.h>

#include "canc.hpp"

void CANC::init(uint16_t _id) {
  // デバイスとしてのIDは11bitで標準IDと同じ扱い
  // 拡張分の18bitは指示コマンドを載せる

  // filter設定1
  uint32_t _u32_fId   = _id << 18;
  uint32_t _u32_fmask = 0x7FF << 18;

  FDCAN_FilterTypeDef _filter;
  _filter.IdType       = FDCAN_EXTENDED_ID;
  _filter.FilterIndex  = 0;
  _filter.FilterType   = FDCAN_FILTER_MASK;
  _filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  _filter.FilterID1    = _u32_fId;
  _filter.FilterID2    = _u32_fmask;

  HAL_FDCAN_ConfigFilter(p_hcan_, &_filter);

  // filter設定2
  // ALL指示用(ID:0x7FF)

  _filter.IdType       = FDCAN_EXTENDED_ID;
  _filter.FilterIndex  = 1;
  _filter.FilterType   = FDCAN_FILTER_MASK;
  _filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  _filter.FilterID1    = 0x7FF << 18;
  _filter.FilterID2    = _u32_fmask;

  HAL_FDCAN_ConfigFilter(p_hcan_, &_filter);

  // deviceIDの保存
  u16_CanDeviceId_ = _id;

  HAL_FDCAN_Start(p_hcan_);
  // HAL_FDCAN_ActivateNotification(p_hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

bool CANC::transmit(uint32_t _cmd_id, uint8_t *_txd) {
  FDCAN_TxHeaderTypeDef _txHeader;

  // txHeader_.StdId = u16_CanDeviceId_;
  _txHeader.Identifier          = (u16_CanDeviceId_ << 18) | _cmd_id;
  _txHeader.IdType              = FDCAN_EXTENDED_ID;
  _txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  _txHeader.DataLength          = FDCAN_DLC_BYTES_8;
  _txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  _txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  _txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  _txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  _txHeader.MessageMarker       = 0;

  return (HAL_FDCAN_AddMessageToTxFifoQ(p_hcan_, &_txHeader, _txd) == HAL_OK) ? true : false;
}

bool CANC::receive(uint32_t &_cmd_id, uint8_t &_dlc, uint8_t *_rxd) {
  bool ret = false;

  FDCAN_RxHeaderTypeDef _rxHeader;

  if(HAL_FDCAN_GetRxMessage(p_hcan_, FDCAN_RX_FIFO0, &_rxHeader, _rxd) == HAL_OK) {
    if(_rxHeader.IdType != FDCAN_EXTENDED_ID) {
      ret = false;
    } else {
      _dlc    = _rxHeader.DataLength >> 16; // FD領域(8byteより大きい)では機能しなくなる
      _cmd_id = _rxHeader.Identifier & 0x3FFFF;
      ret     = true;
    }
  }

  return ret;
}