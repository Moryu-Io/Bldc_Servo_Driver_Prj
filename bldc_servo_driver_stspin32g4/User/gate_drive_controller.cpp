#include "gate_drive_controller.hpp"


GateDriveController::REG_STATUS GateDriveController::get_status_reg(){
    REG_STATUS reg;
    reg.val = get_reg(0x80);
    return reg;
}

void GateDriveController::set_reset(){
  unlock_protect();
  set_reg(0x0C, 0b11111111);
  lock_protect();
}

void GateDriveController::set_clear(){
  set_reg(0x09, 0xFF);
}

void GateDriveController::set_VCC(){
  unlock_protect();
  set_reg(0x01, 0b00000000);
  lock_protect();
}


void GateDriveController::unlock_protect(){
  set_reg(0x0B, 0b00101101);
}

void GateDriveController::lock_protect(){
  set_reg(0x0B, 0b11011101);
}

uint8_t GateDriveController::get_reg(uint8_t _addr) {
  txbuffer_[0] = _addr;
  writes_with_dma(1);
  reads_with_dma(1);

  return rxbuffer_[0];
}

void GateDriveController::set_reg(uint8_t _addr, uint8_t _data) {
  txbuffer_[0] = _addr;
  txbuffer_[1] = _data;
  writes_with_dma(2);
}