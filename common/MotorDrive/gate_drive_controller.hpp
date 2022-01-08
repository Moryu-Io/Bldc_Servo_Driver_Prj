#ifndef GATE_DRIVE_CONTROLLER_HPP_
#define GATE_DRIVE_CONTROLLER_HPP_

#include "i2cc.hpp"

class GateDriveController : public I2CC {
public:
  GateDriveController(I2C_TypeDef *_i2cx, DMA_TypeDef *_dmax, uint32_t _rx_dma_ch, uint32_t _tx_dma_ch)
      : I2CC(_i2cx, _dmax, _rx_dma_ch, _tx_dma_ch){};

  union REG_STATUS{
      uint8_t val;
      struct bit{
          uint8_t lock:(1);
          uint8_t dummy:(3);
          uint8_t reset:(1);
          uint8_t vds_p:(1);
          uint8_t thsd:(1);
          uint8_t vcc_uvlo:(1);
      };
  };

  REG_STATUS get_status_reg();

  void set_reset();
  void set_clear();
  void set_VCC();


private:
  void unlock_protect();
  void lock_protect();

  uint8_t get_reg(uint8_t _addr);
  void set_reg(uint8_t _addr, uint8_t _data);

};

#endif