#ifndef FLASH_INTERFACE_HPP_
#define FLASH_INTERFACE_HPP_

#include "main.h"

#define FLASH_DATA_LENGTH_BYTES (0x00400)
#define FLASH_MAX_LENGTH_BYTES (0x1000)

union FlashSaveParams {
  uint8_t  u8_d[FLASH_DATA_LENGTH_BYTES];
  uint32_t u32_d[FLASH_DATA_LENGTH_BYTES / sizeof(uint32_t)];
  struct Variable {
    /* General */
    uint8_t  u8_is_reset_flash;
    uint8_t  u8_dummy_0001;
    uint16_t u16_fw_ver;
    uint8_t  u8_dummy_0003_000F[12];
    /* CAN Config */
    uint16_t u16_can_device_id;
    uint8_t  u8_dummy_0012_001F[14];
    /* BLDC Config */
    int32_t s32_elec_angle_offset_CNT;
    float   fl_elec_angle_gain_CNTtoDeg;
    int8_t  s8_elec_angle_dir;
    uint8_t u8_dummy_0029_004F[39];
    /* Current Control */
    float   fl_Iq_Pgain;
    float   fl_Iq_Igain;
    float   fl_Iq_Dgain;
    float   fl_Iq_I_Limit;
    float   fl_Id_Pgain;
    float   fl_Id_Igain;
    float   fl_Id_Dgain;
    float   fl_Id_I_Limit;
    uint8_t u8_dummy_0070_008F[32];
    /* Position Control */
    float   fl_PosCtrl_Pgain;
    float   fl_PosCtrl_Igain;
    float   fl_PosCtrl_Dgain;
    float   fl_PosCtrl_I_Limit;
    uint8_t u8_dummy_00A0_00BF[32];
  } var;
};

extern const FlashSaveParams C_FlashInitParams;

class FlashIF {

public:
  FlashIF(uint32_t sector, uint32_t sector_addr)
      : u32_sector_(sector), u32_sector_addr_(sector_addr){};

  bool erase();
  bool save();
  bool load();

  FlashSaveParams mirrorRam;

private:
  uint32_t u32_sector_;
  uint32_t u32_sector_addr_;
};

#endif