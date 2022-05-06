#include "flash_interface.hpp"

const FlashSaveParams C_FlashInitParams __attribute__ ((aligned(8))) = {
    .var = {
        .u16_fw_ver        = 0,
        .u16_can_device_id = 1,
        .s32_elec_angle_offset_CNT   = 15075,                    // PM3505 default
        .fl_elec_angle_gain_CNTtoDeg = 360.0f*11.0f/16384.0f,    // PM3505 default
        .s8_elec_angle_dir           = -1,                       // PM3505 default
    },
};