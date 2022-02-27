#include "flash_interface.hpp"

const FlashSaveParams C_FlashInitParams __attribute__ ((aligned(4))) = {
    .var = {
        .u16_fw_ver        = 0,
        .u16_can_device_id = 1,

    },
};