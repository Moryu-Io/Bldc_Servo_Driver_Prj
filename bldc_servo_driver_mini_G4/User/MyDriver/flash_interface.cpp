#include "flash_interface.hpp"
#include <string.h>

bool FlashIF::erase() {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t               error_sector;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks     = FLASH_BANK_1;
  EraseInitStruct.Page      = 255;
  EraseInitStruct.NbPages   = 1;

  HAL_StatusTypeDef ret = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

  HAL_FLASH_Lock();

  return ret == HAL_OK && error_sector == 0xFFFFFFFF;
}

bool FlashIF::save() {
  if(!erase()) return false;

  mirrorRam.var.u8_is_reset_flash = 0;

  HAL_FLASH_Unlock();

  const int writenum_word = sizeof(mirrorRam) / sizeof(uint64_t);

  HAL_StatusTypeDef ret;

  for(int i = 0; i < writenum_word; i++) {
    ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            u32_sector_addr_ + i * sizeof(uint64_t),
                            mirrorRam.u64_d[i]);
    if(ret != HAL_OK) break;
  }

  HAL_FLASH_Lock();

  return ret == HAL_OK;
}

bool FlashIF::load() {
  memcpy(&mirrorRam, (void *)u32_sector_addr_, sizeof(mirrorRam));
  return true;
}
