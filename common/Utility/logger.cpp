#include <string.h>

#include "logger.hpp"

namespace LOG {

static uint32_t *u32p_Logging_address[LOG_ADDRESS_MAX_NUM] = {}; /* ロギングする変数ポインタを格納する配列 */
static uint8_t u8_Log_Variable_num                         = 0;  /* ロギングする変数の数 */

static uint32_t u32_Log_data[LOG_SIZE_UINT32] = {};              /* ロギングされたデータを格納するBuffer */
static uint32_t u32_Log_data_head             = 0;               /* Bufferの先頭。次書き込まれる位置を示す */
static uint32_t u32_Log_data_size             = LOG_SIZE_UINT32; /* ロギング最大データ数。デフォルトは最大値で指定 */
static uint16_t u16_Log_Mabiki_num            = 0;               /* ロギング間引き数。何回か置きにロギングしたい場合に使用する */
static uint16_t u16_Log_Mabiki_nowCount       = 0;               /* ロギング間引き時に使用する */

static char print_str_buffer[128]; /* ロギング後にprintで出力する際の文字列buffer */

static bool state_logging = false; /* ロギングしているかどうか */

/**
 * @brief ロギングroutine関数
 * @note この関数が呼び出される度にu32p_Logging_addressに保存されている変数ポインタの変数値をロギングする
 * @note ロギング開始しない限りすぐreturnするので、最高周期で常に呼び出しておくのが良い
 * 
 */
void routine() {
  if(state_logging == false) return;

  // 設定された間引き数と同じ回数呼び出されるまで即終了
  if(u16_Log_Mabiki_nowCount < u16_Log_Mabiki_num) {
    u16_Log_Mabiki_nowCount++;
    return;
  } else {
    u16_Log_Mabiki_nowCount = 0;
  }

  for(uint8_t i = 0; i < u8_Log_Variable_num; i++) {
    if(u32_Log_data_head < u32_Log_data_size) {
      u32_Log_data[u32_Log_data_head] = *(u32p_Logging_address[i]);
      u32_Log_data_head++;
    } else {
      // LOG領域が満杯の場合、LOGをストップ
      disable_logging();
    }
  }
}

/**
 * @brief ロギングしたい変数ポインタ配列を設定
 * 
 * @param _u32p_addr : ロギングしたい変数のポインタの配列
 * @param _u32_size : 変数ポインタの数
 */
void set_LogAddressArray(uint32_t *_u32p_addr, uint32_t _u32_size) {
  if(_u32_size > LOG_ADDRESS_MAX_NUM) _u32_size = LOG_ADDRESS_MAX_NUM;

  memcpy(u32p_Logging_address, _u32p_addr, _u32_size * sizeof(uint32_t *));

  u8_Log_Variable_num = (uint8_t)_u32_size;
}

/**
 * @brief ロギングしたい変数ポインタを追加
 * 
 * @param _u32p_addr : push
 */
void put_LogAddress(uint32_t *_u32p_addr) {
  if(u8_Log_Variable_num < LOG_ADDRESS_MAX_NUM) {
    u32p_Logging_address[u8_Log_Variable_num] = _u32p_addr;
    u8_Log_Variable_num++;
  }
}

/**
 * @brief ロギングの最大サイズを設定
 * 
 * @param _u32_ld_size : ロギングサイズ(int単位)
 */
void set_LogData_Size(uint32_t _u32_ld_size) {
  u32_Log_data_size =
      (_u32_ld_size > LOG_SIZE_UINT32) ? LOG_SIZE_UINT32 : _u32_ld_size;
}

/**
 * @brief ロギングの間引き数を設定
 * @note 何回か置きにロギングしたい場合に使用する
 *      ex) routineが1kHzで呼ばれるが、100Hzでロギングしたい場合は
 *          間引き数を9に設定する
 * 
 * @param _u16_mabiki 
 */
void set_Mabiki_Num(uint16_t _u16_mabiki) {
  u16_Log_Mabiki_num = _u16_mabiki;
}

/**
 * @brief 現在設定されているロギングの間引き数を取得する
 * 
 * @return uint16_t 
 */
uint16_t get_Mabiki_Num() { return u16_Log_Mabiki_num; }

/**
 * @brief ログBufferの値をprintする
 * @note 保存されている変数がintであると仮定して出力する
 * @note 実行時、ロギングは強制的に停止される
 * 
 */
void print_LogData_byINT() {
  disable_logging();

  int strsize = 0;

  for(uint32_t i = 0; i < u32_Log_data_head; i++) {
    strsize += sprintf(&print_str_buffer[strsize], "%d", (int)u32_Log_data[i]);

    if((i % u8_Log_Variable_num + 1) == (u8_Log_Variable_num)) {
      print_str_buffer[strsize + 1] = '\n';
      strsize += 2;

      debug_print(print_str_buffer, strsize);
      strsize = 0;
    } else {
      print_str_buffer[strsize + 1] = ',';
      strsize += 2;
    }
  }
}

/**
 * @brief ログBufferの値をprintする
 * @note 保存されている変数がfloatであると仮定して出力する
 * @note 実行時、ロギングは強制的に停止される
 * 
 */
void print_LogData_byFLOAT() {
  disable_logging();

  int strsize = 0;

  for(uint32_t i = 0; i < u32_Log_data_head; i++) {
    // データは整数型で保存されているので、floatに直してprint
    float *putdata = (float *)(&(u32_Log_data[i]));
    strsize += sprintf(&print_str_buffer[strsize], "%f", *putdata);

    if((i % u8_Log_Variable_num + 1) == (u8_Log_Variable_num)) {
      print_str_buffer[strsize + 1] = '\n';
      strsize += 2;

      debug_print(print_str_buffer, strsize);
      strsize = 0;
    } else {
      print_str_buffer[strsize + 1] = ',';
      strsize += 2;
    }
  }
}

/**
 * @brief ロギングする変数ポインタをclearする
 * 
 */
void clear_LogAddressArray() {
  memset(u32p_Logging_address, 0, LOG_ADDRESS_MAX_NUM*4);
  u8_Log_Variable_num = 0;
}

/**
 * @brief ログBufferをclearする
 * 
 */
void clear_LogData() {
  memset(u32_Log_data, 0, LOG_SIZE_BYTES);
  u32_Log_data_head = 0;
}

/**
 * @brief ロギングを開始する
 * 
 */
void enable_logging() {
  u16_Log_Mabiki_nowCount = 0;
  state_logging           = true;
}

/**
 * @brief ロギングを終了する
 * 
 */
void disable_logging() { state_logging = true; }

} // namespace LOG
