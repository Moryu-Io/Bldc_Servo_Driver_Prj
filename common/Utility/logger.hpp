#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "debug_printf.hpp"

#define LOG_SIZE_BYTES (6 * 4096)            // LOGバッファの最大byte数
#define LOG_SIZE_UINT32 (LOG_SIZE_BYTES / 4) // LOGバッファの最大数(4byte単位)
#define LOG_ADDRESS_MAX_NUM (6)              // LOGする変数の最大数

/**
 * @brief Logger
 * @note printfすると諸々の処理のリアルタイム性が悪化する可能性がある
 *       この機能では、一時的に変数値をバッファにためておき、
 * 　　　 あとでprint出力することができる
 *       各種TestCommandなどで使用される
 */
namespace LOG {

void routine();
void set_LogAddressArray(uint32_t *_u32p_addr, uint32_t _u32_size);
void put_LogAddress(uint32_t *_u32p_addr);
void set_LogData_Size(uint32_t _u32_ld_size);
void set_Mabiki_Num(uint16_t _u16_mabiki);
uint16_t get_Mabiki_Num();

void print_LogData_byINT();
void print_LogData_byFLOAT();

void clear_LogAddressArray();
void clear_LogData();

void enable_logging();
void disable_logging();

}; // namespace LOG

#endif /* LOGGING_HPP_ */
