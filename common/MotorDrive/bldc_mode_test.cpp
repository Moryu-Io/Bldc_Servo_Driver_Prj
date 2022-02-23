#include "bldc_mode_test.hpp"
#include "logger.hpp"


void BldcModeTestCurrStep::init()
{
    /* 内部変数初期化 */
    is_comp_ = false;
    u16_test_cnt_ = 0;

    /* Logging設定 */
    LOG::disable_logging();
    LOG::clear_LogData();
    LOG::clear_LogAddressArray();
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_tgt_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Id_tgt_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_meas_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Id_meas_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Vq_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Vd_);
    LOG::enable_logging();
}


void BldcModeTestCurrStep::update()
{    
    BldcDriveMethod::Ref input = {};

    if(u16_test_cnt_ < U16_TEST_CURR_STABLE_COUNT){
        u16_test_cnt_++;
    } else if(u16_test_cnt_ < U16_TEST_CURR_END_COUNT) {
        input.Iq = parts_.fl_tgt_Iq_A;
        input.Id = parts_.fl_tgt_Id_A;

        u16_test_cnt_++;
    } else {
        LOG::disable_logging();
        is_comp_ = true;
    }

    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

}