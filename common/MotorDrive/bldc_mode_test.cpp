#include "bldc_mode_test.hpp"
#include "logger.hpp"



void BldcModeTestElecAngle::init()
{
    /* 内部変数初期化 */
    is_comp_ = false;
    u32_test_cnt_ = 0;
    s32_ElecOffsetSum= 0;
    debug_printf("BldcModeTestElecAngle\n");
}

void BldcModeTestElecAngle::update()
{

    BldcDriveMethod::Ref input = {
        .Vq = 3.0f,
    };

    P_BLDC_->update();
    P_BLDC_->overwrite_elec_angle_forTEST(0.0f);
    //P_BLDC_->overwrite_elec_angle_forTEST((float)(u32_test_cnt_*360/1000));
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

    if(u32_test_cnt_ < U32_TEST_STABLE_COUNT){
        u32_test_cnt_++;
    } else if(u32_test_cnt_ < (U32_TEST_STABLE_COUNT + (1 << U32_TEST_AVARAGE_COUNT_SHIFT))){
        s32_ElecOffsetSum += P_BLDC_->get_angle_raw();
        u32_test_cnt_++;
    }else{
        is_comp_ = true;
    }
}

void BldcModeTestElecAngle::end()
{
    int32_t s32_elecoffset = s32_ElecOffsetSum >> U32_TEST_AVARAGE_COUNT_SHIFT;
    parts_.p_flashif->mirrorRam.var.s32_elec_angle_offset_CNT = s32_elecoffset;
    P_BLDC_->set_elec_angle_offset(s32_elecoffset);
    debug_printf("%d\n", (int)(s32_elecoffset));
}


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