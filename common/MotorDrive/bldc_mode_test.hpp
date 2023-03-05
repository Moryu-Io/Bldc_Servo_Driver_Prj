#ifndef BLDC_MODE_TEST_HPP_
#define BLDC_MODE_TEST_HPP_

#include "bldc_mode_base.hpp"
#include "bldc_mode_pos_control.hpp"
#include "bldc_drive_method.hpp"
#include "controller.hpp"
#include "iir.hpp"
#include "target_interp.hpp"
#include "flash_interface.hpp"

/**
 * @brief BLDC電気角測定テスト
 * 
 */
class BldcModeTestElecAngle : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
        FlashIF* p_flashif;
    };

    BldcModeTestElecAngle(Parts& _parts)
        : parts_(_parts), nowState_(APPLY_OPEN_EANG) {};

    void init() override;
    void update() override;
    void end() override;

    bool isCompleted() override { return is_comp_; };

protected:
    enum state{
        APPLY_OPEN_EANG,
        WAIT_STABLE,
        AVARAGE_EANG,
        FORCE_SINE,
        MEAS_CURR,
        DRV_STOP
    };

    Parts& parts_;
    state  nowState_;

    bool is_comp_;
    int32_t s32_ElecOffsetSum;
    BLDC::CurrentRaw st_curr_max;
    BLDC::CurrentRaw st_curr_min;
    uint32_t u32_test_cnt_;
    const uint32_t U32_TEST_STABLE_COUNT  = 10000;
    const uint32_t U32_TEST_AVARAGE_COUNT_SHIFT = 8;
    const uint32_t U32_TEST_SINE_COUNT  = 10000;

private:
    state apply_open_eang();
    state wait_stable();
    state average_eang();
    state force_sine();
    state measure_current();
    state drive_stop();

};


/**
 * @brief BLDC電流制御ステップ応答テスト
 * 
 */
class BldcModeTestCurrStep : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
    };

    BldcModeTestCurrStep(Parts& _parts)
        : parts_(_parts) {};

    void init() override;
    void update() override;
    void end() override {};

    bool isCompleted() override { return is_comp_; };

    void set_Instruction(Instr *p_instr) override;

protected:
    Parts& parts_;
    
    float fl_tgt_Iq_A_;
    float fl_tgt_Id_A_;

    bool is_comp_;
    uint16_t u16_test_cnt_;
    const uint16_t U16_TEST_CURR_STABLE_COUNT = 200;
    const uint16_t U16_TEST_CURR_END_COUNT    = 1000;
};

/**
 * @brief BLDC位置制御ステップ応答テスト
 * 
 */
class BldcModeTestPosStep : public BldcModeBase {
public:
    struct Parts{
        BldcModePosControl* p_mode_posctrl;
    };

    BldcModeTestPosStep(Parts& _parts)
        : parts_(_parts) {};

    void init() override;
    void update() override;
    void end() override {};

    bool isCompleted() override { return is_comp_; };

    void set_Instruction(Instr *p_instr) override;

protected:
    Parts& parts_;

    int16_t  s16_tgt_pos_deg_;
    uint16_t u16_move_time_ms_;
    uint8_t  u8_mabiki_;

    bool is_comp_;
    uint16_t u16_test_cnt_;
    const uint16_t U16_TEST_CURR_STABLE_COUNT = 200;
    const uint16_t U16_TEST_CURR_END_COUNT    = 10000;

    float fl_tgt_forlog;
    float fl_pos_forlog;
};


/**
 * @brief BLDC正弦波駆動openテスト
 * 
 */
class BldcModeTestSineDriveOpen : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
    };

    BldcModeTestSineDriveOpen(Parts& _parts)
        : parts_(_parts) {};

    void init() override{};
    void update() override;
    void end() override {};

    bool isCompleted() override { return false; };

    void set_Instruction(Instr *p_instr) override;

protected:
    Parts& parts_;
    
    float fl_tgt_Vq_V_;
    float fl_tgt_Vd_V_;

};


#endif
