#ifndef TARGET_INTERP_HPP_
#define TARGET_INTERP_HPP_

#include "main.h"


class TargetInterp {
public:
    TargetInterp(){};

    void set_target(int32_t val, int32_t mtime){
        mtime = (mtime == 0) ? 1 : mtime;
        uint8_t _u8_writePage = u8_nowReadPage_ ^ 1;
        input_[_u8_writePage].s32_move_time_servocnt = mtime;
        input_[_u8_writePage].s32_tgt_final = val;
        input_[_u8_writePage].s32_move_step = ((val - s32_now_target_) >= 0) ? (val - s32_now_target_ + mtime - 1) / mtime  // 切り上げ
                                                                             : (val - s32_now_target_ - mtime + 1) / mtime; // 切り下げ
        u8_nowReadPage_ = _u8_writePage;
    };

    int32_t update_target(){
        int32_t _s32_move_step = input_[u8_nowReadPage_].s32_move_step;
        int32_t _s32_tgt_final = input_[u8_nowReadPage_].s32_tgt_final;

        int32_t _s32_target_now = s32_now_target_ + _s32_move_step;

        if((_s32_move_step > 0 && (_s32_target_now > _s32_tgt_final)) ||
           (_s32_move_step < 0 && (_s32_target_now < _s32_tgt_final))){
            s32_now_target_ = _s32_tgt_final;
        } else {
            s32_now_target_ = _s32_target_now;
        }

        s32_now_target_vel_ = s32_now_target_ - s32_pre_target_;
        s32_pre_target_     = s32_now_target_;

        return s32_now_target_;
    };

    int32_t get_target()     { return s32_now_target_;     };
    int32_t get_target_vel() { return s32_now_target_vel_; };

private:
    struct InputParams{
        int32_t  s32_tgt_final;
        int32_t  s32_move_time_servocnt;
        int32_t  s32_move_step;
    };

    InputParams input_[2];
    uint8_t u8_nowReadPage_;

    int32_t s32_pre_target_;
    int32_t s32_now_target_;
    int32_t s32_now_target_vel_;


};


#endif