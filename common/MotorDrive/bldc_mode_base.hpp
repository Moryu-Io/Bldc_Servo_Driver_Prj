#ifndef BLDC_MODE_BASE_HPP_
#define BLDC_MODE_BASE_HPP_

#include "main.h"
#include "bldc.hpp"

#define CONV_U32TYPE(x) (*((uint32_t*)&(x)))

/**
 * @brief BLDC制御MODE基底クラス
 * 
 */
class BldcModeBase {
public:
    BldcModeBase();

    virtual void init() = 0;
    virtual void update() = 0;
    virtual void end() = 0;

    virtual bool isCompleted(){ return false; };

    virtual void get_status_1(uint32_t *p_mem){};
    virtual void get_status_2(uint32_t *p_mem){};

    static BLDC* P_BLDC_;

};


/**
 * @brief BLDC制御OFFクラス
 * 
 */
class BldcModePowerOff : public BldcModeBase {
public:
    BldcModePowerOff()
        : BldcModeBase() {};

    void init() override {};
    void update() override;
    void end() override {};

};


#endif
