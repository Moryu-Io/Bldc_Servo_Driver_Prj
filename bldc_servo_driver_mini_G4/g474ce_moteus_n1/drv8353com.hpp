#ifndef DRV8353COM_HPP_
#define DRV8353COM_HPP_

#include "main.h"
#include "spic.hpp"

class Drv8353Com {
public:
  enum RegName {
    REG_FAULT_STATUS = 0x00,
    REG_VGS_STATUS   = 0x01,
    REG_DRIVER_CTRL  = 0x02,
    REG_GATE_DRV_HS  = 0x03,
    REG_GATE_DRV_LS  = 0x04,
    REG_OCP_CTRL     = 0x05,
    REG_CSA_CTRL     = 0x06,
    REG_CAL_MODE     = 0x07,
  };

  union unRegFaultStatus {
    uint16_t val;
    struct {
      uint16_t vds_lc  : 1;
      uint16_t vds_hc  : 1;
      uint16_t vds_lb  : 1;
      uint16_t vds_hb  : 1;
      uint16_t vds_la  : 1;
      uint16_t vds_ha  : 1;
      uint16_t otsd    : 1; // 加熱シャットダウン
      uint16_t uvlo    : 1; // 低電圧誤動作防止Fault
      uint16_t gdf     : 1; // ゲート駆動Fault
      uint16_t vds_ocp : 1; // VDSモニタ過電流Fault
      uint16_t Fault   : 1; // どれかのFaultが出ているかどうか
      uint16_t resv    : 5;
    } bit;
  };

  union unRegVGSStatus {
    uint16_t val;
    struct {
      uint16_t vgs_lc  : 1;
      uint16_t vgs_hc  : 1;
      uint16_t vgs_lb  : 1;
      uint16_t vgs_hb  : 1;
      uint16_t vgs_la  : 1;
      uint16_t vgs_ha  : 1;
      uint16_t gduv    : 1; // VCPチャージポンプまたはVGLS低電圧Fault
      uint16_t otw     : 1; // 過熱警告
      uint16_t sc_oc   : 1; // C相 過電流
      uint16_t sb_oc   : 1; // B相 過電流
      uint16_t sa_oc   : 1; // A相 過電流
      uint16_t resv    : 5;
    } bit;
  };

  union unRegDrvCtrl {
    uint16_t val;
    struct {
      uint16_t clr_flt      : 1;  // Faultクリアbit
      uint16_t brake        : 1;  // 3つのL-FETをHighにしてBrake
      uint16_t coast        : 1;  // すべてのFETをHi-Zにする
      uint16_t onepwm_dir   : 1;  // 1PWMモードでこのBitとDIRピン入力の論理和が方向になる
      uint16_t onepwm_com   : 1;  // 1PWMモードで同期(0)/非同期(1)整流 選択
      uint16_t pwm_mode     : 2;  // PWMモード選択
      uint16_t otw_rep      : 1;  // OTWの通知をFaultとしてするか
      uint16_t dis_gdf      : 1;  // ゲート駆動フォルトを無効にする(1)
      uint16_t dis_gduv     : 1;  // VCP/VGLSフォルトを無効にする(1)
      uint16_t ocp_act      : 1;  // vds_ocp, sen_ocpに関連してHalfブリッジをシャットダウン
      uint16_t resv         : 5;
    } bit;
  };

  union unRegGHDrvCtrl {
    uint16_t val;
    struct {
      uint16_t idriven_hs   : 4;
      uint16_t idrivep_hs   : 4;
      uint16_t lock         : 3;
      uint16_t resv         : 5;
    } bit;
  };

  union unRegGLDrvCtrl {
    uint16_t val;
    struct {
      uint16_t idriven_ls   : 4;
      uint16_t idrivep_ls   : 4;
      uint16_t tdrive       : 2;
      uint16_t cbc          : 1;
      uint16_t resv         : 5;
    } bit;
  };

  union unRegOCPCtrl {
    uint16_t val;
    struct {
      uint16_t vds_lvl      : 4;  // VDS閾値レベル
      uint16_t ocp_deg      : 2;  // 過電流グリッチ除去
      uint16_t ocp_mode     : 2;  // フォルトに対する行動
      uint16_t dead_time    : 2;  // デッドタイム
      uint16_t tretry       : 1;  // リトライ時間
      uint16_t resv         : 5;
    } bit;
  };

  union unRegCSACtrl {
    uint16_t val;
    struct {
      uint16_t sen_lvl      : 2;  // 検出OCPレベル
      uint16_t csa_cal_c    : 1;
      uint16_t csa_cal_b    : 1;
      uint16_t csa_cal_a    : 1;
      uint16_t dis_sen      : 1;  // 検出過電流faultを有効(0)
      uint16_t csa_gain     : 2;  // 0から5V/V, 10V/V, 20V/V, 40V/V
      uint16_t ls_ref       : 1;
      uint16_t vref_div     : 1;
      uint16_t csa_fet      : 1;
      uint16_t resv         : 5;
    } bit;
  };

  union unRegCalModeCtrl {
    uint16_t val;
    struct {
      uint16_t cal_mode     : 1;
      uint16_t resv         : 15;
    } bit;
  };

  struct stRegMap{
    unRegFaultStatus  sts_fault;
    unRegVGSStatus    sts_vds;
    unRegDrvCtrl      cfg_drv;
    unRegGHDrvCtrl    cfg_ghs;
    unRegGLDrvCtrl    cfg_gls;
    unRegOCPCtrl      cfg_ocp;
    unRegCSACtrl      cfg_csa;
    unRegCalModeCtrl  cfg_calmode;
  }reg_map_;

  Drv8353Com(SoftSPIC *_spi) : spi_(_spi) {}

  void init() {
    spi_->init();
    doInit();
  }

  virtual void doInit(){};

  /**
   * @brief 
   * 
   * @return true  : Faultが立っている
   * @return false : Faultが立ってない
   */
  bool update_sts_reg(){
    reg_map_.sts_fault.val   = read(REG_FAULT_STATUS);
    reg_map_.sts_vds.val     = read(REG_VGS_STATUS);

    return (reg_map_.sts_fault.bit.Fault) ? true : false;
  }

  void update_all_reg(){
    reg_map_.sts_fault.val   = read(REG_FAULT_STATUS);
    reg_map_.sts_vds.val     = read(REG_VGS_STATUS);
    reg_map_.cfg_drv.val     = read(REG_DRIVER_CTRL);
    reg_map_.cfg_ghs.val     = read(REG_GATE_DRV_HS);
    reg_map_.cfg_gls.val     = read(REG_GATE_DRV_LS);
    reg_map_.cfg_ocp.val     = read(REG_OCP_CTRL);
    reg_map_.cfg_csa.val     = read(REG_CSA_CTRL);
    reg_map_.cfg_calmode.val = read(REG_CAL_MODE);
  }

  unRegFaultStatus get_fault_status(){
    return reg_map_.sts_fault;
  }

  unRegVGSStatus get_vgs_status(){
    return reg_map_.sts_vds;
  }

  bool clear_fault(){
    unRegDrvCtrl wd;
    wd.val         = reg_map_.cfg_drv.val;
    wd.bit.clr_flt = 1;
    write(REG_DRIVER_CTRL, wd.val);
    return update_sts_reg();
  }

protected:
  void write(uint8_t _addr, uint16_t _d) {
    uint16_t data = ((_addr << 11) & 0x7800) | (_d & 0x07FF);
    spi_->transmit(data);
  }

  uint16_t read(uint8_t _addr) {
    uint16_t data = 0x8000 | ((_addr << 11) & 0x7800);
    uint16_t r_d  = spi_->transmit(data);
    return r_d;
  }

  SoftSPIC *spi_;


}
;

#endif