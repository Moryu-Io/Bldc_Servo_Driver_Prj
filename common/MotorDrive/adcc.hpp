#ifndef ADCC_HPP_
#define ADCC_HPP_

#include "main.h"

template<uint8_t ChNum>
class ADCC {
public:
  ADCC(ADC_TypeDef *_adcx, DMA_TypeDef *_dmax, uint32_t _dma_ch) 
  : adcx_(_adcx), dmax_(_dmax), dma_ch_(_dma_ch){};

  virtual void init(){
    LL_DMA_ConfigAddresses(dmax_, dma_ch_,
						   LL_ADC_DMA_GetRegAddr(adcx_,LL_ADC_DMA_REG_REGULAR_DATA),(uint32_t)u16_read_buf_,
						   LL_DMA_GetDataTransferDirection(dmax_, dma_ch_));
	LL_DMA_SetDataLength(dmax_, dma_ch_, ChNum);
  }

  virtual void start(){
    LL_ADC_Enable(adcx_);
	LL_DMA_EnableChannel(dmax_, dma_ch_);
    LL_ADC_REG_StartConversion(adcx_);
  }

  virtual void stop(){
    LL_ADC_REG_StopConversion(adcx_);
    LL_ADC_Disable(adcx_);
	LL_DMA_DisableChannel(dmax_, dma_ch_);
  }

  virtual uint16_t get_adc_data(uint8_t _ch){
      _ch = (_ch >= ChNum) ? (ChNum-1) : _ch;
      return u16_read_buf_[_ch];
  }

protected:
  ADC_TypeDef *adcx_;
  DMA_TypeDef *dmax_;
  uint32_t     dma_ch_;

  uint16_t u16_read_buf_[ChNum];
};

#endif
