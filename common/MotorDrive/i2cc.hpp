#ifndef I2CC_HPP_
#define I2CC_HPP_

#include "stm32g4xx.h"
#include "main.h"

#define BUFFER_LENGTH   (4)

class I2CC{
public:
    I2CC(I2C_TypeDef* _i2cx, DMA_TypeDef *_dmax, uint32_t _rx_dma_ch, uint32_t _tx_dma_ch);

    virtual void init();

    void rx_dma_TC();
    void tx_dma_TC();

protected:
    I2C_TypeDef* i2cx_;
    DMA_TypeDef* dmax_;
    uint32_t rx_dma_ch_;
    uint32_t tx_dma_ch_;
	bool is_rxDMA_completed = false;
	bool is_txDMA_completed = false;

    uint8_t rxbuffer_[BUFFER_LENGTH] = {};
    uint8_t txbuffer_[BUFFER_LENGTH] = {};

    uint8_t u8_device_ic2_addr = 0b10001110;

    void writes_with_dma(uint32_t _dnum);
    void reads_with_dma(uint32_t _dnum);

    void writes(uint32_t _dnum);
    void reads(uint8_t _reg, uint32_t _dnum);

};


#endif
