#include "ad9834.h"

#include <math.h>

/* AD9834 时钟 */
#define AD9834_MCLK 75000000UL

/**
 * @brief AD9834 spi 写数据
 *
 * @param self 实例
 * @param data 数据
 * @param len 长度
 */
static inline void AD9834_SPIWrite(ad9834_t *self, uint8_t *data, size_t len)
{
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(self->hspi, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief AD9834 初始化
 *
 * @param self 实例
 * @param cs_pin CS 引脚号
 * @param cs_port CS 引脚端口
 */
void AD9834_Init(ad9834_t *self, SPI_HandleTypeDef *hspi, uint16_t cs_pin, GPIO_TypeDef *cs_port)
{
    self->hspi = hspi;
    self->cs_pin = cs_pin;
    self->cs_port = cs_port;
    self->control_reg.value = 0;
    self->control_reg.b28 = AD9834_BitSet;
}

/**
 * @brief AD9834 复位/释放复位
 *
 * @param self 实例
 * @param reset AD9834_BitSet=复位, AD9834_BitReset=释放复位
 */
void AD9834_SetReset(ad9834_t *self, ad9834_bit_t reset)
{
    self->control_reg.reset = reset;
    uint8_t set_data[] = {(self->control_reg.value >> 8) & 0xFF, self->control_reg.value & 0xFF};
    AD9834_SPIWrite(self, set_data, sizeof(set_data));
}

/**
 * @brief AD9834 设置频率
 *
 * @param self 实例
 * @param freg 频率寄存器
 * @param frequency 频率(Hz)
 */
void AD9834_SetFrequency(ad9834_t *self, ad9834_frequency_reg_t freg, uint32_t frequency)
{
    uint32_t data = (uint32_t)(frequency * pow(2, 28) / (double)AD9834_MCLK);
    ad9834_frequency_register_t msb = {
        .reserved_db15 = freg == FREQ0_REG ? AD9834_BitReset : AD9834_BitSet,
        .reserved_db14 = freg == FREQ0_REG ? AD9834_BitSet : AD9834_BitReset,
        .data = (uint16_t)(data >> 14),
    };
    ad9834_frequency_register_t lsb = {
        .reserved_db15 = freg == FREQ0_REG ? AD9834_BitReset : AD9834_BitSet,
        .reserved_db14 = freg == FREQ0_REG ? AD9834_BitSet : AD9834_BitReset,
        .data = (uint16_t)(data & 0x3FFF),
    };

    uint8_t spi_data[] = {lsb.value >> 8, lsb.value & 0xFF, msb.value >> 8, msb.value & 0xFF};
    AD9834_SPIWrite(self, spi_data, sizeof(spi_data));
}

/**
 * @brief AD9834 选择频率寄存器输出
 *
 * @param self 实例
 * @param freg 频率寄存器
 */
void AD9834_SelectFrequencyRegOutput(ad9834_t *self, ad9834_frequency_reg_t freg)
{
    self->control_reg.fsel = freg;
    uint8_t set_data[] = {(self->control_reg.value >> 8) & 0xFF, self->control_reg.value & 0xFF};
    AD9834_SPIWrite(self, set_data, sizeof(set_data));
}

/**
 * @brief AD9834 设置相位寄存器输出角度
 *
 * @param self 实例
 * @param preg 相位寄存器
 * @param angle 角度(0~360)
 */
void AD9834_SetPhase(ad9834_t *self, ad9834_phase_reg_t preg, uint16_t angle)
{
    uint16_t data = (uint16_t)(pow(2, 12) / 360.0 * angle);
    ad9834_phase_register_t reg = {
        .reserved_db15 = AD9834_BitSet,
        .reserved_db14 = AD9834_BitSet,
        .reserved_db13 = preg == PHASE0_REG ? AD9834_BitReset : AD9834_BitSet,
        .data = (uint16_t)data,
    };

    uint8_t spi_data[] = {reg.value >> 8, reg.value & 0xFF};
    AD9834_SPIWrite(self, spi_data, sizeof(spi_data));
}

/**
 * @brief AD9834 选择相位寄存器输出
 *
 * @param self 实例
 * @param preg 相位寄存器
 */
void AD9834_SelectPhaseRegOutput(ad9834_t *self, ad9834_phase_reg_t preg)
{
    self->control_reg.psel = preg;
    uint8_t set_data[] = {(self->control_reg.value >> 8) & 0xFF, self->control_reg.value & 0xFF};
    AD9834_SPIWrite(self, set_data, sizeof(set_data));
}

/**
 * @brief AD9834 设置输出波形类型
 *
 * @param self 实例
 * @param wave 波形类型
 */
void AD9834_SetOutputWave(ad9834_t *self, ad9834_wave_t wave)
{
    switch (wave)
    {
    case AD9834_WaveSine:
        self->control_reg.opbiten = AD9834_BitReset;
        self->control_reg.mode = AD9834_BitReset;
        self->control_reg.div2 = AD9834_BitReset;
        break;
    case AD9834_WaveTriangle:
        self->control_reg.opbiten = AD9834_BitReset;
        self->control_reg.mode = AD9834_BitSet;
        self->control_reg.div2 = AD9834_BitReset;
        break;
    case AD9834_WaveSineWithSquare:
        self->control_reg.opbiten = AD9834_BitSet;
        self->control_reg.mode = AD9834_BitReset;
        self->control_reg.div2 = AD9834_BitSet;
        break;
    }
    uint8_t set_data[] = {(self->control_reg.value >> 8) & 0xFF, self->control_reg.value & 0xFF};
    AD9834_SPIWrite(self, set_data, sizeof(set_data));
}
