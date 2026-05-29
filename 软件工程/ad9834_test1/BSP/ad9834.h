#pragma once
#pragma anon_unions

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 角度范围 */
#define AD9834_ANGLE_MIN 0
#define AD9834_ANGLE_MAX 360

/* 频率范围 */
#define AD9834_FREQUENCY_MIN 0
#define AD9834_SQUARE_TRIANGLKE_FREQUENCY_MAX 1000000
#define AD9834_FREQUENCY_MAX 30000000

/* 频率寄存器 */
typedef enum AD9834_FrequencyReg
{
    FREQ0_REG = 0,
    FREQ1_REG = 1,
} ad9834_frequency_reg_t;

/* 相位寄存器 */
typedef enum AD9834_PhaseReg
{
    PHASE0_REG = 0,
    PHASE1_REG = 1,
} ad9834_phase_reg_t;

/**
 * @brief AD9834 波形类型
 *
 */
typedef enum AD9834_Wave
{
    AD9834_WaveSine = 0,
    AD9834_WaveTriangle = 1,
    AD9834_WaveSineWithSquare = 2,
} ad9834_wave_t;

/**
 * @brief AD9834 位
 *
 */
typedef enum AD9834_Bit
{
    AD9834_BitReset = 0,
    AD9834_BitSet,
} ad9834_bit_t;

/**
 * @brief AD9834 控制寄存器
 *
 */
typedef union AD9834_ControlRegister {
    uint16_t value;
    struct
    {
        uint16_t reserved_db0 : 1;
        uint16_t mode : 1;
        uint16_t reserved_db2 : 1;
        uint16_t div2 : 1;
        uint16_t sign_pib : 1;
        uint16_t opbiten : 1;
        uint16_t sleep12 : 1;
        uint16_t sleep1 : 1;
        uint16_t reset : 1;
        uint16_t pin_sw : 1;
        uint16_t psel : 1;
        uint16_t fsel : 1;
        uint16_t hlb : 1;
        uint16_t b28 : 1;
        uint16_t reserved_db14 : 1;
        uint16_t reserved_db15 : 1;
    };
} ad9834_control_register_t;

/**
 * @brief AD9834 频率寄存器
 *
 */
typedef union AD9834_FrequencyRegister {
    uint16_t value;
    struct
    {
        uint16_t data : 14;
        uint16_t reserved_db14 : 1;
        uint16_t reserved_db15 : 1;
    };
} ad9834_frequency_register_t;

/**
 * @brief AD9834 相位寄存器
 *
 */
typedef union AD9834_PhaseRegister {
    uint16_t value;
    struct
    {
        uint16_t data : 12;
        uint16_t reserved_db12 : 1;
        uint16_t reserved_db13 : 1;
        uint16_t reserved_db14 : 1;
        uint16_t reserved_db15 : 1;
    };
} ad9834_phase_register_t;

/**
 * @brief AD9834 实例结构体
 *
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        /* SPI 句柄 */
    uint16_t cs_pin;                /* 片选引脚号 */
    GPIO_TypeDef *cs_port;          /* 片选引脚端口 */
    ad9834_control_register_t control_reg; /* 控制寄存器影子状态 */
} ad9834_t;

/* AD9834 实例方法 */
void AD9834_Init(ad9834_t *self, SPI_HandleTypeDef *hspi, uint16_t cs_pin, GPIO_TypeDef *cs_port);
void AD9834_SetReset(ad9834_t *self, ad9834_bit_t reset);
void AD9834_SetFrequency(ad9834_t *self, ad9834_frequency_reg_t freg, uint32_t frequency);
void AD9834_SetPhase(ad9834_t *self, ad9834_phase_reg_t preg, uint16_t angle);
void AD9834_SelectFrequencyRegOutput(ad9834_t *self, ad9834_frequency_reg_t fsel);
void AD9834_SelectPhaseRegOutput(ad9834_t *self, ad9834_phase_reg_t preg);
void AD9834_SetOutputWave(ad9834_t *self, ad9834_wave_t wave);
