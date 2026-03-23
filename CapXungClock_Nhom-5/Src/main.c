/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Clock lab for NUCLEO-F401RE
 ******************************************************************************
 */

#include <stdint.h>

/* NUCLEO-F401RE receives HSE from the ST-LINK MCO pin at 8 MHz. */
#define HSE_VALUE               ((uint32_t)8000000U)

#include "stm32f401re.h"

/*
 * Change this macro before build:
 * 1 -> Bai 1: output HSI on PA8 (MCO1) with prescaler /4
 * 2 -> Bai 2: enable HSE, switch SYSCLK to HSE, output HSE on PA8 (MCO1) /4
 */
#define ACTIVE_CLOCK_EXERCISE   1u

#if (ACTIVE_CLOCK_EXERCISE != 1u) && (ACTIVE_CLOCK_EXERCISE != 2u)
#error "ACTIVE_CLOCK_EXERCISE must be 1 or 2."
#endif

#define RCC_BASE_ADDR               (0x40023800UL)
#define RCC_CR_REG_OFFSET           (0x00UL)
#define RCC_CFGR_REG_OFFSET         (0x08UL)
#define RCC_AHB1ENR_REG_OFFSET      (0x30UL)

#define RCC_CR_REG_ADDR             (RCC_BASE_ADDR + RCC_CR_REG_OFFSET)
#define RCC_CFGR_REG_ADDR           (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)
#define RCC_AHB1ENR_REG_ADDR        (RCC_BASE_ADDR + RCC_AHB1ENR_REG_OFFSET)

#define GPIOA_BASE_ADDR             (0x40020000UL)
#define GPIOA_MODER_REG_OFFSET      (0x00UL)
#define GPIOA_OSPEEDR_REG_OFFSET    (0x08UL)
#define GPIOA_AFRH_REG_OFFSET       (0x24UL)

#define GPIOA_MODER_REG_ADDR        (GPIOA_BASE_ADDR + GPIOA_MODER_REG_OFFSET)
#define GPIOA_OSPEEDR_REG_ADDR      (GPIOA_BASE_ADDR + GPIOA_OSPEEDR_REG_OFFSET)
#define GPIOA_AFRH_REG_ADDR         (GPIOA_BASE_ADDR + GPIOA_AFRH_REG_OFFSET)

#define RCC_CR_HSEON_BIT        16u
#define RCC_CR_HSERDY_BIT       17u
#define RCC_CR_HSEBYP_BIT       18u

#define RCC_CFGR_SW_SHIFT       0u
#define RCC_CFGR_SWS_SHIFT      2u
#define RCC_CFGR_MCO1_SHIFT     21u
#define RCC_CFGR_MCO1PRE_SHIFT  24u

#define GPIO_PIN_8_SHIFT        16u
#define GPIO_PIN_8_AF_SHIFT     0u

/*
 * These variables are intentionally volatile so they are easy to inspect in
 * the debugger and answer the teacher's questions.
 */
volatile uint32_t g_active_exercise = ACTIVE_CLOCK_EXERCISE;
volatile uint32_t g_expected_sysclk_hz = 16000000u;
volatile uint32_t g_expected_mco1_hz = 4000000u;
volatile uint32_t g_rcc_cr_snapshot = 0u;
volatile uint32_t g_rcc_cfgr_snapshot = 0u;
volatile uint32_t g_rcc_ahb1enr_snapshot = 0u;
volatile uint32_t g_gpioa_moder_snapshot = 0u;
volatile uint32_t g_gpioa_ospeedr_snapshot = 0u;
volatile uint32_t g_gpioa_afrh_snapshot = 0u;
volatile uint32_t g_gpioa_clock_enabled = 0u;
volatile uint32_t g_pa8_mode_is_alternate_function = 0u;
volatile uint32_t g_pa8_af_is_af0 = 0u;
volatile uint32_t g_hse_ready = 0u;
volatile uint32_t g_mco1_source_bits = 0u;
volatile uint32_t g_mco1_prescaler_bits = 0u;
volatile uint32_t g_sysclk_switch_bits = 0u;
volatile uint32_t g_sysclk_status_bits = 0u;

static void capture_debug_state(void)
{
    volatile uint32_t *pRccCrReg = (volatile uint32_t *) RCC_CR_REG_ADDR;
    volatile uint32_t *pRccCfgrReg = (volatile uint32_t *) RCC_CFGR_REG_ADDR;
    volatile uint32_t *pRccAhb1EnrReg = (volatile uint32_t *) RCC_AHB1ENR_REG_ADDR;
    volatile uint32_t *pGpioaModerReg = (volatile uint32_t *) GPIOA_MODER_REG_ADDR;
    volatile uint32_t *pGpioaOspeedrReg = (volatile uint32_t *) GPIOA_OSPEEDR_REG_ADDR;
    volatile uint32_t *pGpioaAfrhReg = (volatile uint32_t *) GPIOA_AFRH_REG_ADDR;

    g_rcc_cr_snapshot = *pRccCrReg;
    g_rcc_cfgr_snapshot = *pRccCfgrReg;
    g_rcc_ahb1enr_snapshot = *pRccAhb1EnrReg;
    g_gpioa_moder_snapshot = *pGpioaModerReg;
    g_gpioa_ospeedr_snapshot = *pGpioaOspeedrReg;
    g_gpioa_afrh_snapshot = *pGpioaAfrhReg;

    g_gpioa_clock_enabled = ((*pRccAhb1EnrReg >> 0u) & 0x1u);
    g_pa8_mode_is_alternate_function = ((*pGpioaModerReg >> GPIO_PIN_8_SHIFT) & 0x3u);
    g_pa8_af_is_af0 = ((*pGpioaAfrhReg >> GPIO_PIN_8_AF_SHIFT) & 0xFu);
    g_hse_ready = ((*pRccCrReg >> RCC_CR_HSERDY_BIT) & 0x1u);
    g_mco1_source_bits = ((*pRccCfgrReg >> RCC_CFGR_MCO1_SHIFT) & 0x3u);
    g_mco1_prescaler_bits = ((*pRccCfgrReg >> RCC_CFGR_MCO1PRE_SHIFT) & 0x7u);
    g_sysclk_switch_bits = ((*pRccCfgrReg >> RCC_CFGR_SW_SHIFT) & 0x3u);
    g_sysclk_status_bits = ((*pRccCfgrReg >> RCC_CFGR_SWS_SHIFT) & 0x3u);
}

static void enable_gpioa_clock(void)
{
    volatile uint32_t *pRccAhb1EnrReg = (volatile uint32_t *) RCC_AHB1ENR_REG_ADDR;

    *pRccAhb1EnrReg |= (1u << 0);
}

static void configure_pa8_as_mco1_output(void)
{
    volatile uint32_t *pGpioaModerReg = (volatile uint32_t *) GPIOA_MODER_REG_ADDR;
    volatile uint32_t *pGpioaOspeedrReg = (volatile uint32_t *) GPIOA_OSPEEDR_REG_ADDR;
    volatile uint32_t *pGpioaAfrhReg = (volatile uint32_t *) GPIOA_AFRH_REG_ADDR;

    /* PA8 -> Alternate Function mode. */
    *pGpioaModerReg &= ~(0x3u << GPIO_PIN_8_SHIFT);
    *pGpioaModerReg |= (0x2u << GPIO_PIN_8_SHIFT);

    /* Increase PA8 output speed for the clock signal. */
    *pGpioaOspeedrReg &= ~(0x3u << GPIO_PIN_8_SHIFT);
    *pGpioaOspeedrReg |= (0x3u << GPIO_PIN_8_SHIFT);

    /* PA8 AFRH[3:0] = AF0, which maps to MCO1. */
    *pGpioaAfrhReg &= ~(0xFu << GPIO_PIN_8_AF_SHIFT);
}

static void run_bai_1_hsi_mco1(void)
{
    /*
     * Buoc 2 + Buoc 3:
     * Khai bao con tro thanh ghi theo dung format cua tai lieu.
     */
    volatile uint32_t *pRccCfgrReg = (volatile uint32_t *) RCC_CFGR_REG_ADDR;
    volatile uint32_t *pRccAhb1Enr = (volatile uint32_t *) RCC_AHB1ENR_REG_ADDR;
    volatile uint32_t *pGpioaModeReg = (volatile uint32_t *) GPIOA_MODER_REG_ADDR;
    volatile uint32_t *pGpioaAltFunHighReg = (volatile uint32_t *) GPIOA_AFRH_REG_ADDR;

    /*
     * Buoc 4:
     * Cau hinh truong bit MCO1 cua RCC_CFGR de lua chon HSI.
     */
    *pRccCfgrReg &= ~(0x3u << RCC_CFGR_MCO1_SHIFT);

    /*
     * Buoc 5:
     * Cai dat he so chia MCO1 la 4.
     */
    *pRccCfgrReg |= (1u << 25);
    *pRccCfgrReg |= (1u << 26);

    /*
     * Buoc 6:
     * Cap clock cho GPIOA va dua PA8 sang alternate function AF0.
     */
    *pRccAhb1Enr |= (1u << 0);
    *pGpioaModeReg &= ~(0x3u << 16);
    *pGpioaModeReg |= (0x2u << 16);
    *pGpioaAltFunHighReg &= ~(0x0Fu << 0);

    /* Tang toc do chan PA8 cho tin hieu clock. */
    configure_pa8_as_mco1_output();

    SystemCoreClockUpdate();
    g_expected_sysclk_hz = SystemCoreClock;
    g_expected_mco1_hz = (HSI_VALUE / 4u);
}

static void run_bai_2_hse_sysclk_and_mco1(void)
{
    /*
     * Buoc 2 + Buoc 3:
     * Dinh nghia dia chi va khai bao con tro thanh ghi theo format tai lieu.
     */
    volatile uint32_t *pRccCrReg = (volatile uint32_t *) RCC_CR_REG_ADDR;
    volatile uint32_t *pRccCfgrReg = (volatile uint32_t *) RCC_CFGR_REG_ADDR;
    volatile uint32_t *pRccAhb1Enr = (volatile uint32_t *) RCC_AHB1ENR_REG_ADDR;
    volatile uint32_t *pGpioaModeReg = (volatile uint32_t *) GPIOA_MODER_REG_ADDR;
    volatile uint32_t *pGpioaAltFunHighReg = (volatile uint32_t *) GPIOA_AFRH_REG_ADDR;

    /*
     * Buoc 4:
     * Cho phep xung HSE hoat dong bang bit HSEON.
     * NUCLEO-F401RE nhan xung ngoai tu ST-LINK MCO, nen can bat HSEBYP.
     */
    *pRccCrReg |= (1u << RCC_CR_HSEBYP_BIT);
    *pRccCrReg |= (1u << RCC_CR_HSEON_BIT);

    /*
     * Buoc 5:
     * Doi cho HSE san sang.
     */
    while (((*pRccCrReg >> RCC_CR_HSERDY_BIT) & 0x1u) == 0u) {
    }

    /*
     * Buoc 6:
     * Chuyen System Clock sang HSE.
     */
    *pRccCfgrReg &= ~(0x3u << RCC_CFGR_SW_SHIFT);
    *pRccCfgrReg |= (1u << 0);

    while (((*pRccCfgrReg >> RCC_CFGR_SWS_SHIFT) & 0x3u) != 0x1u) {
    }

    /*
     * Buoc 7:
     * Chon HSE lam nguon cap cho MCO1.
     * Tai lieu viet "clear 21 and set 22", minh lam ro rang hon bang cach
     * xoa ca truong MCO1 truoc, sau do set bit 22.
     */
    *pRccCfgrReg &= ~(0x3u << RCC_CFGR_MCO1_SHIFT);
    *pRccCfgrReg |= (1u << 22);

    /*
     * Buoc 8:
     * Cai dat he so chia MCO1 la 4.
     */
    *pRccCfgrReg |= (1u << 25);
    *pRccCfgrReg |= (1u << 26);

    /*
     * Buoc 9:
     * Cau hinh chan PA8 o che do alternate function cho MCO1.
     */
    *pRccAhb1Enr |= (1u << 0);
    *pGpioaModeReg &= ~(0x3u << 16);
    *pGpioaModeReg |= (0x2u << 16);
    *pGpioaAltFunHighReg &= ~(0x0Fu << 0);

    /* Tang toc do chan PA8 de xuat clock on dinh hon. */
    configure_pa8_as_mco1_output();

    SystemCoreClockUpdate();
    g_expected_sysclk_hz = SystemCoreClock;
    g_expected_mco1_hz = (HSE_VALUE / 4u);
}

int main(void)
{
    enable_gpioa_clock();
    configure_pa8_as_mco1_output();

    if (g_active_exercise == 1u) {
        run_bai_1_hsi_mco1();
    } else {
        run_bai_2_hse_sysclk_and_mco1();
    }

    capture_debug_state();

    for (;;) {
        capture_debug_state();
    }
}
