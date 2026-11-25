/**
 * @file    sm5800_hal.c
 * @brief   ARM bare-metal HAL for SM-5800 IP core.
 * @note    One source for both PS families, selected by the compiler:
 *          - __aarch64__ : Cortex-A53 (Zynq UltraScale+ APU, AArch64)
 *          - __arm__     : Cortex-A9 (Zynq-7000) and Cortex-R5 (RPU), ARMv7
 * @req     LLR-SM5800-HAL-001
 * @design  SDD-SM5800-2.2
 */

#include "sm5800_hal.h"

#include <stdint.h>

/* 32-bit word alignment mask for AXI4-Lite MMIO registers */
#define HAL_WORD_ALIGN_MASK     (0x03U)

/* IRQ mask bit: CPSR.I (ARMv7) and DAIF.I (AArch64) are both bit 7 */
#define HAL_IRQ_MASK_BIT        (0x00000080U)

#if !defined(__aarch64__) && !defined(__arm__)
#error "sm5800_hal.c: unsupported CPU (expected __aarch64__ or __arm__)"
#endif


uint32_t sm5800_hal_read_reg(uintptr_t base_addr,
                             uint32_t offset_b)
{
    uintptr_t reg_addr = base_addr + (uintptr_t)offset_b;
    uint32_t reg_word = 0U;

    SM5800_ASSERT(0U != base_addr);
    SM5800_ASSERT(0U == (reg_addr & (uintptr_t)HAL_WORD_ALIGN_MASK));

    /* MISRA deviation: M 11.4 - MMIO requires integer-to-pointer cast -
     * base_addr validated at init; offset_b is 4-byte aligned constant. */
    uint32_t const volatile * p_reg = (uint32_t const volatile *)reg_addr;
    reg_word = *p_reg;

    return reg_word;
}


void sm5800_hal_write_reg(uintptr_t base_addr,
                          uint32_t offset_b,
                          uint32_t reg_word)
{
    uintptr_t reg_addr = base_addr + (uintptr_t)offset_b;

    SM5800_ASSERT(0U != base_addr);
    SM5800_ASSERT(0U == (reg_addr & (uintptr_t)HAL_WORD_ALIGN_MASK));

    /* MISRA deviation: M 11.4 - MMIO requires integer-to-pointer cast -
     * base_addr validated at init; offset_b is 4-byte aligned constant. */
    uint32_t volatile * p_reg = (uint32_t volatile *)reg_addr;
    *p_reg = reg_word;
}


void sm5800_hal_dmb(void)
{
    /* MISRA deviation: M Dir 4.3, M 1.2 - ARM architectural DMB barrier -
     * isolated in HAL translation unit for cross-context memory ordering.
     * "sy" (full system) is valid on both ARMv7 and AArch64. */
    __asm__ volatile ("dmb sy" : : : "memory");
}


void sm5800_hal_dsb(void)
{
    /* MISRA deviation: M Dir 4.3, M 1.2 - ARM architectural DSB barrier -
     * isolated in HAL translation unit to drain AXI4-Lite posted writes.
     * "sy" (full system) is valid on both ARMv7 and AArch64. */
    __asm__ volatile ("dsb sy" : : : "memory");
}


uint32_t sm5800_hal_enter_crit(void)
{
    uint32_t result = 0U;

#if defined(__aarch64__)
    uint64_t daif_word = 0U;

    /* MISRA deviation: M Dir 4.3, M 1.2 - AArch64 DAIF IRQ mask
     * instruction (daifset #2 = mask IRQ) - isolated in HAL. */
    __asm__ volatile ("mrs %0, daif\n\t"
                      "msr daifset, #2"
                      : "=r" (daif_word)
                      :
                      : "memory");

    result = (uint32_t)(daif_word & (uint64_t)HAL_IRQ_MASK_BIT);
#else
    uint32_t cpsr_word = 0U;

    /* MISRA deviation: M Dir 4.3, M 1.2 - ARM CPSR IRQ mask instruction -
     * isolated in HAL for deterministic short critical sections on PS. */
    __asm__ volatile ("mrs %0, cpsr\n\t"
                      "cpsid i"
                      : "=r" (cpsr_word)
                      :
                      : "memory");

    result = cpsr_word & HAL_IRQ_MASK_BIT;
#endif

    return result;
}


void sm5800_hal_exit_crit(uint32_t saved_mask)
{
    if (0U == (saved_mask & HAL_IRQ_MASK_BIT))
    {
#if defined(__aarch64__)
        /* MISRA deviation: M Dir 4.3, M 1.2 - AArch64 DAIF IRQ unmask
         * (daifclr #2) - only if IRQs were enabled prior to entry. */
        __asm__ volatile ("msr daifclr, #2" : : : "memory");
#else
        /* MISRA deviation: M Dir 4.3, M 1.2 - ARM CPSR IRQ unmask -
         * restores IRQ enable only if IRQs were enabled prior to entry. */
        __asm__ volatile ("cpsie i" : : : "memory");
#endif
    }
    else
    {
        /* IRQs were already masked before entry; keep masked for nesting */
    }
}
