/**
 * @file panic.c
 * @author Ian Frosst
 * @brief Panic & hard fault handler
 * @date 2020-01-18
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <stdio.h>
#include "stm32f7xx.h"

enum { r0, r1, r2, r3, r12, lr, pc, psr };

__attribute__((used)) static void fault(uint32_t stack[]) {
  fprintf(stderr, "Hard fault!\r\n");
  fprintf(stderr, "  r0   = %08lx\r\n", stack[r0]);
  fprintf(stderr, "  r1   = %08lx\r\n", stack[r1]);
  fprintf(stderr, "  r2   = %08lx\r\n", stack[r2]);
  fprintf(stderr, "  r3   = %08lx\r\n", stack[r3]);
  fprintf(stderr, "  r12  = %08lx\r\n", stack[r12]);
  fprintf(stderr, "  lr   = %08lx\r\n", stack[lr]);
  fprintf(stderr, "  pc   = %08lx\r\n", stack[pc]);
  fprintf(stderr, "  psr  = %08lx\r\n", stack[psr]);
}


/**
 * @brief Pure ASM function to determine which stack pointer we're using
 * and set r0 (first argument), for calling `fault`
 *
 */
__attribute__((naked,used)) static void HardFault_Handler(void) {
  __asm volatile(
    "tst  lr, #4      \n"
    "ite  eq          \n"
    "mrseq  r0, msp   \n"
    "mrsne  r0, psp   \n"
    "b fault          \n"
  );
}
