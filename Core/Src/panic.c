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


static void HardFault(const uint32_t stack[]);
static void MemManage(const uint32_t stack[]);
static void BusFault(const uint32_t stack[]);
static void UsageFault(const uint32_t stack[]);

// Registers pushed to stack on exception entry
enum { r0, r1, r2, r3, r12, lr, pc, psr };

void __attribute__((used)) panic(const uint32_t stack[]) {
  int32_t irqn = __get_IPSR() - 16;
  fprintf(stderr, "Unexpected IRQ %ld!\r\n", irqn);
  fprintf(stderr, "  r0   = %08lx\r\n", stack[r0]);
  fprintf(stderr, "  r1   = %08lx\r\n", stack[r1]);
  fprintf(stderr, "  r2   = %08lx\r\n", stack[r2]);
  fprintf(stderr, "  r3   = %08lx\r\n", stack[r3]);
  fprintf(stderr, "  r12  = %08lx\r\n", stack[r12]);
  fprintf(stderr, "  lr   = %08lx\r\n", stack[lr]);
  fprintf(stderr, "  pc   = %08lx\r\n", stack[pc]);
  fprintf(stderr, "  psr  = %08lx\r\n", stack[psr]);

  switch (irqn) {
    case -13 /* HardFault */:
      HardFault(stack);
      break;
    case MemoryManagement_IRQn:
      MemManage(stack);
      break;
    case BusFault_IRQn:
      BusFault(stack);
      break;
    case UsageFault_IRQn:
      UsageFault(stack);
      break;
    default:
      break;
  }

  while (1) {
  }
}

static void HardFault(const uint32_t stack[]) {
  uint32_t hfsr = SCB->HFSR;

  if ((hfsr & SCB_HFSR_VECTTBL_Msk) != 0) {
    fprintf(stderr, "BusFault on vector talbe read!\r\n");
    BusFault(stack);
  } else if ((hfsr & SCB_HFSR_FORCED_Msk) != 0) {
    fprintf(stderr, "Forced HardFault occurred!\r\n");
    uint32_t cfsr = SCB->CFSR;
    if ((cfsr & SCB_CFSR_MEMFAULTSR_Msk) != 0) {
      MemManage(stack);
    }
    if ((cfsr & SCB_CFSR_BUSFAULTSR_Msk) != 0) {
      BusFault(stack);
    }
    if ((cfsr & SCB_CFSR_USGFAULTSR_Msk) != 0) {
      UsageFault(stack);
    }
  }
}

static void MemManage(const uint32_t stack[]) {
  uint32_t cfsr = SCB->CFSR;

  fprintf(stderr, "MemoryManagementFault occurred!\r\n");

  if ((cfsr & SCB_CFSR_MMARVALID_Msk) != 0) {
    fprintf(stderr, "at address %08lx\r\n", SCB->MMFAR);
    SCB->CFSR = SCB_CFSR_MMARVALID_Msk; // write 1 to clear bit
  }
  if ((cfsr & SCB_CFSR_IACCVIOL_Msk) != 0) {
    fprintf(stderr, "on instruction access violation\r\n");
  }
  if ((cfsr & SCB_CFSR_DACCVIOL_Msk) != 0) {
    fprintf(stderr, "on data access violation\r\n");
  }
  if ((cfsr & SCB_CFSR_MUNSTKERR_Msk) != 0) {
    fprintf(stderr, "on unstacking for exception return\r\n");
  }
  if ((cfsr & SCB_CFSR_MSTKERR_Msk) != 0) {
    fprintf(stderr, "on stacking for exception entry\r\n");
  }
  if ((cfsr & SCB_CFSR_MLSPERR_Msk) != 0) {
    fprintf(stderr, "during floating-point lazy state preservation\r\n");
  }
}

static void BusFault(const uint32_t stack[]) {
  uint32_t cfsr = SCB->CFSR;

  fprintf(stderr, "BusFault occurred!\r\n");

  if ((cfsr & SCB_CFSR_BFARVALID_Msk) != 0) {
    fprintf(stderr, "at address %08lx\r\n", SCB->BFAR);
    SCB->CFSR = SCB_CFSR_BFARVALID_Msk;
  }
  if ((cfsr & SCB_CFSR_IBUSERR_Msk) != 0) {
    fprintf(stderr, "on instruction prefetch\r\n");
  }
  if ((cfsr & SCB_CFSR_PRECISERR_Msk) != 0) {
    fprintf(stderr, "Precise error\r\n");
  }
  if ((cfsr & SCB_CFSR_IMPRECISERR_Msk) != 0) {
    fprintf(stderr, "Imprecise error\r\n");
  }
  if ((cfsr & SCB_CFSR_UNSTKERR_Msk) != 0) {
    fprintf(stderr, "on unstacking for exception return\r\n");
  }
  if ((cfsr & SCB_CFSR_STKERR_Msk) != 0) {
    fprintf(stderr, "on stacking for exception entry\r\n");
  }
  if ((cfsr & SCB_CFSR_LSPERR_Msk) != 0) {
    fprintf(stderr, "during floating-point lazy state preservation\r\n");
  }
}

static void UsageFault(const uint32_t stack[]) {
  uint32_t cfsr = SCB->CFSR;

  fprintf(stderr, "UsageFault occurred!\r\n");

  if ((cfsr & SCB_CFSR_UNDEFINSTR_Msk) != 0) {
    fprintf(stderr, "Undefined instruction\r\n");
  }
  if ((cfsr & SCB_CFSR_INVSTATE_Msk) != 0) {
    fprintf(stderr, "Illegal EPSR access\r\n");
  }
  if ((cfsr & SCB_CFSR_INVPC_Msk) != 0) {
    fprintf(stderr, "Invalid PC load on EXC_RETURN\r\n");
  }
  if ((cfsr & SCB_CFSR_NOCP_Msk) != 0) {
    fprintf(stderr, "Attempted to access non-existent coprocessor\r\n");
  }
  if ((cfsr & SCB_CFSR_UNALIGNED_Msk) != 0) {
    fprintf(stderr, "Unaligned access\r\n");
  }
  if ((cfsr & SCB_CFSR_DIVBYZERO_Msk) != 0) {
    fprintf(stderr, "Divide by zero\r\n");
  }
}
