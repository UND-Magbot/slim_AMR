/* ISR timing probe (2026-09-28).
 *
 * USART1 still overran ~0.9% of bytes with its 16-byte RX FIFO on and its
 * IRQ at priority 2, so something blocks interrupts for more than ~1.4 ms.
 * These counters use the DWT cycle counter to find out what. Read and reset
 * them with the 'u' console command.
 */
#ifndef ISR_PROBE_H
#define ISR_PROBE_H

#include "stm32h7xx.h"
#include <stdint.h>

extern volatile uint32_t g_probeTim6Last;     /* cycle stamp of last TIM6 entry */
extern volatile uint32_t g_probeTim6GapMax;   /* longest gap between TIM6 entries */
extern volatile uint32_t g_probeOtgMax;       /* longest OTG_FS handler */
extern volatile uint32_t g_probeU2Max;        /* longest USART2 handler */
extern volatile uint32_t g_probeTickMax;      /* longest SysTick handler */
extern volatile uint32_t g_probeU1Max;        /* longest USART1 handler */
extern volatile uint32_t g_probeU1BurstMax;   /* most bytes drained in one USART1 entry */

static inline uint32_t probe_now(void) { return DWT->CYCCNT; }

static inline void probe_max(volatile uint32_t *m, uint32_t v)
{
  if (v > *m) { *m = v; }
}

void ISR_ProbeInit(void);

#endif
