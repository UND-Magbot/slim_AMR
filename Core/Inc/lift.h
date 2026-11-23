/* lift.h - lifting outputs on PC0 (up) / PE1 (down) (2026-10-07, pins 2026-10-30) */
#ifndef LIFT_H
#define LIFT_H

#include <stdint.h>

void Lift_Init(void);
void Lift_Up(uint32_t ms);
void Lift_Down(uint32_t ms);
void Lift_Stop(void);
void Lift_Poll(void);
void Lift_Print(void);
void Lift_ConsoleCmd(char *args);

#endif /* LIFT_H */
