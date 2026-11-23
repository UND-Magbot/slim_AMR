/* lift.c - lifting outputs (2026-10-07, up moved PA1 -> PA4;
 *          2026-10-30 wiring moved PA4 / PA6 -> PC0 / PE1)
 *
 *   PC0 HIGH = lift goes up,   LOW again once it is fully up
 *   PE1 HIGH = lift goes down, LOW again once it is fully down
 *
 * There is no end sensor wired, so a move ends by time: the output is
 * dropped LOW after the up / down travel time (settable with 'l t').
 * Both outputs are never HIGH together - the other side is dropped LOW
 * before a move starts. L2DB_StopAll (x, z, boot) also stops the lift.
 *
 * Console (the line must carry a " %XXXX" CRC-16 check value):
 *   l                 show state
 *   l up [ms]         PC0 HIGH for the up time (or ms), then LOW
 *   l down [ms]       PE1 HIGH for the down time (or ms), then LOW
 *   l stop            both LOW now
 *   l t <up> <down>   set travel times in ms (RAM only, back to default
 *                     on reset)
 *
 * Both outputs start LOW. The output register is written LOW before the
 * pins are switched to output mode so they never pulse HIGH at boot.
 * While the MCU is in reset / the bootloader the pins float - the lift
 * side should hold its inputs LOW with pull-downs.                       */
#include "lift.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define LIFT_PORT_UP     GPIOC
#define LIFT_PIN_UP      GPIO_PIN_0     /* PC0 */
#define LIFT_PORT_DOWN   GPIOE
#define LIFT_PIN_DOWN    GPIO_PIN_1     /* PE1 */

#define LIFT_DEFAULT_MS  10000U         /* up / down travel time (2026-10-07) */
#define LIFT_MAX_MS      60000U         /* never drive longer than this */

typedef enum { LIFT_IDLE = 0, LIFT_UP, LIFT_DOWN } LiftState_t;

static LiftState_t s_state   = LIFT_IDLE;
static uint32_t    s_startMs = 0U;
static uint32_t    s_runMs   = 0U;
static uint32_t    s_upMs    = LIFT_DEFAULT_MS;
static uint32_t    s_downMs  = LIFT_DEFAULT_MS;

static void Lift_Outputs(uint8_t up, uint8_t down)
{
  /* Drop first, raise second: the two are never HIGH together. */
  if (!up)
  {
    HAL_GPIO_WritePin(LIFT_PORT_UP, LIFT_PIN_UP, GPIO_PIN_RESET);
  }
  if (!down)
  {
    HAL_GPIO_WritePin(LIFT_PORT_DOWN, LIFT_PIN_DOWN, GPIO_PIN_RESET);
  }
  if (up)
  {
    HAL_GPIO_WritePin(LIFT_PORT_UP, LIFT_PIN_UP, GPIO_PIN_SET);
  }
  if (down)
  {
    HAL_GPIO_WritePin(LIFT_PORT_DOWN, LIFT_PIN_DOWN, GPIO_PIN_SET);
  }
}

static uint32_t Lift_Clamp(uint32_t ms)
{
  if (ms == 0U)
  {
    return 0U;
  }
  return (ms > LIFT_MAX_MS) ? LIFT_MAX_MS : ms;
}

void Lift_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(LIFT_PORT_UP, LIFT_PIN_UP, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIFT_PORT_DOWN, LIFT_PIN_DOWN, GPIO_PIN_RESET);
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = LIFT_PIN_UP;
  HAL_GPIO_Init(LIFT_PORT_UP, &g);
  g.Pin   = LIFT_PIN_DOWN;
  HAL_GPIO_Init(LIFT_PORT_DOWN, &g);
  s_state = LIFT_IDLE;
}

void Lift_Up(uint32_t ms)
{
  s_runMs = Lift_Clamp(ms ? ms : s_upMs);
  if (s_runMs == 0U)
  {
    Lift_Stop();
    return;
  }
  Lift_Outputs(1U, 0U);
  s_state   = LIFT_UP;
  s_startMs = HAL_GetTick();
  printf("[LIFT] up start (PC0 HIGH, %lu ms)\r\n", (unsigned long)s_runMs);
}

void Lift_Down(uint32_t ms)
{
  s_runMs = Lift_Clamp(ms ? ms : s_downMs);
  if (s_runMs == 0U)
  {
    Lift_Stop();
    return;
  }
  Lift_Outputs(0U, 1U);
  s_state   = LIFT_DOWN;
  s_startMs = HAL_GetTick();
  printf("[LIFT] down start (PE1 HIGH, %lu ms)\r\n", (unsigned long)s_runMs);
}

void Lift_Stop(void)
{
  LiftState_t was = s_state;
  Lift_Outputs(0U, 0U);
  s_state = LIFT_IDLE;
  if (was != LIFT_IDLE)
  {
    printf("[LIFT] stop after %lu ms\r\n", (unsigned long)(HAL_GetTick() - s_startMs));
  }
}

/* Called from the main loop. Ends a move when its time is up. */
void Lift_Poll(void)
{
  if (s_state == LIFT_IDLE)
  {
    return;
  }
  if ((HAL_GetTick() - s_startMs) >= s_runMs)
  {
    LiftState_t was = s_state;
    Lift_Outputs(0U, 0U);
    s_state = LIFT_IDLE;
    printf("[LIFT] %s done (%lu ms) - PC0/PE1 LOW\r\n",
           (was == LIFT_UP) ? "up" : "down", (unsigned long)s_runMs);
  }
}

void Lift_Print(void)
{
  /* Read back the real pin levels, not just what was asked. */
  printf("[LIFT] %s PC0=%u PE1=%u up=%lums down=%lums\r\n",
         (s_state == LIFT_UP) ? "UP" : (s_state == LIFT_DOWN) ? "DOWN" : "IDLE",
         (unsigned)(HAL_GPIO_ReadPin(LIFT_PORT_UP, LIFT_PIN_UP) == GPIO_PIN_SET),
         (unsigned)(HAL_GPIO_ReadPin(LIFT_PORT_DOWN, LIFT_PIN_DOWN) == GPIO_PIN_SET),
         (unsigned long)s_upMs, (unsigned long)s_downMs);
}

void Lift_ConsoleCmd(char *args)
{
  char word[8] = {0};
  unsigned long a = 0UL, b = 0UL;

  while (*args == ' ')
  {
    args++;
  }
  if (*args == '\0')
  {
    Lift_Print();
    return;
  }
  if (sscanf(args, "%7s", word) != 1)
  {
    return;
  }
  const char *rest = args + strlen(word);

  if (strcmp(word, "up") == 0)
  {
    (void)sscanf(rest, "%lu", &a);
    Lift_Up((uint32_t)a);
  }
  else if (strcmp(word, "down") == 0)
  {
    (void)sscanf(rest, "%lu", &a);
    Lift_Down((uint32_t)a);
  }
  else if (strcmp(word, "stop") == 0)
  {
    Lift_Stop();
    Lift_Print();
  }
  else if (strcmp(word, "t") == 0 && sscanf(rest, "%lu %lu", &a, &b) == 2
           && a > 0UL && b > 0UL)
  {
    s_upMs   = Lift_Clamp((uint32_t)a);
    s_downMs = Lift_Clamp((uint32_t)b);
    Lift_Print();
  }
  else
  {
    printf("[LIFT] usage: l | l up [ms] | l down [ms] | l stop | l t <up_ms> <down_ms>\r\n");
  }
}
