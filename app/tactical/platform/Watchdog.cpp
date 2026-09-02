/*******************************************************************************
 * @file Watchdog.cpp
 * @brief Independent watchdog bring-up and refresh (RM0481 §41).
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "platform/Watchdog.hpp"

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>

/* File-local constants ------------------------------------------------------*/

/*******************************************************************************
 * @brief IWDG key register values (RM0481 §41.4.1).
 *
 * The key register is write-only and needs no read-modify-write, which is why
 * losing HAL_IWDG_Init() costs nothing here.
 ******************************************************************************/
namespace IwdgKey
{
  /// Reload the down-counter from IWDG_RLR.
  static constexpr uint32_t kReload = 0x0000AAAAU;

  /// Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR. Any other key
  /// value written afterwards closes the window again.
  static constexpr uint32_t kWriteAccess = 0x00005555U;

  /// Start the counter. There is no key that stops it again.
  static constexpr uint32_t kStart = 0x0000CCCCU;
} // namespace IwdgKey

/*******************************************************************************
 * @brief Prescaler and reload chosen for Watchdog::kReloadMs.
 *
 * LSI is LSI_VALUE (32 kHz nominal, stm32h5xx_hal_conf.h). IWDG_PR code 5 is a
 * /128 divide, giving a 250 Hz counter and so 4 ms per count; 125 counts is
 * therefore 500 ms.
 *
 * The divider was picked to put the reload value in the middle of IWDG_RLR's
 * 12-bit range rather than at either end: at /4 the same period needs 4000
 * counts and there is almost no room left to lengthen the period later, and at
 * /1024 one count is 32 ms, which is coarser than the thing being measured.
 * 4 ms per count keeps the period adjustable in both directions in steps far
 * finer than the 100 ms heartbeat tick that sets it.
 ******************************************************************************/
namespace IwdgTiming
{
  /// IWDG_PR code for a /128 divide of the LSI.
  static constexpr uint32_t kPrescalerCode = 5U;

  /// LSI divider the code above selects. Kept beside it so the arithmetic
  /// below is checkable without the reference manual open.
  static constexpr uint32_t kPrescalerDivider = 128U;

  /// Counter frequency in Hz, after the prescaler.
  static constexpr uint32_t kCounterHz = LSI_VALUE / kPrescalerDivider;

  /// IWDG_RLR value: the counts in Watchdog::kReloadMs. 125 at 250 Hz.
  static constexpr uint32_t kReloadCounts = (kCounterHz * Watchdog::kReloadMs) / 1000U;

  static_assert(kReloadCounts <= 0x0FFFU, "IWDG_RLR is 12 bits - lengthen the prescaler");
  static_assert(kReloadCounts > 0U, "Reload rounds to zero - shorten the prescaler");
} // namespace IwdgTiming

/// Milliseconds to wait for RCC_BDCR_LSIRDY. The LSI needs tens of
/// microseconds; this is the HAL's own RCC_LSI_TIMEOUT_VALUE, generously round.
static constexpr uint32_t kLsiReadyTimeoutMs = 2U;

/// Milliseconds to wait for IWDG_SR to report the prescaler and reload writes
/// transferred into the LSI clock domain. The transfer takes a few LSI cycles,
/// so single-digit milliseconds is already an outlier; 10 ms means "never".
static constexpr uint32_t kRegisterUpdateTimeoutMs = 10U;

/* Public interface ----------------------------------------------------------*/

bool watchdogStart()
{
  // Before the counter starts, not after: a debugger attached at reset can halt
  // the core between these two statements, and this is the bit that makes that
  // survivable. Without it, every breakpoint in the firmware becomes a reset
  // and the board appears to be crashing on code that is merely paused.
  __HAL_DBGMCU_FREEZE_IWDG();

  // The IWDG runs from the LSI, which nothing else in this firmware turns on -
  // the .ioc has no LSI consumer, so SystemClock_Config() leaves it stopped.
  // The prescaler and reload writes below are synchronised into the LSI domain,
  // so they cannot be issued until it is actually running.
  __HAL_RCC_LSI_ENABLE();

  const uint32_t lsiStartTick = HAL_GetTick();
  while ((RCC->BDCR & RCC_BDCR_LSIRDY) == 0U)
  {
    if ((HAL_GetTick() - lsiStartTick) > kLsiReadyTimeoutMs)
    {
      return false;
    }
  }

  // Start first, then configure - the order ST's own HAL_IWDG_Init() uses. The
  // counter therefore runs on its reset defaults (/4, 4095 counts, about
  // 512 ms) for the handful of microseconds the writes below take, which is
  // both harmless and, if this function were to hang in the SR poll, the reason
  // the board still resets rather than sitting there with a dead watchdog.
  IWDG->KR  = IwdgKey::kStart;
  IWDG->KR  = IwdgKey::kWriteAccess;
  IWDG->PR  = IwdgTiming::kPrescalerCode;
  IWDG->RLR = IwdgTiming::kReloadCounts;

  // IWDG_WINR is deliberately left at its reset value. A window would turn an
  // early refresh into a reset, and the heartbeat task's refresh is gated on
  // the control task's liveness counter, so its spacing is a property of the
  // control cycle rather than something this file can promise.

  // PVU and RVU stay set until the values have crossed into the LSI domain.
  // Reloading before then would reload from the old RLR.
  const uint32_t updateStartTick = HAL_GetTick();
  while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
  {
    if ((HAL_GetTick() - updateStartTick) > kRegisterUpdateTimeoutMs)
    {
      return false;
    }
  }

  // Load the new reload value into the counter and close the write window.
  IWDG->KR = IwdgKey::kReload;

  return true;
}

void watchdogRefresh()
{
  IWDG->KR = IwdgKey::kReload;
}

/* EOF -----------------------------------------------------------------------*/
