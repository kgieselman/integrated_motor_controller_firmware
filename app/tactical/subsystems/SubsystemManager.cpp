/*******************************************************************************
 * @file SubsystemManager.cpp
 * @brief Walks the robot's subsystem array and calls the Subsystem contract.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "subsystems/SubsystemManager.hpp"

/* Construction --------------------------------------------------------------*/

SubsystemManager::SubsystemManager(std::span<Subsystem* const> subsystems)
    : m_subsystems(subsystems.data())
    , m_count(subsystems.size())
{
}

/* Dispatch ------------------------------------------------------------------*/

bool SubsystemManager::initAll()
{
  bool allOk = true;

  for (std::size_t i = 0U; i < m_count; ++i)
  {
    // Deliberately not `allOk = allOk && ...`: && short-circuits, and after the
    // first failure that would skip every remaining subsystem's onInit().
    if (!m_subsystems[i]->onInit())
    {
      allOk = false;
    }
  }

  return allOk;
}

void SubsystemManager::periodic(const RobotContext& ctx)
{
  for (std::size_t i = 0U; i < m_count; ++i)
  {
    m_subsystems[i]->onPeriodic(ctx);
  }
}

void SubsystemManager::disableAll()
{
  for (std::size_t i = 0U; i < m_count; ++i)
  {
    // No enabled/disabled bookkeeping and no early exit - see the header.
    m_subsystems[i]->onDisable();
  }
}

/* EOF -----------------------------------------------------------------------*/
