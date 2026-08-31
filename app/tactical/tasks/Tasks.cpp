/*******************************************************************************
 * @file Tasks.cpp
 * @brief Creates the four application tasks of §3.1.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "tasks/Tasks.hpp"

#include "tasks/CommsTask.hpp"
#include "tasks/ControlTask.hpp"
#include "tasks/HeartbeatTask.hpp"
#include "tasks/TelemetryTask.hpp"

bool tasksCreateAll()
{
  // Every one is attempted even if an earlier one failed, so that a single
  // report tells you whether it was one task or the heap.
  bool ok = commsTaskCreate();
  ok      = telemetryTaskCreate() && ok;
  ok      = heartbeatTaskCreate() && ok;
  ok      = controlTaskCreate() && ok;

  return ok;
}

/* EOF -----------------------------------------------------------------------*/
