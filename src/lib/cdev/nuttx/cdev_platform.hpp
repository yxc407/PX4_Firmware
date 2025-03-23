
#pragma once

#include <inttypes.h>

#include <nuttx/arch.h>

#include <tc_platform_common/micro_hal.h>
//#include "/home/thuncloud/TC-Autopilot/platforms/nuttx/src/tc/common/include/tc_platform/micro_hal.h"

#define ATOMIC_ENTER irqstate_t flags = tc_enter_critical_section()
#define ATOMIC_LEAVE tc_leave_critical_section(flags)

namespace cdev
{

using tc_file_operations_t = struct file_operations;
using file_t = struct file;

} // namespace cdev
