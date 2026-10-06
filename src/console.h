// Serial console. See docs/SIMULATOR_PLAN.md section 7, "Serial console".
// Every command replies with one line starting "ok" or "err". While
// streaming, CSV lines (starting with a digit) are interleaved.
#pragma once

void console_start_task();
