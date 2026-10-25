#include "vc_log.h"

#include <stdarg.h>
#include <stdio.h>

namespace vc {

namespace {
LogSink g_sink = nullptr;
}

void setLogSink(LogSink sink) { g_sink = sink; }

void logf(const char* fmt, ...) {
  if (!g_sink) return;
  char line[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);  // truncates long lines, always terminated
  va_end(ap);
  g_sink(line);
}

}  // namespace vc
