// Line-oriented logging hook for the shared cores. The firmware routes lines
// to Serial; host tests may capture or silence them. Never log secrets.
#pragma once

namespace vc {

typedef void (*LogSink)(const char* line);

void setLogSink(LogSink sink);  // nullptr = discard

#if defined(__GNUC__)
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
#else
void logf(const char* fmt, ...);
#endif

}  // namespace vc
