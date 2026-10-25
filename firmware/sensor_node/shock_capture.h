// Interrupt-driven capture of shock-sensor pulses as "impact" records.
//
// The ISR only timestamps the edge and updates counters / a small ring under a
// spinlock. The first edge after a refractory interval starts a new impact
// record; edges inside the interval are counted as pulses of that same impact
// (vibration switches chatter). The application pops a record once its
// refractory window has closed, so its pulse count is final.
//
// Limits (see README "Impact capture"): the GPIO interrupt is edge-latched in
// hardware, so a pulse that arrives while interrupts are masked on this core
// (e.g. ~5 ms during a DHT read) is still serviced afterwards, but its
// timestamp is delayed and several edges inside that window coalesce into one.
// Pulses shorter than the module's own comparator/RC response never reach the
// pin at all; guaranteed capture of very short pulses needs a hardware latch.
#pragma once

#include <stdint.h>

struct ImpactRecord {
  uint32_t detectedMs;  // millis()-compatible time of the first edge
  uint16_t pulses;      // edges within the refractory window
};

namespace shock {

void begin(int pin, bool activeHigh, uint32_t refractoryMs);
// Returns the oldest impact whose refractory window has closed.
bool pop(ImpactRecord& out, uint32_t nowMs);
uint32_t edges();
uint32_t impacts();
uint32_t overflows();  // impacts lost because the ring was full

}  // namespace shock
