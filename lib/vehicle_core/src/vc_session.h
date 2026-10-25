// Replay protection and session tracking for the authenticated local link.
//
// ESP-NOW peer encryption (CCMP with a per-peer LMK) authenticates who sent a
// frame, but this layer decides whether an authentic frame is *fresh*:
//
//  * The sensor's bootId is a counter persisted in its NVS and incremented on
//    every boot; its bootNonce is random per boot. The gateway persists the
//    highest bootId it has granted a session to, with that boot's nonce, and
//    refuses Hello frames from an older boot, or from the same boot id with a
//    different nonce (a boot id reused after the sensor's NVS was erased).
//  * Each Hello is answered with a fresh random 32-bit session id from the
//    gateway. All later sensor frames must carry that session id, so frames
//    captured in an earlier session (or before a gateway reboot) are rejected.
//  * Within a session, seq numbers pass through a 64-entry sliding window.
//
// A replayed Hello from the *current* sensor boot can force a new session (an
// availability nuisance: the sensor re-handshakes on its next SessionReject),
// but it can never cause stale telemetry or events to be accepted.
#pragma once

#include <stdint.h>

namespace vc {

class ReplayWindow {
 public:
  static const uint32_t kWindow = 64;

  void reset() {
    highest_ = 0;
    bitmap_ = 0;
  }

  bool wouldAccept(uint32_t seq) const {
    if (seq == 0) return false;
    if (seq > highest_) return true;
    uint32_t offset = highest_ - seq;
    if (offset >= kWindow) return false;
    return (bitmap_ & (1ULL << offset)) == 0;
  }

  // Checks and records seq. Returns false for duplicates, too-old and zero seq.
  bool accept(uint32_t seq) {
    if (!wouldAccept(seq)) return false;
    if (seq > highest_) {
      uint32_t shift = seq - highest_;
      bitmap_ = shift >= kWindow ? 0 : bitmap_ << shift;
      highest_ = seq;
      bitmap_ |= 1;
    } else {
      bitmap_ |= 1ULL << (highest_ - seq);
    }
    return true;
  }

  uint32_t highest() const { return highest_; }

 private:
  uint32_t highest_ = 0;
  uint64_t bitmap_ = 0;
};

class PeerSession {
 public:
  enum class Verdict : uint8_t { Accept, NoSession, StaleBoot, WrongSession, Replay };

  // Highest sensor boot ever granted a session and its boot nonce (loaded
  // from NVS at gateway boot; nonce 0 = unknown, accepted once and recorded).
  void restorePersistedBoot(uint32_t boot, uint32_t bootNonce = 0) {
    persistedBoot_ = boot;
    persistedNonce_ = bootNonce;
  }
  uint32_t persistedBoot() const { return persistedBoot_; }
  uint32_t persistedNonce() const { return persistedNonce_; }

  // Handles a Hello. `freshSessionId` must come from a hardware RNG and be
  // non-zero. Returns the granted session id, or 0 if the Hello is refused.
  // When the granted (boot, nonce) differs from the persisted one,
  // *persistNeeded is set so the caller can write both to NVS.
  uint32_t onHello(uint32_t boot, uint32_t bootNonce, uint32_t freshSessionId, bool* persistNeeded) {
    if (persistNeeded) *persistNeeded = false;
    if (boot == 0 || bootNonce == 0 || freshSessionId == 0) return 0;
    if (boot < persistedBoot_) return 0;
    if (boot == persistedBoot_ && persistedNonce_ != 0 && bootNonce != persistedNonce_) return 0;
    if (boot > persistedBoot_ || bootNonce != persistedNonce_) {
      persistedBoot_ = boot;
      persistedNonce_ = bootNonce;
      if (persistNeeded) *persistNeeded = true;
    }
    boot_ = boot;
    session_ = freshSessionId;
    window_.reset();
    active_ = true;
    return session_;
  }

  Verdict classify(uint32_t boot, uint32_t session, uint32_t seq) const {
    if (!active_) return Verdict::NoSession;
    if (boot < persistedBoot_) return Verdict::StaleBoot;
    if (boot != boot_ || session != session_) return Verdict::WrongSession;
    if (!window_.wouldAccept(seq)) return Verdict::Replay;
    return Verdict::Accept;
  }

  // classify() followed by recording seq on acceptance.
  Verdict onData(uint32_t boot, uint32_t session, uint32_t seq) {
    Verdict v = classify(boot, session, seq);
    if (v == Verdict::Accept) window_.accept(seq);
    return v;
  }

  void drop() { active_ = false; }
  bool active() const { return active_; }
  uint32_t boot() const { return boot_; }
  uint32_t session() const { return session_; }

 private:
  uint32_t persistedBoot_ = 0;
  uint32_t persistedNonce_ = 0;
  uint32_t boot_ = 0;
  uint32_t session_ = 0;
  bool active_ = false;
  ReplayWindow window_;
};

// Remembers recently accepted (boot, eventCounter) pairs so a retransmitted
// event is acknowledged again but not processed twice.
template <unsigned N>
class RecentEventIds {
 public:
  bool contains(uint32_t boot, uint32_t counter) const {
    for (unsigned i = 0; i < count_; ++i)
      if (ids_[i].boot == boot && ids_[i].counter == counter) return true;
    return false;
  }
  void insert(uint32_t boot, uint32_t counter) {
    ids_[next_].boot = boot;
    ids_[next_].counter = counter;
    next_ = (next_ + 1) % N;
    if (count_ < N) ++count_;
  }

 private:
  struct Id {
    uint32_t boot;
    uint32_t counter;
  };
  Id ids_[N] = {};
  unsigned next_ = 0;
  unsigned count_ = 0;
};

}  // namespace vc
