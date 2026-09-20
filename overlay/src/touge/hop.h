#pragma once
//
// Moving the ride off a channel somebody else is sitting on.
//
// 2.4 GHz is crowded and a convoy cannot pick a quiet corner in advance, so
// the group has to be able to move. The hard part is not deciding to move, it
// is moving without leaving anyone behind.
//
// ## Nothing depends on hearing the order
//
// The obvious design is a hop command from whoever is in charge. It is also
// the one that strands people: a car that misses the command sits on the old
// channel, deaf, and the thing that would tell it what happened is the thing
// it cannot hear. The failure is permanent and silent.
//
// So there is no command. Every car carries the channel it believes in, on
// every beacon, and a newer belief replaces an older one wherever the two
// meet. A car that missed the change hears it from the next car it hears from,
// not from a single announcement it had one chance at.
//
// And when it hears from nobody at all, it goes looking. The candidate set is
// three channels, so a car that has fallen off the back has three places to
// check and finds the group in a few seconds. That scan is what makes the
// whole thing safe: it is the floor under every other mechanism here, and it
// works whether the car missed a hop, was switched off during one, or joined
// the ride an hour late.

#include <stdint.h>
#include <stddef.h>

namespace touge {

// One, six and eleven: the only three channels in the 2.4 GHz band that do not
// overlap each other. Hopping between adjacent channels would move the ride
// without moving it out of the interference.
static const uint8_t FAST_CHANNELS = 3;
extern const uint8_t HOP_CHANNELS[FAST_CHANNELS];

// The generation is six bits and wraps, so "newer" is a comparison on a circle
// rather than a straight line.
static const uint8_t HOP_GEN_MASK = 0x3F;
static const uint8_t HOP_GEN_HALF = 0x20;

// True if `a` is a later generation than `b`, across the wrap.
bool hopNewer(uint8_t a, uint8_t b);

// Packs into one byte on the wire: two bits of index, six of generation.
uint8_t hopPack(uint8_t index, uint8_t generation);
void hopUnpack(uint8_t packed, uint8_t& index, uint8_t& generation);

class Hop {
 public:
  // `seed` picks which of the three the ride starts on, so two groups in the
  // same car park do not both begin on channel one.
  void begin(uint8_t seed);

  uint8_t index() const { return index_; }
  uint8_t generation() const { return generation_; }
  uint8_t channel() const { return HOP_CHANNELS[index_ % FAST_CHANNELS]; }

  // Adopt another car's belief if it is newer than ours. Returns true if we
  // moved, which is the caller's cue to retune the radio.
  bool observe(uint8_t packed);

  // Advance to the next channel and claim a new generation. Only the car
  // keeping time does this; everyone else finds out by hearing it.
  void advance();

  // Step the radio through the candidates while lost. Returns the channel to
  // listen on next. Does not touch the generation: this is a search, not a
  // decision, and a car that finds the group must take the group's answer
  // rather than imposing the one it happened to stop on.
  uint8_t scanNext();

 private:
  uint8_t index_ = 0;
  uint8_t generation_ = 0;
  uint8_t scan_ = 0;
};

} // namespace touge
