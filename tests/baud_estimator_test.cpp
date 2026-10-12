#include <cassert>
#include <cstdio>
#include "../components/toshiba_ab/baud_estimator.h"
using namespace esphome::toshiba_ab;
void known(BaudEstimator &e,uint32_t bit_us) {
  for(uint32_t i=0;i<64;++i) {
    const uint32_t bits[]={1,2,3,7};
    e.observe(bit_us*bits[i%4]+(i%3));
  }
}
int main() {
  BaudEstimator e; uint32_t rates[3];
  assert(!e.enough() && e.candidates(rates)==0);
  known(e,417); assert(e.enough() && e.candidates(rates)==1 && rates[0]==2400);
  e.reset(); known(e,208); assert(e.candidates(rates)==1 && rates[0]==4800);
  e.reset(); known(e,104); assert(e.candidates(rates)==1 && rates[0]==9600);
  e.reset(); e.set_configured(3600); known(e,278);
  assert(e.candidates(rates)>=1 && rates[0]==3600);
  // Repeated intervals can also fit faster rates with multiple-bit runs.
  e.reset(); e.set_configured(2400);
  for(int i=0;i<64;++i) e.observe(417);
  assert(e.candidates(rates)==3 && rates[0]==2400 && rates[1]==4800 && rates[2]==9600);
  // Silence and too few timings cannot produce a suggestion.
  e.reset(); for(int i=0;i<100;++i) { e.observe(0); e.observe(5000000); }
  assert(!e.enough()); for(int i=0;i<63;++i) e.observe(417);
  assert(!e.enough() && e.candidates(rates)==0);
  // Grossly inconsistent timing is inconclusive, not a baud-rate guess.
  e.reset(); for(int i=0;i<64;++i) e.observe((i%2) ? 417 : 691);
  assert(e.enough() && e.candidates(rates)==0);
  // A common bit grid can be supported without any single-bit pulses.
  e.reset(); for(int i=0;i<64;++i) e.observe(417*(i%2 ? 2 : 9));
  assert(e.candidates(rates)>=1 && rates[0]==2400);
  uint32_t pulse_matches, pulse_considered, pulse_singles;
  e.configured_fit(pulse_matches,pulse_considered,pulse_singles);
  assert(pulse_matches==64 && pulse_singles==0);
  // One rare coprime run is not enough to establish the fundamental grid.
  e.reset(); for(int i=0;i<63;++i) e.observe(417*3); e.observe(417*2);
  assert(e.candidates(rates)==0);
  // Long pauses within a burst must not invalidate otherwise consistent bits.
  e.reset(); known(e,417);
  for(int i=0;i<32;++i) e.observe(9876); // ~24 bit times, still below the 50ms burst-gap cutoff.
  assert(e.candidates(rates)>=1 && rates[0]==2400);
  uint32_t matches, considered, singles;
  e.configured_fit(matches,considered,singles);
  assert(matches==64 && considered==64 && singles==16);
  // A slower rate must not win by fitting pauses while ignoring real short bits.
  e.reset();
  for(int i=0;i<80;++i) e.observe(417*(1+i%3));
  for(int i=0;i<128;++i) e.observe(3333); // Looks like single-bit timing at 300 baud.
  assert(e.candidates(rates)>=1 && rates[0]==2400);
  // A small matching subset is insufficient even with many long intervals.
  e.reset(); for(int i=0;i<16;++i) e.observe(417);
  for(int i=0;i<100;++i) e.observe(9876);
  assert(e.candidates(rates)==0);
  // The ISR queue preserves intervals, excludes burst gaps, and wraps uptime.
  RxEdgeIntervals q; uint32_t value;
  q.put(UINT32_MAX-100); q.put(316); assert(q.pop(value) && value==417);
  q.put(5000316); assert(!q.pop(value));
  q.put(5000733); assert(q.pop(value) && value==417);
  uint32_t now=5000733;
  for(int i=0;i<256;++i) { now+=417; q.put(now); }
  assert(q.dropped()==1); q.discard(); assert(!q.pop(value));
  for(int i=0;i<1000;++i) { now+=417; q.put(now); assert(q.pop(value) && value==417); }
  puts("PASS: edge timing, standard/custom candidates, harmonic ambiguity, insufficient/inconsistent evidence, queue loss, gaps, timestamp and ring wrap");
}
