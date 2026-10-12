#include <cassert>
#include <cstdio>
#include "../components/toshiba_ab/parity_detector.h"
using namespace esphome::toshiba_ab;
using Action = ParityDetector::Action;
ParitySample good() { ParitySample s; s.bytes=36; s.frames=4; return s; }
ParitySample parity_bad() { auto s=good(); s.parity_events=32; s.parity_frames=4; return s; }
ParitySample framing_bad() { auto s=good(); s.framing_events=4; s.framing_frames=4; return s; }
void repeats(ParityDetector &d) {
  CapturedFrame f; f.data[0]=0x01; f.data[1]=0xFF; f.length=2; f.final=true;
  d.observe(f); d.observe(f); d.observe(f);
}
int main() {
  ParityDetector d;
  d.start(0);
  for(uint32_t t=20000;t<=200000;t+=20000) assert(d.step(t,{})==Action::NONE);
  assert(d.parity()==1 && d.scanning());
  auto short_frames=good(); short_frames.bytes=4;
  assert(d.step(200000,short_frames)==Action::NONE);
  d.start(0);
  assert(d.step(19999,good())==Action::NONE);
  assert(d.step(20000,good())==Action::NEXT && d.parity()==2);
  assert(d.step(40000,parity_bad())==Action::NEXT && d.parity()==0);
  assert(d.step(60000,framing_bad())==Action::LOCK && d.parity()==1 && d.locked());
  // Errors and time never trigger rediscovery after a conclusive result.
  for(uint32_t t=80000;t<=1000000;t+=20000) assert(d.step(t,framing_bad())==Action::NONE);
  assert(d.parity()==1 && d.locked());
  d.start(0);
  d.step(20000,parity_bad()); d.step(40000,good());
  assert(d.step(60000,framing_bad())==Action::LOCK && d.parity()==2);
  d.start(0); d.step(20000,parity_bad()); d.step(40000,parity_bad()); repeats(d);
  assert(d.step(60000,good())==Action::LOCK && d.parity()==0);
  // NONE cannot win because its parity-error counter is disabled.
  d.start(0); d.step(20000,good()); d.step(40000,parity_bad()); repeats(d);
  assert(d.step(60000,good())==Action::UNCERTAIN && d.parity()==1 && !d.scanning());
  assert(d.step(200000,framing_bad())==Action::NONE && d.parity()==1);
  // NONE also needs consistent repeated captures, beyond failure of E and O.
  d.start(0); d.step(20000,parity_bad()); d.step(40000,parity_bad());
  assert(d.step(60000,good())==Action::UNCERTAIN);
  // A lost capture invalidates a trial; retries stay at the same parity.
  d.start(0); auto loss=good(); loss.losses=1;
  assert(d.step(20000,loss)==Action::RETRY && d.parity()==1);
  assert(d.step(39999,good())==Action::NONE);
  assert(d.step(40000,good())==Action::NEXT && d.parity()==2);
  // Trials work when uptime wraps during the scan.
  const uint32_t start=UINT32_MAX-9999;
  d.start(start);
  assert(d.step(start+19999,good())==Action::NONE);
  assert(d.step(start+20000,good())==Action::NEXT);
  puts("PASS: one-time parity policy, traffic minimums, E/O/N selection, ambiguity, repeated NONE evidence, losses, no rediscovery, uptime wrap");
}
