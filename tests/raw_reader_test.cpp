#include "../components/toshiba_ab/toshiba_ab.cpp"
using esphome::toshiba_ab::ToshibaAbLogger;
using esphome::toshiba_ab::CaptureQueue;
using esphome::toshiba_ab::CapturedFrame;
void interrupt(uint32_t time, uint32_t flags) {
  fake_now=time; status_reg=flags; assert(irq_enabled && irq_callback);
  in_irq=true; irq_callback(irq_arg); in_irq=false;
  assert(cleared_reg==flags);
}
void byte(uint32_t time, uint8_t value) {
  fifo.push(value);
  if (fifo.size()>=2) interrupt(time,1U<<UIFF);
}
int main() {
  ToshibaAbLogger r; r.setup(); logs.clear();
  assert(configured_pin==13 && configured_mode==FUNCTION_4);
  assert(swap_reg & (1U<<IOSWAPU0));
  assert(clock_reg==33333 && ((conf1_reg>>UCTOT)&127)==5);
  assert(conf1_reg & (1U<<UCTOE));
  // A single byte remains in the FIFO until hardware silence, regardless of loop time.
  byte(1000,0xFF); fake_now=2000000; r.loop(); assert(logs.empty());
  interrupt(23917,1U<<UITO); assert(logs.empty() && fifo.empty());
  r.loop(); assert(logs.size()==1 && logs[0]=="FF");
  logs.clear();
  // Two frames captured while the ESPHome loop is delayed must stay separate.
  byte(30000,0xFE); byte(34583,0x00); assert(fifo.size()==1);
  interrupt(57500,1U<<UITO);
  byte(70000,0x12); byte(74583,0x34); interrupt(97500,1U<<UITO);
  assert(logs.empty()); fake_now=300000; r.loop();
  assert(logs.size()==2 && logs[0].find("FE:00")!=std::string::npos && logs[1].find("12:34")!=std::string::npos);
  assert(logs[0]=="FE:00" && logs[1]=="12:34"); logs.clear();
  // Preserve every byte across fragment boundaries, including the held FIFO byte.
  for(int i=0;i<129;++i) byte(400000+i*4583,static_cast<uint8_t>(i));
  interrupt(1000000,1U<<UITO); r.loop();
  assert(logs.size()==3 && logs[0].find("logging in fragments")!=std::string::npos);
  assert(logs[1].size()==128*3-1 && logs[2]=="80");
  logs.clear();
  // UART errors remain visible on the queued frame.
  byte(1100000,0xAA); interrupt(1122917,(1U<<UITO)|(1U<<UIPE)|(1U<<UIOF)); r.loop();
  assert(logs.size()==3 && logs[0].find("FIFO overflow")!=std::string::npos && logs[1].find("parity error")!=std::string::npos && logs[2]=="AA"); logs.clear();
  // Exhaust the queue without overwriting already queued records.
  for(int i=0;i<9;++i) { byte(1200000+i*30000,i); interrupt(1222917+i*30000,1U<<UITO); }
  r.loop(); assert(logs.size()==8); // Seven intact captures and one loss warning.
  assert(logs[0].find("2 records dropped")!=std::string::npos); logs.clear();
  byte(1500000,0xBB); interrupt(1522917,1U<<UITO); r.loop();
  assert(logs.size()==2 && logs[0].find("queue loss")!=std::string::npos && logs[1]=="BB"); logs.clear();
  // IRQ timestamps retain uint32 wrap; logging never substitutes loop time.
  byte(UINT32_MAX-100,0xCC); interrupt(22816,1U<<UITO); r.loop();
  assert(logs.size()==1 && logs[0]=="CC");
  r.on_shutdown(); assert(!irq_enabled && irq_callback==nullptr && enabled_reg==0);
  // The original normal parser receives queued bytes outside the ISR.
  ToshibaAbLogger normal;
  normal.set_debug_mode("normal"); normal.set_master_address(1);
  fake_now=2000000; normal.setup(); logs.clear();
  const uint8_t packet[] = {0x01,0x52,0x11,0x04,0x80,0x86,0xA1,0x05,0xE4};
  for (size_t i=0;i<sizeof(packet);++i) byte(2000000+i*4583,packet[i]);
  interrupt(2060000,1U<<UITO); assert(logs.empty()); normal.loop();
  bool decoded=false;
  for(const auto &line:logs) if(line.find("MASTER PARAMETERS")!=std::string::npos) decoded=true;
  assert(decoded); normal.on_shutdown();
  // Capture queue preserves timestamps and IDs through queue wrap and exact-size frames.
  CaptureQueue q; CapturedFrame f;
  for(int pass=0;pass<20;++pass) {
    for(int i=0;i<128;++i) q.put(i,100);
    q.finish(200); assert(q.pop(f)); assert(f.final && f.length==128);
    assert(f.first_irq_us==100 && f.boundary_irq_us==200 && f.frame_id==static_cast<uint32_t>(pass));
    for(int i=0;i<128;++i) assert(f.data[i]==i);
    assert(!q.pop(f));
  }
  puts("PASS: owned UART configuration, receiver-time boundaries, delayed loop, short frames, FIFO tail, fragmentation, errors, queue loss, timestamp/queue wrap, shutdown");
}
