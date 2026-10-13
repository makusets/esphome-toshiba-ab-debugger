#include "../components/toshiba_ab/toshiba_ab.cpp"
using esphome::toshiba_ab::ToshibaAbLogger;
using esphome::text_sensor::TextSensor;
void edges(uint32_t start,uint32_t bit_us) {
  assert(edge_callback && gpio_irq_enabled && configured_mode==INPUT_PULLUP);
  assert(enabled_reg==0 && !irq_enabled); // GPIO measurement cannot overlap UART reception.
  const uint32_t pattern[]={1,2,3,7}; fake_now=start;
  rx_level = !rx_level; in_irq=true; edge_callback(edge_arg); in_irq=false;
  for(int i=0;i<16;++i) {
    fake_now+=bit_us*pattern[i%4];
    rx_level = !rx_level; in_irq=true; edge_callback(edge_arg); in_irq=false;
  }
}
void sample_and_handoff(ToshibaAbLogger &r) {
  for(uint32_t i=1;i<=4;++i) { edges(i*1000000,208); r.loop(); }
  assert(enabled_reg==0);
  rx_level=HIGH; fake_now+=25000; r.loop();
  assert(edge_callback==nullptr && enabled_reg!=0 && configured_mode==FUNCTION_4);
  assert(fake_now<20000000);
}
void irq(uint32_t time,uint32_t flags) {
  assert(irq_enabled && irq_callback); fake_now=time; status_reg=flags;
  in_irq=true; irq_callback(irq_arg); in_irq=false;
}
void frame(ToshibaAbLogger &r,uint32_t start,uint32_t error) {
  assert(edge_callback==nullptr); // GPIO IRQs are unavailable while UART is muxed.
  for(int i=0;i<9;++i) {
    fifo.push(i);
    if(fifo.size()>=2) irq(start+i*4583,1U<<UIFF);
  }
  irq(start+65000,(1U<<UITO)|error); r.loop();
  fake_now=start+90000; r.loop();
}
void window(ToshibaAbLogger &r,uint32_t error) {
  uint32_t start=fake_now;
  for(uint32_t i=1;i<=4;++i) frame(r,start+i*5000000,error);
}
int main() {
  ToshibaAbLogger r; TextSensor sensor; r.set_auto_parity(true); r.set_diagnostic(&sensor);
  fake_now=0; rx_level=HIGH; r.setup();
  assert(sensor.state.find("Measuring RX timing (UART paused)")==0);
  assert(configured_mode==INPUT_PULLUP && enabled_reg==0 && edge_callback);
  sample_and_handoff(r);
  assert(sensor.state.find("Discovering: 2400 baud, 8E1; baud candidates: 4800; cycle fit ")==0);
  assert(clock_reg==33333);
  // Parity receives a full 20-second window after the startup sampling period.
  window(r,0); assert(sensor.state.find("Discovering: 2400 baud, 8O1")==0);
  window(r,1U<<UIPE); window(r,1U<<UIFR);
  assert(sensor.state.find("Detected: 2400 baud, 8E1; baud candidates: 4800; cycle fit ")==0);
  assert(edge_callback==nullptr && clock_reg==33333);
  const size_t updates=sensor.history.size(); window(r,1U<<UIFR);
  assert(sensor.history.size()==updates); r.on_shutdown();
  // Explicit parity performs the same startup measurement without a parity scan.
  ToshibaAbLogger manual; TextSensor configured; manual.set_diagnostic(&configured);
  fake_now=0; manual.setup(); sample_and_handoff(manual);
  assert(configured.state.find("Configured: 2400 baud, 8E1; baud candidates: 4800; cycle fit ")==0);
  manual.on_shutdown();
  // Real ISR captures polarity; unequal rise/fall delays cancel in cycles.
  ToshibaAbLogger delayed; TextSensor delayed_sensor; delayed.set_diagnostic(&delayed_sensor);
  fake_now=0; rx_level=HIGH; delayed.setup();
  fake_now=1000; rx_level=LOW; in_irq=true; edge_callback(edge_arg); in_irq=false;
  const uint32_t runs[]={1,1,2,1,3,2,1,3};
  for (int i=0;i<160;++i) {
    rx_level=!rx_level;
    fake_now+=417*runs[i%8]+(rx_level ? 150 : -150);
    in_irq=true; edge_callback(edge_arg); in_irq=false;
  }
  delayed.loop(); rx_level=HIGH; fake_now+=25000; delayed.loop();
  assert(delayed_sensor.state.find("Configured: 2400 baud, 8E1; baud candidates: 2400, 4800, 9600 (ambiguous);")==0);
  assert(delayed_sensor.state.find("cycle fit 159/159")!=std::string::npos);
  assert(delayed_sensor.state.find("L 567-1401us, H 267-1101us")!=std::string::npos);
  assert(clock_reg==33333 && edge_callback==nullptr); delayed.on_shutdown();
  // Silence cannot hold UART reception off for longer than the 20-second deadline.
  ToshibaAbLogger no_edges; TextSensor no_timing; no_edges.set_diagnostic(&no_timing);
  fake_now=0; no_edges.setup(); fake_now=19999000; no_edges.loop();
  assert(enabled_reg==0 && configured_mode==INPUT_PULLUP);
  fake_now=20000000; no_edges.loop();
  assert(enabled_reg!=0 && edge_callback==nullptr && configured_mode==FUNCTION_4);
  assert(no_timing.state=="Configured: 2400 baud, 8E1; baud candidates: unavailable (no RX edge interrupts)");
  no_edges.on_shutdown();
  ToshibaAbLogger few; TextSensor few_samples; few.set_diagnostic(&few_samples);
  fake_now=0; few.setup(); edges(10000,208); few.loop();
  fake_now=20000000; few.loop();
  assert(few_samples.state.find("inconclusive (16/64 timing samples)")!=std::string::npos);
  few.on_shutdown();
  ToshibaAbLogger unmatched; TextSensor bad_timing; unmatched.set_diagnostic(&bad_timing);
  fake_now=0; unmatched.setup();
  for(int i=0;i<65;++i) {
    fake_now+=(i%2 ? 417 : 691); rx_level = !rx_level; in_irq=true; edge_callback(edge_arg); in_irq=false;
  }
  unmatched.loop(); fake_now=20000000; unmatched.loop();
  assert(bad_timing.state.find("inconclusive (no confident timing match;")!=std::string::npos);
  unmatched.on_shutdown();
  // Lost intervals reset the evidence, then fresh GPIO-mode samples can recover.
  ToshibaAbLogger loss; TextSensor lossy; loss.set_diagnostic(&lossy);
  fake_now=0; loss.setup();
  for(int i=0;i<300;++i) { fake_now+=208; rx_level = !rx_level; in_irq=true; edge_callback(edge_arg); in_irq=false; }
  loss.loop(); assert(lossy.state.find("baud candidates: collecting")!=std::string::npos);
  sample_and_handoff(loss); assert(lossy.state.find("baud candidates: 4800")!=std::string::npos);
  loss.on_shutdown();
  // A forced handoff on an active line must drop the partial frame's tail.
  ToshibaAbLogger busy; TextSensor busy_sensor; busy.set_diagnostic(&busy_sensor);
  fake_now=0; busy.setup(); rx_level=LOW; fake_now=20000000; busy.loop(); logs.clear();
  assert(enabled_reg!=0 && edge_callback==nullptr);
  fifo.push(0xAA); fifo.push(0xBB); irq(20005000,1U<<UIFF);
  assert(fifo.size()==1); busy.loop(); assert(logs.empty());
  rx_level=HIGH; irq(20030000,1U<<UITO); busy.loop(); assert(logs.empty() && fifo.empty());
  fifo.push(1); fifo.push(2); irq(20040000,1U<<UIFF); irq(20070000,1U<<UITO); busy.loop();
  assert(logs.size()==1 && logs[0]=="01:02"); busy.on_shutdown();
  // Shutdown while sampling must never start UART later.
  ToshibaAbLogger stop; TextSensor stop_sensor; stop.set_diagnostic(&stop_sensor);
  fake_now=0; stop.setup(); stop.on_shutdown(); fake_now=30000000; stop.loop();
  assert(edge_callback==nullptr && enabled_reg==0 && !irq_enabled);
  // Startup deadline uses elapsed time safely across uptime wrap.
  ToshibaAbLogger wrap; TextSensor wrap_sensor; wrap.set_diagnostic(&wrap_sensor);
  override_millis=true; fake_millis=UINT32_MAX-5000; fake_now=1000000;
  wrap.setup(); fake_millis+=19999; fake_now+=19999000; wrap.loop();
  assert(enabled_reg==0 && edge_callback!=nullptr);
  ++fake_millis; fake_now+=1000; wrap.loop();
  assert(enabled_reg!=0 && edge_callback==nullptr); wrap.on_shutdown(); override_millis=false;
  // Unswapped GPIO3 is restored to its original UART pin function as well.
  ToshibaAbLogger pin3; TextSensor pin3_sensor; pin3.set_uart_config(3,4800,8,1,1); pin3.set_diagnostic(&pin3_sensor);
  fake_now=0; pin3.setup(); assert(configured_pin==3 && configured_mode==INPUT_PULLUP);
  for(uint32_t i=1;i<=4;++i) { edges(i*1000000,208); pin3.loop(); }
  rx_level=HIGH; fake_now+=25000; pin3.loop();
  assert(configured_pin==3 && configured_mode==SPECIAL && !(swap_reg & (1U<<IOSWAPU0)) && clock_reg==16667);
  pin3.on_shutdown();
  puts("PASS: real GPIO startup mode, UART pause, early/deadline handoff, fixed baud, parity windows, frozen diagnostics, overflow recovery, partial-frame discard, shutdown, clock wrap, GPIO3 restore");
}
