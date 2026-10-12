#include "../components/toshiba_ab/toshiba_ab.cpp"
using esphome::toshiba_ab::ToshibaAbLogger;
using esphome::text_sensor::TextSensor;
void edges(uint32_t start,uint32_t bit_us) {
  assert(edge_callback && gpio_irq_enabled);
  const uint32_t pattern[]={1,2,3,7}; fake_now=start;
  in_irq=true; edge_callback(edge_arg); in_irq=false;
  for(int i=0;i<16;++i) {
    fake_now+=bit_us*pattern[i%4];
    in_irq=true; edge_callback(edge_arg); in_irq=false;
  }
}
void frame(ToshibaAbLogger &r,uint32_t start,uint32_t error) {
  edges(start,208); // Real edge timing suggests 4800 while configured UART stays at 2400.
  for(int i=0;i<9;++i) {
    fifo.push(i); fake_now=start+i*4583; status_reg=1U<<UIFF;
    if(fifo.size()>=2) { in_irq=true; irq_callback(irq_arg); in_irq=false; }
  }
  fake_now=start+65000; status_reg=(1U<<UITO)|error;
  in_irq=true; irq_callback(irq_arg); in_irq=false; r.loop();
  fake_now=start+90000; r.loop();
}
void window(ToshibaAbLogger &r,uint32_t error) {
  uint32_t start=fake_now;
  for(uint32_t i=1;i<=4;++i) frame(r,start+i*5000000,error);
}
int main() {
  ToshibaAbLogger r; TextSensor sensor; r.set_auto_parity(true); r.set_diagnostic(&sensor);
  fake_now=0; r.setup();
  assert(sensor.state.find("baud candidates: collecting")!=std::string::npos);
  window(r,0);
  assert(sensor.state.find("baud candidates: 4800")!=std::string::npos && clock_reg==33333);
  window(r,1U<<UIPE); window(r,1U<<UIFR);
  assert(sensor.state=="Detected: 2400 baud, 8E1; baud candidates: 4800");
  assert(edge_callback==nullptr && clock_reg==33333);
  const size_t updates=sensor.history.size(); fake_now+=100000000; r.loop();
  assert(sensor.history.size()==updates); r.on_shutdown();
  // Explicit parity also gets timing suggestions; collection ends after 60s.
  ToshibaAbLogger manual; TextSensor configured; manual.set_diagnostic(&configured);
  fake_now=0; manual.setup(); assert(edge_callback);
  for(uint32_t i=1;i<=4;++i) { edges(i*5000000,208); manual.loop(); }
  assert(configured.state.find("Configured: 2400 baud, 8E1; baud candidates: 4800")==0);
  fake_now=60000000; manual.loop(); assert(edge_callback==nullptr && clock_reg==33333);
  manual.on_shutdown();
  // Lost edge timing discards stale evidence and allows fresh samples.
  ToshibaAbLogger loss; TextSensor lossy; loss.set_auto_parity(true); loss.set_diagnostic(&lossy);
  fake_now=0; loss.setup();
  edges(10000,208); loss.loop();
  for(int i=0;i<300;++i) { fake_now+=208; in_irq=true; edge_callback(edge_arg); in_irq=false; }
  loss.loop(); assert(lossy.state.find("baud candidates: collecting")!=std::string::npos);
  for(uint32_t i=1;i<=4;++i) { edges(i*5000000,208); loss.loop(); }
  assert(lossy.state.find("baud candidates: 4800")!=std::string::npos && clock_reg==33333);
  loss.on_shutdown();
  puts("PASS: baud timing sensor integration, different measured/configured baud, fixed baud, final frozen suggestions, manual mode, edge overflow recovery");
}
