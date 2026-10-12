#include "../components/toshiba_ab/toshiba_ab.cpp"
using esphome::toshiba_ab::ToshibaAbLogger;
using esphome::text_sensor::TextSensor;
void irq(uint32_t time, uint32_t flags) {
  fake_now=time; status_reg=flags; assert(irq_enabled && irq_callback);
  in_irq=true; irq_callback(irq_arg); in_irq=false;
}
void receive_frame(ToshibaAbLogger &r, uint32_t start, uint32_t error) {
  const uint8_t data[]={1,2,3,4,5,6,7,8,9};
  for(size_t i=0;i<sizeof(data);++i) {
    fifo.push(data[i]);
    if(fifo.size()>=2) irq(start+i*2300,1U<<UIFF);
  }
  irq(start+45000,(1U<<UITO)|error); r.loop();
  fake_now=start+70000; r.loop();
}
void window(ToshibaAbLogger &r,uint32_t error) {
  const uint32_t start=fake_now;
  for(uint32_t i=1;i<=4;++i) receive_frame(r,start+i*5000000,error);
}
int main() {
  gpio_read_level=LOW; // UART pad reads LOW through GPIO even while UART RXD is idle HIGH.
  // A configured baud other than 2400 remains unchanged across every trial.
  ToshibaAbLogger r; TextSensor sensor;
  r.set_uart_config(13,4800,8,1,1); r.set_auto_parity(true); r.set_diagnostic(&sensor);
  fake_now=0; r.setup(); logs.clear();
  assert(sensor.state.find("Discovering: 4800 baud, 8E1")==0 && clock_reg==16667);
  window(r,0);
  assert(sensor.state.find("Discovering: 4800 baud, 8O1")==0 && clock_reg==16667);
  assert((conf0_reg & (1U<<UCPA)) && (conf0_reg & (1U<<UCPAE)));
  window(r,1U<<UIPE);
  assert(sensor.state.find("Discovering: 4800 baud, 8N1")==0 && clock_reg==16667);
  assert(!(conf0_reg & (1U<<UCPAE)));
  window(r,1U<<UIFR);
  assert(sensor.state.find("Detected: 4800 baud, 8E1")==0 && clock_reg==16667);
  assert(edge_callback==nullptr);
  assert((conf0_reg & (1U<<UCPAE)) && !(conf0_reg & (1U<<UCPA)));
  const size_t updates=sensor.history.size();
  for(int i=0;i<4;++i) window(r,(1U<<UIPE)|(1U<<UIFR));
  assert(sensor.state.find("Detected: 4800 baud, 8E1")==0 && sensor.history.size()==updates);
  assert(clock_reg==16667); r.on_shutdown();
  // Silence cannot advance the scan, and repeated status states are not republished.
  ToshibaAbLogger silent; TextSensor waiting;
  silent.set_auto_parity(true); silent.set_diagnostic(&waiting);
  fake_now=0; silent.setup();
  for(uint32_t t=20000000;t<=300000000;t+=20000000) { fake_now=t; silent.loop(); }
  assert(waiting.state.find("Waiting for traffic: 2400 baud, 8E1")==0 && waiting.history.size()==2);
  assert(clock_reg==33333 && !(conf0_reg & (1U<<UCPA))); silent.on_shutdown();
  ToshibaAbLogger odd; TextSensor detected_odd;
  odd.set_auto_parity(true); odd.set_diagnostic(&detected_odd); fake_now=0; odd.setup();
  window(odd,1U<<UIPE); window(odd,0); window(odd,1U<<UIFR);
  assert(detected_odd.state.find("Detected: 2400 baud, 8O1")==0); odd.on_shutdown();
  // NONE can be selected only after both parity modes fail and captures repeat.
  ToshibaAbLogger none; TextSensor detected_none;
  none.set_auto_parity(true); none.set_diagnostic(&detected_none); fake_now=0; none.setup();
  window(none,1U<<UIPE); window(none,1U<<UIPE); window(none,0);
  assert(detected_none.state.find("Detected: 2400 baud, 8N1")==0); none.on_shutdown();
  // A clean NONE trial beside a clean EVEN trial is ambiguous and final.
  ToshibaAbLogger ambiguous; TextSensor uncertain;
  ambiguous.set_auto_parity(true); ambiguous.set_diagnostic(&uncertain); fake_now=0; ambiguous.setup();
  window(ambiguous,0); window(ambiguous,1U<<UIPE); window(ambiguous,0);
  assert(uncertain.state.find("Uncertain: 2400 baud, 8E1")==0);
  const size_t final_updates=uncertain.history.size();
  window(ambiguous,1U<<UIFR); window(ambiguous,1U<<UIFR);
  assert(uncertain.history.size()==final_updates); ambiguous.on_shutdown();
  // An active or queued partial frame postpones switching until after silence.
  ToshibaAbLogger busy; TextSensor busy_sensor;
  busy.set_auto_parity(true); busy.set_diagnostic(&busy_sensor); fake_now=0; busy.setup();
  for(uint32_t i=1;i<=3;++i) receive_frame(busy,i*5000000,0);
  fifo.push(0xAA); fake_now=20000000; busy.loop();
  assert(!(conf0_reg & (1U<<UCPA)) && busy_sensor.state.find("8E1")!=std::string::npos);
  fifo.push(0xBB); irq(20004600,1U<<UIFF); busy.loop();
  assert(!(conf0_reg & (1U<<UCPA)));
  irq(20030000,1U<<UITO); busy.loop();
  assert(!(conf0_reg & (1U<<UCPA)));
  fake_now=20060000;
  assert(edge_callback); in_irq=true; edge_callback(edge_arg); in_irq=false;
  busy.loop(); assert(!(conf0_reg & (1U<<UCPA)));  // A new RX edge blocks switching even before a byte enters FIFO.
  fake_now=20090000; rx_level=LOW; busy.loop(); assert(!(conf0_reg & (1U<<UCPA)));
  rx_level=HIGH; busy.loop();
  assert((conf0_reg & (1U<<UCPA)) && busy_sensor.state.find("Discovering: 2400 baud, 8O1")==0);
  busy.on_shutdown();
  // Explicit parity publishes configured settings without discovery.
  ToshibaAbLogger manual; TextSensor configured;
  manual.set_uart_config(3,9600,7,0,2); manual.set_diagnostic(&configured); manual.setup();
  assert(configured.state.find("Configured: 9600 baud, 7N2")==0); manual.on_shutdown();
  puts("PASS: parity receiver integration, fixed configured/default baud, diagnostic states, no rediscovery, silence, NONE evidence, ambiguity, idle switching, manual settings");
}
