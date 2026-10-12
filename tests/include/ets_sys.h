#pragma once
inline bool irq_enabled=false;
inline void (*irq_callback)(void *)=nullptr;
inline void *irq_arg=nullptr;
#define ETS_UART_INTR_DISABLE() (irq_enabled=false)
#define ETS_UART_INTR_ENABLE() (irq_enabled=true)
#define ETS_UART_INTR_ATTACH(f,a) do { irq_callback=f; irq_arg=a; } while (0)

inline bool gpio_irq_enabled=true;
#define ETS_GPIO_INTR_DISABLE() (gpio_irq_enabled=false)
#define ETS_GPIO_INTR_ENABLE() (gpio_irq_enabled=true)
