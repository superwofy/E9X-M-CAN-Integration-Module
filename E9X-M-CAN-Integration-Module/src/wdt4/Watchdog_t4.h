#if !defined(_WATCHDOG_T4_H_)
#define _WATCHDOG_T4_H_

#include "Arduino.h"

typedef void (*watchdog_class_ptr)();

typedef enum WDT_DEV_TABLE {
  WDT1 = (uint32_t)0x400B8000,
  WDT2 = (uint32_t)0x400D0000,
  WDT3 = (uint32_t)0x400BC000,
  EWM  = (uint32_t)0x400B4000
} WDT_DEV_TABLE;

typedef enum RTWDOG_CLK_TABLE {
  BUS_CLK = 0,
  LPO_CLK = 1,
  INT_CLK = 2,
  ER_CLK = 3
} RTWDOG_CLK_TABLE;

typedef struct WDT_timings_t {
  double trigger = 5;
  double timeout = 10;
  double window = 0;
  uint8_t pin = 0;
  RTWDOG_CLK_TABLE clock = LPO_CLK; /* default clock, 32KHz */
  bool lp_suspend = 0;
  uint8_t input = 0;
  bool update = 1;
  bool cmd32en = 1;
  watchdog_class_ptr callback = nullptr;
} WDT_timings_t;

#define WDT_CLASS template<WDT_DEV_TABLE _device>
#define WDT_FUNC template<WDT_DEV_TABLE _device>
#define WDT_OPT WDT_T4<_device>

class WDT_T4_Base {
  public:
    virtual void watchdog_isr() = 0;
    virtual void ewatchdog_isr() = 0;
    watchdog_class_ptr watchdog_class_handler = 0;
  private:
};

static WDT_T4_Base* _WDT1 = nullptr;
static WDT_T4_Base* _WDT2 = nullptr;
static WDT_T4_Base* _WDT3 = nullptr;
static WDT_T4_Base* _EWM = nullptr;

WDT_CLASS class WDT_T4 : public WDT_T4_Base {
  public:
    void begin(WDT_timings_t config);
    void callback(watchdog_class_ptr handler) { watchdog_class_handler = handler; }
    void reset();
    void feed();
    bool expired();
  private:
    watchdog_class_ptr watchdog_class_handler;
    void watchdog_isr();
    void ewatchdog_isr();
};

#include "Watchdog_t4.tpp"
#endif
