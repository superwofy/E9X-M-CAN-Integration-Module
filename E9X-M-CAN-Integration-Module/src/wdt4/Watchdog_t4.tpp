#include "Watchdog_t4.h"
#include "Arduino.h"

#define WDOGb_WCR(b)		(*(volatile uint16_t*)(b))
#define WDOGb_WSR(b)		(*(volatile uint16_t*)(b+0x2))
#define WDOGb_WRSR(b)		(*(volatile uint16_t*)(b+0x4))
#define WDOGb_WICR(b)		(*(volatile uint16_t*)(b+0x6))
#define WDOGb_WMCR(b)		(*(volatile uint16_t*)(b+0x8))
#define WDOGb_CS(b)		(*(volatile uint16_t*)(b))
#define WDOGb_CNT(b)		(*(volatile uint16_t*)(b+0x4))
#define WDOGb_CNT32(b)	(*(volatile uint32_t*)(b+0x4))
#define WDOGb_TOVAL(b)	(*(volatile uint16_t*)(b+0x8))
#define WDOGb_WIN(b)		(*(volatile uint16_t*)(b+0xC))
#define WDOG_WICR_WIE				((uint16_t)(1<<15))
#define WDOG_WICR_WTIS			((uint16_t)(1<<14))
#define WDOG_WICR_WICT(n)			((uint16_t)((n) & 0xFF))
#define EWM_CTRL_INTEN			((uint8_t)(1<<3))
#define EWM_CTRL_EWMEN			((uint8_t)(1<<0))

static void watchdog1_isr();
static void watchdog2_isr();
static void watchdog3_isr();
static void ewm_isr();

WDT_FUNC void WDT_OPT::begin(WDT_timings_t config) {
  IOMUXC_GPR_GPR16 |= 0x200000; /* undocumented register found by PaulS to fix reset */
  uint32_t nvicIRQ = 0;
  watchdog_class_handler = config.callback;

  if ( _device == WDT1 ) {
    CCM_CCGR3 |= (3UL << 16); /* enable WDOG1 clocks */
    _WDT1 = this;
    nvicIRQ = IRQ_WDOG1;
    _VectorsRam[16 + nvicIRQ] = watchdog1_isr;
  }
  if ( _device == WDT2 ) {
    CCM_CCGR5 |= (3UL << 10); /* enable WDOG2 clocks */
    _WDT2 = this;
    nvicIRQ = IRQ_WDOG2;
    _VectorsRam[16 + nvicIRQ] = watchdog2_isr;
  }
  if ( _device == WDT3 ) {
    CCM_CCGR5 |= (3UL << 4); /* enable WDOG3 clocks */
    _WDT3 = this;
    nvicIRQ = IRQ_RTWDOG;
    _VectorsRam[16 + nvicIRQ] = watchdog3_isr;

    bool preScaler = 0;
    uint16_t toVal = 0;

    if ( config.clock == LPO_CLK || config.clock == INT_CLK ) { /* LPO_CLOCK & INT_CLOCK are 32KHz */
      double highest_limit_without_prescaler = ((1.0f/32.0f)*65535.0f);
      double lowest_limit_without_prescaler = ((1.0f/32.0f)*1.0f);
      double highest_limit_with_prescaler = ((255.0f/32.0f)*65535.0f);
      double lowest_limit_with_prescaler = ((255.0f/32.0f)*1.0f);
      (void)lowest_limit_with_prescaler; /* unused, kept for reference */
      config.timeout = constrain(config.timeout, lowest_limit_without_prescaler, highest_limit_with_prescaler);
      config.window = constrain(config.window, lowest_limit_without_prescaler, highest_limit_with_prescaler);
      if ( config.timeout < highest_limit_without_prescaler ) { /* prescaler not needed */
        toVal = (config.timeout/(1.0f/32.0f));
        if ( config.window ) config.window = (config.window/(1.0f/32.0f));
      }
      else { /* use prescaler */
        preScaler = 1;
        toVal = (config.timeout/(255.0f/32.0f));
        if ( config.window ) config.window = (config.window/(255.0f/32.0f));
      }
    }

    __disable_irq();
    if ( WDOGb_CS(_device) & WDOG_CS_CMD32EN ) WDOGb_CNT32(_device) = 0xD928C520;
    else {
      WDOGb_CNT(_device) = 0xC520;
      WDOGb_CNT(_device) = 0xD928;
    }
    WDOGb_WIN(_device) = config.window;
    WDOGb_TOVAL(_device) = toVal;
    WDOGb_CS(_device) = ((config.window) ? WDOG_CS_WIN : 0) | ((preScaler) ? WDOG_CS_PRES : 0) | WDOG_CS_FLG | (config.update << 5) | (config.cmd32en << 13) | (config.clock << 8) | ((config.callback) ? WDOG_CS_INT : 0) | WDOG_CS_EN;
    __enable_irq();
    NVIC_ENABLE_IRQ(nvicIRQ);
    return;
  }

  if ( _device == EWM ) {
    CCM_CCGR3 |= (3UL << 14); /* enable EWM clocks */
    _EWM = this;
    nvicIRQ = IRQ_EWM;
    _VectorsRam[16 + nvicIRQ] = ewm_isr;

    if ( config.pin == 21 ) {
      IOMUXC_CSI_DATA06_SELECT_INPUT = 0;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 = 0x11; // pin21
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11 = 0x30b0; // pin21
    }
    else if ( config.pin == 25 ) {
      IOMUXC_CSI_DATA06_SELECT_INPUT = 0;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_13 = 0x13; // pin25
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_13 = 0x30b0; // pin25
    }
    else config.pin = 0;

    double highest_limit_without_prescaler = ((1.0f/32.0f)*254.0f);
    (void)highest_limit_without_prescaler; /* unused, kept for reference */
    double lowest_limit_without_prescaler = ((1.0f/32.0f)*1.0f);
    double highest_limit_with_prescaler = ((255.0f/32.0f)*254.0f);
    double lowest_limit_with_prescaler = ((255.0f/32.0f)*1.0f);
    (void)lowest_limit_with_prescaler; /* unused, kept for reference */
    config.timeout = constrain(config.timeout, lowest_limit_without_prescaler, highest_limit_with_prescaler);
    config.window = constrain(config.window, lowest_limit_without_prescaler, highest_limit_with_prescaler);
    config.timeout = (config.timeout/(255.0f/32.0f));
    config.window = (config.window/(255.0f/32.0f));
    EWM_CLKPRESCALER = 0xFF;
    EWM_CLKCTRL = 0x0;
    EWM_CMPL = (uint8_t)config.window;
    EWM_CMPH = (uint8_t)config.timeout;
    EWM_CTRL = ((config.callback) ? EWM_CTRL_INTEN : 0) | EWM_CTRL_EWMEN;
    NVIC_ENABLE_IRQ(nvicIRQ);
    return;
  }

  if ( config.pin ) {
    /* WDOG1 PINS */
    if ( config.pin == 19 ) {
      IOMUXC_QTIMER3_TIMER0_SELECT_INPUT = 1;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 0x14; // pin19
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_00 = 0x30b0; // pin19
    }
    else if ( config.pin == 20 ) {
      IOMUXC_CSI_DATA07_SELECT_INPUT = 0;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = 0x11; // pin20
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10 = 0x30b0; // pin20
    }

    /* WDOG2 PINS */
    else if ( config.pin == 24 ) {
      IOMUXC_LPI2C4_SCL_SELECT_INPUT = 1;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12 = 0x13; // pin24
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_12 = 0x30b0; // pin24
    }
    else if ( config.pin == 13 ) {
      IOMUXC_LPSPI4_SCK_SELECT_INPUT = 0;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x16; // pin13
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = 0x30b0; // pin13
    }
    else config.pin = 0;
  }

  WDOGb_CS(_device) = WDOG_CS_FLG | (config.update << 5) | (config.cmd32en << 13) | (config.clock << 8) | ((config.callback) ? WDOG_CS_INT : 0) | WDOG_CS_EN;
  config.timeout = constrain(config.timeout, 0.5f, 128.0f); /* timeout to reset */
  config.timeout = (config.timeout - 0.5f)/0.5f;
  WDOGb_WCR(_device) = WDOG_WCR_SRS | WDOG_WCR_WT((uint8_t)config.timeout); /* do NOT negate the reset signal when clearing register */
  config.trigger = constrain(config.trigger, 0.0f, 127.5); /* callback trigger before timeout */
  config.trigger /= 0.5f;
  WDOGb_WICR(_device) = ((config.callback) ? WDOG_WICR_WIE : 0) | WDOG_WICR_WTIS | WDOG_WICR_WICT((uint8_t)config.trigger); /* enable interrupt, clear interrupt */
  WDOGb_WCR(_device) |= WDOG_WCR_WDE | ((config.lp_suspend) ? WDOG_WCR_WDZST : 0) | WDOG_WCR_WDA | WDOG_WCR_WDT | WDOG_WCR_SRE;
  WDOGb_WMCR(_device) = 0; /* Disable power down counter, else GPIO will force LOW indefinately after 16 seconds */
  NVIC_ENABLE_IRQ(nvicIRQ);
}

WDT_FUNC bool WDT_OPT::expired() {
  if ( WDT3 == _device ) return SRC_SRSR & SRC_SRSR_WDOG3_RST_B;
  if ( EWM == _device ) return 0; /* No status register? */
  return WDOGb_WRSR(_device) & WDOG_WRSR_TOUT;
}

WDT_FUNC void WDT_OPT::reset() {
  if ( WDT3 == _device || EWM == _device ) SCB_AIRCR = 0x05FA0004; /* WDT3 & EWM doesn't have a reset register, fall back to ARM */
  WDOGb_WCR(_device) &= ~WDOG_WCR_SRS;
}

WDT_FUNC void WDT_OPT::feed() {
  if ( _device == EWM ) {
    EWM_SERV = 0xB4;
    EWM_SERV = 0x2C;
    return;
  }
  if ( _device == WDT1 || _device == WDT2 ) {
    WDOGb_WSR(_device) = 0x5555;
    WDOGb_WSR(_device) = 0xAAAA;
  }
  else {
    if ( WDOGb_CS(_device) & WDOG_CS_CMD32EN ) WDOGb_CNT32(_device) = 0xB480A602;
    else {
      WDOGb_CNT(_device) = 0xA602;
      WDOGb_CNT(_device) = 0xB480;
    }
  }
}

void watchdog1_isr() {
  if ( _WDT1 ) _WDT1->watchdog_isr();
}

void watchdog2_isr() {
  if ( _WDT2 ) _WDT2->watchdog_isr();
}

void watchdog3_isr() {
  if ( _WDT3 ) _WDT3->watchdog_isr();
}

WDT_FUNC void WDT_OPT::watchdog_isr() {
  if ( watchdog_class_handler ) watchdog_class_handler();
  if ( WDT3 == _device ) WDOGb_CS(_device) |= WDOG_CS_FLG;
  else WDOGb_WICR(_device) |= WDOG_WICR_WTIS;
  asm volatile ("dsb"); /* disable double firing of interrupt */
}

void ewm_isr() {
  if ( _EWM ) _EWM->ewatchdog_isr();
}

WDT_FUNC void WDT_OPT::ewatchdog_isr() {
  if ( watchdog_class_handler ) watchdog_class_handler();
  EWM_CTRL &= ~(1U << 3); /* disable further interrupts, EWM has been triggered, only way out is reset */
  asm volatile ("dsb"); /* disable double firing of interrupt */
}
