#include <stdlib.h>
#include "../lib/arduino-lmic/lmic/hal.h"
#include "pins.h"
#include "SPI.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include "../lib/arduino-lmic/lmic/lmic.h"
#include <assert.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* some shortcuts for better readability */
#define _RFM95_NSS_DDR          DDR(RFM95_NSS_PORT)
#define _RFM95_NSS_PORT         PORT(RFM95_NSS_PORT)
#define _RFM95_NSS_PORTPIN      PORTPIN(RFM95_NSS_PORT, RFM95_NSS_PIN)
#define _RFM95_DIO0_DDR         DDR(RFM95_DIO0_PORT)
#define _RFM95_DIO0_PORTIN      PORTIN(RFM95_DIO0_PORT)
#define _RFM95_DIO0_PORTPIN     PORTPIN(RFM95_DIO0_PORT, RFM95_DIO0_PIN)
#define _RFM95_DIO1_DDR         DDR(RFM95_DIO1_PORT)
#define _RFM95_DIO1_PORTIN      PORTIN(RFM95_DIO1_PORT)
#define _RFM95_DIO1_PORTPIN     PORTPIN(RFM95_DIO1_PORT, RFM95_DIO1_PIN)

struct lmic_pinmap {
    int nix;
};
const struct lmic_pinmap lmic_pins;

static hal_failure_handler_t* custom_hal_failure_handler = NULL;

//--------------------
// Interrupt handling
//--------------------
#define NUM_DIO_INTERRUPT 3
//static_assert(NUM_DIO_INTERRUPT <= NUM_DIO, "Number of interrupt-sensitive lines must be less than number of GPIOs");
static ostime_t interrupt_time[NUM_DIO_INTERRUPT] = {0};

#if !defined(LMIC_USE_INTERRUPTS)
static void hal_interrupt_init() {
    _RFM95_DIO0_DDR &= ~(1 << _RFM95_DIO0_PORTPIN);
#ifdef RFM95_DIO1_PORT
    _RFM95_DIO1_DDR &= ~(1 << _RFM95_DIO1_PORTPIN);
#endif
#ifdef RFM95_DIO2_PORT
    _RFM95_DIO2_DDR &= ~(1 << _RFM95_DIO2_PORTPIN);
#endif
}

static uint8_t dio_states[3] = {0};
void hal_pollPendingIRQs_helper() {
    uint8_t u;

    u = (_RFM95_DIO0_PORTIN & (1 << _RFM95_DIO0_PORTPIN)) ? 0xFF : 0x00;
    if (dio_states[0] != u) {
        dio_states[0] = !dio_states[0];
        if (dio_states[0] && interrupt_time[0] == 0) {
            ostime_t const now = os_getTime();
            interrupt_time[0] = now ? now : 1;
        }
    }

#ifdef RFM95_DIO1_PORT
    u = (_RFM95_DIO1_PORTIN & (1 << _RFM95_DIO1_PORTPIN)) ? 0xFF : 0x00;
    if (dio_states[1] != u) {
        dio_states[1] = !dio_states[1];
        if (dio_states[1] && interrupt_time[1] == 0) {
            ostime_t const now = os_getTime();
            interrupt_time[1] = now ? now : 1;
        }
    }
#endif /* RFM95_DIO1_PORT */

#ifdef RFM95_DIO2_PORT
    u = (_RFM95_DIO2_PORTIN & (1 << _RFM95_DIO2_PORTPIN)) ? 0xFF : 0x00;
    if (dio_states[2] != u) {
        dio_states[2] = !dio_states[2];
        if (dio_states[2] && interrupt_time[2] == 0) {
            ostime_t const now = os_getTime();
            interrupt_time[2] = now ? now : 1;
        }
    }
#endif /* RFM95_DIO2_PORT */

}
#else
    #warning not implemented!
#endif /* !LMIC_USE_INTERRUPTS */

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    cli();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        sei();

#if !defined(LMIC_USE_INTERRUPTS)
        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs.
        // We merely collect the edges and timestamps here; we wait for
        // a call to hal_processPendingIRQs() before dispatching.
        hal_pollPendingIRQs_helper();
#endif /* !defined(LMIC_USE_INTERRUPTS) */
    }
}

static void hal_timer_init() {

    // this needs to be called before setup() or some functions won't
    // work there
    sei();
    
    // on the ATmega168, timer 0 is also used for fast hardware pwm
    // (using phase-correct PWM would mean that timer 0 overflowed half as often
    // resulting in different millis() behavior on the ATmega8 and ATmega168)
#if defined(TCCR0A) && defined(WGM01)
    sbi(TCCR0A, WGM01);
    sbi(TCCR0A, WGM00);
#endif

    // set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
    // CPU specific: different values for the ATmega128
    sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
    // this combination is for the standard atmega8
    sbi(TCCR0, CS01);
    sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
    // this combination is for the standard 168/328/1280/2560
    sbi(TCCR0B, CS01);
    sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
    // this combination is for the __AVR_ATmega645__ series
    sbi(TCCR0A, CS01);
    sbi(TCCR0A, CS00);
#else
    #error Timer 0 prescale factor 64 not set correctly
#endif

    // enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
    sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
    sbi(TIMSK0, TOIE0);
#else
    #error  Timer 0 overflow interrupt not set correctly
#endif

    // timers 1 and 2 are used for phase-correct hardware pwm
    // this is better for motors as it ensures an even waveform
    // note, however, that fast pwm mode can achieve a frequency of up
    // 8 MHz (with a 16 MHz clock) at 50% duty cycle

#if defined(TCCR1B) && defined(CS11) && defined(CS10)
    TCCR1B = 0;

    // set timer 1 prescale factor to 64
    sbi(TCCR1B, CS11);
#if F_CPU >= 8000000L
    sbi(TCCR1B, CS10);
#endif
#elif defined(TCCR1) && defined(CS11) && defined(CS10)
    sbi(TCCR1, CS11);
#if F_CPU >= 8000000L
    sbi(TCCR1, CS10);
#endif
#endif
    // put timer 1 in 8-bit phase correct pwm mode
#if defined(TCCR1A) && defined(WGM10)
    sbi(TCCR1A, WGM10);
#endif

    // set timer 2 prescale factor to 64
#if defined(TCCR2) && defined(CS22)
    sbi(TCCR2, CS22);
#elif defined(TCCR2B) && defined(CS22)
    sbi(TCCR2B, CS22);
//#else
    // Timer 2 not finished (may not be present on this CPU)
#endif

    // configure timer 2 for phase correct pwm (8-bit)
#if defined(TCCR2) && defined(WGM20)
    sbi(TCCR2, WGM20);
#elif defined(TCCR2A) && defined(WGM20)
    sbi(TCCR2A, WGM20);
//#else
    // Timer 2 not finished (may not be present on this CPU)
#endif

#if defined(TCCR3B) && defined(CS31) && defined(WGM30)
    sbi(TCCR3B, CS31);      // set timer 3 prescale factor to 64
    sbi(TCCR3B, CS30);
    sbi(TCCR3A, WGM30);     // put timer 3 in 8-bit phase correct pwm mode
#endif

#if defined(TCCR4A) && defined(TCCR4B) && defined(TCCR4D) /* beginning of timer4 block for 32U4 and similar */
    sbi(TCCR4B, CS42);      // set timer4 prescale factor to 64
    sbi(TCCR4B, CS41);
    sbi(TCCR4B, CS40);
    sbi(TCCR4D, WGM40);     // put timer 4 in phase- and frequency-correct PWM mode 
    sbi(TCCR4A, PWM4A);     // enable PWM mode for comparator OCR4A
    sbi(TCCR4C, PWM4D);     // enable PWM mode for comparator OCR4D
#else /* beginning of timer4 block for ATMEGA1280 and ATMEGA2560 */
#if defined(TCCR4B) && defined(CS41) && defined(WGM40)
    sbi(TCCR4B, CS41);      // set timer 4 prescale factor to 64
    sbi(TCCR4B, CS40);
    sbi(TCCR4A, WGM40);     // put timer 4 in 8-bit phase correct pwm mode
#endif
#endif /* end timer4 block for ATMEGA1280/2560 and similar */   

#if defined(TCCR5B) && defined(CS51) && defined(WGM50)
    sbi(TCCR5B, CS51);      // set timer 5 prescale factor to 64
    sbi(TCCR5B, CS50);
    sbi(TCCR5A, WGM50);     // put timer 5 in 8-bit phase correct pwm mode
#endif

#if defined(ADCSRA)
    // set a2d prescaler so we are inside the desired 50-200 KHz range.
    #if F_CPU >= 16000000 // 16 MHz / 128 = 125 KHz
        sbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
    #elif F_CPU >= 8000000 // 8 MHz / 64 = 125 KHz
        sbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        cbi(ADCSRA, ADPS0);
    #elif F_CPU >= 4000000 // 4 MHz / 32 = 125 KHz
        sbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
    #elif F_CPU >= 2000000 // 2 MHz / 16 = 125 KHz
        sbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        cbi(ADCSRA, ADPS0);
    #elif F_CPU >= 1000000 // 1 MHz / 8 = 125 KHz
        cbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
    #else // 128 kHz / 2 = 64 KHz -> This is the closest you can get, the prescaler is 2
        cbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
    #endif
    // enable a2d conversions
    sbi(ADCSRA, ADEN);
#endif

    // the bootloader connects pins 0 and 1 to the USART; disconnect them
    // here so they can be used as normal digital i/o; they will be
    // reconnected in Serial.begin()
#if defined(UCSRB)
    UCSRB = 0;
#elif defined(UCSR0B)
    UCSR0B = 0;
#endif

}

void hal_init (void) {
    // NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK
#ifndef RFM95_DIO0_PORT
    #error DIO0 required!
#endif
#ifndef RFM95_DIO1_PORT
    #error DIO1 required!
#endif

    // initialize SPI chip select to high (it's active low)
    _RFM95_NSS_DDR |= (1 << _RFM95_NSS_PORTPIN);
    _RFM95_NSS_PORT |= (1 << _RFM95_NSS_PORTPIN);   /* set NSS HIGH */

#ifdef RFM95_RXTX_PORT
    if (plmic_pins->rxtx != LMIC_UNUSED_PIN) {
        // initialize to RX
        digitalWrite(plmic_pins->rxtx, LOW != plmic_pins->rxtx_rx_active);
        pinMode(plmic_pins->rxtx, OUTPUT);
    }
#endif /* RFM95_RXTX_PORT */

#ifdef RFM95_RST_PORT
    if (plmic_pins->rst != LMIC_UNUSED_PIN) {
        // initialize RST to floating
        pinMode(plmic_pins->rst, INPUT);
    }
#endif /* RFM95_RST_PORT */

    hal_interrupt_init();
    hal_timer_init();
}

// hal_init_ex is a C API routine, written in C++, and it's called
// with a pointer to an lmic_pinmap.
void hal_init_ex (const void *pContext) {
    /*
    const lmic_pinmap * const pHalPinmap = (const lmic_pinmap *) pContext;
    if (! Arduino_LMIC::hal_init_with_pinmap(pHalPinmap)) {
        hal_failed(__FILE__, __LINE__);
    }
    */
    hal_init();
}

void hal_spi_write(u1_t cmd, const u1_t* buf, size_t len) {
    /* set NSS pin LOW to start communication */
    _RFM95_NSS_PORT &= ~(1 << _RFM95_NSS_PORTPIN);

    (void)SPI_transfer(cmd);

    for (; len > 0; --len, ++buf) {
        (void)SPI_transfer(*buf);
    }

    /* set NSS pin HIGH to end communication */
    _RFM95_NSS_PORT |= (1 << _RFM95_NSS_PORTPIN);
}

void hal_spi_read(u1_t cmd, u1_t* buf, size_t len) {
    /* set NSS pin LOW to start communication */
    _RFM95_NSS_PORT &= ~(1 << _RFM95_NSS_PORTPIN);

    (void)SPI_transfer(cmd);

    for (; len > 0; --len, ++buf) {
        *buf = SPI_transfer(0x00);
    }

    /* set NSS pin HIGH to end communication */
    _RFM95_NSS_PORT |= (1 << _RFM95_NSS_PORTPIN);

}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
#ifdef RFM95_RST_PORT
    if(val == 0 || val == 1) { // drive pin
        digitalWrite(plmic_pins->rst, val);
        pinMode(plmic_pins->rst, OUTPUT);
    } else { // keep pin floating
        pinMode(plmic_pins->rst, INPUT);
    }
#endif /* RFM95_RST_PORT */
}

// val == 1  => tx
void hal_pin_rxtx (u1_t val) {
#ifdef RFM95_RXTX_PORT
    if (plmic_pins->rxtx != LMIC_UNUSED_PIN)
        digitalWrite(plmic_pins->rxtx, val != plmic_pins->rxtx_rx_active);
#endif /* RFM95_RXTX_PORT */
}

void hal_failed (const char *file, u2_t line) {
    if (custom_hal_failure_handler != NULL) {
        (*custom_hal_failure_handler)(file, line);
    }

#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif

    hal_disableIRQs();

    // Infinite loop
    while (1) {
        ;
    }
}

/* porting of "micros()" from Arduino to AVR */

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

#if defined(TIM0_OVF_vect)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
    // copy these to local variables so they can be stored in registers
    // (volatile variables must be read from memory on every access)
    unsigned long m = timer0_millis;
    unsigned char f = timer0_fract;

    m += MILLIS_INC;
    f += FRACT_INC;
    if (f >= FRACT_MAX) {
        f -= FRACT_MAX;
        m += 1;
    }

    timer0_fract = f;
    timer0_millis = m;
    timer0_overflow_count++;
}

static unsigned long micros() {
    unsigned long m;
    uint8_t oldSREG = SREG, t;
    
    cli();
    m = timer0_overflow_count;
#if defined(TCNT0)
    t = TCNT0;
#elif defined(TCNT0L)
    t = TCNT0L;
#else
    #error TIMER 0 not defined
#endif

  
#ifdef TIFR0
    if ((TIFR0 & _BV(TOV0)) && (t < 255))
        m++;
#else
    if ((TIFR & _BV(TOV0)) && (t < 255))
        m++;
#endif

    SREG = oldSREG;
    
    return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

static void delay(unsigned long ms)
{
    uint32_t start = micros();

    while (ms > 0) {
        //yield();
        while ( ms > 0 && (micros() - start) >= 1000) {
            ms--;
            start += 1000;
        }
    }
}

u4_t hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // check for already too late.
    if (delta < 0)
        return -delta;

    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383. Also, STM32 does a better
    // job with delay is less than 10,000 us; so reduce in steps.
    // It's nice to use delay() for the longer times.
    while (delta > (9000 / US_PER_OSTICK)) {
        // deliberately delay 8ms rather than 9ms, so we
        // will exit loop with delta typically positive.
        // Depends on BSP keeping time accurately even if interrupts
        // are disabled.
        delay(8);
        // re-synchronize.
        delta = delta_time(time);
    }

    // unluckily, delayMicroseconds() isn't very accurate.
    // so spin using delta_time().
    while (delta_time(time) > 0)
        /* loop */;

    // we aren't "late". Callers are interested in gross delays, not
    // necessarily delays due to poor timekeeping here.
    return 0;
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

void hal_sleep () {
    // Not implemented
}

bit_t hal_queryUsingTcxo(void) {
    return(0);
}

uint8_t hal_getTxPowerPolicy(
    u1_t inputPolicy,
    s1_t requestedPower,
    u4_t frequency
    ) {
    return(LMICHAL_radio_tx_power_policy_paboost);
/*
    return (uint8_t) pHalConfig->getTxPowerPolicy(
                        Arduino_LMIC::HalConfiguration_t::TxPowerPolicy_t(inputPolicy),
                        requestedPower,
                        frequency
                        );
*/
}

s1_t hal_getRssiCal (void) {
    /* ###TODO### adjust! TTGO LoRa32: 10 */
    return 0;
}

ostime_t hal_setModuleActive (bit_t val) {
    // setModuleActive() takes a c++ bool, so
    // it effectively says "val != 0". We
    // don't have to.
    return(0);  /* no delay */
}

void hal_processPendingIRQs() {
    uint8_t i;
    for (i = 0; i < NUM_DIO_INTERRUPT; ++i) {
        ostime_t iTime;
        // if (plmic_pins->dio[i] == LMIC_UNUSED_PIN) continue;

        // NOTE(tmm@mcci.com): if using interrupts, this next step
        // assumes uniprocessor and fairly strict memory ordering
        // semantics relative to ISRs. It would be better to use
        // interlocked-exchange, but that's really far beyond
        // Arduino semantics. Because our ISRs use "first time
        // stamp" semantics, we don't have a value-race. But if
        // we were to disable ints here, we might observe a second
        // edge that we'll otherwise miss. Not a problem in this
        // use case, as the radio won't release IRQs until we
        // explicitly clear them.
        iTime = interrupt_time[i];
        if (iTime) {
            interrupt_time[i] = 0;
            radio_irq_handler_v2(i, iTime);
        }
    }
}
