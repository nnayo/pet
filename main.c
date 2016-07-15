#include "minut.h"
#include "servo.h"
#include "tk-off.h"

#include "drivers/timer2.h"
#include "utils/pt.h"
#include "utils/time.h"

#include "dispatcher.h"
#include "basic.h"
#include "reconf.h"
#include "dna.h"
#include "common.h"
#include "nat.h"
#include "log.h"
#include "time_sync.h"
#include "alive.h"
#include "cpu.h"

#include "avr/io.h"
#include "avr/interrupt.h"

#if 0
arduino        atmega        function
-------+-------+--------

D8      PB0     take-off detection
D9      PB1     servo pwm
sck     PB5     led

+9V     PWR     power in
GND     GND     ground

#endif

// ------------------------------------------
// simavr options
//

#include "avr_mcu_section.h"

AVR_MCU(16000000, "atmega328p");

const struct avr_mmcu_vcd_trace_t simavr_conf[]  _MMCU_ = {
        { AVR_MCU_VCD_SYMBOL("take_off"), .mask = _BV(PORTB0), .what = (void*)&PORTB, },
        { AVR_MCU_VCD_SYMBOL("servo"), .mask = _BV(PORTB1), .what = (void*)&PORTB, },
        { AVR_MCU_VCD_SYMBOL("led"), .mask = _BV(PORTB5), .what = (void*)&PORTB, },

//        { AVR_MCU_VCD_SYMBOL("TWDR"), .what = (void*)&TWDR, },
//
//        { AVR_MCU_VCD_SYMBOL("SPDR"), .what = (void*)&SPDR, },
//
//        { AVR_MCU_VCD_SYMBOL("TCCR1A"), .what = (void*)&TCCR1A, },
//        { AVR_MCU_VCD_SYMBOL("TCCR1B"), .what = (void*)&TCCR1B, },
//        { AVR_MCU_VCD_SYMBOL("TCCR1C"), .what = (void*)&TCCR1C, },
//        { AVR_MCU_VCD_SYMBOL("TCNT1H"), .what = (void*)&TCNT1H, },
//        { AVR_MCU_VCD_SYMBOL("TCNT1L"), .what = (void*)&TCNT1L, },
//        { AVR_MCU_VCD_SYMBOL("OCR1AH"), .what = (void*)&OCR1AH, },
//        { AVR_MCU_VCD_SYMBOL("OCR1AL"), .what = (void*)&OCR1AL, },
//        { AVR_MCU_VCD_SYMBOL("OCR1BH"), .what = (void*)&OCR1BH, },
//        { AVR_MCU_VCD_SYMBOL("OCR1BL"), .what = (void*)&OCR1BL, },
//        { AVR_MCU_VCD_SYMBOL("ICR1H"), .what = (void*)&ICR1H, },
//        { AVR_MCU_VCD_SYMBOL("ICR1L"), .what = (void*)&ICR1L, },
//
//        { AVR_MCU_VCD_SYMBOL("TIMSK1"), .what = (void*)&TIMSK1, },
//        { AVR_MCU_VCD_SYMBOL("TIFR1"), .what = (void*)&TIFR1, },
//
//        { AVR_MCU_VCD_SYMBOL("TCNT2"), .what = (void*)&TCNT2, },
//
//        { AVR_MCU_VCD_SYMBOL("UDR0"), .what = (void*)&UDR0, },
//        { AVR_MCU_VCD_SYMBOL("UDRIE0"), .mask = _BV(UDRIE0), .what = (void*)&UCSR0B, },
//        { AVR_MCU_VCD_SYMBOL("UCSR0A"), .what = (void*)&UCSR0A, },

        { AVR_MCU_VCD_SYMBOL("PINB"), .what = (void*)&PINB, },
        { AVR_MCU_VCD_SYMBOL("PORTB"), .what = (void*)&PORTB, },
        { AVR_MCU_VCD_SYMBOL("DDRB"), .what = (void*)&DDRB, },
};


// ------------------------------------------
// private definitions
//

//#define TIMER2_TOP_VALUE        78        // @ 8 MHz ==> 10 ms
#define TIMER2_TOP_VALUE        156        // @ 16 MHz ==> 10 ms


// ------------------------------------------
// private variables
//


// ------------------------------
// private functions
//

static void time(void* misc)
{
        (void)misc;

        // time update
        TIME_incr();
}


static u32 time_adjust(void)
{
        u8 val;
        u32 incr;

        val = TMR2_get_value();
        incr = TIME_get_incr();

        return incr * val / TIMER2_TOP_VALUE;
}


// ------------------------------
// public variables
//


// ------------------------------
// public functions
//

int main(void)
{
//        // if bad reset conditions
//        if ( MCUSR & ( _BV(WDRF) | _BV(BORF) | _BV(EXTRF) ) ) {
//                // loop for ever
//                while (1)
//                        ;
//        }
//        else {
//                MCUSR = _BV(WDRF) | _BV(BORF) | _BV(EXTRF);
//        }

        // init on-board time
        TIME_init(time_adjust);
        TIME_set_incr(10 * TIME_1_MSEC);

        // program and start timer2 for interrupt on compare every 10 ms
        TMR2_init(TMR2_WITH_COMPARE_INT, TMR2_PRESCALER_1024, TMR2_WGM_CTC, TIMER2_TOP_VALUE, time, NULL);
        TMR2_start();

        // enable interrupts
        sei();

        // init every common module
        dpt_init();
        BSC_init();
        CMN_init();
        //NAT_init();
        //LOG_init();
        //CPU_init();

        mnt_init();
        srv_init();
        tkf_init();

        while (1) {
                // run every common module
                dpt_run();
                BSC_run();
                CMN_run();
                //NAT_run();
                //LOG_run();
                //CPU_run();

                mnt_run();
                srv_run();
                tkf_run();

                //#define DEBUG
#if DEBUG
                if ( TIME_get() > 20 * TIME_1_SEC ) {
                        cli();
#include <avr/sleep.h>
                        sleep_mode();
                }
#endif
        }

        // this point is never reached
        return 0;
}
