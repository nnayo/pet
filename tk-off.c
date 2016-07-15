#include "tk-off.h"

#include "dispatcher.h"

#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include "avr/io.h"

#include <stdbool.h>

// the take-off pin is pull down by a jumper
// when taking-off, the jumper is removed
// and the internal pin pull-up drives the pin to 1

// ------------------------------------------
// private definitions
//

#define IN_FIFO_SIZE        1

#define TKOFF_DDR       DDRB
#define TKOFF_PORT      PORTB
#define TKOFF_PIN       PINB
#define TKOFF_PARA      _BV(PB0)

#define TKF_THRES_HI    5 // high threshold for debounce
#define TKF_THRES_LO    0 // low threshold for debounce

#define TKF_PERIOD      (10 * TIME_1_MSEC) // 100 Hz

// ------------------------------------------
// private variables
//

struct {
        pt_t pt_com;                    // pt for sending thread
        pt_t pt_dbnc;                   // pt for debouncing take-off signal thread
        dpt_interface_t interf;         // interface to the dispatcher

        frame_t in_buf[IN_FIFO_SIZE];   // incoming buffer and fifo for acquisitions or commands
        fifo_t in_fifo;
        frame_t in_fr;                  // incoming frame

        frame_t out_fr;                 // outgoing frame

        s8 dbnc;                        // debouncing counter
        u32 period;                     // 100 Hz period net occurrence
        u8 is_in_waiting_state;       // take off check is only done in waiting state
} tkf;


// ------------------------------------------
// private functions
//

static PT_THREAD( tkf_thread_com(pt_t* pt) )
{
        PT_BEGIN(pt);

        // wait incoming commands
        PT_WAIT_UNTIL(pt, OK == FIFO_get(&tkf.in_fifo, &tkf.in_fr));

        switch (tkf.in_fr.cmde) {
        case FR_TAKE_OFF:
                // take off response is ignored
                break;

        case FR_STATE:
                // state response is ignored
                if (tkf.in_fr.resp)
                        break;

                // if state is set to waiting
                if (tkf.in_fr.argv[0] == FR_STATE_SET && tkf.in_fr.argv[1] == FR_STATE_WAITING) {
                        // enable take-off pin polling and init period
                        tkf.is_in_waiting_state = true;
                        tkf.period = TIME_get() + TKF_PERIOD;
                } else {
                        tkf.is_in_waiting_state = false;
                }
                break;

        default:
                break;
        }

        PT_RESTART(pt);

        PT_END(pt);
}

static PT_THREAD( tkf_thread_dbnc(pt_t* pt) )
{
        PT_BEGIN(pt);

        PT_WAIT_UNTIL(pt, TIME_get() > tkf.period);
        tkf.period += TKF_PERIOD;

        // read take-off pin (0 before take-off, 1 after)
        u8 tk_off = TKOFF_PIN & TKOFF_PARA;
//        if (tk_off)
//                PORTD |= _BV(PD7);
//        else
//                PORTD &= ~_BV(PD7);

        // debouncing
        tkf.dbnc += tk_off ? 1 : -1;
        if (tkf.dbnc < TKF_THRES_LO)
                tkf.dbnc = TKF_THRES_LO;

        // check if take-off is effective
        if (tkf.dbnc > TKF_THRES_HI) {
                tkf.out_fr.dest = DPT_SELF_ADDR;
                //tkf.out_fr.orig = DPT_SELF_ADDR;
                //tkf.out_fr.t_id = ?;
                tkf.out_fr.cmde = FR_TAKE_OFF;

                // send it throught the dispatcher
                dpt_lock(&tkf.interf);

                // some retry may be necessary
                PT_WAIT_UNTIL(pt, OK == dpt_tx(&tkf.interf, &tkf.out_fr));

                // release the dispatcher
                dpt_unlock(&tkf.interf);
        }

        PT_RESTART(pt);

        PT_END(pt);
}

// ------------------------------------------
// public functions
//

void tkf_init(void)
{
        // init
        FIFO_init(&tkf.in_fifo, &tkf.in_buf, IN_FIFO_SIZE, sizeof(frame_t));

        tkf.interf.channel = 8;
        tkf.interf.cmde_mask = _CM(FR_TAKE_OFF) | _CM(FR_STATE);
        tkf.interf.queue = &tkf.in_fifo;
        dpt_register(&tkf.interf);

        tkf.is_in_waiting_state = false;
        tkf.dbnc = TKF_THRES_LO;
        tkf.period = 0;

        PT_INIT(&tkf.pt_com);
        PT_INIT(&tkf.pt_dbnc);

        // set take-off detection pin as input with pull-up on
        TKOFF_DDR &= ~TKOFF_PARA;
        TKOFF_PORT |= TKOFF_PARA;

        // PD5 is the pull-up
        DDRD |= _BV(PD5);
        PORTD |= _BV(PD5);
}


void tkf_run(void)
{
        (void)PT_SCHEDULE(tkf_thread_com(&tkf.pt_com));
        // enable take-off pin polling only in waiting state
        if (tkf.is_in_waiting_state)
                (void)PT_SCHEDULE(tkf_thread_dbnc(&tkf.pt_dbnc));
}
