#include "servo.h"

#include "dispatcher.h"

#include "drivers/timer1.h"
#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include "avr/io.h"


// ------------------------------------------
// private definitions
//

#define IN_FIFO_SIZE    3
#define OUT_FIFO_SIZE   3

#define SERVO_DDR       DDRB
#define SERVO_PORT      PORTB
#define SERVO_PIN       PINB
#define SERVO_PARA      _BV(PB1)


// ------------------------------------------
// private variables
//

struct {
        pt_t pt_out;        // pt for sending thread
        pt_t pt_in;                // pt for receiving thread

        dpt_interface_t interf;        // interface to the dispatcher

        struct {
                s8 open_pos;                // open position
                s8 close_pos;                // closed position
        } para;

        // incoming frames fifo
        fifo_t in;
        frame_t in_buf[IN_FIFO_SIZE];

        // outgoing frames fifo
        fifo_t out;
        frame_t out_buf[OUT_FIFO_SIZE];

        frame_t out_fr;        // frame for the sending thread
        frame_t in_fr;        // frame for the cmde thread

} srv;


// ------------------------------------------
// private functions
//

// activate the para servo to drive it to the given position
static void srv_para_on(s8 position)
{
        // compute the compare value according to the required position and the prescaler
        // for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
        // for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
        // for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
        // compare = (position / 90) * 1000 + 3000
        // the computation shall be modified to fit in s16
        // the result is sure to fit in u16
        TMR1_compare_set(TMR1_A, ((s16)position * 100 / 9) + 3000);
}

// deactivate the para servo to save power
static void srv_para_off(void)
{
        // setting the compare value to 0, ensure output pin is driven lo
        TMR1_compare_set(TMR1_A, 0);
}

static void srv_drive(u8 servo, u8 sense)
{
        switch (servo) {
        case FR_SERVO_PARA:
                switch (sense) {
                case FR_SERVO_OPEN:        // open
                        srv_para_on(srv.para.open_pos);
                        break;

                case FR_SERVO_CLOSE:        // close
                        srv_para_on(srv.para.close_pos);
                        break;

                case FR_SERVO_OFF:
                        srv_para_off();
                        break;

                default:
                        break;
                }
                break;

        default:
                break;
        }
}

static void srv_para_save(frame_t* fr)
{
        switch ( fr->argv[2] ) {
        case FR_SERVO_OPEN:        // open position
                srv.para.open_pos = fr->argv[3];
                break;

        case FR_SERVO_CLOSE:        // closed position
                srv.para.close_pos = fr->argv[3];
                break;

        default:
                // shall never happen
                fr->error = 1;
                break;
        }
}

static void srv_para_read(frame_t* fr)
{
        switch ( fr->argv[2] ) {
        case FR_SERVO_OPEN:        // open position
                fr->argv[3] = srv.para.open_pos;
                break;

        case FR_SERVO_CLOSE:        // closed position
                fr->argv[3] = srv.para.close_pos;
                break;

        default:
                // shall never happen
                fr->error = 1;
        }
}

static void srv_position(frame_t* fr)
{
        switch ( fr->argv[0] ) {
        case FR_SERVO_PARA:
                switch ( fr->argv[1] ) {
                case FR_SERVO_SAVE:        // save
                        srv_para_save(fr);
                        break;

                case FR_SERVO_READ:        // read
                        srv_para_read(fr);
                        break;

                default:
                        // shall never happen
                        fr->error = 1;
                        break;
                }
                break;

        default:
                // shall never happen
                fr->error = 1;
                break;
        }
}

static PT_THREAD( srv_in(pt_t* pt) )
{
        u8 swap;

        PT_BEGIN(pt);

        // if no incoming frame is available
        PT_WAIT_UNTIL(pt, OK == FIFO_get(&srv.in, &srv.in_fr) );

        // if it is a response
        if (srv.in_fr.resp) {
                // ignore it

                // release the dispatcher
//                dpt_unlock(&srv.interf);

                // and restart waiting
                PT_RESTART(pt);
        }

        srv.in_fr.error = 0;

        switch (srv.in_fr.cmde) {
                case FR_MINUT_SERVO_CMD:
                        // drive the servo
                        srv_drive(srv.in_fr.argv[0], srv.in_fr.argv[1]);
                        break;

                case FR_MINUT_SERVO_INFO:
                        srv_position(&srv.in_fr);
                        break;

                default:
                        // shall never happen
                        srv.in_fr.error = 1;
                        break;
        }

        // send the response
        swap = srv.in_fr.orig;
        srv.in_fr.orig = srv.in_fr.dest;
        srv.in_fr.dest = swap;
        srv.in_fr.resp = 1;
        //srv.in_fr.nat = 0;
        PT_WAIT_UNTIL(pt, OK == FIFO_put(&srv.out, &srv.in_fr));

        // and restart waiting for incoming command
        PT_RESTART(pt);

        PT_END(pt);
}

static PT_THREAD( srv_out(pt_t* pt) )
{
        PT_BEGIN(pt);

        // wait until a frame to send is available
        PT_WAIT_UNTIL(pt, OK == FIFO_get(&srv.out, &srv.out_fr));

        // send it throught the dispatcher
        dpt_lock(&srv.interf);

        // some retry may be necessary
        PT_WAIT_UNTIL(pt, OK == dpt_tx(&srv.interf, &srv.out_fr));

        // release the dispatcher if a reply
 //       if (srv.out_fr.resp)
                dpt_unlock(&srv.interf);

        // loop back at start
        PT_RESTART(pt);

        PT_END(pt);
}


// ------------------------------------------
// public functions
//

void srv_init(void)
{
        // init
        FIFO_init(&srv.in, &srv.in_buf, IN_FIFO_SIZE, sizeof(frame_t));
        FIFO_init(&srv.out, &srv.out_buf, OUT_FIFO_SIZE, sizeof(frame_t));

        srv.interf.channel = 10;
        srv.interf.cmde_mask = _CM(FR_MINUT_SERVO_CMD) | _CM(FR_MINUT_SERVO_INFO);
        srv.interf.queue = &srv.in;
        dpt_register(&srv.interf);

        PT_INIT(&srv.pt_in);
        PT_INIT(&srv.pt_out);

        // configure port
        SERVO_DDR |= SERVO_PARA;

        // init the driver, by default, the pwm is zero
        TMR1_init(TMR1_WITHOUT_INTERRUPT, TMR1_PRESCALER_8, TMR1_WGM_FAST_PWM_ICR1, COM1AB_1010, NULL, NULL);

        TMR1_compare_set(TMR1_CAPT, 40000);

        // launch the pwm generation
        TMR1_start();
}

void srv_run(void)
{
        // if incoming command available
        (void)PT_SCHEDULE(srv_in(&srv.pt_in));

        // if outgoing frame to send
        (void)PT_SCHEDULE(srv_out(&srv.pt_out));
}
