#include "minut.h"

#include "type_def.h"
#include "dispatcher.h"
#include "dna.h"

#include "utils/pt.h"
#include "utils/time.h"
#include "utils/fifo.h"
#include "utils/state_machine.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

//for debug
#define static


// ------------------------------------------
// private definitions
//

#define NB_EVENTS	5
#define NB_IN_FR	3
#define NB_OUT_FR	4

#define CONE_DDR			DDRB
#define CONE				PINB
#define CONE_PIN			PB3
#define CONE_STATE_CLOSED	0
#define CONE_STATE_OPEN		_BV(CONE_PIN)

#define SAMPLING_START		(2 * TIME_1_SEC)
#define SAMPLING_PERIOD		(100 * TIME_1_MSEC)


// ------------------------------------------
// private types
//

typedef enum {
	mnt_EV_NONE,
	mnt_EV_TIME_OUT,
	mnt_EV_TAKE_OFF,
} mnt_event_t;


// ------------------------------------------
// private variables
//

struct {
	dpt_interface_t interf;	// dispatcher interface

	pt_t pt_chk_time_out;	// checking time-out thread
	pt_t pt_chk_cmds;	// checking commands thread
	pt_t pt_out;		// sending thread

	stm_t stm;

	volatile u32 time_out;	// time-out target time
	u32 sampling_rate;	// sampling rate for door changings
	u32 take_off_time_out;	// take-off scan interval time
	u32 door_time_out;	// door scan interval time
	u32 check_time_out;	// state scan interval time

	u8 open_time;		// open time [0.0; 25.5] seconds from take-off detection

	// events fifo
	fifo_t ev_fifo;
	mnt_event_t ev_buf[NB_EVENTS];

	// incoming commands fifo
	fifo_t in_fifo;
	frame_t in_buf[NB_IN_FR];
	frame_t in_fr;

	// outcoming frames fifo
	fifo_t out_fifo;
	frame_t out_buf[NB_OUT_FR];
	frame_t out_fr;		// frame for the sending thread

	u8 started:1;		// signal to application can be started
} mnt;

static const stm_transition_t init2para_opening;
static const stm_transition_t para_opening2para_closing;
static const stm_transition_t para_closing2waiting;
static const stm_transition_t waiting2flight;
static const stm_transition_t flight2parachute;

static const stm_state_t init;
static const stm_state_t para_opening;
static const stm_state_t para_closing;
static const stm_state_t waiting;
static const stm_state_t flight;
static const stm_state_t parachute;

static u8 action_init(pt_t* pt, void* args);
static u8 action_para_opening(pt_t* pt, void* args);
static u8 action_para_closing(pt_t* pt, void* args);
static u8 action_waiting(pt_t* pt, void* args);
static u8 action_flight(pt_t* pt, void* args);
static u8 action_parachute(pt_t* pt, void* args);

// transitions
static const stm_transition_t init2para_opening = {
	.ev = mnt_EV_TIME_OUT,
	.st = &para_opening,
	.tr = NULL,
};

static const stm_transition_t para_opening2para_closing = {
	.ev = mnt_EV_TIME_OUT,
	.st = &para_closing,
	.tr = NULL,
};

static const stm_transition_t para_closing2waiting = {
	.ev = mnt_EV_TIME_OUT,
	.st = &waiting,
	.tr = NULL,
};

static const stm_transition_t waiting2flight = {
	.ev = mnt_EV_TAKE_OFF,
	.st = &flight,
	.tr = NULL,
};

static const stm_transition_t flight2parachute = {
	.ev = mnt_EV_TIME_OUT,
	.st = &parachute,
	.tr = NULL,
};

// states
static const stm_state_t init = {
	.action = action_init,
	.transition = &init2para_opening,
};

static const stm_state_t para_opening = {
	.action = action_para_opening,
	.transition = &para_opening2para_closing,
};

static const stm_state_t para_closing = {
	.action = action_para_closing,
	.transition = &para_closing2waiting,
};

static const stm_state_t waiting = {
	.action = action_waiting,
	.transition = &waiting2flight,
};

static const stm_state_t flight = {
	.action = action_flight,
	.transition = &flight2parachute,
};

static const stm_state_t parachute = {
	.action = action_parachute,
	.transition = NULL,
};

// ------------------------------------------
// private functions
//

// actions
static u8 action_init(pt_t* pt, void* args)
{
        (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// preset container #1
	PT_WAIT_UNTIL(pt, frame_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_CONTAINER, 0, 0, 0, 0, 1)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 1s
	mnt.time_out = TIME_get() + 1 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_para_opening(pt_t* pt, void* args)
{
        (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// preset container #2
	PT_WAIT_UNTIL(pt, frame_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_CONTAINER, 0, 0, 0, 0, 2)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 5s
	mnt.time_out = TIME_get() + 5 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_para_closing(pt_t* pt, void* args)
{
        (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// preset container #3
	PT_WAIT_UNTIL(pt, frame_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_CONTAINER, 0, 0, 0, 0, 3)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 2s
	mnt.time_out = TIME_get() + 2 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_waiting(pt_t* pt, void* args)
{
        (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// preset container #4
	PT_WAIT_UNTIL(pt, frame_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_CONTAINER, 0, 0, 0, 0, 4)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_flight(pt_t* pt, void* args)
{
        (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// preset container #5
	PT_WAIT_UNTIL(pt, frame_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_CONTAINER, 0, 0, 0, 0, 5)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out = flight time
	mnt.time_out = TIME_get() + mnt.open_time * TIME_1_SEC / 10;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_parachute(pt_t* pt, void* args)
{
        (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// preset container #6
	PT_WAIT_UNTIL(pt, frame_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_CONTAINER, 0, 0, 0, 0, 6)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

// check a time-out has elapsed
static PT_THREAD( mnt_check_time_out(pt_t* pt) )
{
	mnt_event_t ev;

	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, TIME_get() > mnt.time_out);

	// prevent any further time-out
	mnt.time_out = TIME_MAX;

	// generate the time-out event
	PT_WAIT_UNTIL(pt, (ev = mnt_EV_TIME_OUT) && OK == FIFO_put(&mnt.ev_fifo, &ev) );

	PT_RESTART(pt);

	PT_END(pt);
}

static void mnt_open_time(frame_t* fr)
{
	switch (fr->argv[0]) {
		case 0x00:
			// save new open time value
			mnt.open_time = fr->argv[1];
			break;

		case 0xff:
			// read open time value
			fr->argv[1] = mnt.open_time;
			break;

		default:
			// bad sub-command
			fr->error = 1;
			break;
	}
}

static PT_THREAD( mnt_check_commands(pt_t* pt) )
{
	mnt_event_t ev;
	u8 swap;

	PT_BEGIN(pt);

	// as long as there are no command
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&mnt.in_fifo, &mnt.in_fr));

	// silently ignore incoming response
	if ( mnt.in_fr.resp == 1 ) {
		//dpt_unlock(&mnt.interf);
		PT_RESTART(pt);
	}

	switch (mnt.in_fr.cmde) {
		case FR_TAKE_OFF:
			// generate take-off event
			PT_WAIT_UNTIL(pt, (ev = mnt_EV_TAKE_OFF) && OK == FIFO_put(&mnt.ev_fifo, &ev) );
			break;

		case FR_MINUT_TIME_OUT:
			mnt_open_time(&mnt.in_fr);
			break;

		case FR_STATE:
			if ( (mnt.in_fr.argv[0] == 0x7a) || (mnt.in_fr.argv[0] == 0x8b) ) {
				//mnt.state = mnt.in_fr.argv[1];
			}

			// don't respond, response will be done by CMN
			PT_RESTART(pt);
			break;

		case FR_APPLI_START:
			mnt.started = 1;

			// don't respond
			PT_RESTART(pt);
			break;

		default:
			// shall never happen
			break;
	}

	// build the response to the current command
	swap = mnt.in_fr.orig;
	mnt.in_fr.orig = mnt.in_fr.dest;
	mnt.in_fr.dest = swap;
	mnt.in_fr.resp = 1;

	// enqueue it
	PT_WAIT_UNTIL(pt, OK == FIFO_put(&mnt.out_fifo, &mnt.in_fr));

	PT_RESTART(pt);

	PT_END(pt);
}

static PT_THREAD( mnt_send_frame(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until an outgoing frame is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&mnt.out_fifo, &mnt.out_fr));

	// send the frame throught the dispatcher
	dpt_lock(&mnt.interf);

	// some retry may be needed
	PT_WAIT_UNTIL(pt, OK == dpt_tx(&mnt.interf, &mnt.out_fr));

	// release the dispatcher
	dpt_unlock(&mnt.interf);

	// loop back for the next frame to send
	PT_RESTART(pt);
	
	PT_END(pt);
}

// ------------------------------------------
// public functions
//

void mnt_init(void)
{
	// init state machine
	STM_init(&mnt.stm, &init);

	// init fifoes
	FIFO_init(&mnt.ev_fifo, mnt.ev_buf, NB_EVENTS, sizeof(mnt_event_t));
	FIFO_init(&mnt.in_fifo, mnt.in_buf, NB_IN_FR, sizeof(frame_t));
	FIFO_init(&mnt.out_fifo, mnt.out_buf, NB_OUT_FR, sizeof(frame_t));

	// register to dispatcher
	mnt.interf.channel = 7;
	mnt.interf.cmde_mask = _CM(FR_TAKE_OFF) | _CM(FR_MINUT_TIME_OUT) | _CM(FR_STATE) | _CM(FR_APPLI_START);
	mnt.interf.queue = &mnt.in_fifo;
	dpt_register(&mnt.interf);

	// init threads
	PT_INIT(&mnt.pt_chk_time_out);
	PT_INIT(&mnt.pt_chk_cmds);
	PT_INIT(&mnt.pt_out);

	// prevent any time-out
	mnt.time_out = TIME_MAX;
	mnt.sampling_rate = SAMPLING_START;

	// the application start signal shall be received
	mnt.started = 0;
}

void mnt_run(void)
{
	// event sources are :
	//  - take-off detector
	//  - time-out
	//  - door detectors
	//  - frame commands
	//
	//  each generated event is stored in a fifo

	if ( mnt.started ) {
		// check if a time-out has elapsed
		(void)PT_SCHEDULE(mnt_check_time_out(&mnt.pt_chk_time_out));
	}

	// treat each incoming commands
	(void)PT_SCHEDULE(mnt_check_commands(&mnt.pt_chk_cmds));

	if ( mnt.started ) {
		// treat each new event
		mnt_event_t ev;

		// if there is an event
		if ( OK == FIFO_get(&mnt.ev_fifo, &ev) ) {
			// send it to the state machine
			STM_event(&mnt.stm, ev);
		}

		// update state machine
		STM_run(&mnt.stm);
	}

	// send outgoing frame(s) if any
	(void)PT_SCHEDULE(mnt_send_frame(&mnt.pt_out));
}
