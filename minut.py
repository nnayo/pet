"""
generate the eeprom image for egere minuterie frames
"""


from frame import *
from frame import Frame

# servo info
SERVO_PARA = 0xc0

SERVO_SAVE = 0x5a
SERVO_READ = 0x4e

SERVO_OPEN_POS = 0x09
SERVO_CLOSE_POS = 0xc1

# servo command
SERVO_OPEN = 0x09
SERVO_CLOSE = 0xc1
SERVO_OFF = 0x0f

# time-out
TIME_OUT_SAVE = 0x00

# state
STATE_GET = 0x9e
STATE_SET = 0x5e

STATE_INIT = 0x00
STATE_OPEN = 0x01
STATE_CLOSE = 0x02
STATE_WAITING = 0x04
STATE_FLIGHT = 0x08
STATE_PARACHUTE = 0x10

# led
LED_ALIVE = 0xa1
LED_OPEN = 0x09
LED_SET = 0x00
LED_GET = 0xff

# generic data
I2C_SELF_ADDR = Frame.I2C_SELF_ADDR
T_ID = Frame.T_ID
CMD = Frame.CMD


# slots number
slots_nb = 7

slots = [
	#--------------------------------
	# slot #0 : reset
	[
		# set cone servo open position: -15deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_SAVE, SERVO_OPEN_POS, -90),

		# set cone servo closed position: +30deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_SAVE, SERVO_CLOSE_POS, 45),

		# set flight time-out: 8.5s
		minut_time_out(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, TIME_OUT_SAVE, 85),

		# send application start signal
		appli_start(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD),
	],

	#--------------------------------
	# slot #1 : minut init state
	[
                state(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_INIT),
                #minut_servo_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_OFF),
                led_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 10, 5),
	],

	#--------------------------------
	# slot #2 : minut open state
	[
                state(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_OPEN),
                minut_servo_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_OPEN),
                led_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 10, 40),
	],

	#--------------------------------
	# slot #3 : minut close state
	[
                state(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_CLOSE),
                minut_servo_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_CLOSE),
                led_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 40, 10),
	],

	#--------------------------------
	# slot #4 : minut waiting state
	[
                state(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_WAITING),
                #minut_servo_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_OFF),
                led_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 90, 10),
	],

	#--------------------------------
	# slot #5 : minut flight state
	[
                state(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_PARACHUTE),
                #minut_servo_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_CLOSE),
                led_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 10, 10),
	],

	#--------------------------------
	# slot #6 : minut parachute state
	[
                state(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_PARACHUTE),
                minut_servo_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_PARA, SERVO_OPEN),
                led_cmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 20, 20),
	],
]

