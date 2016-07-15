#!/usr/bin/python
# -*- coding: latin-1 -*-

project_name = 'minut'

import os
try:
    troll_path = os.environ['TROLL_PROJECTS']
except KeyError:
    troll_path = os.environ['HOME'] + '/TRoll/projects'
    os.environ['TROLL_PROJECTS'] = troll_path


minut	= [
	'main.c',			\
	'minut.c',			\
	'servo.c',			\
	'tk-off.c',			\
	'eeprom_frames.c',	\
]

libs = ['scalp', 'nanoK']
libpath = [troll_path + '/scalp', troll_path + '/nanoK']

mcu_target      = 'atmega328p'
optimize        = '-Os -mcall-prologues -fshort-enums -std=c99 '
#optimize        = '-Os -mcall-prologues -fshort-enums '
includes	= [
				'.',
				troll_path + '/nanoK',
				troll_path + '/scalp',
                troll_path + '/simavr/simavr/sim/avr',
			]
cflags		= '-g -Wall -Wextra ' + optimize + '-mmcu=' + mcu_target
ldflags		= '-g -Wall ' + optimize + '-mmcu=' + mcu_target + ' -Wl,-Map,' + project_name + '.map,--cref '
ldflags		+= '-Wl,--undefined=_mmcu,--section-start=.mmcu=0x8000'


builder_hex = Builder(
	action = Action(
		"avr-objcopy -O ihex -R .eeprom -R .mmcu $SOURCE $TARGET", cmdstr = "$HEXCOMSTR"), 
		suffix = ".hex", 
		src_suffix = ".elf"
)

env = Environment(
	CC = 'avr-gcc',		\
	AR = 'avr-ar',		\
	CFLAGS = cflags,	\
	CPPPATH = includes,	\
	LINKFLAGS = ldflags,	\
)

env.Append( BUILDERS = { 'Hex': builder_hex, } )
env.Hex(project_name, project_name)

Export('env')

SConscript([troll_path + '/scalp/SConscript', troll_path + '/nanoK/SConscript'], exports='env')

elf = env.Program(project_name + '.elf', minut, LIBS = libs, LIBPATH = libpath)
env.Default(elf)

# autogen eeprom_frame.c file
env.Depends('eeprom_frames.c', ['./gen_eeprom_frames.py', 'frame.py', 'minut.py'])
env.Command('eeprom_frames.c', '', './gen_eeprom_frames.py eeprom_frames.c')

# generate a file with code and source
env.Alias('lix', project_name + '.elf', 'avr-objdump -h -sdx ' + project_name + '.elf > ' + project_name + '.lix')
env.AlwaysBuild('lix')


# give the size of the binary
env.Alias('size', project_name + '.elf', 'avr-size -t ' + project_name + '.elf')
env.AlwaysBuild('size')


# load binary in flash
env.Depends(project_name + '.flash.hex', project_name + '.elf')
env.Command(project_name + '.flash.hex', project_name + '.elf', 'avr-objcopy -j .text -j .data -O ihex ' + project_name + '.elf' + ' ' + project_name + '.flash.hex')
load = env.Alias('load', project_name + '.flash.hex', 'avrdude -V -c arduino -p ATMEGA328P -P /dev/ttyACM0 -b 57600 -U flash:w:' + project_name + '.flash.hex')
env.AlwaysBuild(load)


# download the frames in eeprom
env.Depends(project_name + '.eeprom.hex', project_name + '.elf')
env.Command(project_name + '.eeprom.hex', project_name + '.elf', 'avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex ' + project_name + '.elf' + ' ' + project_name + '.eeprom.hex')

env.Alias('eeprom', project_name + '.eeprom.hex', 'avrdude -V -c arduino -p ATMEGA328P -D -P /dev/ttyACM0 -b 57600 -U eeprom:w:' + project_name + '.eeprom.hex')
env.AlwaysBuild('eeprom')

env.Alias('eeprom_check', '', 'avrdude -V -c arduino -p ATMEGA328P -D -P /dev/ttyACM0 -b 57600 -U eeprom:r:eeprom_content.raw:h')
env.AlwaysBuild('eeprom_check')

# connect to target with a terminal
env.Alias('term', '', 'avrdude -V -c arduino -p ATMEGA328P -D -P /dev/ttyACM0 -b 57600 -t')
env.AlwaysBuild('term')


# test with simavr
#env.Alias('sim', project_name + '.elf', 'time ' + troll_path + '/simavr/simavr/run_avr -t -v -s ' + troll_path + '/simavr/examples/sim/mpu6050.so ' + troll_path + '/égère/soft/minut.elf > log')
env.Alias('sim', (project_name + '.elf', project_name + '.eeprom.hex'), troll_path + '/simavr/simavr/run_avr -t -v -g ' + troll_path + '/pet/minut.elf')
env.AlwaysBuild('sim')

# extract frames from gtkwave_trace.vcd file
env.Alias('log', 'gtkwave_trace.vcd', troll_path + '/interface_server/streamer.py localhost:7777 gtkwave_trace.vcd UDR0& sleep 1; ' + troll_path + '/interface_server/onflight_decode.py localhost:7777 > decoded.log')
env.AlwaysBuild('log')


# test with simavr & avr-gdb
env.Alias('debug', project_name + '.elf', '~/TRoll/projects/simavr/simavr/run_avr -g -t -v ' + project_name + '.elf')
env.AlwaysBuild('debug')

env.Alias('gdb', project_name + '.elf', 'avr-gdb ' + project_name + '.elf -n -x gdbinit_avr')
env.AlwaysBuild('gdb')


# suppress reliquat files
env.Alias('clean', '', 'rm -f *~ *o */*.o *.a *.lis')
env.AlwaysBuild('clean')

