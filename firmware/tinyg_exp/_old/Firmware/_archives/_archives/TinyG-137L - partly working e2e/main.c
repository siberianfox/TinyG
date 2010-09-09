/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl
  Copyright (c) 2009 Simen Svale Skogsrud

  < insert GPL language here >

---- TinyG Notes ----

	Modified Grbl to support Xmega family processors
	Alden S. Hart, Jr., May 2010

---- In order to succecssfully compile this... ----

	To compile and link you must use libm.a otherwise the floating point will fail.
		In AVRstudio select Project / Configuration Options
		Select Libraries
		Move libm.a from the left pane to the right pane
		ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0

	Configure project
		32000000 Hz processor, and also set 32.0000 Mhz in debug configs

  	Another annoying avr20100110 bug: 
  		If you are running WinAVR-20100110 you may be asked to locate libraries or 
		include files that were known to a previous avr-gcc version. 

		When asked to browse for stdlib files, go to: C:\WinAVR-20100110\avr\lib\avrxmega6
		When asked to browse for include files go to: C:\WinAVR-20100110\avr\include

---- Using OSX screen to drive it ----

  Procedure to use the USB port from mac OSX:
	- Install the FTDI virtual serial port driver
	- Find your tty device in /dev directory, e.g.
		/dev/tty.usbserial-A700eUQo
	- Invoke screen using your tty device at 115200 baud. From terminal prompt, e.g:
		screen /dev/tty.usbserial-A700eUQo 115200

  If you are running screen (under terminal) in OSX you may want to do this first:
	in terminal, enter: "defaults write com.apple.Terminal TermCapString xterm"
						"export TERM=xterm"
  (ref: http://atomized.org/2006/05/fixing-backspace-in-screen-in-terminal-on-os-x/)

---- Coding conventions ----

  Adopted the following xmega and C variable naming conventions
  (See AVR1000: Getting Started Writing C-code for XMEGA [doc8075.pdf] )

	varname_bm		- single bit mask, e.g. 0x40 aka (1<<4)
	varname_bp		- single bit position, e.g. 4 for the above example
	varname_gm		- group bit mask, e.g. 0x0F
	varname_gc		- group configuration, e.g. 0x0A is 2 bits in the above _gm
	varname_ptr		- indicates a pointer. (but NOT array indexes)
	varname_idx		- indicates an array index (if not simply called i or j)
	varname_vect	- interrupt or other vectors

  These conventions are used for internal variables but may be relaxed for old 
  UNIX vars and DEFINES that don't follow these conventions.
*/

#include <stdio.h>			// only needed by fgets2 in xio.h
#include <avr/interrupt.h>

#include "xmega_support.h"	// must precede <util/delay> and app defines
#include "xmega_interrupts.h"
// required for all those inits...
#include "xio.h"
#include "tinyg.h"
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "encoder.h"
#include "gcode.h"
#include "parsers.h"
#include "config.h"

int main(void) 
{
	/* These inits are order dependent */
	cli();
	xmega_init();				// xmega setup
	xio_init();					// xmega io subsystem

	config_init();				// get config record from eeprom
//	config_test();				// we don't need no stinkin test
	st_init(); 					// stepper subsystem
	mc_init();					// motion control subsystem
	spindle_init();				// spindle controller
	en_init();					// encoders
	gc_init();					// gcode-parser
	tg_init();					// tinyg parsers

	PMIC_SetVectorLocationToApplication();  // as opposed to boot rom
//	PMIC_EnableLowLevel();		// nothing at this level
	PMIC_EnableMediumLevel(); 	// enable serial IO
	PMIC_EnableHighLevel();		// enable stepper timers
	sei();						// enable global interrupts

	for(;;){
		tg_process();			// get next line to process
		st_execute_line();		// run next motion 
	}
}
