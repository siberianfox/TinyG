/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

NOTES by Alden
---	To compile and link you must use libm.a otherwise the floating point will fail.
	In AVRstudio select Project / Configuration Options
	Select Libraries
	Move libm.a from the left pane to the right pane
	ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0

---	Stuff to do:
	- replace the bresenham line estimator with 4 actual timers doing interpolation

*/

#include <avr/io.h>
#include <avr/sleep.h>
#include "xmega_support.h"	// must precede <util/delay> and app defines
#include <util/delay.h>
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "serial_protocol.h"
#include "config.h"
#include "wiring_serial.h"


int main(void) 
{
	/* These inits are order dependent */
	xmega_init();			// xmega setup
	config_init();			// get config record from eeprom
//	beginSerial(BAUD_RATE);	// serial IO system - redundant init
	st_init(); 				// stepper subsystem
	mc_init();				// motion control subsystem
	spindle_init();			// spindle controller
	en_init();				// encoders
	gc_init();				// gcode-parser
	sp_init();				// serial protocol

	for(;;){
		sleep_mode();
		sp_process(); // process the serial protocol
	}
	return 0;   /* never reached */
}
