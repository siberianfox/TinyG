/*
 * stepper.h - stepper motor interface
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#ifndef stepper_h
#define stepper_h

/*
 * Global Scope Functions
 */

void st_init(void);					// Initialize and start stepper motor subsystem
void st_motor_test(void);			// Test stepper motor subsystem
void st_execute_move(void);			// Dequeue and start next linear move in the move buffer
void st_execute_move2(void);		// Dequeue and start next linear move in the move buffer
void st_kill(void);					// Kill current move
void st_terminate(void);			// Terminate moves after the current move
void st_go_home(void);				// Execute the homing cycle

#endif
