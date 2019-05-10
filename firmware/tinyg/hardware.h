/*
 * hardware.h - system hardware configuration
 *				THIS FILE IS HARDWARE PLATFORM SPECIFIC - AVR Xmega version
				这个文件是硬件平台参数-AVR Xmega版本
 *
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * 中断使用 - TinyG 在各个地方大量使用了它们（AVR xmega的中断有3个优先级）
 *
 * 	高优先级 步进电机 DDA 脉冲生成            (在stepper.h中设置)
 *  高优先级 
 *	HI	Stepper load routine SW interrupt	(set in stepper.h)
 *	HI	Dwell timer counter 				(set in stepper.h)
 *  LO	Segment execution SW interrupt		(set in stepper.h)
 *	MED	GPIO1 switch port					(set in gpio.h)
 *  MED	Serial RX for USB & RS-485			(set in xio_usart.h)
 *  MED	Serial TX for USB & RS-485			(set in xio_usart.h) (* see note)
 *	LO	Real time clock interrupt			(set in xmega_rtc.h)
 *
 *	(*) The TX cannot run at LO level or exception reports and other prints
 *		called from a LO interrupt (as in prep_line()) will kill the system in a
 *		permanent sleep_mode() call in xio_putc_usb() (xio.usb.c) as no interrupt
 *		can release the sleep mode.
 *  (*) 发送不能在低中断优先级，否则
 */

#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE

/*--- 硬件平台枚举值 ---*/

enum hwPlatform {
	HM_PLATFORM_NONE = 0,

	HW_PLATFORM_TINYG_XMEGA,	// TinyG代码基于Xmega.
								//	hwVersion 7 = TinyG v7以及更早的
								//	hwVersion 8 = TinyG v8

	HW_PLATFORM_G2_DUE,			// G2 代码基于原生的 Arduino Due

	HW_PLATFORM_TINYG_V9		// G2 代码基于v9主板
								//  hwVersion 0 = v9c
								//  hwVersion 1 = v9d
								//  hwVersion 2 = v9f
								//  hwVersion 3 = v9h
								//  hwVersion 4 = v9i
};

#define HW_VERSION_TINYGV6		6
#define HW_VERSION_TINYGV7		7
#define HW_VERSION_TINYGV8		8

#define HW_VERSION_TINYGV9C		0
#define HW_VERSION_TINYGV9D		1
#define HW_VERSION_TINYGV9F		2
#define HW_VERSION_TINYGV9H		3
#define HW_VERSION_TINYGV9I		4

////////////////////////////
/////// AVR 版本 ////////
////////////////////////////

#include "config.h"						// 需要用到里面的stat_t typedef
#include <avr/interrupt.h>
#include "xmega/xmega_rtc.h"			// 只用于Xmega. Goes away with RTC refactoring

// 在motate的Xmega 引脚可以使用的时候取消注释
//#include "motatePins.h"
//#include "motateTimers.h"				// for Motate::timer_number

/*************************
 * 全局系统定义 *
 *************************/

#undef F_CPU							// CPU 时钟 - 设置用于延时 
#define F_CPU 32000000UL				// 应一直在 <avr/delay.h>之前
#define MILLISECONDS_PER_TICK 1			// MS for system tick (systick * N)
#define SYS_ID_LEN 12					// 从sys_get_id()得到的系统ID字符串长度

/************************************************************************************
 **** AVR XMEGA 特定硬件 ***************************************************
 ************************************************************************************/

// 晶振配置，选取一个:
//#define __CLOCK_INTERNAL_32MHZ TRUE	// 使用内部晶振
//#define __CLOCK_EXTERNAL_8MHZ	TRUE	// 使用PLL来提供32MHz系统时钟
#define __CLOCK_EXTERNAL_16MHZ TRUE		// 使用PLL来提供32MHz系统时钟

/*** 电机，输出位和开关引脚分配***
 *** These are not all the same, and must line up in multiple places in gpio.h ***
 * Sorry if this is confusing - it's a board routing issue
 */
#define PORT_MOTOR_1	PORTA			// 端机映射到端口 
#define PORT_MOTOR_2 	PORTF
#define PORT_MOTOR_3	PORTE
#define PORT_MOTOR_4	PORTD

#define PORT_SWITCH_X 	PORTA			// 轴开关映射到端口
#define PORT_SWITCH_Y 	PORTD
#define PORT_SWITCH_Z 	PORTE
#define PORT_SWITCH_A 	PORTF

#define PORT_OUT_V7_X	PORTA			// v7 映射 - 输出位映射到端口
#define PORT_OUT_V7_Y 	PORTF
#define PORT_OUT_V7_Z	PORTD
#define PORT_OUT_V7_A	PORTE

#define PORT_OUT_V6_X	PORTA			// v6 和更早的 - 输出位映射到端口
#define PORT_OUT_V6_Y 	PORTF
#define PORT_OUT_V6_Z	PORTE
#define PORT_OUT_V6_A	PORTD

// 以下四个必须在PORT_MOTOR_*定义变化的时候跟随着一起更改
#define PORTCFG_VP0MAP_PORT_MOTOR_1_gc PORTCFG_VP0MAP_PORTA_gc
#define PORTCFG_VP1MAP_PORT_MOTOR_2_gc PORTCFG_VP1MAP_PORTF_gc
#define PORTCFG_VP2MAP_PORT_MOTOR_3_gc PORTCFG_VP2MAP_PORTE_gc
#define PORTCFG_VP3MAP_PORT_MOTOR_4_gc PORTCFG_VP3MAP_PORTD_gc

#define PORT_MOTOR_1_VPORT	VPORT0
#define PORT_MOTOR_2_VPORT	VPORT1
#define PORT_MOTOR_3_VPORT	VPORT2
#define PORT_MOTOR_4_VPORT	VPORT3

/*
 * 端口设置 - 步进电机 / 开关端口
 *	b0	(out) step			(SET is step,  CLR is rest)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b3	(out) microstep 0
 *	b4	(out) microstep 1
 *	b5	(out) output bit for GPIO port1
 *	b6	(in) min limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 *	b7	(in) max limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 */
#define MOTOR_PORT_DIR_gm 0x3F	// 方向设置：低6位输出，高2位输入
//#define MOTOR_PORT_DIR_gm 0x00	// 方向设置:所有都是输入

enum cfgPortBits {			// motor control port bit positions
	STEP_BIT_bp = 0,		// bit 0
	DIRECTION_BIT_bp,		// bit 1
	MOTOR_ENABLE_BIT_bp,	// bit 2
	MICROSTEP_BIT_0_bp,		// bit 3
	MICROSTEP_BIT_1_bp,		// bit 4
	GPIO1_OUT_BIT_bp,		// bit 5 (4 gpio1 output bits; 1 from each axis)
	SW_MIN_BIT_bp,			// bit 6 (4 input bits for homing/limit switches)
	SW_MAX_BIT_bp			// bit 7 (4 input bits for homing/limit switches)
};

#define STEP_BIT_bm			(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm	(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm (1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm	(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm	(1<<MICROSTEP_BIT_1_bp)
#define GPIO1_OUT_BIT_bm	(1<<GPIO1_OUT_BIT_bp)	// 主轴和水冷输出位
#define SW_MIN_BIT_bm		(1<<SW_MIN_BIT_bp)		// 负方向开关输入
#define SW_MAX_BIT_bm		(1<<SW_MAX_BIT_bp)		// 正方向开关输入

/* GPIO1输出位分配，用于主轴，PWM和水冷*/

#define SPINDLE_BIT			0x08		// 主轴开或者关
#define SPINDLE_DIR			0x04		// 主轴方向 1=顺时针 0=逆时针
#define SPINDLE_PWM			0x02		// 主轴PWM输出位
#define MIST_COOLANT_BIT	0x01		// 水冷开关 - 由于输出端口数量限制，它和下面的共享端口
#define FLOOD_COOLANT_BIT	0x01		// 水冷开关

#define SPINDLE_LED			0
#define SPINDLE_DIR_LED		1
#define SPINDLE_PWM_LED		2
#define COOLANT_LED			3

#define INDICATOR_LED		SPINDLE_DIR_LED	// 可以用主轴方向的led灯作为指示灯

/* 定时器分配 - 查看具体的模组获取详情 */

#define TIMER_DDA			TCC0		// DDA 定时器 	(see stepper.h)
#define TIMER_DWELL	 		TCD0		// Dwell 定时器	(see stepper.h)
#define TIMER_LOAD			TCE0		// Loader 定时器(see stepper.h)
#define TIMER_EXEC			TCF0		// Exec 定时器	(see stepper.h)
#define TIMER_5				TCC1		// 未使用定时器 
#define TIMER_PWM1			TCD1		// PWM 定时器 #1 (see pwm.c)
#define TIMER_PWM2			TCE1		// PWM 定时器 #2 (see pwm.c)

/* 步进电机和dewlls的定时器设置 */

#define FREQUENCY_DDA 		(float)50000	// DDA 频率 单位为Hz
#define FREQUENCY_DWELL		(float)10000	// Dwell 计数频率，单位为hz.
#define LOAD_TIMER_PERIOD 	100				// cycles you have to shut off SW interrupt
#define EXEC_TIMER_PERIOD 	100				// cycles you have to shut off SW interrupt
#define EXEC_TIMER_PERIOD_LONG 100			// cycles you have to shut off SW interrupt

#define STEP_TIMER_TYPE		TC0_struct 		// stepper subsybstem uses all the TC0's
#define STEP_TIMER_DISABLE 	0				// turn timer off (clock = 0 Hz)
#define STEP_TIMER_ENABLE	1				// turn timer clock on (F_CPU = 32 Mhz)
#define STEP_TIMER_WGMODE	0				// normal mode (count to TOP and rollover)

#define LOAD_TIMER_DISABLE 	0				// turn load timer off (clock = 0 Hz)
#define LOAD_TIMER_ENABLE	1				// turn load timer clock on (F_CPU = 32 Mhz)
#define LOAD_TIMER_WGMODE	0				// normal mode (count to TOP and rollover)

#define EXEC_TIMER_DISABLE 	0				// turn exec timer off (clock = 0 Hz)
#define EXEC_TIMER_ENABLE	1				// turn exec timer clock on (F_CPU = 32 Mhz)
#define EXEC_TIMER_WGMODE	0				// normal mode (count to TOP and rollover)

#define TIMER_DDA_ISR_vect	TCC0_OVF_vect	// must agree with assignment in system.h
#define TIMER_DWELL_ISR_vect TCD0_OVF_vect	// must agree with assignment in system.h
#define TIMER_LOAD_ISR_vect	TCE0_OVF_vect	// must agree with assignment in system.h
#define TIMER_EXEC_ISR_vect	TCF0_OVF_vect	// must agree with assignment in system.h

#define TIMER_OVFINTLVL_HI	3				// timer interrupt level (3=hi)
#define	TIMER_OVFINTLVL_MED 2;				// timer interrupt level (2=med)
#define	TIMER_OVFINTLVL_LO  1;				// timer interrupt level (1=lo)

#define TIMER_DDA_INTLVL 	TIMER_OVFINTLVL_HI
#define TIMER_DWELL_INTLVL	TIMER_OVFINTLVL_HI
#define TIMER_LOAD_INTLVL	TIMER_OVFINTLVL_HI
#define TIMER_EXEC_INTLVL	TIMER_OVFINTLVL_LO


/**** Device singleton - global structure to allow iteration through similar devices ****/
/*
	端口都是在步进电机和GPIO之间共享的，所以我们需要一个全局结构。
	每个xmega端口有三个绑定，电机，开关和输出位

	初始化顺序非常重要。它们的顺序是：
		- sys_init() 绑定所有端口到device结构体。
		- st_init()  设置IO方向和设置步进电机


	The initialization sequence is important. the order is:
		- sys_init()	binds all ports to the device struct
		- st_init() 	sets IO directions and sets stepper VPORTS and stepper specific functions
		- gpio_init()	sets up input and output functions and required interrupts

	Care needs to be taken in routines that use ports not to write to bits that are
	not assigned to the designated function - ur unpredicatable results will occur
*/

typedef struct hmSingleton {
	PORT_t *st_port[MOTORS];		// 绑定用于步进电机端口 (stepper.c)
	PORT_t *sw_port[MOTORS];		// 绑定用于开关端口
	PORT_t *out_port[MOTORS];		// 绑定用于输出端口(GPIO1)
} hwSingleton_t;
hwSingleton_t hw;

/*** 函数原型 ***/

void hardware_init(void);			// 主硬件初始化
void hw_request_hard_reset();
void hw_hard_reset(void);
stat_t hw_hard_reset_handler(void);

void hw_request_bootloader(void);
stat_t hw_bootloader_handler(void);
stat_t hw_run_boot(nvObj_t *nv);

stat_t hw_set_hv(nvObj_t *nv);
stat_t hw_get_id(nvObj_t *nv);

#ifdef __TEXT_MODE

	void hw_print_fb(nvObj_t *nv);
	void hw_print_fv(nvObj_t *nv);
	void hw_print_hp(nvObj_t *nv);
	void hw_print_hv(nvObj_t *nv);
	void hw_print_id(nvObj_t *nv);

#else

	#define hw_print_fb tx_print_stub
	#define hw_print_fv tx_print_stub
	#define hw_print_hp tx_print_stub
	#define hw_print_hv tx_print_stub
	#define hw_print_id tx_print_stub

#endif // __TEXT_MODE

#endif	// end of include guard: HARDWARE_H_ONCE
