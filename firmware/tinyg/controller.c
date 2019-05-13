/*
 * controller.c - tinyg controller and top level parser
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

#include "tinyg.h"				// #1
#include "config.h"				// #2
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"

#include "encoder.h"
#include "hardware.h"
#include "switch.h"
#include "gpio.h"
#include "report.h"
#include "help.h"
#include "util.h"
#include "xio.h"

#ifdef __ARM
#include "Reset.h"
#endif

/***********************************************************************************
 **** 结构体分配 ********************************************************************
 ***********************************************************************************/

controller_t cs;		// 控制器状态结构体 

/***********************************************************************************
 **** 静态和本地函数 ****************************************************************
 ***********************************************************************************/

static void _controller_HSM(void);
static stat_t _shutdown_idler(void);
static stat_t _normal_idler(void);
static stat_t _limit_switch_handler(void);
static stat_t _system_assertions(void);
static stat_t _sync_to_planner(void);
static stat_t _sync_to_tx_buffer(void);
static stat_t _command_dispatch(void);

// prep for export to other modules:
stat_t hardware_hard_reset_handler(void);
stat_t hardware_bootloader_handler(void);

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - 控制器初始化 
 */

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err)
{
	memset(&cs, 0, sizeof(controller_t));			// 清除所有值，jobid，指针和状态
	controller_init_assertions();

	cs.fw_build = TINYG_FIRMWARE_BUILD;
	cs.fw_version = TINYG_FIRMWARE_VERSION;
	cs.hw_platform = TINYG_HARDWARE_PLATFORM;		// NB: 硬件版本从EEPROM中设置 

#ifdef __AVR
	cs.state = CONTROLLER_STARTUP;					// 准备好运行起始行了
	xio_set_stdin(std_in);
	xio_set_stdout(std_out);
	xio_set_stderr(std_err);
	cs.default_src = std_in;
	tg_set_primary_source(cs.default_src);
#endif

#ifdef __ARM
	cs.state = CONTROLLER_NOT_CONNECTED;			// find USB next
	IndicatorLed.setFrequency(100000);
#endif
}

/*
 * controller_init_assertions()
 * controller_test_assertions() - 为控制器检查内存完整性 
 */

void controller_init_assertions()
{
	cs.magic_start = MAGICNUM;
	cs.magic_end = MAGICNUM;
}

stat_t controller_test_assertions()
{
	if ((cs.magic_start != MAGICNUM) || (cs.magic_end != MAGICNUM)) return (STAT_CONTROLLER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * controller_run() - 主循环 —— 顶层控制器 
 *
 * 分发任务的顺序是非常重要的。
 * 通过增加依赖性（阻塞层次结构）来排序任务。
 * 依赖于更底层任务的完成的任务在列表中必须排在他们所依赖的任务的后面。
 *
 * 任务必须写成连续的模式，因为它们会被重复地调用，即使他们在当前都没有实际的生效。
 *
 * DISPATAH 宏调用函数并在函数返回STAT_EAGAIN（未完成）的时候返回controller主体。
 * 以阻止后面的函数被执行（它们仍然处于堵塞状态）在其它情况下- OK 或者 ERR ，抛弃并
 * 执行列表下面的函数
 *
 * 一个没有动作的函数必须返回(STAT_NOOP)，例如OFF或者idle
 */

void controller_run()
{
	while (true) {
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == STAT_EAGAIN) return;
static void _controller_HSM()
{
//----- Interrupt Service Routines are the highest priority controller functions ----//
//      See hardware.h for a list of ISRs and their priorities.
//
//----- 核心层中断处理器 ----(标志都是在中断中设置的)------------------------//
												// 顺序很重要:
	DISPATCH(hw_hard_reset_handler());			// 1. 处理硬件复位请求 
	DISPATCH(hw_bootloader_handler());			// 2. 处理进入bootloader请求
	DISPATCH(_shutdown_idler());				// 3. idle in shutdown state
//	DISPATCH( poll_switches());					// 4. run a switch polling cycle
	DISPATCH(_limit_switch_handler());			// 5. limit switch has been thrown

	DISPATCH(cm_feedhold_sequencing_callback());// 6a. feedhold state machine runner
	DISPATCH(mp_plan_hold_callback());			// 6b. plan a feedhold from line runtime
	DISPATCH(_system_assertions());				// 7. system integrity assertions

//----- G代码和循环的规划器结构 ---------------------------------------//

	DISPATCH(st_motor_power_callback());		// stepper motor power sequencing
//	DISPATCH(switch_debounce_callback());		// debounce switches
	DISPATCH(sr_status_report_callback());		// conditionally send status report
	DISPATCH(qr_queue_report_callback());		// conditionally send queue report
	DISPATCH(rx_report_callback());             // conditionally send rx report
	DISPATCH(cm_arc_callback());				// arc generation runs behind lines
	DISPATCH(cm_homing_callback());				// G28.2 continuation
	DISPATCH(cm_jogging_callback());			// jog function
	DISPATCH(cm_probe_callback());				// G38.2 continuation
	DISPATCH(cm_deferred_write_callback());		// persist G10 changes when not in machining cycle

//----- command readers and parsers --------------------------------------------------//

	DISPATCH(_sync_to_planner());				// 确保规划队列中这里至少有一个空余的缓冲
	DISPATCH(_sync_to_tx_buffer());				// sync with TX buffer (pseudo-blocking)
#ifdef __AVR
	DISPATCH(set_baud_callback());				// 执行波特率更新（必须在TX SYNC后面）
#endif
	DISPATCH(_command_dispatch());				// 读取并执行下一条命令 
	DISPATCH(_normal_idler());					// blink LEDs slowly to show everything is OK
}

/*****************************************************************************
 * _command_dispatch() - 分发从实际输入设备中接收到的行数据
 *
 *  读取下一个命令行并分发到相关的解析器并执行。
 *  当运动队列中有空间的时候接收命令,当没有空间的时候返回EAGAINS。
 *  管理EOF截断。
 *  同时也有协助流控制的功能
 */

static stat_t _command_dispatch()
{
#ifdef __AVR
	stat_t status;

	// 读取输入行并在当前不是完整行的时候返回
	// xio_gets() 是一个非堵塞，类fgets()的函数
	while (true) {
		if ((status = xio_gets(cs.primary_src, cs.in_buf, sizeof(cs.in_buf))) == STAT_OK) {
			cs.bufp = cs.in_buf;
			break;
		}
		// 从file devices中处理 end-of-line
		if (status == STAT_EOF) {						// EOF 只能源于文件设备(file devices)
			if (cfg.comm_mode == TEXT_MODE) {
				fprintf_P(stderr, PSTR("End of command file\n"));
			} else {
				rpt_exception(STAT_EOF);				// 不是一个真正的异常
			}
			tg_reset_source();							// 复位到默认的源
		}
		return (status);								// Note: STAT_EAGAIN, errors, etc. will drop through
	}
#endif // __AVR
#ifdef __ARM
	// detect USB connection and transition to disconnected state if it disconnected
	if (SerialUSB.isConnected() == false) cs.state = CONTROLLER_NOT_CONNECTED;

	// read input line and return if not a completed line
	if (cs.state == CONTROLLER_READY) {
		if (read_line(cs.in_buf, &cs.read_index, sizeof(cs.in_buf)) != STAT_OK) {
			cs.bufp = cs.in_buf;
			return (STAT_OK);	// This is an exception: returns OK for anything NOT OK, so the idler always runs
		}
	} else if (cs.state == CONTROLLER_NOT_CONNECTED) {
		if (SerialUSB.isConnected() == false) return (STAT_OK);
		cm_request_queue_flush();
		rpt_print_system_ready_message();
		cs.state = CONTROLLER_STARTUP;

	} else if (cs.state == CONTROLLER_STARTUP) {		// run startup code
		cs.state = CONTROLLER_READY;

	} else {
		return (STAT_OK);
	}
	cs.read_index = 0;
#endif // __ARM

	// 设置缓冲
	cs.linelen = strlen(cs.in_buf)+1;					// linelen only tracks primary input
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);	// save input buffer for reporting

	// dispatch the new text line
	switch (toupper(*cs.bufp)) {						// first char

		case '!': { cm_request_feedhold(); break; }		// include for AVR diagnostics and ARM serial
		case '%': { cm_request_queue_flush(); break; }
		case '~': { cm_request_cycle_start(); break; }

		case NUL: { 									// blank line (just a CR)
			if (cfg.comm_mode != JSON_MODE) {
				text_response(STAT_OK, cs.saved_buf);
			}
			break;
		}
		case '$': case '?': case 'H': { 				// text mode input
			cfg.comm_mode = TEXT_MODE;
			text_response(text_parser(cs.bufp), cs.saved_buf);
			break;
		}
		case '{': { 									// JSON input
			cfg.comm_mode = JSON_MODE;
			json_parser(cs.bufp);
			break;
		}
		default: {										// anything else must be Gcode
			if (cfg.comm_mode == JSON_MODE) {			// run it as JSON...
				strncpy(cs.out_buf, cs.bufp, INPUT_BUFFER_LEN -8);					// use out_buf as temp
				sprintf((char *)cs.bufp,"{\"gc\":\"%s\"}\n", (char *)cs.out_buf);	// '-8' is used for JSON chars
				json_parser(cs.bufp);
			} else {									//...or run it as text
				text_response(gc_gcode_parser(cs.bufp), cs.saved_buf);
			}
		}
	}
	return (STAT_OK);
}


/**** Local Utilities ********************************************************/
/*
 * _shutdown_idler() - 快速闪烁并阻止更多的操作发生
 * _normal_idler() -   缓慢闪烁指示LED指示一切都OK
 *
 *	Shutdown idler flashes indicator LED rapidly to show everything is not OK.
 *	Shutdown idler returns EAGAIN causing the control loop to never advance beyond
 *	this point. It's important that the reset handler is still called so a SW reset
 *	(ctrl-x) or bootloader request can be processed.
 */

static stat_t _shutdown_idler()
{
	if (cm_get_machine_state() != MACHINE_SHUTDOWN) { return (STAT_OK);}

	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + LED_ALARM_TIMER;
		IndicatorLed_toggle();
	}
	return (STAT_EAGAIN);	// EAGAIN 阻止其它更低优先级的任务被执行
}

static stat_t _normal_idler()
{
#ifdef __ARM
	/*
	 * S-curve heartbeat code. Uses forward-differencing math from the stepper code.
	 * See plan_line.cpp for explanations.
	 * Here, the "velocity" goes from 0.0 to 1.0, then back.
	 * t0 = 0, t1 = 0, t2 = 0.5, and we'll complete the S in 100 segments.
	 */

	// These are statics, and the assignments will only evaluate once.
	static float indicator_led_value = 0.0;
	static float indicator_led_forward_diff_1 = 50.0 * square(1.0/100.0);
	static float indicator_led_forward_diff_2 = indicator_led_forward_diff_1 * 2.0;


	if (SysTickTimer.getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer.getValue() + LED_NORMAL_TIMER / 100;

		indicator_led_value += indicator_led_forward_diff_1;
		if (indicator_led_value > 100.0)
			indicator_led_value = 100.0;

		if ((indicator_led_forward_diff_2 > 0.0 && indicator_led_value >= 50.0) || (indicator_led_forward_diff_2 < 0.0 && indicator_led_value <= 50.0)) {
			indicator_led_forward_diff_2 = -indicator_led_forward_diff_2;
		}
		else if (indicator_led_value <= 0.0) {
			indicator_led_value = 0.0;

			// Reset to account for rounding errors
			indicator_led_forward_diff_1 = 50.0 * square(1.0/100.0);
		} else {
			indicator_led_forward_diff_1 += indicator_led_forward_diff_2;
		}

		IndicatorLed = indicator_led_value/100.0;
	}
#endif
#ifdef __AVR
/*
	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + LED_NORMAL_TIMER;
//		IndicatorLed_toggle();
	}
*/
#endif
	return (STAT_OK);
}

/*
 * tg_reset_source() 		 - reset source to default input device (see note)
 * tg_set_primary_source() 	 - set current primary input source
 * tg_set_secondary_source() - set current primary input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 * be expanded to also set the stdout/stderr console device so the prompt
 * and other messages are sent to the active device.
 */

void tg_reset_source() { tg_set_primary_source(cs.default_src);}
void tg_set_primary_source(uint8_t dev) { cs.primary_src = dev;}
void tg_set_secondary_source(uint8_t dev) { cs.secondary_src = dev;}

/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 * _sync_to_time() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer()
{
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

static stat_t _sync_to_planner()
{
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { // allow up to N planner buffers for this line
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

/*
static stat_t _sync_to_time()
{
	if (cs.sync_to_time_time == 0) {		// initial pass
		cs.sync_to_time_time = SysTickTimer_getValue() + 100; //ms
		return (STAT_OK);
	}
	if (SysTickTimer_getValue() < cs.sync_to_time_time) {
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}
*/
/*
 * _limit_switch_handler() - 当限位开关触发的时候，关闭系统 
 */
static stat_t _limit_switch_handler(void)
{
	if (cm_get_machine_state() == MACHINE_ALARM) { return (STAT_NOOP);}

	if (get_limit_switch_thrown() == false) return (STAT_NOOP);
	return(cm_hard_alarm(STAT_LIMIT_SWITCH_HIT));
	return (STAT_OK);
}

/*
 * _system_assertions() - 测试内存完整性和其他校验
 */
#define emergency___everybody_to_get_from_street(a) if((status_code=a) != STAT_OK) return (cm_hard_alarm(status_code));

stat_t _system_assertions()
{
	emergency___everybody_to_get_from_street(config_test_assertions());
	emergency___everybody_to_get_from_street(controller_test_assertions());
	emergency___everybody_to_get_from_street(canonical_machine_test_assertions());
	emergency___everybody_to_get_from_street(planner_test_assertions());
	emergency___everybody_to_get_from_street(stepper_test_assertions());
	emergency___everybody_to_get_from_street(encoder_test_assertions());
	emergency___everybody_to_get_from_street(xio_test_assertions());
	return (STAT_OK);
}
