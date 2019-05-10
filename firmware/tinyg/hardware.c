/*
 * hardware.c - general hardware support functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

#ifdef __AVR
#include <avr/wdt.h>			// used for software reset
#endif

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "hardware.h"
#include "switch.h"
#include "controller.h"
#include "text_parser.h"
#ifdef __AVR
#include "xmega/xmega_init.h"
#include "xmega/xmega_rtc.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif

/*
 * _port_bindings  - 端口绑定，绑定XMEGA端口到硬件 - these changed at board revision 7
 * hardware_init() - 最低层硬件初始化
 */

static void _port_bindings(float hw_version)
{
#ifdef __AVR
	hw.st_port[0] = &PORT_MOTOR_1;
	hw.st_port[1] = &PORT_MOTOR_2;
	hw.st_port[2] = &PORT_MOTOR_3;
	hw.st_port[3] = &PORT_MOTOR_4;

	hw.sw_port[0] = &PORT_SWITCH_X;
	hw.sw_port[1] = &PORT_SWITCH_Y;
	hw.sw_port[2] = &PORT_SWITCH_Z;
	hw.sw_port[3] = &PORT_SWITCH_A;

	if (hw_version > 6.9) {
		hw.out_port[0] = &PORT_OUT_V7_X;
		hw.out_port[1] = &PORT_OUT_V7_Y;
		hw.out_port[2] = &PORT_OUT_V7_Z;
		hw.out_port[3] = &PORT_OUT_V7_A;
		} else {
		hw.out_port[0] = &PORT_OUT_V6_X;
		hw.out_port[1] = &PORT_OUT_V6_Y;
		hw.out_port[2] = &PORT_OUT_V6_Z;
		hw.out_port[3] = &PORT_OUT_V6_A;
	}
#endif
}

void hardware_init()
{
#ifdef __AVR
	xmega_init();							// 设置系统时钟 
	_port_bindings(TINYG_HARDWARE_VERSION);
	rtc_init();								// 实时时钟计数器 
#endif
}

/*
 * _get_id() - 获取一个便于用户识别的签名
 *
 * 对AVR而已：
 * 唯一设备ID是基于工厂校准数据来生成的.
 * 		格式是：12345-ABC
 *
 *  数字部分是直接从6位批号码中读取出来的
 *  字母部分是wafer number和XY coords的低5位
 *	有关细节参考 iox192a3.h 中的NVM_PROD_SIGNATURES_t
 *
 * 对于ARM而言：
 *  当前还没有实现
 */

/* 未使用 
static uint8_t _read_calibration_byte(uint8_t index)
{
	NVM_CMD = NVM_NV_READ_CALIB_ROW_gc; 	// Load NVM Command register to read the calibration row
	uint8_t result = pgm_read_byte(index);
	NVM_CMD = NVM_NV_NO_OPERATION_gc; 	 	// Clean up NVM Command register
	return(result);
}
*/

enum {
	LOTNUM0=8,  // Lot Number Byte 0, ASCII
	LOTNUM1,    // Lot Number Byte 1, ASCII
	LOTNUM2,    // Lot Number Byte 2, ASCII
	LOTNUM3,    // Lot Number Byte 3, ASCII
	LOTNUM4,    // Lot Number Byte 4, ASCII
	LOTNUM5,    // Lot Number Byte 5, ASCII
	WAFNUM =16, // Wafer Number
	COORDX0=18, // Wafer Coordinate X Byte 0
	COORDX1,    // Wafer Coordinate X Byte 1
	COORDY0,    // Wafer Coordinate Y Byte 0
	COORDY1,    // Wafer Coordinate Y Byte 1
};

static void _get_id(char_t *id)
{
#ifdef __AVR
	char printable[33] = {"ABCDEFGHJKLMNPQRSTUVWXYZ23456789"};
	uint8_t i;

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 	// Load NVM Command register to read the calibration row

	for (i=0; i<6; i++) {
		id[i] = pgm_read_byte(LOTNUM0 + i);
	}
	id[i++] = '-';
	id[i++] = printable[(pgm_read_byte(WAFNUM) & 0x1F)];
	id[i++] = printable[(pgm_read_byte(COORDX0) & 0x1F)];
//	id[i++] = printable[(pgm_read_byte(COORDX1) & 0x1F)];
	id[i++] = printable[(pgm_read_byte(COORDY0) & 0x1F)];
//	id[i++] = printable[(pgm_read_byte(COORDY1) & 0x1F)];
	id[i] = 0;

	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 	 	// 清除 NVM  命令寄存器
#endif
}

/*
 * 硬件复位处理函数 
 *
 * hw_request_hard_reset()
 * hw_hard_reset()			- 使用看门狗定时器来进行硬复位
 * hw_hard_reset_handler()	- controller's rest handler
 */
void hw_request_hard_reset() { cs.hard_reset_requested = true; }

void hw_hard_reset(void)			// 软件硬复位，使用看门狗定时器
{
#ifdef __AVR
	wdt_enable(WDTO_15MS);
	while (true);					// 循环大概15ms，然后复位
#endif
}

stat_t hw_hard_reset_handler(void)
{
	if (cs.hard_reset_requested == false)
        return (STAT_NOOP);
	hw_hard_reset();				// 硬复位 - 按到复位按钮触发
	return (STAT_EAGAIN);
}

/*
 * Bootloader 处理函数 
 *
 * hw_request_bootloader()
 * hw_request_bootloader_handler() - 使用CCPWrite来完成软件复位
 */

void hw_request_bootloader() { cs.bootloader_requested = true;}

stat_t hw_bootloader_handler(void)
{
#ifdef __AVR
	if (cs.bootloader_requested == false)
        return (STAT_NOOP);
	cli();
	CCPWrite(&RST.CTRL, RST_SWRST_bm);  // 触发一个软件限位 
#endif
	return (STAT_EAGAIN);				// 永远不会到这里，但是让编译器保持happy，不提示warnning。
}

/*****系统函数结束 *****/


/***********************************************************************************
 * 配置和接口功能
 * 用于从cfgArray表格中获取和设置值得函数
 ***********************************************************************************/

/*
 * hw_get_id() - 获取设备id（签名） 
 */

stat_t hw_get_id(nvObj_t *nv)
{
	char_t tmp[SYS_ID_LEN];
	_get_id(tmp);
	nv->valuetype = TYPE_STRING;
	ritorno(nv_copy_string(nv, tmp));
	return (STAT_OK);
}

/*
 * hw_run_boot() - 从cfgArray中唤起软件复位(bootloader)
 */
stat_t hw_run_boot(nvObj_t *nv)
{
	hw_request_bootloader();
	return(STAT_OK);
}

/*
 * hw_set_hv() - 设置硬件版本号 
 */
stat_t hw_set_hv(nvObj_t *nv)
{
	if (nv->value > TINYG_HARDWARE_VERSION_MAX)
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
	set_flt(nv);					// 记录硬件版本 
	_port_bindings(nv->value);		// 复位端口绑定 
	switch_init();					// 重新初始化GPIO端口
//++++	gpio_init();				// 重新初始化GPIO端口 
	return (STAT_OK);
}

/***********************************************************************************
 * 文本模式支持 
 * 包含从cfgArray 表格中输出值的函数
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_fb[] PROGMEM = "[fb]  firmware build%18.2f\n";
static const char fmt_fv[] PROGMEM = "[fv]  firmware version%16.2f\n";
static const char fmt_hp[] PROGMEM = "[hp]  hardware platform%15.2f\n";
static const char fmt_hv[] PROGMEM = "[hv]  hardware version%16.2f\n";
static const char fmt_id[] PROGMEM = "[id]  TinyG ID%30s\n";

void hw_print_fb(nvObj_t *nv) { text_print_flt(nv, fmt_fb);}
void hw_print_fv(nvObj_t *nv) { text_print_flt(nv, fmt_fv);}
void hw_print_hp(nvObj_t *nv) { text_print_flt(nv, fmt_hp);}
void hw_print_hv(nvObj_t *nv) { text_print_flt(nv, fmt_hv);}
void hw_print_id(nvObj_t *nv) { text_print_str(nv, fmt_id);}

#endif //__TEXT_MODE

#ifdef __cplusplus
}
#endif
