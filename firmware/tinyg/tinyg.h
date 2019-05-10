/*
 * tinyg.h - tinyg main header
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* Is this code over documented? Possibly.
 * We try to follow this (at least we are evolving to it). It's worth a read.
 * ftp://ftp.idsoftware.com/idstuff/doom3/source/CodeStyleConventions.doc
 */
/* Xmega project setup notes:
 * from: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=117023
 * "Yes it's definitely worth making WinAVR work. To install WinAVR for the project use
 * Project-Configuration Options and under Custom Options untick the "Use toolchain" box
 * then set the top one to \winavr\bin\avr-gcc.exe  (C:\WinAVR-20100110\bin\avr-gcc.exe)
 * and the lower one to \winavr\utils\bin\make.exe  (C:\WinAVR-20100110\utils\bin\make.exe)"
 */

#ifndef TINYG_H_ONCE
#define TINYG_H_ONCE

// 通用系统include 
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

//#include "MotatePins.h"

/****** 修订 ******/

#ifndef TINYG_FIRMWARE_BUILD
#define TINYG_FIRMWARE_BUILD        440.20	// arc test

#endif
#define TINYG_FIRMWARE_VERSION		0.97					// 主固件版本 
#define TINYG_HARDWARE_PLATFORM		HW_PLATFORM_TINYG_XMEGA	// 查看 hardware.h
#define TINYG_HARDWARE_VERSION		HW_VERSION_TINYGV8		// 查看 hardware.h
#define TINYG_HARDWARE_VERSION_MAX	TINYG_HARDWARE_VERSION

/****** 编译时设置 ******/

#define __STEP_CORRECTION
//#define __NEW_SWITCHES					// 使用v9版本的switch 代码
//#define __JERK_EXEC						// Use computed jerk (versus forward difference based exec)
//#define __KAHAN							// Use Kahan summation in aline exec functions

#define __TEXT_MODE							// 使能 text 模式	(~10Kb)
#define __HELP_SCREENS						// 使能 帮助 (~3.5Kb)
#define __CANNED_TESTS 						// 使能 $tests 		(~12Kb)
#define __TEST_99 							// 使能诊断测试99（独立于其他测试）

/****** 开发设置 ******/

#define __DIAGNOSTIC_PARAMETERS				// 使能系统诊断参数，位于config_app中的(_xx) 
//#define __DEBUG_SETTINGS					// 特殊测试，详情在settings.h
//#define __CANNED_STARTUP					// run any canned startup moves

//#ifndef WEAK
//#define WEAK  __attribute__ ((weak))
//#endif

/************************************************************************************
 ****************** 平台兼容性 *******************************************************
 ************************************************************************************/
#undef __AVR
#define __AVR
//#undef __ARM
//#define __ARM

/*********************
 * AVR兼容 *
 *********************/
#ifdef __AVR

#include <avr/pgmspace.h>		// 定义 PROGMEM 和 PSTR

typedef char char_t;			// ARM/C++ 版本使用uint8_t作为char_t

																	// 获取功能的宏都依赖于预先设置好的nv->index
#define GET_TABLE_WORD(a)  pgm_read_word(&cfgArray[nv->index].a)	// 从 cfgArray 中获取一个字(word)
#define GET_TABLE_BYTE(a)  pgm_read_byte(&cfgArray[nv->index].a)	// 从 cfgArray 中获取一个字节
#define GET_TABLE_FLOAT(a) pgm_read_float(&cfgArray[nv->index].a)	// 从 cfgArray 中获取一个浮点数
#define GET_TOKEN_BYTE(a)  (char_t)pgm_read_byte(&cfgArray[i].a)	// 从 cfgArray 中获取一个token 字节

// populate the shared buffer with the token string given the index
#define GET_TOKEN_STRING(i,a) strcpy_P(a, (char *)&cfgArray[(index_t)i].token);

// 从PGM中的字符串数组获取文本并转化成内存(RAM)字符串
#define GET_TEXT_ITEM(b,a) strncpy_P(global_string_buf,(const char *)pgm_read_word(&b[a]), MESSAGE_LEN-1)

// 从PGM中的字符串数组获取单位并转化到内存中去
#define GET_UNITS(a) strncpy_P(global_string_buf,(const char *)pgm_read_word(&msg_units[cm_get_units_mode(a)]), MESSAGE_LEN-1)

// IO 设置 
#define STD_IN 	XIO_DEV_USB		// 默认IO设置 
#define STD_OUT	XIO_DEV_USB
#define STD_ERR	XIO_DEV_USB

// 字符串兼容 
#define strtof strtod			// strtof 没有在 AVR lib 中

#endif // __AVR

/*********************
 * ARM 兼容 *
 *********************/
#ifdef __ARM
								// Use macros to fake out AVR's PROGMEM and other AVRisms.
#define PROGMEM					// ignore PROGMEM declarations in ARM/GCC++
#define PSTR (const char *)		// AVR macro is: PSTR(s) ((const PROGMEM char *)(s))

typedef char char_t;			// In the ARM/GCC++ version char_t is typedef'd to uint8_t
								// because in C++ uint8_t and char are distinct types and
								// we want chars to behave as uint8's

													// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  cfgArray[nv->index].a	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TOKEN_BYTE(i,a) (char_t)cfgArray[i].a	// get token byte value from cfgArray

#define GET_TOKEN_STRING(i,a) cfgArray[(index_t)i].a
//#define GET_TOKEN_STRING(i,a) (char_t)cfgArray[i].token)// populate the token string given the index

#define GET_TEXT_ITEM(b,a) b[a]						// get text from an array of strings in flash
#define GET_UNITS(a) msg_units[cm_get_units_mode(a)]

// IO settings
#define DEV_STDIN 0				// STDIO defaults - stdio is not yet used in the ARM version
#define DEV_STDOUT 0
#define DEV_STDERR 0

/* String compatibility
 *
 * The ARM stdio functions we are using still use char as input and output. The macros
 * below do the casts for most cases, but not all. Vararg functions like the printf()
 * family need special handling. These like char * as input and require casts as per:
 *
 *   printf((const char *)"Good Morning Hoboken!\n");
 *
 * The AVR also has "_P" variants that take PROGMEM strings as args.
 * On the ARM/GCC++ the _P functions are just aliases of the non-P variants.
 */
#define strncpy(d,s,l) (char_t *)strncpy((char *)d, (char *)s, l)
#define strpbrk(d,s) (char_t *)strpbrk((char *)d, (char *)s)
#define strcpy(d,s) (char_t *)strcpy((char *)d, (char *)s)
#define strcat(d,s) (char_t *)strcat((char *)d, (char *)s)
#define strstr(d,s) (char_t *)strstr((char *)d, (char *)s)
#define strchr(d,s) (char_t *)strchr((char *)d, (char)s)
#define strcmp(d,s) strcmp((char *)d, (char *)s)
#define strtod(d,p) strtod((char *)d, (char **)p)
#define strtof(d,p) strtof((char *)d, (char **)p)
#define strlen(s) strlen((char *)s)
#define isdigit(c) isdigit((char)c)
#define isalnum(c) isalnum((char)c)
#define tolower(c) (char_t)tolower((char)c)
#define toupper(c) (char_t)toupper((char)c)

#define printf_P printf		// these functions want char * as inputs, not char_t *
#define fprintf_P fprintf	// just sayin'
#define sprintf_P sprintf
#define strcpy_P strcpy

#endif // __ARM

/******************************************************************************
 ***** TINYG 应用定义 ******************************************
 ******************************************************************************/

typedef uint16_t magic_t;		// “魔法数”的大小 
#define MAGICNUM 0x12EF			// 用于内存完整性assertions校验

/***** 轴，电机和PWM通道 *****/
// 轴，电机和PWM通道必须被定义（不是枚举）所以#ifdef <值> 可以被使用
#define AXES		6			// 在这个版本中支持多少个轴
#define HOMING_AXES	4			// 多少个轴可以被归位（假设Zxyabc顺序）
#define MOTORS		4			// 板上有多少个电机
#define COORDS		6			// 支持多少个坐标系统(1-6)
#define PWMS		2			// 支持多少个PWMM通道

// 注意： 如果你改变了 COORDS 你必须调整config.c中的cfgArray表格中的入口。

#define AXIS_X		0
#define AXIS_Y		1
#define AXIS_Z		2
#define AXIS_A		3
#define AXIS_B		4
#define AXIS_C		5
#define AXIS_U		6			// 保留 
#define AXIS_V		7			// 保留
#define AXIS_W		8			// 保留

#define MOTOR_1		0 			// 定义电机号码和数组索引
#define MOTOR_2		1			// 必须被定义。枚举不起作用
#define MOTOR_3		2
#define MOTOR_4		3
#define MOTOR_5		4
#define MOTOR_6		5

#define PWM_1		0
#define PWM_2		1

/************************************************************************************
 * 状态码 
 *
 * 第一个代码范围(0-19)是和XIO codes对齐的，且必须要这样做。
 * 请不要在没有检查xio.h中的相关值的情况下改动它们
 *
 * 状态码被分成几个范围，为了更好的分类和扩展。在某些情况下，这可能会导致更加混乱，但是一旦
 * 这些已存在的状态码已经处于分发出去的状态的时候，明智的做法是不要去改变这些值。
 *
 * 范围是:
 *
 *	 0 - 19		OS,通讯和底层状态（必须和xio.h中的XIO_xxx 码对齐）
 *
 *  20 - 99		普通内部错误和应用程序错误。内部错误从20开始，且
 * 				Generic internal and application errors. Internal errors start at 20 and work up,
 *				Assertion failures start at 99 and work down.
 *
 * 100 - 129	普通数据和输入错误 - 未指定Generic data and input errors - not specific to Gcode or TinyG
 *
 * 130 -		G代码和TinyG 应用程序错误和警告
 *
 * 可以在main.c中查看关联的消息字符串。有关状态码的改变都需要同时修改main.c中字符串数组中的消息字符串。
 *
 * 大多数下方的的状态码（除了STAT_OK)都是会导致命令失败的错误，且都会由失败命令返回并通过JSON格式或者
 * text格式输出报告。
 * 一些状态码是警告且不会导致命令失败。这些可以用来生成异常报告。它们都用WARNING做为标签
 */

typedef uint8_t stat_t;
extern stat_t status_code;				// 在 main.c 中分配了

#define MESSAGE_LEN 80					// 全局信息字符串存储
extern char global_string_buf[];				// 在main.c中分配了

char *get_status_message(stat_t status);

// ritorno是一个非常方便提供异常返回的方法
// 它只在错误发生的时候才返回(ritorno是意大利文的return的意思)
#define ritorno(a) if((status_code=a) != STAT_OK) { return(status_code); }

// OS, 通讯和底层状态（必须和xio.h中的XIO_xxxx(XioCode枚举)代码对齐)
#define	STAT_OK 0						// 函数正常执行完成 
#define	STAT_ERROR 1					// 通用错误返回(EPERM)
#define	STAT_EAGAIN 2					// 函数将堵塞在这里(再次调用)
#define	STAT_NOOP 3						// 函数没有进行操作
#define	STAT_COMPLETE 4					// 操作完成
#define STAT_TERMINATE 5				// 操作终止(gracefully)
#define STAT_RESET 6					// operation was hard reset (sig kill)
#define	STAT_EOL 7						// 函数返回行结束  end-of-line
#define	STAT_EOF 8						// 函数返回文件结束end-of-file
#define	STAT_FILE_NOT_OPEN 9
#define	STAT_FILE_SIZE_EXCEEDED 10
#define	STAT_NO_SUCH_DEVICE 11
#define	STAT_BUFFER_EMPTY 12
#define	STAT_BUFFER_FULL 13
#define	STAT_BUFFER_FULL_FATAL 14
#define	STAT_INITIALIZING 15			// 初始化中，还未准备好可以使用 
#define	STAT_ENTERING_BOOT_LOADER 16	// 这个码实际上是从boot loader中发出，而不是TinyG
#define	STAT_FUNCTION_IS_STUBBED 17
#define	STAT_ERROR_18 18
#define	STAT_ERROR_19 19				// 注意：XIO 码对齐到这里

// 内部错误和启动信息
#define	STAT_INTERNAL_ERROR 20			// 不可恢复内部错误
#define	STAT_INTERNAL_RANGE_ERROR 21	// 用户输入数值范围不符
#define	STAT_FLOATING_POINT_ERROR 22	// 数值转化错误
#define	STAT_DIVIDE_BY_ZERO 23
#define	STAT_INVALID_ADDRESS 24
#define	STAT_READ_ONLY_ADDRESS 25
#define	STAT_INIT_FAIL 26
#define	STAT_ALARMED 27
#define	STAT_FAILED_TO_GET_PLANNER_BUFFER 28
#define STAT_GENERIC_EXCEPTION_REPORT 29	// 用于测试 

#define	STAT_PREP_LINE_MOVE_TIME_IS_INFINITE 30
#define	STAT_PREP_LINE_MOVE_TIME_IS_NAN 31
#define	STAT_FLOAT_IS_INFINITE 32
#define	STAT_FLOAT_IS_NAN 33
#define	STAT_PERSISTENCE_ERROR 34
#define	STAT_BAD_STATUS_REPORT_SETTING 35
#define	STAT_ERROR_36 36
#define	STAT_ERROR_37 37
#define	STAT_ERROR_38 38
#define	STAT_ERROR_39 39

#define	STAT_ERROR_40 40
#define	STAT_ERROR_41 41
#define	STAT_ERROR_42 42
#define	STAT_ERROR_43 43
#define	STAT_ERROR_44 44
#define	STAT_ERROR_45 45
#define	STAT_ERROR_46 46
#define	STAT_ERROR_47 47
#define	STAT_ERROR_48 48
#define	STAT_ERROR_49 49

#define	STAT_ERROR_50 50
#define	STAT_ERROR_51 51
#define	STAT_ERROR_52 52
#define	STAT_ERROR_53 53
#define	STAT_ERROR_54 54
#define	STAT_ERROR_55 55
#define	STAT_ERROR_56 56
#define	STAT_ERROR_57 57
#define	STAT_ERROR_58 58
#define	STAT_ERROR_59 59

#define	STAT_ERROR_60 60
#define	STAT_ERROR_61 61
#define	STAT_ERROR_62 62
#define	STAT_ERROR_63 63
#define	STAT_ERROR_64 64
#define	STAT_ERROR_65 65
#define	STAT_ERROR_66 66
#define	STAT_ERROR_67 67
#define	STAT_ERROR_68 68
#define	STAT_ERROR_69 69

#define	STAT_ERROR_70 70
#define	STAT_ERROR_71 71
#define	STAT_ERROR_72 72
#define	STAT_ERROR_73 73
#define	STAT_ERROR_74 74
#define	STAT_ERROR_75 75
#define	STAT_ERROR_76 76
#define	STAT_ERROR_77 77
#define	STAT_ERROR_78 78
#define	STAT_ERROR_79 79

#define	STAT_ERROR_80 80
#define	STAT_ERROR_81 81
#define	STAT_ERROR_82 82
#define	STAT_ERROR_83 83
#define	STAT_ERROR_84 84
#define	STAT_ERROR_85 85
#define	STAT_ERROR_86 86
#define	STAT_ERROR_87 87
#define	STAT_ERROR_88 88
#define	STAT_ERROR_89 89

// Assertion failures - build down from 99 until they meet the system internal errors
// 校验失败 - 

#define	STAT_CONFIG_ASSERTION_FAILURE 90
#define	STAT_XIO_ASSERTION_FAILURE 91
#define	STAT_ENCODER_ASSERTION_FAILURE 92
#define	STAT_STEPPER_ASSERTION_FAILURE 93
#define	STAT_PLANNER_ASSERTION_FAILURE 94
#define	STAT_CANONICAL_MACHINE_ASSERTION_FAILURE 95
#define	STAT_CONTROLLER_ASSERTION_FAILURE 96
#define	STAT_STACK_OVERFLOW 97
#define	STAT_MEMORY_FAULT 98					// 被“魔法数”捕获的通用内存错误
#define	STAT_GENERIC_ASSERTION_FAILURE 99		// 通用校验失败-unclassified

// 程序和数据输入错误 

// 通用数据输入错误
#define	STAT_UNRECOGNIZED_NAME 100              // 解析器无法识别名字
#define	STAT_INVALID_OR_MALFORMED_COMMAND 101   // 解析器捕获无效或未识别的行
#define	STAT_BAD_NUMBER_FORMAT 102              // 数字格式错误
#define	STAT_UNSUPPORTED_TYPE 103               // An otherwise valid number or JSON type is not supported
#define	STAT_PARAMETER_IS_READ_ONLY 104         // 输入错误：参数不能被设置
#define	STAT_PARAMETER_CANNOT_BE_READ 105       // 输入错误：参数不能被设置
#define	STAT_COMMAND_NOT_ACCEPTED 106			// 命令在此时不能够被接受
#define	STAT_INPUT_EXCEEDS_MAX_LENGTH 107       // 输入字符串过长
#define	STAT_INPUT_LESS_THAN_MIN_VALUE 108      // 输入错误：数值低于最小值
#define	STAT_INPUT_EXCEEDS_MAX_VALUE 109        // 输入错误：数值大于最大值

#define	STAT_INPUT_VALUE_RANGE_ERROR 110        // 输入错误：数值超过范围
#define	STAT_JSON_SYNTAX_ERROR 111              // JSON输入字符串没有被完全格式化 
#define	STAT_JSON_TOO_MANY_PAIRS 112            // JSON输入字符串有太多JSON对
#define	STAT_JSON_TOO_LONG 113					// JSON输入或者输出超过缓冲大小
#define	STAT_ERROR_114 114
#define	STAT_ERROR_115 115
#define	STAT_ERROR_116 116
#define	STAT_ERROR_117 117
#define	STAT_ERROR_118 118
#define	STAT_ERROR_119 119

#define	STAT_ERROR_120 120
#define	STAT_ERROR_121 121
#define	STAT_ERROR_122 122
#define	STAT_ERROR_123 123
#define	STAT_ERROR_124 124
#define	STAT_ERROR_125 125
#define	STAT_ERROR_126 126
#define	STAT_ERROR_127 127
#define	STAT_ERROR_128 128
#define	STAT_ERROR_129 129

// G代码错误和警告(大多数都是源于NIST - 指的是它的概念不是这里的号码数字)
// Fascinating: http://www.cncalarms.com/

#define	STAT_GCODE_GENERIC_INPUT_ERROR 130				// 通用G代码输入错误
#define	STAT_GCODE_COMMAND_UNSUPPORTED 131				// G代码命令不支持
#define	STAT_MCODE_COMMAND_UNSUPPORTED 132				// M代码命令不支持
#define	STAT_GCODE_MODAL_GROUP_VIOLATION 133			// G代码模型组错误
#define	STAT_GCODE_AXIS_IS_MISSING 134					// 命令需要至少有一个轴作为参数
#define STAT_GCODE_AXIS_CANNOT_BE_PRESENT 135			// 如果G80带有轴作为参数，则出错
#define STAT_GCODE_AXIS_IS_INVALID 136					// 为命令指定了一个不合法的轴
#define STAT_GCODE_AXIS_IS_NOT_CONFIGURED 137			// 警告：尝试去编程一个被关闭(disable)的轴
#define STAT_GCODE_AXIS_NUMBER_IS_MISSING 138			// 轴缺少数值
#define STAT_GCODE_AXIS_NUMBER_IS_INVALID 139	 		// 轴的数值不合法

#define STAT_GCODE_ACTIVE_PLANE_IS_MISSING 140			// 生效平面还未被指定
#define STAT_GCODE_ACTIVE_PLANE_IS_INVALID 141			// 生效平面对于当前命令是无效的
#define	STAT_GCODE_FEEDRATE_NOT_SPECIFIED 142			// 移动没有指定进给
#define STAT_GCODE_INVERSE_TIME_MODE_CANNOT_BE_USED 143	// G38.2 和其他一些 canned cycles不能接受inverse time模式
#define STAT_GCODE_ROTARY_AXIS_CANNOT_BE_USED 144		// G38.2 和其他的一些命令不能有旋转轴
#define STAT_GCODE_G53_WITHOUT_G0_OR_G1 145				// G0 或者 G1必须在G53下生效
#define STAT_REQUESTED_VELOCITY_EXCEEDS_LIMITS 146
#define STAT_CUTTER_COMPENSATION_CANNOT_BE_ENABLED 147
#define STAT_PROGRAMMED_POINT_SAME_AS_CURRENT_POINT 148
#define	STAT_SPINDLE_SPEED_BELOW_MINIMUM 149

#define	STAT_SPINDLE_SPEED_MAX_EXCEEDED 150
#define	STAT_S_WORD_IS_MISSING 151
#define	STAT_S_WORD_IS_INVALID 152
#define	STAT_SPINDLE_MUST_BE_OFF 153
#define	STAT_SPINDLE_MUST_BE_TURNING 154				// 一些固定循环在调用时需要主轴被开启
#define	STAT_ARC_SPECIFICATION_ERROR 155				// 通用弧线参数错误
#define STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE 156	// 当前选择平面，圆弧缺少对应的轴
#define STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE 157 // 一个或者多个偏移没有被指定
#define STAT_ARC_RADIUS_OUT_OF_TOLERANCE 158			// 警告 - 圆弧半径过小或者过大 - 精度存在问题
#define STAT_ARC_ENDPOINT_IS_STARTING_POINT 159

#define STAT_P_WORD_IS_MISSING 160						// 在dwell或者其他功能中P值必须指定
#define STAT_P_WORD_IS_INVALID 161						// 通用P值错误
#define STAT_P_WORD_IS_ZERO 162
#define STAT_P_WORD_IS_NEGATIVE 163						// dewlls 请求正数的P值
#define STAT_P_WORD_IS_NOT_AN_INTEGER 164				// G10s 和其他命令需要整数的P值
#define STAT_P_WORD_IS_NOT_VALID_TOOL_NUMBER 165
#define STAT_D_WORD_IS_MISSING 166
#define STAT_D_WORD_IS_INVALID 167
#define STAT_E_WORD_IS_MISSING 168
#define STAT_E_WORD_IS_INVALID 169

#define STAT_H_WORD_IS_MISSING 170
#define STAT_H_WORD_IS_INVALID 171
#define STAT_L_WORD_IS_MISSING 172
#define STAT_L_WORD_IS_INVALID 173
#define STAT_Q_WORD_IS_MISSING 174
#define STAT_Q_WORD_IS_INVALID 175
#define STAT_R_WORD_IS_MISSING 176
#define STAT_R_WORD_IS_INVALID 177
#define STAT_T_WORD_IS_MISSING 178
#define STAT_T_WORD_IS_INVALID 179

#define	STAT_ERROR_180 180									// 为G代码错误保留 
#define	STAT_ERROR_181 181
#define	STAT_ERROR_182 182
#define	STAT_ERROR_183 183
#define	STAT_ERROR_184 184
#define	STAT_ERROR_185 185
#define	STAT_ERROR_186 186
#define	STAT_ERROR_187 187
#define	STAT_ERROR_188 188
#define	STAT_ERROR_189 189

#define	STAT_ERROR_190 190
#define	STAT_ERROR_191 191
#define	STAT_ERROR_192 192
#define	STAT_ERROR_193 193
#define	STAT_ERROR_194 194
#define	STAT_ERROR_195 195
#define	STAT_ERROR_196 196
#define	STAT_ERROR_197 197
#define	STAT_ERROR_198 198
#define	STAT_ERROR_199 199

// TinyG 错误和警告 

#define STAT_GENERIC_ERROR 200
#define	STAT_MINIMUM_LENGTH_MOVE 201					// 移动小于最小长度 
#define	STAT_MINIMUM_TIME_MOVE 202						// 移动小于最小时间
#define	STAT_MACHINE_ALARMED 203						// 机器处于警报状态。命令没有被执行
#define	STAT_LIMIT_SWITCH_HIT 204						// 限位开被处罚导致停止
#define	STAT_PLANNER_FAILED_TO_CONVERGE 205				// trapezoid generator can through this exception
#define	STAT_ERROR_206 206
#define	STAT_ERROR_207 207
#define	STAT_ERROR_208 208
#define	STAT_ERROR_209 209

#define	STAT_ERROR_210 210
#define	STAT_ERROR_211 211
#define	STAT_ERROR_212 212
#define	STAT_ERROR_213 213
#define	STAT_ERROR_214 214
#define	STAT_ERROR_215 215
#define	STAT_ERROR_216 216
#define	STAT_ERROR_217 217
#define	STAT_ERROR_218 218
#define	STAT_ERROR_219 219

#define	STAT_SOFT_LIMIT_EXCEEDED 220					// 软限位错误 - 轴未指定
#define	STAT_SOFT_LIMIT_EXCEEDED_XMIN 221				// 软限位错误 - X最小
#define	STAT_SOFT_LIMIT_EXCEEDED_XMAX 222				// 软限位错误 - X最大
#define	STAT_SOFT_LIMIT_EXCEEDED_YMIN 223				// 软限位错误 - Y最小
#define	STAT_SOFT_LIMIT_EXCEEDED_YMAX 224				// 软限位错误 - Y最大
#define	STAT_SOFT_LIMIT_EXCEEDED_ZMIN 225				// 软限位错误 - Z最小
#define	STAT_SOFT_LIMIT_EXCEEDED_ZMAX 226				// 软限位错误 - Z最大
#define	STAT_SOFT_LIMIT_EXCEEDED_AMIN 227				// 软限位错误 - A最小
#define	STAT_SOFT_LIMIT_EXCEEDED_AMAX 228				// 软限位错误 - A最大
#define	STAT_SOFT_LIMIT_EXCEEDED_BMIN 229				// 软限位错误 - B最小

#define	STAT_SOFT_LIMIT_EXCEEDED_BMAX 220				// 软限位错误 - B最大
#define	STAT_SOFT_LIMIT_EXCEEDED_CMIN 231				// 软限位错误 - C最小
#define	STAT_SOFT_LIMIT_EXCEEDED_CMAX 232				// 软限位错误 - C最大
#define	STAT_ERROR_233 233
#define	STAT_ERROR_234 234
#define	STAT_ERROR_235 235
#define	STAT_ERROR_236 236
#define	STAT_ERROR_237 237
#define	STAT_ERROR_238 238
#define	STAT_ERROR_239 239

#define	STAT_HOMING_CYCLE_FAILED 240					// 归位循环未完成
#define	STAT_HOMING_ERROR_BAD_OR_NO_AXIS 241
#define	STAT_HOMING_ERROR_ZERO_SEARCH_VELOCITY 242
#define	STAT_HOMING_ERROR_ZERO_LATCH_VELOCITY 243
#define	STAT_HOMING_ERROR_TRAVEL_MIN_MAX_IDENTICAL 244
#define	STAT_HOMING_ERROR_NEGATIVE_LATCH_BACKOFF 245
#define	STAT_HOMING_ERROR_SWITCH_MISCONFIGURATION 246
#define	STAT_ERROR_247 247
#define	STAT_ERROR_248 248
#define	STAT_ERROR_249 249

#define	STAT_PROBE_CYCLE_FAILED 250						// 对刀循环未完成 
#define STAT_PROBE_ENDPOINT_IS_STARTING_POINT 251
#define	STAT_JOGGING_CYCLE_FAILED 252					// 手轮引导未完成

// !!! 不要在没有同时修改stat_t 定义的时候超过255

#endif // End of include guard: TINYG2_H_ONCE
