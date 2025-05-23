//$file${.::motor.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: blinky.qm
// File:  ${.::motor.h}
//
// This code has been generated by QM 7.0.1 <www.state-machine.com/qm>.
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// Copyright (c) 2005 Quantum Leaps, LLC. All rights reserved.
//
//                 ____________________________________
//                /                                   /
//               /    GGGGGGG    PPPPPPPP   LL       /
//              /   GG     GG   PP     PP  LL       /
//             /   GG          PP     PP  LL       /
//            /   GG   GGGGG  PPPPPPPP   LL       /
//           /   GG      GG  PP         LL       /
//          /     GGGGGGG   PP         LLLLLLL  /
//         /___________________________________/
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This generated code is open-source software licensed under the GNU
// General Public License (GPL) as published by the Free Software Foundation
// (see <https://www.gnu.org/licenses>).
//
// NOTE:
// The GPL does NOT permit the incorporation of this code into proprietary
// programs. Please contact Quantum Leaps for commercial licensing options,
// which expressly supersede the GPL and are designed explicitly for
// closed-source distribution.
//
// Quantum Leaps contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//
//$endhead${.::motor.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef MOTOR_H
#define MOTOR_H
#include "rtdef.h"
#include <rtthread.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>

enum motor_type
{
    MOTOR_TYPE_LINE           = 0x00,  // 线性电机类型
    MOTOR_TYPE_CIRCLE         = 0x01,  // 环形电机类型
    MOTOR_TYPE_SPECIAL_CIRCLE = 0x02,  // 特殊环形电机类型
};
/*
 * 电机方向
 */
enum motor_direction
{
    DIRECTION_POSITIVE = 1,  // 正方向
    DIRECTION_NEGATIVE = 0,  // 反方向
};

/*
 * 电机电源模式
 */
enum motor_power_mode
{
    LOW_POWER_MODE    = 0,  // 低功耗模式
    NORMAL_POWER_MODE = 1,  // 正常模式
};

/*
 * 复位状态
 */
enum motor_reset_status
{
    MOTOR_NO_RESET_STATUS = 0,  // 没有复位
    MOTOR_RESET_STATUS    = 1,  // 已复位
};

/*
 * 电机停止模式
 */
enum motor_stop_mode
{
    MOTOR_INSTANT_STOP = 0,  // 立即停止
    MOTOR_DEC_TO_STOP  = 1,  // 减速停止
};

/*
 * 使能与不使能
 * 开启与关闭
 */
enum switch_value
{
    MOTOR_DISABLE = 0,  // 不使能
    MOTOR_ENABLE  = 1,  // 使能
    MOTOR_OPEN    = 2,  // 打开
    MOTOR_CLOSE   = 3,  // 关闭
};

/*
 * 电机事件标志
 */
enum motor_event_flag
{
    MOTOR_EVENT_COMPLETED_FLAG = (1 << 0x00),  // 电机运动完成标志
    MOTOR_EVENT_ERROR_FLAG     = (1 << 0x01),  // 电机运动错误标志
    MOTOR_EVENT_TRIGGER_FLAG   = (1 << 0x02),  // 电机运动触发标志
    MOTOR_EVENT_ARRIVED_FLAG   = (1 << 0x03),  // 电机抵达触发标志
};

/*
 * 电机硬件控制错误
 */
enum motor_error
{
    MOTOR_ERROR_TIMEOUT_FLAG = (1 << 0x00),  // FPGA硬件控制上报: 电机运动超时
    MOTOR_ERROR_ERROR1_FLAG  = (1 << 0x01),  // FPGA硬件控制上报: 电机错误1
    MOTOR_ERROR_ERROR2_FLAG  = (1 << 0x02),  // FPGA硬件控制上报: 电机错误2
};

/*
 * 电机错误码
 */
enum motor_error_code
{
    MOTOR_ERROR_ERR_NONE    = 0x00,  //
    MOTOR_ERROR_NO_MEMERY   = 0x01,  // 电机系统资源不足
    MOTOR_ERROR_TIMEOUT     = 0x02,  // 电机运动超时
    MOTOR_ERROR_NOT_RESET   = 0x03,  // 电机运动没有复位
    MOTOR_ERROR_RESET_FAIL  = 0x04,  // 电机运动复位失败
    MOTOR_ERROR_CRASH       = 0x05,  // 电机运动碰撞错误
    MOTOR_ERROR_EXCEL_LIMIT = 0x06,  // 电机运动范围超限
    MOTOR_ERROR_LOSE_STEP   = 0x07,  // 电机运动丢步
};

/*
 * 电机函数类型
 */
enum function_type
{
    FUNCTION_TRIGGER_IRQ = 0x01,  // 触发中断回调函数
    FUNCTION_ERROR_IRQ,           // 错误中断回调函数
    FUNCTION_COMPLETED_IRQ,       // 完整包中断回调函数
};

/*
 * 信号触发源
 */
enum signal_source
{
    NULL_SIGNAL_SOURCE   = 0x00U,  // 无触发源
    RESET_SIGNAL_SOURCE  = 0x02U,  // 复位信号源
    CRASH_SIGNAL_SOURCE  = 0x04U,  // 碰撞信号源
    ARRIVE_SIGNAL_SOURCE = 0x08U,  // 抵达型号源
    CODE_SIGNAL_SOURCE   = 0x10U,  // 码齿信号源
};

/*
 *  电机预定义速度档位
 */
enum
{
    MOTOR_GRADE_1 = 0x00,  // 该档位给复位运动
    MOTOR_GRADE_2,
    MOTOR_GRADE_3,
    MOTOR_GRADE_4,
    MOTOR_GRADE_5,
    MOTOR_GRADE_6,
    MOTOR_GRADE_7,
    MOTOR_GRADE_8,
    MOTOR_GRADE_9,
    MOTOR_GRADE_10,
    MOTOR_GRADE_11,
    MOTOR_GRADE_12,
    MOTOR_GRADE_13,
    MOTOR_GRADE_14,
    MOTOR_GRADE_15,
    MOTOR_GRADE_16,
    MOTOR_GRADE_MAX_NUM = 0x10,
};

/*
 * 电机速度结构体
 */
typedef struct motor_speed
{
    uint32_t start_frequency;  // 起始频率值
    uint32_t max_frequency;    // 匀速频率值
    uint32_t end_frequency;    // 结束频率值
    uint32_t step_t[7];        // 第一阶步数
} motor_speed_t;

/*
 * 电机应用层配置参数(初始化)
 */
typedef struct motor_configure
{
    uint8_t motor_id;
    const char *name;    // 电机名称
    uint8_t motor_type;  // 电机类型

    TIM_TypeDef *timerType;  // 定时类型
    uint32_t Channel;        // 定时器通道号

    uint32_t pwm_pin;     // 脉冲输出引脚
    GPIO_TypeDef *GPIOx;  // 脉冲所在的引脚位置
    uint32_t dir_pin;    // 方向输出引脚
    uint32_t lpw_pin;    // 低功耗控制引脚

    bool direction_level;  // 正方向的有效电平
    bool low_power_level;  // 低功耗的有效电平

    uint32_t reset_pin;   // 复位检测引脚
    uint32_t crash_pin;   // 碰撞检测引脚
    uint32_t arrive_pin;  // 抵达触发引脚
    uint32_t code_pin;    // 码盘检测引脚

    uint8_t reset_signal_level;   // 复位光耦检测到的有效电平
    uint8_t crash_signal_level;   // 碰撞光耦检测到的有效电平
    uint8_t arrive_signal_level;  // 抵达光耦检测到的有效电平
    uint8_t code_signal_level;    // 码齿光耦检测到的有效电平
    // 要求触发时的值为1

    int32_t min_position;  // 最小位置
    int32_t max_position;  // 最大位置

    int32_t steps_reset_signal_posi;  // 复位触发后运动的步数(正方向找光耦)
    int32_t steps_reset_signal_nega;  // 复位触发后运动的步数(反方向离开光耦)
    int32_t steps_code_signal;        // 液面触发后运动的步数
    int32_t steps_crash_signal;       // 碰撞触发后运动的步数
    int32_t steps_arrive_signal;      // 抵达触发后运动的步数

    //   uint8_t bubble_signal_level; // 气泡传感器检测的有效电平
    motor_speed_t speed_para[MOTOR_GRADE_MAX_NUM];  // 电机速度档位
} rt_motor_config_t;

/* 电机状态 */
typedef enum
{
    MOTOR_STATE_INACTIVE = 0x00,                  //空闲
    MOTOR_STATE_ACCEL    = 0x01,                  //加速
    MOTOR_STATE_RUN      = 0x02,                  //匀速
    MOTOR_STATE_DECEL    = 0x03,                  //减速
    MOTOR_STATE_STOP     = MOTOR_STATE_INACTIVE,  //停止
} motorState_t;

typedef struct speed_t
{
    int32_t accMax;  // 加加速度最大值
    int32_t decMax;  // 减加速度最大值
    int32_t acc1;    // 加加速加速度的斜率(T1)
    int32_t acc2;    // 加减速加速度的斜率(T3)
    int32_t dec1;    // 减加速加速度的斜率(T5)
    int32_t dec2;    // 减减速加速度的斜率(T7)
    int32_t stepAcc;  // 加速的步数
    int32_t stepDec;  // 减速的步数
};

#define ACCTIMARR 1000
#define DECTIMARR 1000

//$declare${AOs::Motor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::Motor} ..............................................................
typedef struct Motor {
// protected:
    QActive super;

// public:
    QTimeEvt timeEvt;
    rt_motor_config_t* config;

    //定时器句柄
    TIM_HandleTypeDef timerHandle;

    // 中断计数值
    uint32_t timerCount;

    // 运动状态
    motorState_t motionState;

    // 相对位置
    int32_t relativePosition;

    // 当前的位置,相对复位原点的位置
    int32_t currentPosition;

    // 当前频率
    uint32_t currentFreq;

    // 步数加载索引
    uint32_t timeLoadIndex;

    // 定时器加载值
    uint32_t timeLoadValue;

    // 触发检测源
    uint32_t triggle_source;

    // T1阶段步数(包含前阶段)
    uint32_t stepT[7];

    // 当前的运动方向
    uint8_t direction;

    // 当前错误码
    uint8_t error_code;

    // 复位停止方式（减速|立即）停止
    uint8_t reset_signal_stop_way;

    // 碰撞停止方式（减速|立即）停止
    uint8_t crash_signal_stop_way;

    // 到达停止方式（减速|立即）停止
    uint8_t arrive_signal_stop_way;

    // 液面检测停止方式（减速|立即）停止
    uint8_t liquid_signal_stop_way;
    motor_speed_t current_speed;

    // 加速度的加速计数值
    uint16_t accTimArr[ACCTIMARR];
    uint16_t decTimArr[DECTIMARR];

    // 电机复位状态
    uint8_t reset_status;

    // 电机起始触发状态
    uint8_t start_trigger_status;

    // 记录触发状态（值是触发源）
    uint16_t trigger_status;

    // 单次运动绝对步数
    uint32_t absolute_steps;

    // 触发后要走的减速步数
    uint32_t trigger_after_move_steps;
} Motor;

extern Motor Motor_inst[MOTOR_MAX_NUM];

// public:
uint32_t Motor_Init(Motor * const me,
    rt_motor_config_t * config);

// 电机复位
uint32_t Motor_Reset(Motor * const me,
    uint32_t grade,
    uint32_t timeout);

/******************************************************
* @decription:
*      电机移动目标步数
* @para:
*     @grade：    速度档位
*     @target_position： 目标步数
*     @target_dir： 目标方向
*     @timeout: 超时时间

*******************************************************/
uint32_t Motor_Move(Motor * const me,
    uint32_t grade,
    int32_t target_steps,
    uint32_t timeout);

// protected:
QState Motor_initial(Motor * const me, void const * const par);
QState Motor_IDLE(Motor * const me, QEvt const * const e);
QState Motor_RUN(Motor * const me, QEvt const * const e);
//$enddecl${AOs::Motor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// 应用层直接调用接口
uint32_t rt_motor_init(Motor *motor, rt_motor_config_t *config);
uint32_t rt_motor_reset(Motor *motor, uint32_t mtGrade, uint32_t timeout);
uint32_t rt_motor_goto(Motor *motor, int32_t target_position, uint32_t grade, uint32_t timeout);
uint32_t rt_motor_move(Motor *motor, uint32_t grade, int32_t target_steps, uint32_t timeout);
uint32_t Motorrigger_move(Motor *motor, uint16_t trigger_source, uint32_t grade, int32_t trigger_steps, uint32_t step2, uint32_t timeout);
uint32_t rt_motor_set_ad_collect_enable(Motor *motor, uint8_t enable_flag, uint32_t dynamic_steps);
uint32_t rt_motor_wait(Motor *motor, uint32_t timeout);
uint32_t rt_motor_start(Motor *motor);
uint32_t rt_motor_instant_stop(Motor *motor);
uint32_t rt_motor_dec_stop(Motor *motor, uint32_t target_steps);
uint32_t rt_motor_set_direction(Motor *motor, uint8_t direction);
void rt_motor_set_low_power(Motor *motor, uint8_t mode);
uint32_t rt_motor_set_error_code(Motor *motor, uint32_t error_code);
uint32_t rt_motor_get_error_code(Motor *motor, uint32_t *error_code);
uint32_t rt_motor_set_current_position(Motor *motor, int32_t position);
int32_t rt_motor_get_current_position(Motor *motor);
void rt_motor_set_reset_status(Motor *motor, uint8_t reset_status);
uint32_t rt_motor_get_reset_status(Motor *motor);
uint32_t rt_motor_get_absolute_steps(Motor *motor);
uint32_t rt_motor_get_sensor(Motor *motor, uint16_t trigger_source);
uint32_t rt_motor_move_to_zero(Motor *motor, uint32_t mtGrade, uint32_t timeout);
int rt_motor_get_id(Motor *motor);
int rt_motor_get_reset_value(Motor *motor);
#endif


