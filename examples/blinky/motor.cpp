#include <qpc.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtconfig.h>
#include <stdint.h>
#include <drv_common.h>
#include <protocol.h>
#include "motor.h"

#define LOG_TAG "motor"
#define DBG_SECTION_NAME "motor"
#define LOG_LVL LOG_LVL_DBG
#include <rtdbg.h>
#define MT_TIM_PERSCALER 35

uint32_t rt_motor_calcaulate_speed(rt_motor_t *motor, uint32_t target_steps);
void rt_motor_set_enable(rt_motor_t *motor);

int rt_motor_get_crash_value(rt_motor_t *motor);
int rt_motor_get_arrive_value(rt_motor_t *motor);
int rt_motor_get_reset_value(rt_motor_t *motor);
int rt_motor_get_code_value(rt_motor_t *motor);

// 电机复位
uint32_t rt_motor_move_to_zero(rt_motor_t *motor, uint32_t grade, uint32_t timeout)
{
    uint32_t reg_value = ENABLE;
    uint32_t result    = RT_EOK;
    int32_t acctiarr;

    /* 复位触发后运动的步数(反方向离开光耦)是否大于加速数组 */
    if (motor->config->steps_reset_signal_nega > ACCTIMARR)
        acctiarr = ACCTIMARR - 1;
    else
        acctiarr = motor->config->steps_reset_signal_nega;

    // 获取复位光耦的输入值(指定信号源)
    reg_value = rt_motor_get_sensor(motor, RESET_SIGNAL_SOURCE);
    if (reg_value == 0)
    {
        result = rt_motor_trigger_move(motor, RESET_SIGNAL_SOURCE, grade, -motor->config->max_position, acctiarr, timeout);

        if (result == RT_ERROR)
        {
            return RT_ERROR;
        }
    }

    // 复位后设置当前位置为0
    motor->currentPosition = 0;
    motor->reset_status    = MOTOR_RESET_STATUS;

    return RT_EOK;
}

/******************************************************
* @decription:
*      电机复位
* @para:
*     @dev:             设备
            @mtGrade：    速度档位
*     @timeout:     超时时间
* @return : rt_err_t
* @author： sicheng.lai
* @date: 2021-03-02
*******************************************************/
uint32_t rt_motor_reset(rt_motor_t *motor, uint32_t grade, uint32_t timeout)
{
    uint32_t reg_value      = ENABLE;
    uint32_t result         = RT_EOK;
    rt_uint32_t motor_grade = 0;

    LOG_I("%s %d", __func__, rt_motor_get_id(motor));
    // 复位时先清除错误码
    rt_motor_set_error_code(motor, MOTOR_ERROR_ERR_NONE);

    // (1)判断是否在光耦，如果不在光耦就执行第一次找光耦，在光耦就执行下一步
    // 获取复位光耦的输入值(指定信号源)
    reg_value = rt_motor_get_sensor(motor, RESET_SIGNAL_SOURCE);

    if (reg_value == 0)  // 光耦外
    {
        LOG_I("step1: find oc:%d", motor->config->steps_reset_signal_nega);
        rt_thread_delay(10);  // 延时10ms 消抖操作
        reg_value = rt_motor_get_sensor(motor, RESET_SIGNAL_SOURCE);

        if (reg_value == 0)
        {
            /* 将当前位置设置为最大位置 */
            motor->currentPosition = motor->config->max_position;

            /* 触发运动 参数：设备 速度档位 触发源（复位信号） 触发后走的步数（减速） 目标方向 超时时间*/
            result = rt_motor_trigger_move(motor, RESET_SIGNAL_SOURCE, grade, -motor->config->max_position, motor->config->steps_reset_signal_nega,
                                           timeout);
            if (result == RT_ERROR)
            {
                return result;
            }
        }
    }

    // (2) 正向出光耦        此时电机一定在光耦内。但在光耦的位置未知（存在一开始就在光耦内的情况），因此进行正向出光耦操作
    // 脱离光耦，走大于加速的步数，让后面(3)反向找光耦动作执行时, 找到光耦状态是匀速的

    LOG_I("step2: out oc:%d", motor->config->steps_reset_signal_posi);

    /* 当前位置设置为最小位置 */
    motor->currentPosition = motor->config->min_position;
    /* 参数：设备 速度档位 运动步数 运动方向 超时时间 */
    result = rt_motor_trigger_move(motor, RESET_SIGNAL_SOURCE, grade, motor->config->max_position, motor->config->steps_reset_signal_posi, timeout);
    if (result == RT_ERROR)
    {
        return RT_ERROR;
    }

    // 特殊环形电机
    if (motor->config->motor_type == MOTOR_TYPE_SPECIAL_CIRCLE)
    {
        motor_grade = 1;
    }
    else
    {
        motor_grade = grade;
    }

    rt_thread_delay(100);

    // (3) 反向找光耦，此时找到光耦为匀速阶段  出光耦后，电机的位置是一定的。此时再进光耦，到达机械零点

    // 反向找光耦
    LOG_I("step3: find oc:%d", motor->config->steps_reset_signal_nega);
    motor->currentPosition = motor->config->max_position;  // 将当前位置设置为最大位置
    // 触发运动 参数：设备 速度档位 触发源（复位信号） 触发后走的步数 目标方向 超时时间
    result =
        rt_motor_trigger_move(motor, RESET_SIGNAL_SOURCE, motor_grade, -motor->config->max_position, motor->config->steps_reset_signal_nega, timeout);
    if (result == RT_ERROR)
    {

        return RT_ERROR;
    }

    // 复位后设置当前位置为0
    motor->currentPosition = 0;
    motor->reset_status    = MOTOR_RESET_STATUS;  // 电机复位状态设置为复位

    return RT_EOK;
}

// 电机到指定的位置
uint32_t rt_motor_goto(rt_motor_t *motor, int32_t target_position, uint32_t grade, uint32_t timeout)
{
    int32_t finaltarget_position = 0;

    // 当前电机没有复位，不能执行goto
    if (motor->reset_status == MOTOR_NO_RESET_STATUS)
    {
        motor->error_code = MOTOR_ERROR_NOT_RESET;  // 设置电机错误码为电机未复位
        LOG_E("MOTOR_ERROR_NOT_RESET,%d,%d", motor->config->motor_id, __LINE__);
        return RT_ERROR;
    }

    if (target_position == motor->currentPosition)  // 如果目标位置等于当前位置
    {
        return RT_EOK;
    }

    // 盘电机使用转最短距离到达目标位置（如果电机类型为环形电机【比如盘电机】）
    if (motor->config->motor_type == MOTOR_TYPE_CIRCLE || motor->config->motor_type == MOTOR_TYPE_SPECIAL_CIRCLE)
    {
        // 目标位置 - 当前位置 > 最大位置的一半
        if ((target_position - motor->currentPosition) > (motor->config->max_position) / 2)
        {
            /* 要运动的步数 =  目标位置 - 当前位置 */
            finaltarget_position = target_position - motor->currentPosition;
            /* 要运动的步数 = 电机最大位置 - 要运动的步数 （进行反转） */
            finaltarget_position = motor->config->max_position - finaltarget_position;
            /* 运动方向改为反方向 */
        }
        else
        {
            /* 小于一半则直接正向运动 */
            finaltarget_position = target_position - motor->currentPosition;
        }
    }
    else
    {
        /* 如果不为盘电机，则直接正向运动 */
        finaltarget_position = target_position - motor->currentPosition;
    }

    // 电机运动
    return rt_motor_move(motor, grade, finaltarget_position, timeout);
}

/******************************************************
* @decription:
*      电机移动目标步数
* @para:
*     @dev:                      设备
*     @grade：    速度档位
*     @target_position：    目标步数
*     @target_dir：         目标方向
*     @timeout:                  超时时间
* @return : rt_err_t
* @author： sicheng.lai
* @date: 2021-03-02
*******************************************************/
uint32_t rt_motor_move(rt_motor_t *motor, uint32_t grade, int32_t target_steps, uint32_t timeout)
{
    uint32_t result;

    LOG_I("motor_move:ID=%d,G=%d,%d,T=%d", motor->config->motor_id, grade, target_steps, timeout);

    uint32_t target_dir = DIRECTION_POSITIVE;
    if (target_steps < 0)
    {
        target_steps = -target_steps;
        target_dir   = DIRECTION_NEGATIVE;
    }

    motor->direction = target_dir;  // 运动方向

    /* 判断电机复位状态 */
    if (motor->triggle_source == NULL_SIGNAL_SOURCE)
    {
// 当前电机没有复位，不能执行move
#if (RT_CANTP_RX_ID != 4)
        if (motor->reset_status == MOTOR_NO_RESET_STATUS)
        {
            motor->error_code = MOTOR_ERROR_NOT_RESET;  // 设置电机错误码为电机未复位

            LOG_E("MOTOR_ERROR_NOT_RESET,%d,%d", motor->config->motor_id, __LINE__);
            return RT_ERROR;
        }
#endif
    }
    else if (motor->reset_status == RESET_SIGNAL_SOURCE)
    {
        /* 程序继续运行，不做任何操作 */
    }

    // (0) 根据速度档位，更新速度参数
    if (grade >= MOTOR_GRADE_MAX_NUM)
    {
        return -1;
    }

    rt_memcpy(&(motor->current_speed), &(motor->config->speed_para[grade]), sizeof(motor_speed_t));

    // 计算加速度并设置
    rt_motor_calcaulate_speed(motor, target_steps);

    // 设置方向 设置方向引脚的值
    rt_motor_set_direction(motor, motor->direction);

    // 设置正常模式
    rt_motor_set_low_power(motor, NORMAL_POWER_MODE);

    // 使能启动电机
    rt_motor_start(motor);

    // 等待电机运动完成
    result = rt_motor_wait(motor, timeout);

    return result;
}

// 电机移动目标步数（触发）
uint32_t rt_motor_trigger_move(rt_motor_t *motor, uint16_t trigger_source, uint32_t grade, int32_t trigger_steps, uint32_t step2, uint32_t timeout)
{
    uint32_t result = RT_EOK;
    LOG_W("%s:ID=%d,G=%d,%d", __func__, motor->config->motor_id, grade, trigger_steps);

    motor->trigger_after_move_steps = step2;  // 触发后要走的减速的步数
    motor->error_code               = 0;

    motor->start_trigger_status = rt_motor_get_sensor(motor, trigger_source);  // 记录光耦起始状态
    motor->trigger_status       = NULL_SIGNAL_SOURCE;                          // 设置触发状态为空（清空当前电机的触发状态）
    motor->triggle_source |= trigger_source;                                   // 设置触发源

    if (trigger_source == RESET_SIGNAL_SOURCE)
    {
        if (trigger_steps >= 0)
        {
            motor->currentPosition = motor->config->min_position;
        }
        else
        {
            motor->currentPosition = motor->config->max_position;
        }
    }

    result = rt_motor_move(motor, grade, trigger_steps, timeout);

    return result;
}

// 等待运动完成
uint32_t rt_motor_wait(rt_motor_t *motor, uint32_t timeout)
{
    uint32_t receiveEvent;
    uint32_t result = RT_EOK;

    if (timeout == 0)
    {
        // 非阻塞运动
        result = RT_EOK;
    }
    else
    {
        // 阻塞运动，等待电机事件
        if (rt_event_recv(&(motor->motor_event), (MOTOR_EVENT_COMPLETED_FLAG | MOTOR_EVENT_ERROR_FLAG | MOTOR_EVENT_ARRIVED_FLAG),
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, &receiveEvent)
            == RT_EOK)
        // 电机在指定时间完成或出现错误 ， 并将时间标志清除
        {
            // 如果电机运行错误
            if (receiveEvent == MOTOR_EVENT_ERROR_FLAG)
            {
                // 电机中断会存出现把错误码
                // 出错后，位置已异常，电机标记未复位状态
                motor->reset_status = MOTOR_NO_RESET_STATUS;
                LOG_E("MOTOR_NO_RESET_STATUS:%d, %d", motor->config->motor_id, __LINE__);
                result = RT_ERROR;
            }
            else if (receiveEvent == MOTOR_EVENT_ARRIVED_FLAG)
            {
                LOG_W("MOTOR_EVENT_ARRIVED_FLAG");
            }
            else
            {
                // 碰撞运动是属于触发运动，会产生完成标志，不会产生错误标志，等完成减速后再判断标记错误
                if (motor->error_code)  // 判断当前错误码
                {
                    // 出错后，位置已异常，电机标记未复位状态
                    LOG_E("MOTOR_NO_RESET_STATUS:%d, %d,%d", motor->config->motor_id, motor->error_code, __LINE__);
                    motor->reset_status = MOTOR_NO_RESET_STATUS;
                    return RT_ERROR;
                }
            }
        }
        else
        {
            // 出错后，位置已异常，电机标记未复位状态
            motor->reset_status = MOTOR_NO_RESET_STATUS;
            motor->error_code   = MOTOR_ERROR_TIMEOUT;
            LOG_E("MOTOR_ERROR_TIMEOUT:%d, %d,currentPosition=%d", motor->config->motor_id, __LINE__, motor->currentPosition);

            rt_motor_instant_stop(motor);  // 立即停止，关闭定时器的中断
            result = RT_ERROR;
        }
    }
    return result;
}

// 启动电机
uint32_t rt_motor_start(rt_motor_t *motor)
{
    uint32_t result = RT_EOK;

    uint32_t reg_value = ENABLE;

    RT_ASSERT(motor != RT_NULL);

    rt_motor_set_enable(motor);

    return result;
}

// 电机减速停止
uint32_t rt_motor_dec_stop(rt_motor_t *motor, uint32_t target_steps)
{
    RT_ASSERT(motor != RT_NULL);

    // 获取当前驱动层运动的步数，然后加上减速需要的步数
    uint32_t remain_steps = target_steps + motor->absolute_steps;

    // 更改速度参数为线性减速
    motor_speed_t mspeed_para = {motor->currentFreq, motor->currentFreq, motor->current_speed.end_frequency, 0, 0, 0, 0, 0, target_steps, 0};

    rt_memcpy(&(motor->current_speed), (void *)&mspeed_para, sizeof(motor_speed_t));

    // 计算电机参数
    rt_motor_calcaulate_speed(motor, remain_steps);

    return RT_EOK;
}

void rt_motor_set_reset_status(rt_motor_t *motor, uint8_t reset_status)
{
    motor->reset_status = reset_status;
}

uint32_t rt_motor_get_reset_status(rt_motor_t *motor)
{
    return motor->reset_status;
}

// 设置运动方向
uint32_t rt_motor_set_direction(rt_motor_t *motor, uint8_t direction)
{
    uint32_t reg_value = direction;
    RT_ASSERT(motor != RT_NULL);

    uint32_t dir = direction ^ (motor->config->direction_level);

    if (dir)
    {
        rt_pin_write(motor->config->dir_pin, PIN_HIGH);
    }
    else
    {
        rt_pin_write(motor->config->dir_pin, PIN_LOW);
    }

    return RT_EOK;
}

// 获取匀速（最大频率）
int rt_motor_get_id(rt_motor_t *motor)
{
    return motor->config->motor_id;
}

// 使能/失能AD采集
uint32_t rt_motor_set_ad_collect_enable(rt_motor_t *motor, uint8_t enable_flag, uint32_t dynamic_steps)
{
    RT_ASSERT(motor != RT_NULL);
    UNUSED(enable_flag);

    RT_ASSERT(motor != RT_NULL);
    uint32_t reg_value = 0;

    /*
     *  寄存器0位： 使能位
     *  寄存器1位：电平有效位
     */
    if (enable_flag > 0)
    {
        reg_value = 0x01;
    }
    else
    {
        reg_value = 0;
    }

    if (motor->config->code_signal_level > 0)
    {
        reg_value += 0x02;
    }
    else
    {
        reg_value += 0x00;
    }

    // 动态AD采集偏移步数
    if (dynamic_steps > 0)
    {
        reg_value = reg_value + (dynamic_steps << 16);
    }

    reg_value = 0;
    return RT_EOK;
}

// 设置错误码
uint32_t rt_motor_set_error_code(rt_motor_t *motor, uint32_t error_code)
{
    RT_ASSERT(motor != RT_NULL);

    motor->error_code = error_code;

    return RT_EOK;
}

// 获取错误码
uint32_t rt_motor_get_error_code(rt_motor_t *motor, uint32_t *error_code)
{
    RT_ASSERT(motor != RT_NULL);

    *error_code = motor->error_code;

    return RT_EOK;
}

// 获取传感器的值
uint32_t rt_motor_get_sensor(rt_motor_t *motor, uint16_t trigger_source)
{
    RT_ASSERT(motor != RT_NULL);

    uint32_t temp_reg_value = 0;

    if (trigger_source == RESET_SIGNAL_SOURCE)
    {
        temp_reg_value = rt_motor_get_reset_value(motor);
    }
    else if (trigger_source == CRASH_SIGNAL_SOURCE)
    {
        temp_reg_value = rt_motor_get_crash_value(motor);
    }
    else if (trigger_source == ARRIVE_SIGNAL_SOURCE)
    {
        temp_reg_value = rt_motor_get_arrive_value(motor);
    }
    else if (trigger_source == CODE_SIGNAL_SOURCE)
    {
        temp_reg_value = rt_motor_get_code_value(motor);
    }

    return temp_reg_value;
}

// 设置当前位置
uint32_t rt_motor_set_current_position(rt_motor_t *motor, int32_t position)
{
    RT_ASSERT(motor != RT_NULL);

    motor->currentPosition = position;

    return RT_EOK;
}

// 获取当前位置
int32_t rt_motor_get_current_position(rt_motor_t *motor)
{
    RT_ASSERT(motor != RT_NULL);

    LOG_W("M%d, currentPosition=%d", motor->config->motor_id, motor->currentPosition);

    return motor->currentPosition;
}

// 获取当前绝对步数
uint32_t rt_motor_get_absolute_steps(rt_motor_t *motor)
{
    RT_ASSERT(motor != RT_NULL);

    return (motor->absolute_steps);
}

// 计算加速度并设置
uint32_t rt_motor_calcaulate_speed(rt_motor_t *motor, uint32_t target_steps)
{
    RT_ASSERT(motor != RT_NULL);
    motor_speed_t *current_speed = &motor->current_speed;

    uint32_t area = 0;

    // 计算加速需要的步数
    motor->stepAcc = current_speed->step_t[0] + current_speed->step_t[1] + current_speed->step_t[2];

    // 计算减速需要的步数
    motor->stepDec = current_speed->step_t[4] + current_speed->step_t[5] + current_speed->step_t[6];

    // 如果目标步数小于整个加减速需要步数, 按线性加减速处理， 并重新计算最大速度
    if (target_steps < (motor->stepAcc + motor->stepDec))
    {
        uint32_t temp_acc_all_steps = motor->stepAcc;

        // 去除变加速减速阶段
        current_speed->step_t[0] = 0;
        current_speed->step_t[1] = target_steps * motor->stepAcc / (motor->stepAcc + motor->stepDec);
        current_speed->step_t[2] = 0;
        current_speed->step_t[4] = 0;
        current_speed->step_t[5] = target_steps * motor->stepDec / (motor->stepAcc + motor->stepDec);
        current_speed->step_t[6] = 0;

        // 根绝加速步数比例计算最大频率
        uint32_t detSpeed = current_speed->max_frequency - (current_speed->start_frequency / 2 + current_speed->end_frequency / 2);

        current_speed->max_frequency = current_speed->start_frequency + current_speed->step_t[1] * detSpeed / motor->stepAcc;

        motor->stepAcc = current_speed->step_t[1];
        motor->stepDec = current_speed->step_t[5];
    }

    current_speed->step_t[3] = target_steps - (motor->stepAcc + motor->stepDec);

    // 目标步数大于整个加减速，正常赋值
    motor->stepT[0] = current_speed->step_t[0];
    motor->stepT[1] = motor->stepT[0] + current_speed->step_t[1];
    motor->stepT[2] = motor->stepT[1] + current_speed->step_t[2];
    motor->stepT[3] = motor->stepT[2] + current_speed->step_t[3];
    motor->stepT[4] = motor->stepT[3] + current_speed->step_t[4];
    motor->stepT[5] = motor->stepT[4] + current_speed->step_t[5];
    motor->stepT[6] = motor->stepT[5] + current_speed->step_t[6];

    // 变加减速阶段的加速度
    if ((current_speed->max_frequency) <= (current_speed->start_frequency))  // 如果匀速（最大）频率小于起始频率
    {
        motor->acc1 = 0;
        motor->acc2 = 0;
    }
    else
    {
        // 计算加速梯形面积（用最大频率减去起始频率）
        area = (current_speed->max_frequency) - (current_speed->start_frequency);

        // 计算变加速阶段的加速度
        if (((current_speed->step_t[0]) == 0) && ((current_speed->step_t[1]) == 0) && ((current_speed->step_t[2]) == 0))
        {
            motor->acc1 = 0;
            motor->acc2 = 0;
        }
        else
        {
            if ((current_speed->step_t[0] == 0) || (current_speed->step_t[2] == 0))
            {
                if (current_speed->step_t[1] != 0)
                {
                    motor->accMax = (area << 10) / (current_speed->step_t[1]);
                }
                else
                {
                    motor->accMax = 0;
                }
            }
            else
            {
                // 梯形面积计算（放大 1024）
                motor->accMax = (area << 11) / (2 * (current_speed->step_t[1]) + (current_speed->step_t[0]) + (current_speed->step_t[2]));
                // 计算加加速度的值
                motor->acc1 = motor->accMax / (current_speed->step_t[0]);
                motor->acc2 = motor->accMax / (current_speed->step_t[2]);
            }
        }
    }

    if ((current_speed->max_frequency) <= (current_speed->end_frequency))  // 如果匀速（最大）频率小于结束频率
    {
        motor->dec1 = 0;
        motor->dec2 = 0;
    }
    else
    {
        // 计算减速梯形面积
        area = (current_speed->max_frequency) - (current_speed->end_frequency);

        // 计算变减速阶段的加速度
        if (((current_speed->step_t[4]) == 0) && ((current_speed->step_t[5]) == 0) && ((current_speed->step_t[6]) == 0))
        {
            motor->dec1 = 0;
            motor->dec2 = 0;
        }
        else
        {
            if ((current_speed->step_t[4] == 0) || (current_speed->step_t[6] == 0))
            {
                if (current_speed->step_t[5] != 0)
                {
                    motor->decMax = (area << 10) / (current_speed->step_t[5]);
                }
                else
                {
                    motor->decMax = 0;
                }
            }
            else
            {
                motor->decMax = (area << 11) / (2 * (current_speed->step_t[5]) + (current_speed->step_t[4]) + (current_speed->step_t[6]));

                motor->dec1 = motor->decMax / (current_speed->step_t[4]);
                motor->dec2 = motor->decMax / (current_speed->step_t[6]);
            }
        }
    }

    // 梯形高被放大计算，这里需要处理加速度值
    // acc1 = (acc1)>>2;
    // acc2 = (acc2)>>2;
    // motor->dec1 = (motor->dec1)>>2;
    // motor->dec2 = (motor->dec2)>>2;
#if 0
    for (int i = 0; i < 7; i++)
    {
        LOG_I("stepT[%d]=%d", i, motor->stepT[i]);
    }

    LOG_I("acc1= %d", motor->acc1);
    LOG_I("acc2= %d", motor->acc2);
    LOG_I("dec1= %d", motor->dec1);
    LOG_I("dec2= %d", motor->dec2);

    LOG_I("max_acc= %d", motor->accMax);
    LOG_I("max_dec= %d", motor->decMax);
    LOG_I("stepAcc= %d", motor->stepAcc);
    LOG_I("stepAcc= %d", motor->stepDec);

    LOG_I("start_frequency= %d", current_speed->start_frequency);
    LOG_I("max_frequency= %d", current_speed->max_frequency);
    LOG_I("end_frequency= %d", current_speed->end_frequency);
#endif /* endif RT_USING_MOTOR_DEBUG */

    int32_t AddFreq = 0;
    int32_t CurFreq = current_speed->start_frequency << 10;

    // step3: 计算加速过程的频率变化
    // LOG_I("ACC");
    for (int i = 0; i < motor->stepT[2]; i++)
    {
        if (i < motor->stepT[0])
        {
            AddFreq = motor->acc1 * i;
        }
        else if (i < motor->stepT[1])
        {
            AddFreq = motor->accMax;
        }
        else if (i < motor->stepT[2])
        {
            AddFreq = motor->accMax - motor->acc2 * (i - motor->stepT[1]);
        }

        CurFreq += AddFreq;
        if (0)
        {
            LOG_RAW("%d: %d\n", i, CurFreq >> 10);
            for (int j = 0; j < 10000; j++)
                ;
        }

        motor->accTimArr[i] = (CurFreq >> 10);
    };

    // step4: 计算减速过程的频率变化
    // 减速的最大频率为加速后的频率

    // LOG_I("DEC");
    uint16_t T5 = motor->stepT[4] - motor->stepT[3];
    uint16_t T6 = motor->stepT[5] - motor->stepT[3];
    uint16_t T7 = motor->stepT[6] - motor->stepT[3];

    for (int i = 0; i < T7; i++)
    {
        if (i < T5)
        {
            AddFreq = motor->dec1 * i;
        }
        else if (i < T6)
        {
            AddFreq = motor->decMax;
        }
        else if (i < T7)
        {
            AddFreq = (motor->decMax + (motor->dec2 * (T6 - i)));
        }
        CurFreq -= AddFreq;
        if (0)
        {
            LOG_RAW("%d: %d\n", i, CurFreq >> 10);
            for (int j = 0; j < 10000; j++)
                ;
        }

        motor->decTimArr[i] = (CurFreq >> 10);
    }

    return 0;
}

int rt_motor_get_reset_value(rt_motor_t *motor)
{
    if (motor->config->reset_pin)
    {
        return (rt_pin_read(motor->config->reset_pin) != motor->config->reset_signal_level);
    }
    LOG_E("%s: reset_pin is not used", __func__);

    return RT_ENOSYS;
}

int rt_motor_get_arrive_value(rt_motor_t *motor)
{
    if (motor->config->arrive_pin)
    {
        return (rt_pin_read(motor->config->arrive_pin) != motor->config->arrive_signal_level);
    }

    LOG_E("%s: arrive_pin is not used", __func__);

    return RT_ENOSYS;
}

int rt_motor_get_crash_value(rt_motor_t *motor)
{
    if (motor->config->crash_pin)
    {
        return (rt_pin_read(motor->config->crash_pin) != motor->config->crash_signal_level);
    }

    LOG_E("%s: crash_pin is not used", __func__);

    return RT_ENOSYS;
}

int rt_motor_get_code_value(rt_motor_t *motor)
{
    if (motor->config->code_pin)
    {
        return (rt_pin_read(motor->config->code_pin) != motor->config->code_signal_level);
    }

    LOG_E("%s: code_pin is not used", __func__);

    return RT_ENOSYS;
}
/*
 * 操作电机底层的接口
 */
void rt_motor_set_enable(rt_motor_t *motor)
{
    motor->timeLoadIndex    = 0;
    motor->relativePosition = 0;
    motor->timerCount       = 0;
    motor->currentFreq      = motor->accTimArr[motor->timeLoadIndex];
    motor->motionState      = MOTOR_STATE_ACCEL;

    // 设置定时器加载值
    if (motor->currentFreq > 50000)
    {
        LOG_E("error Freq %d", motor->currentFreq);
        motor->currentFreq = 1000;
    }
    motor->timeLoadValue = 2329600 / (motor->currentFreq);  // (SystemCoreClock / (MT_TIM_PERSCALER + 1)/2  167731200/36/2
    __HAL_TIM_SET_AUTORELOAD(&(motor->timerHandle), motor->timeLoadValue);

    __HAL_TIM_SET_COUNTER(&(motor->timerHandle), 0);
    // 开启定时器中断
    HAL_TIM_OC_Start_IT(&(motor->timerHandle), motor->config->Channel);
}

// 设置使能模式
void rt_motor_set_low_power(rt_motor_t *motor, uint8_t mode)
{
    rt_pin_write(motor->config->lpw_pin, motor->config->low_power_level ^ mode);
}

/******************************************************
 * @decription: 电机立即停止
 * @para:
       @motor  ： 电机结构体对象
 * @return: void
 * @author： sicheng.lai
 * @date: 2021-12-22
 *******************************************************/
uint32_t rt_motor_instant_stop(rt_motor_t *motor)
{
    HAL_TIM_OC_Stop_IT(&(motor->timerHandle), motor->config->Channel);
    motor->motionState = MOTOR_STATE_STOP;

    rt_pin_write(motor->config->lpw_pin, motor->config->low_power_level ^ LOW_POWER_MODE);

    return RT_EOK;
}

/******************************************************
 * @decription: 触发信号检测
 * @para:
       @motor  ： 电机结构体对象
 * @return: void
 * @author： sicheng.lai
 * @date: 2021-12-22
 *******************************************************/
uint32_t stm32_motor_siganal_triggle_check(rt_motor_t *motor)
{
    if ((motor->triggle_source & RESET_SIGNAL_SOURCE) == RESET_SIGNAL_SOURCE)  // 电机驱动层复位触发检测
    {
        uint32_t reg_value = rt_motor_get_sensor(motor, RESET_SIGNAL_SOURCE);
        if (reg_value != motor->start_trigger_status)  // 光耦跳变
        {
            motor->triggle_source &= ~(uint32_t)RESET_SIGNAL_SOURCE;
            motor->trigger_status |= RESET_SIGNAL_SOURCE;           // 设置触发状态为复位触发
            if (motor->reset_signal_stop_way == MOTOR_DEC_TO_STOP)  // 如果复位停止方式为减速停止 TODO
            {
                rt_motor_dec_stop(motor, motor->trigger_after_move_steps);  // 减速停止
            }
            else
            {
                rt_motor_instant_stop(motor);  // 立即停止
            }
        }
    }

    if ((motor->triggle_source & CRASH_SIGNAL_SOURCE) == CRASH_SIGNAL_SOURCE)
    {
        // 电机驱动层碰撞触发检测
        if (rt_motor_get_sensor(motor, CRASH_SIGNAL_SOURCE) == 1)
        {
            motor->trigger_status |= CRASH_SIGNAL_SOURCE;
            motor->triggle_source &= ~(uint32_t)CRASH_SIGNAL_SOURCE;
            motor->error_code = MOTOR_ERROR_CRASH;
            if (motor->crash_signal_stop_way == MOTOR_DEC_TO_STOP)
            {
                rt_motor_dec_stop(motor, motor->config->steps_crash_signal);
            }
            else
            {
                rt_motor_instant_stop(motor);
            }
            rt_event_send(&(motor->motor_event), MOTOR_EVENT_ERROR_FLAG);
        }
    }

    // 电机驱动层抵达触发检测
    if ((motor->triggle_source & ARRIVE_SIGNAL_SOURCE) == ARRIVE_SIGNAL_SOURCE)
    {
        if (rt_motor_get_sensor(motor, ARRIVE_SIGNAL_SOURCE) == 1)
        {
            motor->trigger_status |= ARRIVE_SIGNAL_SOURCE;
            motor->triggle_source &= ~(uint32_t)ARRIVE_SIGNAL_SOURCE;
            if (motor->arrive_signal_stop_way == MOTOR_DEC_TO_STOP)
            {
                rt_motor_dec_stop(motor, motor->config->steps_arrive_signal);
            }
            else
            {
                rt_motor_instant_stop(motor);
            }
            rt_event_send(&(motor->motor_event), MOTOR_EVENT_ARRIVED_FLAG);
        }
    }
    // motor->triggle_source = NULL_SIGNAL_SOURCE;  // 将触发检测位清除，防止重复触发运动

    return RT_EOK;
}

/******************************************************
 * @decription: 电机中断回调函数
 * @para:
       @motor  ： 电机结构体对象
 * @return: void
 * @author： sicheng.lai
 * @date: 2021-12-22
 *******************************************************/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *timerHandle)
{
    rt_interrupt_enter();

    rt_motor_t *motor = (rt_motor_t *)(timerHandle->object);

    // 判断，如果计数器的值大于重装载值，则手动清零
    uint32_t counter    = __HAL_TIM_GET_COUNTER(&(motor->timerHandle));
    uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(&(motor->timerHandle));
    if (counter >= autoreload)
    {
        LOG_E("__HAL_TIM_SET_COUNTER ID=%d,%d,%d,%d", motor->config->motor_id, counter, autoreload, motor->timerCount / 2);

        __HAL_TIM_SET_COUNTER(&(motor->timerHandle), 0);
    }

    motor->timerCount++;

    // 判断完整脉冲
    if (motor->timerCount % 2 == 0)
    {
        if (motor->motionState != MOTOR_STATE_RUN)
        {
            // 设置电机当前脉冲频率
            if (motor->currentFreq > 50000)
            {
                LOG_W("error timeLoadValue %d", motor->timeLoadValue);
                motor->currentFreq = 1000;
            }

            motor->timeLoadValue = 2329600 / motor->currentFreq;  // (SystemCoreClock / (MT_TIM_PERSCALER + 1)/2
            __HAL_TIM_SET_AUTORELOAD(&(motor->timerHandle), motor->timeLoadValue);
        }

        // 电机不同状态处理
        switch (motor->motionState)
        {
            case MOTOR_STATE_STOP:  //     停止/空闲
            {
                rt_motor_instant_stop(motor);
                break;
            }
            case MOTOR_STATE_ACCEL:  //     加速
            {
                motor->relativePosition++;                   //     电机相对位置增加
                if (motor->direction == DIRECTION_POSITIVE)  //     如果为正向运动
                {
                    motor->currentPosition++;
                }
                else
                {
                    motor->currentPosition--;
                }

                if (motor->relativePosition < motor->stepAcc)
                {
                    if (motor->timeLoadIndex < motor->stepAcc)
                    {
                        motor->timeLoadIndex++;
                        motor->currentFreq = motor->accTimArr[motor->timeLoadIndex];
                    }
                }
                else
                {
                    motor->motionState = MOTOR_STATE_RUN;
                }
                break;
            }

            case MOTOR_STATE_RUN:  //     匀速
            {
                motor->relativePosition++;
                if (motor->direction == DIRECTION_POSITIVE)
                {
                    motor->currentPosition++;
                }
                else
                {
                    motor->currentPosition--;
                }

                if ((motor->stepT[6] - motor->relativePosition) <= motor->stepDec)
                {
                    if (motor->stepDec == 0)
                    {
                        rt_motor_instant_stop(motor);
                    }
                    else
                    {
                        motor->motionState   = MOTOR_STATE_DECEL;
                        motor->timeLoadIndex = 0;
                    }
                }

                break;
            }
            case MOTOR_STATE_DECEL:  //     減速
            {
                motor->relativePosition++;
                if (motor->direction == DIRECTION_POSITIVE)
                {
                    motor->currentPosition++;
                }
                else
                {
                    motor->currentPosition--;
                }

                if (motor->relativePosition < motor->stepT[6])
                {
                    if (motor->timeLoadIndex < motor->stepDec)
                    {
                        motor->currentFreq = motor->decTimArr[motor->timeLoadIndex];
                        motor->timeLoadIndex++;
                    }
                }
                else
                {
                    rt_motor_instant_stop(motor);
                    int result = RT_EOK;

                    // 检查抵达状态
                    if ((motor->triggle_source == ARRIVE_SIGNAL_SOURCE) && (motor->trigger_status == NULL_SIGNAL_SOURCE))
                    {
                        rt_motor_set_error_code(motor, MOTOR_ERROR_TIMEOUT);
                        motor->triggle_source = NULL_SIGNAL_SOURCE;
                        LOG_E("ARRIVE_SIGNAL_SOURCE trigger fail\n");
                        motor->currentPosition = 0;

                        result = RT_ERROR;
                    }

                    // 检查复位状态
                    if ((motor->triggle_source == RESET_SIGNAL_SOURCE) && (motor->trigger_status & RESET_SIGNAL_SOURCE) != RESET_SIGNAL_SOURCE)
                    {
                        motor->reset_status = MOTOR_NO_RESET_STATUS;
                        rt_motor_set_error_code(motor, MOTOR_ERROR_RESET_FAIL);
                        result = RT_ERROR;
                    }

                    // 环形电机，没有尽头
                    if (motor->config->motor_type == MOTOR_TYPE_CIRCLE || motor->config->motor_type == MOTOR_TYPE_SPECIAL_CIRCLE)
                    {
                        if (motor->currentPosition > (motor->config->max_position - 1))
                        {
                            motor->currentPosition %= motor->config->max_position;
                        }

                        result = RT_EOK;
                    }
                    // 线性或其他电机:运动范围超限制判断
                    else if (motor->currentPosition > motor->config->max_position || motor->currentPosition < motor->config->min_position)
                    {
                        motor->error_code = MOTOR_ERROR_EXCEL_LIMIT;
                        result            = RT_ERROR;
                    }

                    // 触发后的执行函数（立即停止或减速停止）
                    if (result != RT_EOK)
                    {
                        rt_event_send(&(motor->motor_event), MOTOR_EVENT_ERROR_FLAG);  // 发送错误标志
                    }
                    else
                    {
                        rt_event_send(&(motor->motor_event), MOTOR_EVENT_COMPLETED_FLAG);  // 发送完成事件标志
                    }

                    if (motor->triggle_source)
                    {
                        motor->triggle_source = NULL_SIGNAL_SOURCE;
                    }
                }

                break;
            }
        }  // end switch

        /* 更新当前绝对位置 */
        motor->absolute_steps = motor->relativePosition;

        if (motor->triggle_source)  // 如果设置了触发源
        {
            stm32_motor_siganal_triggle_check(motor);  // 触发信号检测
        }

        if (motor->triggle_source == NULL_SIGNAL_SOURCE)
        {
            if (motor->currentPosition > motor->config->max_position || motor->currentPosition < motor->config->min_position)
            {
                motor->error_code = MOTOR_ERROR_EXCEL_LIMIT;
                LOG_E("motor_id=%d,currentPosition=%d, min_position=%d,max_position=%d", motor->config->motor_id, motor->currentPosition,
                      motor->config->min_position, motor->config->max_position);
                rt_motor_instant_stop(motor);
                rt_event_send(&(motor->motor_event), MOTOR_EVENT_ERROR_FLAG);
            }
        }
    }  // end if(motor->timerCount% 2 == 0)

    rt_interrupt_leave();
}

/******************************************************
 * @decription: 初始化定时器
 * @para:
 *            @timerHandle： 定时器句柄
       @timerType  ： 定时器编号
       @timerIrqNum： 定时器中断编号
 * @return: void
 * @author： sicheng.lai
 * @date: 2021-12-22
 *******************************************************/
void stm32_motor_timer_init(TIM_HandleTypeDef *timerHandle, TIM_TypeDef *timerType, uint32_t Channel)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_OC_InitTypeDef sConfigOC              = {0};
    TIM_HandleTypeDef *timerHandleType        = timerHandle;

    timerHandleType->Instance               = timerType;
    timerHandleType->Init.CounterMode       = TIM_COUNTERMODE_UP;
    timerHandleType->Init.Period            = 1000;
    timerHandleType->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    timerHandleType->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (timerType == TIM1 || timerType == TIM8 || timerType == TIM9 || timerType == TIM10 || timerType == TIM11)
    {

        timerHandleType->Init.Prescaler = 34;
    }
    else
    {

        timerHandleType->Init.Prescaler = 17;
    }

    // 1、基础定时器初始化
    if (HAL_TIM_Base_Init(timerHandleType) != RT_EOK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(timerHandleType, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(timerHandleType) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    // 2、主机模式配置同步
    if (HAL_TIMEx_MasterConfigSynchronization(timerHandleType, &sMasterConfig) != RT_EOK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_OC_ConfigChannel(timerHandleType, &sConfigOC, Channel) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(timerHandleType, Channel);
}

void stm32_motor_pin_clk_enable(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (GPIOx == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (GPIOx == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (GPIOx == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (GPIOx == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (GPIOx == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (GPIOx == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
}

void stm32_motor_timer_MspPostInit(uint32_t pwm_pin, GPIO_TypeDef *GPIOx, TIM_TypeDef *TIMx, uint32_t Channel)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    stm32_motor_pin_clk_enable(GPIOx);

    GPIO_InitStruct.Pin   = pwm_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(STM32F407xx)

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    if (TIMx == TIM1)
    {
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    }
    else if (TIMx == TIM2)
    {
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    }
    else if (TIMx == TIM3)
    {
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    }
    else if (TIMx == TIM4)
    {
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    }
    else if (TIMx == TIM5)
    {
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    }
    else if (TIMx == TIM8)
    {
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    }
    else if (TIMx == TIM9)
    {
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    }
    else if (TIMx == TIM10)
    {
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
    }
    else if (TIMx == TIM11)
    {
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    }
    else if (TIMx == TIM12)
    {
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    }
    else if (TIMx == TIM13)
    {
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    }
    else if (TIMx == TIM14)
    {
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    }
#endif /* endif of defined(STM32F407xx) */

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

#if defined(STM32F103xE)

    /**
   * @brief Enable the remapping of TIM4 alternate function channels 1 to 4.
   * @note  ENABLE: Full remap (TIM4_CH1/PD12, TIM4_CH2/PD13, TIM4_CH3/PD14, TIM4_CH4/PD15)
   * @note  TIM4_ETR on PE0 is not re-mapped.
   * @retval None
   */
    if (GPIOx == GPIOD && pwm_pin == GPIO_PIN_12 && TIMx == TIM4 && Channel == TIM_CHANNEL_1)
    {

        __HAL_AFIO_REMAP_TIM4_ENABLE();  //     打开复用
    }
#endif
}

uint32_t rt_motor_init(rt_motor_t *motor, rt_motor_config_t *config)
{
    // 创建电机控制器
    motor->config = config;
    // 初始化定时器
    stm32_motor_timer_init(&motor->timerHandle, motor->config->timerType, motor->config->Channel);
    motor->timerHandle.object = motor;

    // 电机引脚初始化
    if (motor->config->dir_pin)
        rt_pin_mode(motor->config->dir_pin, PIN_MODE_OUTPUT);  // 电机方向引脚
    if (motor->config->lpw_pin)
        rt_pin_mode(motor->config->lpw_pin, PIN_MODE_OUTPUT);  // 电机低功耗引脚

    // 电机时钟引脚初始化
    stm32_motor_timer_MspPostInit(motor->config->pwm_pin, motor->config->GPIOx, motor->config->timerType, motor->config->Channel);
    if (motor->config->reset_pin)
        rt_pin_mode(motor->config->reset_pin, PIN_MODE_INPUT);  // 电机复位检测引脚
    if (motor->config->crash_pin)
        rt_pin_mode(motor->config->crash_pin, PIN_MODE_INPUT);  // 电机碰撞检测引脚
    if (motor->config->arrive_pin)
        rt_pin_mode(motor->config->arrive_pin, PIN_MODE_INPUT);  // 电机抵达检测引脚

    LOG_I("%s %d ok", __func__, motor->config->motor_id);

    // 状态初始化
    motor->reset_status = MOTOR_NO_RESET_STATUS;

    // 电机默认静止
    HAL_TIM_OC_Stop_IT(&(motor->timerHandle), motor->config->Channel);

    rt_pin_write(motor->config->lpw_pin, motor->config->low_power_level ^ LOW_POWER_MODE);

    motor->triggle_source = NULL_SIGNAL_SOURCE;

    // 创建或初始化事件集
    rt_event_init(&(motor->motor_event), "event", RT_IPC_FLAG_FIFO);

    // 设置默认的速度档位
    rt_memcpy(&(motor->current_speed), (void *)&motor->config->speed_para[MOTOR_GRADE_1], sizeof(motor_speed_t));

    // 初始化设置触发后的执行方式
    motor->reset_signal_stop_way  = MOTOR_DEC_TO_STOP;
    motor->crash_signal_stop_way  = MOTOR_INSTANT_STOP;
    motor->arrive_signal_stop_way = MOTOR_DEC_TO_STOP;
    motor->liquid_signal_stop_way = MOTOR_INSTANT_STOP;
    char mq_str[20];
    rt_sprintf(mq_str, "mq_%s", motor->config->name);
    motor->mq = rt_mq_create((const char *)mq_str, sizeof(msg), 5, RT_IPC_FLAG_FIFO);
    if (motor->mq == RT_NULL)
    {
        LOG_E("%s create fail!\n", mq_str);
        return RT_ERROR;
    }
    LOG_I("create %s ok", mq_str);

    return RT_EOK;
}

