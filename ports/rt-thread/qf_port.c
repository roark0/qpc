/*============================================================================
* QP/C Real-Time Embedded Framework (RTEF)
* Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-QL-commercial
*
* This software is dual-licensed under the terms of the open source GNU
* General Public License version 3 (or any later version), or alternatively,
* under the terms of one of the closed source Quantum Leaps commercial
* licenses.
*
* The terms of the open source GNU General Public License version 3
* can be found at: <www.gnu.org/licenses/gpl-3.0>
*
* The terms of the closed source Quantum Leaps commercial licenses
* can be found at: <www.state-machine.com/licensing>
*
* Redistributions in source code must retain this top-level comment block.
* Plagiarizing this software to sidestep the license obligations is illegal.
*
* Contact information:
* <www.state-machine.com>
* <info@state-machine.com>
============================================================================*/
/*!
* @date Last updated on: 2023-04-12
* @version Last updated for: @ref qpc_7_2_2
*
* @file
* @brief QF/C, port to RT-Thread
*/
#define QP_IMPL          /* this is QP implementation */
#include "qp_port.h"     /* QF port */
#include "qp_pkg.h"      // QP package-scope interface
#include "qsafe.h"       // QP Functional Safety (FuSa) Subsystem
#ifdef Q_SPY             /* QS software tracing enabled? */
    #include "qs_port.h" /* QS port */
    #include "qs_pkg.h"  /* QS package-scope internal interface */
#else
    #include "qs_dummy.h" /* disable the QS software tracing */
#endif                    /* Q_SPY */

Q_DEFINE_THIS_MODULE("qf_port")

/*..........................................................................*/
void QF_init(void)
{
}
/*..........................................................................*/
int_t QF_run(void)
{
    QS_CRIT_ENTRY();

    QF_onStartup(); /* QF callback to configure and start interrupts */

    /* produce the QS_QF_RUN trace record */
    QS_BEGIN_PRE_(QS_QF_RUN, 0U)
    QS_END_PRE_()
    QS_CRIT_EXIT();

    return 0; /* return success */
}
/*..........................................................................*/
void QF_stop(void)
{
    QF_onCleanup(); /* cleanup callback */
}
/*..........................................................................*/
static void thread_function(void *parameter)
{ /* RT-Thread signature */
    QActive *act = (QActive *)parameter;

    /* event-loop */
    for (;;)
    { /* for-ever */
        QEvt const *e = QActive_get_(act);
        QASM_DISPATCH(&act->super, e, act->prio);
        QF_gc(e); /* check if the event is garbage, and collect it if so */
    }
}
/*..........................................................................*/
void QActive_start_(QActive *const me, QPrioSpec const prioSpec, QEvt const **const qSto, uint_fast16_t const qLen, void *const stkSto,
                    uint_fast16_t const stkSize, void const *const par)
{
    /* allege that the RT-Thread queue is created successfully */
    Q_ASSERT_INCRIT(210, rt_mb_init(&me->eQueue, me->thread.name, (void *)qSto, (qLen), RT_IPC_FLAG_FIFO) == RT_EOK);

    me->prio  = (uint8_t)(prioSpec & 0xFFU); /* QF-priority */
    me->pthre = (uint8_t)(prioSpec >> 8U);   /* preemption-threshold */
    QActive_register_(me);                   /* register this AO */

    QASM_INIT(&me->super, par, me->prio); /* initial tran. (virtual) */
    QS_FLUSH();                           /* flush the trace buffer to the host */

    Q_ASSERT_INCRIT(220, rt_thread_init(&me->thread,              /* RT-Thread thread control block */
                                    me->thread.name,          /* unique thread name */
                                    &thread_function,         /* thread function */
                                    me,                       /* thread parameter */
                                    stkSto,                   /* stack start */
                                    stkSize,                  /* stack size in bytes */
                                    QF_MAX_ACTIVE - me->prio, /* RT-Thread priority */
                                    5)
                         == RT_EOK);

    rt_thread_startup(&me->thread);
}
/*..........................................................................*/
void QActive_setAttr(QActive *const me, uint32_t attr1, void const *attr2)
{
    /* this function must be called before QACTIVE_START(),
    */
    QF_CRIT_ENTRY();
    Q_REQUIRE_INCRIT(300, me->thread.name == (char *)0);
    switch (attr1)
    {
        case THREAD_NAME_ATTR:
            rt_memset(me->thread.name, 0x00, RT_NAME_MAX);
            rt_strncpy(me->thread.name, (char *)attr2, RT_NAME_MAX - 1);
            break;
            /* ... */
    }
    QF_CRIT_EXIT();
}
/*..........................................................................*/
bool QActive_post_(QActive *const me, QEvt const *const e, uint_fast16_t const margin, void const *const sender)
{
    uint_fast16_t nFree;
    bool status;

    QF_CRIT_ENTRY();
    nFree = (uint_fast16_t)(me->eQueue.size - me->eQueue.entry);

    if (margin == QF_NO_MARGIN)
    {
        if (nFree > 0U)
        {
            status = true; /* can post */
        }
        else
        {
            status = false;      /* cannot post */
            Q_ERROR_INCRIT(510); /* must be able to post the event */
        }
    }
    else if (nFree > (QEQueueCtr)margin)
    {
        status = true; /* can post */
    }
    else
    {
        status = false; /* cannot post */
    }

    if (status)
    { /* can post the event? */

        QS_BEGIN_PRE_(QS_QF_ACTIVE_POST, me->prio)
        QS_TIME_PRE_();                      /* timestamp */
        QS_OBJ_PRE_(sender);                 /* the sender object */
        QS_SIG_PRE_(e->sig);                 /* the signal of the event */
        QS_OBJ_PRE_(me);                     /* this active object (recipient) */
        QS_2U8_PRE_(QEvt_getPoolNum_(me), e->refCtr_); /* pool Id & ref Count */
        QS_EQC_PRE_(nFree);                  /* # free entries available */
        QS_EQC_PRE_(0U);                     /* min # free entries (unknown) */
        QS_END_PRE_()
       

        if (QEvt_getPoolNum_(e) != 0U)
        {                        /* is it a pool event? */
            QEvt_refCtr_inc_(e); /* increment the reference counter */
        }

        QF_CRIT_EXIT();

        /* posting to the RT-Thread message queue must succeed */
        Q_ASSERT_INCRIT(520, rt_mb_send(&me->eQueue, (rt_ubase_t)e) == RT_EOK);
    }
    else
    {

        QS_BEGIN_PRE_(QS_QF_ACTIVE_POST_ATTEMPT, me->prio)
        QS_TIME_PRE_();                      /* timestamp */
        QS_OBJ_PRE_(sender);                 /* the sender object */
        QS_SIG_PRE_(e->sig);                 /* the signal of the event */
        QS_OBJ_PRE_(me);                     /* this active object (recipient) */
        QS_2U8_PRE_(QEvt_getPoolNum_(me), e->refCtr_); /* pool Id & ref Count */
        QS_EQC_PRE_(nFree);                  /* # free entries available */
        QS_EQC_PRE_(0U);                     /* min # free entries (unknown) */
        QS_END_PRE_()

        QF_CRIT_EXIT();
    }

    return status;
}
/*..........................................................................*/
void QActive_postLIFO_(QActive *const me, QEvt const *const e)
{
    QF_CRIT_ENTRY();

    QS_BEGIN_PRE_(QS_QF_ACTIVE_POST_LIFO, me->prio)
    QS_TIME_PRE_();                                  /* timestamp */
    QS_SIG_PRE_(e->sig);                             /* the signal of this event */
    QS_OBJ_PRE_(me);                                 /* this active object */
    QS_2U8_PRE_(QEvt_getPoolNum_(e), e->refCtr_);             /* pool Id & ref Count */
    QS_EQC_PRE_(me->eQueue.size - me->eQueue.entry); /* # free */
    QS_EQC_PRE_(0U);                                 /* min # free entries (unknown) */
    QS_END_PRE_()

    if (QEvt_getPoolNum_(e) != 0U)
    {                        /* is it a pool event? */
        QEvt_refCtr_inc_(e); /* increment the reference counter */
    }
    
    QF_CRIT_EXIT();

    /* LIFO posting must succeed */
    Q_ASSERT_INCRIT(610, rt_mb_urgent(&me->eQueue, (rt_ubase_t)e) == RT_EOK);
}
/*..........................................................................*/
QEvt const *QActive_get_(QActive *const me)
{
    QEvt const *e;

    Q_ASSERT_INCRIT(710, rt_mb_recv(&me->eQueue, (rt_ubase_t *)&e, RT_WAITING_FOREVER) == RT_EOK);
  
    QS_BEGIN_PRE_(QS_QF_ACTIVE_GET, me->prio)
    QS_TIME_PRE_();                                  /* timestamp */
    QS_SIG_PRE_(e->sig);                             /* the signal of this event */
    QS_OBJ_PRE_(me);                                 /* this active object */
    QS_2U8_PRE_(e->poolId_, e->refCtr_);             /* pool Id & ref Count */
    QS_EQC_PRE_(me->eQueue.size - me->eQueue.entry); /* # free */
    QS_END_PRE_()

    return e;
}
