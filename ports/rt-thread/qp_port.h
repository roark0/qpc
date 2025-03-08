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
* @date Last updated on: 2022-06-12
* @version Last updated for: @ref qpc_7_0_1
*
* @file
* @brief QF/C, port to RT-Thread
*/
#ifndef QF_PORT_H
#define QF_PORT_H

#include <rtthread.h>   /* RT-Thread API */

/* RT-Thread event queue and thread types */
// #define QF_EQUEUE_TYPE      struct rt_mailbox
#define QACTIVE_EQUEUE_TYPE struct rt_mailbox
#define QACTIVE_THREAD_TYPE struct rt_thread

/* The maximum number of active objects in the application, see NOTE2 */
#define QF_MAX_ACTIVE       (RT_THREAD_PRIORITY_MAX)

/* QF critical section for RT-Thread, see NOTE3 */
#define QF_CRIT_ENTRY() (rt_enter_critical())
#define QF_CRIT_EXIT()   (rt_exit_critical())

#define Q_NORETURN void

enum RT_Thread_ThreadAttrs {
    THREAD_NAME_ATTR
};

#include "qep_port.h" /* QEP port */
#include "qequeue.h"  /* native QF event queue for deferring events */
#include "qmpool.h"   /* native QF event pool */
#include "qp.h"       /* QF platform-independent public interface */
/*****************************************************************************
* interface used only inside QF, but not in applications
*/
#ifdef QP_IMPL

    #define QF_SCHED_STAT_
    #define QF_SCHED_LOCK_(prio_)   rt_enter_critical()
    #define QF_SCHED_UNLOCK_()      rt_exit_critical()

    /* native QF event pool operations */
    #define QF_EPOOL_TYPE_            QMPool
    #define QF_EPOOL_INIT_(p_, poolSto_, poolSize_, evtSize_) \
        (QMPool_init(&(p_), (poolSto_), (poolSize_), (evtSize_)))
    #define QF_EPOOL_EVENT_SIZE_(p_)  ((uint_fast16_t)(p_).blockSize)
    #define QF_EPOOL_GET_(p_, e_, m_, qs_id_) \
        ((e_) = (QEvt *)QMPool_get(&(p_), (m_), (qs_id_)))
    #define QF_EPOOL_PUT_(p_, e_, qs_id_) \
        (QMPool_put(&(p_), (e_), (qs_id_)))

#endif /* ifdef QP_IMPL */

#endif /* QF_PORT_H */

