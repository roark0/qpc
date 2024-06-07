//$file${include::qpc.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${include::qpc.h}
//
// This code has been generated by QM 6.1.1 <www.state-machine.com/qm>.
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// This code is covered by the following QP license:
// License #    : LicenseRef-QL-dual
// Issued to    : Any user of the QP/C real-time embedded framework
// Framework(s) : qpc
// Support ends : 2024-12-31
// License scope:
//
// Copyright (C) 2005 Quantum Leaps, LLC <state-machine.com>.
//
//                    Q u a n t u m  L e a P s
//                    ------------------------
//                    Modern Embedded Software
//
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-QL-commercial
//
// This software is dual-licensed under the terms of the open source GNU
// General Public License version 3 (or any later version), or alternatively,
// under the terms of one of the closed source Quantum Leaps commercial
// licenses.
//
// The terms of the open source GNU General Public License version 3
// can be found at: <www.gnu.org/licenses/gpl-3.0>
//
// The terms of the closed source Quantum Leaps commercial licenses
// can be found at: <www.state-machine.com/licensing>
//
// Redistributions in source code must retain this top-level comment block.
// Plagiarizing this software to sidestep the license obligations is illegal.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//
//$endhead${include::qpc.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef QPC_H_
#define QPC_H_

//============================================================================
#include "qp_port.h"      // QP port from the port directory
#include "qsafe.h"        // QP Functional Safety (FuSa) Subsystem
#ifdef Q_SPY // software tracing enabled?
    #include "qs_port.h"  // QS/C port from the port directory
#else
    #include "qs_dummy.h" // QS/C dummy (inactive) interface
#endif

//============================================================================
#ifndef QP_API_VERSION

#define QP_API_VERSION 0

#endif // #ifndef QP_API_VERSION

//============================================================================
// QP API compatibility layer...

#if (QP_API_VERSION < 740)

#ifdef QEVT_DYN_CTOR
//! @deprecated #QEVT_DYN_CTOR, please use #QEVT_PAR_INIT
#define QEVT_PAR_INIT
#endif

//! @deprecated plain 'char' is no longer forbidden in MISRA-C:2023
typedef char char_t;

//! @deprecated assertion failure handler
//! Use Q_onError() instead.
#define Q_onAssert(module_, id_) Q_onError(module_, id_)

//! @deprecated #Q_NASSERT preprocessor switch to disable QP assertions
#ifdef Q_NASSERT

    // #Q_UNSAFE now replaces the functionality of Q_NASSERT
    #define Q_UNSAFE

    //! @deprecated general purpose assertion with user-specified ID
    //! number that **always** evaluates the `expr_` expression.
    #define Q_ALLEGE_ID(id_, expr_) ((void)(expr_))

#else // QP FuSa Subsystem enabled

    //! @deprecated general purpose assertion with user-specified ID
    //! number that **always** evaluates the `expr_` expression.
    //! @note
    //! The use of this macro is no longer recommended.
    #define Q_ALLEGE_ID(id_, expr_) if (!(expr_)) { \
        QF_CRIT_STAT \
        QF_CRIT_ENTRY(); \
        Q_onError(&Q_this_module_[0], (id_)); \
        QF_CRIT_EXIT(); \
    } else ((void)0)

#endif

//! @deprecated general purpose assertion without ID number
//! that **always** evaluates the `expr_` expression.
//! Instead of ID number, this macro is based on the standard
//! `__LINE__` macro.
//!
//! @note The use of this macro is no longer recommended.
#define Q_ALLEGE(expr_)         Q_ALLEGE_ID(__LINE__, (expr_))

//! Static (compile-time) assertion.
//! @deprecated
//! Use Q_ASSERT_STATIC() or better yet `_Static_assert()` instead.
#define Q_ASSERT_COMPILE(expr_) Q_ASSERT_STATIC(expr_)

//! @static @public @memberof QF
//! @deprecated
static inline void QF_psInit(
    QSubscrList * const subscrSto,
    enum_t const maxSignal)
{
    QActive_psInit(subscrSto, maxSignal);
}

//! @deprecated instead use: QASM_INIT()
#define QHSM_INIT(me_, par_, qsId_)   QASM_INIT((me_), (par_), (qsId_))

//! @deprecated instead use: QASM_DISPATCH()
#define QHSM_DISPATCH(me_, e_, qsId_) QASM_DISPATCH((me_), (e_), (qsId_))

//! @deprecated instead use: QASM_IS_IN()
#define QHsm_isIn(me_, state_)         QHsm_isIn_((QAsm *)(me_), (state_))

//============================================================================
#if (QP_API_VERSION < 691)

//! @deprecated enable the QS global filter
#define QS_FILTER_ON(rec_)        QS_GLB_FILTER((rec_))

//! @deprecated disable the QS global filter
#define QS_FILTER_OFF(rec_)       QS_GLB_FILTER(-(rec_))

//! @deprecated enable the QS local filter for SM (state machine) object
#define QS_FILTER_SM_OBJ(obj_)    ((void)0)

//! @deprecated enable the QS local filter for AO (active objects)
#define QS_FILTER_AO_OBJ(obj_)    ((void)0)

//! @deprecated enable the QS local filter for MP (memory pool) object
#define QS_FILTER_MP_OBJ(obj_)    ((void)0)

//! @deprecated enable the QS local filter for EQ (event queue) object
#define QS_FILTER_EQ_OBJ(obj_)    ((void)0)

//! @deprecated enable the QS local filter for TE (time event) object
#define QS_FILTER_TE_OBJ(obj_)    ((void)0)

#ifdef Q_SPY

//! @deprecated local Filter for a generic application object `obj_`.
#define QS_FILTER_AP_OBJ(obj_)  (QS_filt_.loc_AP = (obj_))

//! @deprecated begin of a user QS record, instead use QS_BEGIN_ID()
#define QS_BEGIN(rec_, obj_)                            \
    if (((QS_filt_.glb[(uint_fast8_t)(rec_) >> 3U]      \
          & (1U << ((uint_fast8_t)(rec_) & 7U))) != 0U) \
        && ((QS_priv_.locFilter_AP == (void *)0)        \
            || (QS_priv_.locFilter_AP == (obj_))))      \
    {                                                   \
        QS_CRIT_STAT                                    \
        QS_CRIT_ENTRY();                                \
        QS_beginRec_((uint_fast8_t)(rec_));             \
        QS_TIME_PRE_(); {

//! @deprecated Output formatted uint32_t to the QS record
#define QS_U32_HEX(width_, data_) \
    (QS_u32_fmt_((uint8_t)(((width_) << 4)) | QS_HEX_FMT, (data_)))

#else

#define QS_FILTER_AP_OBJ(obj_)    ((void)0)
#define QS_BEGIN(rec_, obj_)      if (false) {
#define QS_U32_HEX(width_, data_) ((void)0)

#endif

//============================================================================
#if (QP_API_VERSION < 660)

//! @deprecated casting to QXThreadHandler
//! instead use: the new signature of QXThreadHandler and don't cast
#define Q_XTHREAD_CAST(handler_) ((QXThreadHandler)(handler_))

//============================================================================
#if (QP_API_VERSION < 580)

//! @deprecated instead use: QASM_INIT()
#define QMSM_INIT(me_, par_, qsId_)   QASM_INIT((me_), (par_), (qsId_))

//! @deprecated instead use: QASM_DISPATCH()
#define QMSM_DISPATCH(me_, e_, qsId_) QASM_DISPATCH((me_), (e_), (qsId_))

#endif // QP_API_VERSION < 580
#endif // QP_API_VERSION < 660
#endif // QP_API_VERSION < 691
#endif // QP_API_VERSION < 700

#endif // QPC_H_
