//$file${.::blinky.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: blinky.qm
// File:  ${.::blinky.c}
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
//$endhead${.::blinky.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include <rtthread.h>

#ifdef QPC_USING_BLINKY_EXAMPLE
#ifdef RT_USING_FINSH

#include <finsh.h>
#include "qpc.h"
Q_DEFINE_THIS_FILE

#define LOG_TAG "blinky"
#define DBG_SECTION_NAME "blinky"
#define LOG_LVL DBG_LOG
#include <rtdbg.h>

enum { BSP_TICKS_PER_SEC = 1000 };

void BSP_ledOff(void) {
    LOG_I("LED OFF\n");
}

void BSP_ledOn(void) {
    LOG_I("LED ON\n");
}

enum BlinkySignals {
    TIMEOUT_SIG = Q_USER_SIG,
    MAX_SIG
};

//*============== ask QM to declare the Blinky class ================*/
//$declare${AOs::Blinky} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::Blinky} .............................................................
typedef struct Blinky {
// protected:
    QActive super;

// public:

// private:
    QTimeEvt timeEvt;
} Blinky;

extern Blinky Blinky_inst;

// protected:
static QState Blinky_initial(Blinky * const me, void const * const par);
static QState Blinky_off(Blinky * const me, QEvt const * const e);
static QState Blinky_on(Blinky * const me, QEvt const * const e);
//$enddecl${AOs::Blinky} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

QActive * const AO_Blinky = &Blinky_inst.super;

static void Blinky_ctor(void) {
    Blinky *me = &Blinky_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&Blinky_initial));
    QTimeEvt_ctorX(&me->timeEvt, &me->super, TIMEOUT_SIG, 0U);
}

int qpc_blinky_start(void) {
    /* statically allocate event queue buffer for the Blinky AO */
    static QEvt const *blinky_queueSto[10];
    static uint8_t blinky_stack[1024];

    QF_init(); /* initialize the framework */

    Blinky_ctor(); /* explicitly call the "constructor" */
    QACTIVE_START(AO_Blinky,
                  1U, /* priority */
                  blinky_queueSto, Q_DIM(blinky_queueSto),
                  (void *)blinky_stack, sizeof(blinky_stack), /* no stack */
                  (void *)0);    /* no initialization parameter */
    return QF_run(); /* run the QF application */
}

MSH_CMD_EXPORT(qpc_blinky_start, start qpc blinky example);

/*================ ask QM to define the Blinky class ================*/
//$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Check for the minimum required QP version
#if (QP_VERSION < 730U) || (QP_VERSION != ((QP_RELEASE^4294967295U)%0x2710U))
#error qpc version 7.3.0 or higher required
#endif
//$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${AOs::Blinky} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::Blinky} .............................................................
Blinky Blinky_inst;

//${AOs::Blinky::SM} .........................................................
static QState Blinky_initial(Blinky * const me, void const * const par) {
    //${AOs::Blinky::SM::initial}
    (void)par; /* unused parameter */
    QTimeEvt_armX(&me->timeEvt,
    BSP_TICKS_PER_SEC/2, BSP_TICKS_PER_SEC/2);

    QS_FUN_DICTIONARY(&Blinky_off);
    QS_FUN_DICTIONARY(&Blinky_on);

    return Q_TRAN(&Blinky_off);
}

//${AOs::Blinky::SM::off} ....................................................
static QState Blinky_off(Blinky * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${AOs::Blinky::SM::off}
        case Q_ENTRY_SIG: {
            BSP_ledOff();
            status_ = Q_HANDLED();
            break;
        }
        //${AOs::Blinky::SM::off::TIMEOUT}
        case TIMEOUT_SIG: {
            status_ = Q_TRAN(&Blinky_on);
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}

//${AOs::Blinky::SM::on} .....................................................
static QState Blinky_on(Blinky * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${AOs::Blinky::SM::on}
        case Q_ENTRY_SIG: {
            BSP_ledOn();
            status_ = Q_HANDLED();
            break;
        }
        //${AOs::Blinky::SM::on::TIMEOUT}
        case TIMEOUT_SIG: {
            status_ = Q_TRAN(&Blinky_off);
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}
//$enddef${AOs::Blinky} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#endif /* RT_USING_FINSH */
#endif /* QPC_USING_BLINKY_EXAMPLE */

