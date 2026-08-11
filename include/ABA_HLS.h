#ifndef ABA_HLS_H
#define ABA_HLS_H

#include <T_R_out.h>
#include <v_ori.h>
#include <c.h>
#include <cal_p.h>
#include <back.h>
#include <pass3.h>

void ABA(
    const data_t q[DOF],
    const data_t dq[DOF],
    const data_t tau[DOF],
    data_t ddq[DOF]
);

#endif