/**
 * @file test\ABA_HLS.cpp
 * @brief 用于串联ABA代码的所有计算部分，输入仅有q，dq，tau，输出为ddq
 *
 *
 * @author YZJ
 * @date 2026-08-09
 * @version t0.1
 */

#include "ABA_fixed.h"

void ABA(
    const joint_pos_t q[DOF],
    const joint_vel_t dq[DOF],
    const joint_tau_t tau[DOF],
    data_t ddq[DOF]
)
{
#pragma HLS INTERFACE ap_ctrl_chain port=return
#pragma HLS ARRAY_PARTITION variable=q complete dim=1
#pragma HLS ARRAY_PARTITION variable=dq complete dim=1
#pragma HLS ARRAY_PARTITION variable=tau complete dim=1
#pragma HLS ARRAY_PARTITION variable=ddq complete dim=1

    // Resource-balanced throughput target. II=12 allows fixed-point operators
    // to be shared instead of constructing a complete II=1 arithmetic graph.
#pragma HLS PIPELINE II=12

    // The former fmul/fadd/fdiv limits no longer apply: there are no
    // floating-point operations in this datapath.

    // 定义输出数组
    kinematic_t v[DOF][6];

    kinematic_t c[DOF][6];

    force_t h[DOF][6];
    force_t p[DOF][6];

    inertia_t U[DOF][6];
    inverse_t inv_D[DOF];
    transform_t E[DOF][3][3];
    transform_t F[DOF][3][3];
    force_t u[DOF];

#pragma HLS ARRAY_PARTITION variable=v complete dim=0
#pragma HLS ARRAY_PARTITION variable=c complete dim=0
#pragma HLS ARRAY_PARTITION variable=h complete dim=0
#pragma HLS ARRAY_PARTITION variable=p complete dim=0
#pragma HLS ARRAY_PARTITION variable=U complete dim=0
#pragma HLS ARRAY_PARTITION variable=inv_D complete dim=1
#pragma HLS ARRAY_PARTITION variable=E complete dim=0
#pragma HLS ARRAY_PARTITION variable=F complete dim=0
#pragma HLS ARRAY_PARTITION variable=u complete dim=1

    // 调用函数
    // 计算v
    v_fina(q, dq, E, F, v);

    // 计算c
    c_fina(v,dq,c);

    // 计算h
    // I_space(mass,c_of_mass,I,I_spa);
    h_fina(I_spa,v,h);
    p_fina(v,h,p);

    // 逆推
    back_pass(I_spa,p,tau,c,E,F,U,inv_D,u);

    // 正推
    tip_pass3(E, F, c, inv_D, u, U, ddq);
}
