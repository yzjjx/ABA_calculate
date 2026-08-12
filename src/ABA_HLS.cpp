/**
 * @file test\ABA_HLS.cpp
 * @brief 用于串联ABA代码的所有计算部分,输入仅有q，dq，tau，输出为ddq
 *
 *
 * @author YZJ
 * @date 2026-08-09
 * @version t0.1
 */

#include <ABA_parms.h>
#include <ABA_HLS.h>

void ABA(
    const data_t q[DOF],
    const data_t dq[DOF],
    const data_t tau[DOF],
    data_t ddq[DOF]
)
{
    // 定义输出数组
    data_t v[DOF][6];

    data_t c[DOF][6];

    data_t h[DOF][6];
    data_t p[DOF][6];

    data_t U[DOF][6];
    data_t inv_D[DOF];
    data_t E[DOF][3][3];
    data_t F[DOF][3][3];
    data_t u[DOF];

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