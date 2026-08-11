/**
 * @file test\ABA_HLS.cpp
 * @brief 用于串联ABA代码的所有计算部�?,输入仅有q，dq，tau，输出为ddq
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
    data_t v_J[6][DOF];

    data_t X_lam[DOF][6][6];
    data_t v[DOF][6];

    data_t c[DOF][6];

    data_t h[DOF][6];
    data_t p[DOF][6];

    data_t I_A[DOF][6][6];
    data_t p_A[DOF][6];
    data_t U[DOF][6];
    data_t D[DOF];
    data_t u[DOF];
    data_t I_a[DOF][6][6];
    data_t p_a[DOF][6];

    data_t a_s[DOF][6];
    data_t a_a[DOF][6];

    // 调用函数
    // 计算v
    v_fina(q,dq,X_lam,v);

    // 计算c
    c_fina(v,dq,c);

    // 计算h
    // I_space(mass,c_of_mass,I,I_spa);
    h_fina(I_spa,v,h);
    p_fina(v,h,p);

    // 逆推
    back_pass(I_spa,p,tau,c,X_lam,I_A,p_A,U,D,u,I_a,p_a);

    // 正推
    tip_pass3(X_lam,c,D,u,U,ddq);
}