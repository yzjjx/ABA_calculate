#ifndef ABA_FIXED_H
#define ABA_FIXED_H

#include <ap_fixed.h>

// Central fixed-point configuration for the complete HLS datapath.
//
// data_t range (default): [-32768, 32767.9999847412109375]
// data_t resolution:      2^-16 = 0.0000152587890625
//
// AP_RND_CONV avoids the systematic bias of truncation. AP_SAT turns an
// unexpected range violation into a bounded value instead of wrap-around.
// The width macros make software accuracy sweeps possible without editing the
// algorithm. Keep DATA_W at 32 for a 32-bit AXI memory word.
#ifndef ABA_DATA_W
#define ABA_DATA_W 32
#endif

#ifndef ABA_DATA_I
#define ABA_DATA_I 16
#endif

typedef ap_fixed<ABA_DATA_W, ABA_DATA_I, AP_RND_CONV, AP_SAT> data_t;

// All external buffers remain 32 bits wide. Their binary-point positions are
// matched to the deployment ranges q=[-pi,pi], dq=[-2,2], tau=[-10,10], and
// ddq approximately [-2.3e4,2.3e4].
typedef ap_fixed<32, 4, AP_RND_CONV, AP_SAT> joint_pos_t;
typedef ap_fixed<32, 4, AP_RND_CONV, AP_SAT> joint_vel_t;
typedef ap_fixed<32, 8, AP_RND_CONV, AP_SAT> joint_tau_t;

// Range-driven internal formats. Keeping narrow, domain-specific operands is
// materially cheaper than using one 36+ bit format everywhere, while the
// 24 fractional bits in inertia_t preserve the robot's smallest coefficients.
typedef ap_fixed<22, 2, AP_RND_CONV, AP_SAT> transform_t;
typedef ap_fixed<28, 8, AP_RND_CONV, AP_SAT> kinematic_t;
typedef ap_fixed<32, 8, AP_RND_CONV, AP_SAT> inertia_t;
typedef ap_fixed<32, 12, AP_RND_CONV, AP_SAT> force_t;
typedef ap_fixed<34, 14, AP_RND_CONV, AP_SAT> inverse_t;

// q is restricted to [-pi, pi] by the deployment input contract. A dedicated
// angle type retains 20 fractional bits for the fixed-point CORDIC, while the
// sine/cosine result retains 20 fractional bits in [-1, 1].
typedef ap_fixed<24, 4> angle_t;
typedef ap_fixed<22, 2> trig_t;

constexpr int DOF = 6;
constexpr int ABA_BATCH_SIZE = 1000;

extern const data_t alpha[DOF];
extern const data_t alpha_cos[DOF];
extern const data_t alpha_sin[DOF];
extern const transform_t P_const[DOF][3];
extern const data_t a[DOF];
extern const data_t d[DOF];
extern const data_t I[DOF][9];
extern const data_t mass[DOF];
extern const data_t c_of_mass[DOF][3];
extern const inertia_t I_spa[DOF][6][6];

void cal_R_Trans(
    const joint_pos_t q[DOF],
    transform_t R_Trans[DOF][3][3]);

void v_mat_cal_kinematic(
    const transform_t E[3][3],
    const transform_t F[3][3],
    const kinematic_t in[6],
    kinematic_t out[6]);

void v_mat_cal_acceleration(
    const transform_t E[3][3],
    const transform_t F[3][3],
    const data_t in[6],
    data_t out[6]);

void f_mat_cal(
    const transform_t E[3][3],
    const transform_t F[3][3],
    const force_t in[6],
    force_t out[6]);

void transform_inertia_ef(
    const transform_t E[3][3],
    const transform_t F[3][3],
    const inertia_t Ia[6][6],
    inertia_t result[6][6]);

void v_fina(
    const joint_pos_t q[DOF],
    const joint_vel_t dq[DOF],
    transform_t E[DOF][3][3],
    transform_t F[DOF][3][3],
    kinematic_t v[DOF][6]);

void space_cross(
    const kinematic_t in_1[6],
    const kinematic_t in_2[6],
    kinematic_t out[6]);

void c_fina(
    const kinematic_t v[DOF][6],
    const joint_vel_t dq[DOF],
    kinematic_t c[DOF][6]);

void h_fina(
    const inertia_t I_spa_in[DOF][6][6],
    const kinematic_t v[DOF][6],
    force_t h[DOF][6]);

void f_space_cross(
    const kinematic_t in_1[6],
    const force_t in_2[6],
    force_t out[6]);

void p_fina(
    const kinematic_t v[DOF][6],
    const force_t h[DOF][6],
    force_t p[DOF][6]);

void back_pass(
    const inertia_t I_spa_in[DOF][6][6],
    const force_t p[DOF][6],
    const joint_tau_t tau[DOF],
    const kinematic_t c[DOF][6],
    const transform_t E[DOF][3][3],
    const transform_t F[DOF][3][3],
    inertia_t U[DOF][6],
    inverse_t inv_D[DOF],
    force_t u[DOF]);

void tip_pass3(
    const transform_t E[DOF][3][3],
    const transform_t F[DOF][3][3],
    const kinematic_t c[DOF][6],
    const inverse_t inv_D[DOF],
    const force_t u[DOF],
    const inertia_t U[DOF][6],
    data_t ddq[DOF]);

void ABA(
    const joint_pos_t q[DOF],
    const joint_vel_t dq[DOF],
    const joint_tau_t tau[DOF],
    data_t ddq[DOF]);

void ABA_batch(
    const joint_pos_t q[ABA_BATCH_SIZE][DOF],
    const joint_vel_t dq[ABA_BATCH_SIZE][DOF],
    const joint_tau_t tau[ABA_BATCH_SIZE][DOF],
    data_t ddq[ABA_BATCH_SIZE][DOF]);

void I_space(
    const data_t mass_in[DOF],
    const data_t c_of_mass_in[DOF][3],
    const data_t inertia[DOF][9],
    inertia_t spatial_inertia[DOF][6][6]);

#endif
