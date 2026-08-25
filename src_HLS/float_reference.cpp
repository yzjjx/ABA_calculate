// Host-only validation adapter. It compiles the untouched src implementation
// under distinct symbol names so fixed_accuracy_test can compare both models
// on exactly the same inputs. This file is not part of the HLS source set.
#define ABA ABA_float
#define cal_R_Trans cal_R_Trans_float
#define v_fina v_fina_float
#define v_mat_cal v_mat_cal_float
#define f_mat_cal f_mat_cal_float
#define transform_inertia_ef transform_inertia_ef_float
#define space_cross space_cross_float
#define c_fina c_fina_float
#define h_fina h_fina_float
#define f_space_cross f_space_cross_float
#define p_fina p_fina_float
#define back_pass back_pass_float
#define tip_pass3 tip_pass3_float

#define PI PI_float
#define alpha alpha_float
#define alpha_cos alpha_cos_float
#define alpha_sin alpha_sin_float
#define P_const P_const_float
#define a a_float
#define d d_float
#define I I_float
#define mass mass_float
#define c_of_mass c_of_mass_float
#define I_spa I_spa_float

#include "../src/T_R_out.cpp"
#include "../src/v_ori.cpp"
#include "../src/c.cpp"
#include "../src/cal_p.cpp"
#include "../src/back.cpp"
#include "../src/pass3.cpp"
#include "../src/ABA_parms.cpp"
#include "../src/block_mat.cpp"
#include "../src/ABA_HLS.cpp"
