#ifndef ABA_PARMS_H
#define ABA_PARMS_H

#include <T_R_out.h>
#include <v_ori.h>
#include <c.h>
#include <cal_p.h>
#include <back.h>
#include <pass3.h>


extern const data_t alpha[DOF];

extern const data_t alpha_cos[DOF];

extern const data_t alpha_sin[DOF];

extern const data_t P_const[DOF][3];

extern const data_t a[DOF];

extern const data_t d[DOF];

extern const data_t I[DOF][9];

extern const data_t mass[DOF];

extern const data_t c_of_mass[DOF][3];

extern const data_t I_spa[DOF][6][6];

#endif