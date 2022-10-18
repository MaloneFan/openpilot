#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2312207852105212569);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3492233489560761085);
void gnss_H_mod_fun(double *state, double *out_6464982121424259234);
void gnss_f_fun(double *state, double dt, double *out_4539264253469581544);
void gnss_F_fun(double *state, double dt, double *out_8063886126610994275);
void gnss_h_6(double *state, double *sat_pos, double *out_6421619825315491896);
void gnss_H_6(double *state, double *sat_pos, double *out_1129091903658590088);
void gnss_h_20(double *state, double *sat_pos, double *out_5307285830992173381);
void gnss_H_20(double *state, double *sat_pos, double *out_8963679878168358303);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3350868428259612938);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_738916608976085322);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3350868428259612938);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_738916608976085322);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}