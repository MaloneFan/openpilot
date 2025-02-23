#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_155765044166347563);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5147265498242855152);
void gnss_H_mod_fun(double *state, double *out_2441002244825976478);
void gnss_f_fun(double *state, double dt, double *out_5834456982122438687);
void gnss_F_fun(double *state, double dt, double *out_6970338126122875519);
void gnss_h_6(double *state, double *sat_pos, double *out_2137796404954608038);
void gnss_H_6(double *state, double *sat_pos, double *out_6474332688499420280);
void gnss_h_20(double *state, double *sat_pos, double *out_3064986068187853552);
void gnss_H_20(double *state, double *sat_pos, double *out_1015241364259705761);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6705706645024539856);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1489560582702204123);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6705706645024539856);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1489560582702204123);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}