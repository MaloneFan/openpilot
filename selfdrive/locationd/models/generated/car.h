#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4673356681073551746);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5111845765447207624);
void car_H_mod_fun(double *state, double *out_2465835830106009632);
void car_f_fun(double *state, double dt, double *out_1423529977948406845);
void car_F_fun(double *state, double dt, double *out_2292801506792658940);
void car_h_25(double *state, double *unused, double *out_274171705799066031);
void car_H_25(double *state, double *unused, double *out_1362543829109268691);
void car_h_24(double *state, double *unused, double *out_8682175579901995671);
void car_H_24(double *state, double *unused, double *out_2195381769693054445);
void car_h_30(double *state, double *unused, double *out_3363277822454956319);
void car_H_30(double *state, double *unused, double *out_1155789129397979936);
void car_h_26(double *state, double *unused, double *out_5967683143263653970);
void car_H_26(double *state, double *unused, double *out_5104047147983324915);
void car_h_27(double *state, double *unused, double *out_2144573907268787995);
void car_H_27(double *state, double *unused, double *out_1018974182402444975);
void car_h_29(double *state, double *unused, double *out_8364951090115801964);
void car_H_29(double *state, double *unused, double *out_1666020473712372120);
void car_h_28(double *state, double *unused, double *out_8652824993129429279);
void car_H_28(double *state, double *unused, double *out_3416378543357158454);
void car_h_31(double *state, double *unused, double *out_6718782836820559302);
void car_H_31(double *state, double *unused, double *out_5730255250216676391);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}