#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8430055075784696659);
void live_err_fun(double *nom_x, double *delta_x, double *out_3254368280690439304);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8788489577648386071);
void live_H_mod_fun(double *state, double *out_2792624268178059430);
void live_f_fun(double *state, double dt, double *out_5459361766403996839);
void live_F_fun(double *state, double dt, double *out_4154060615378714711);
void live_h_4(double *state, double *unused, double *out_2212251112735887867);
void live_H_4(double *state, double *unused, double *out_3397264096299453329);
void live_h_9(double *state, double *unused, double *out_820401519696436064);
void live_H_9(double *state, double *unused, double *out_3889954838964994141);
void live_h_10(double *state, double *unused, double *out_507373885694143139);
void live_H_10(double *state, double *unused, double *out_491213846029255995);
void live_h_12(double *state, double *unused, double *out_3794464210217895999);
void live_H_12(double *state, double *unused, double *out_1622192311732508466);
void live_h_35(double *state, double *unused, double *out_4994379186538344813);
void live_H_35(double *state, double *unused, double *out_30602038926845953);
void live_h_32(double *state, double *unused, double *out_6498826032295832395);
void live_H_32(double *state, double *unused, double *out_3485115101030013887);
void live_h_13(double *state, double *unused, double *out_2161486960338920047);
void live_H_13(double *state, double *unused, double *out_2943230005699876328);
void live_h_14(double *state, double *unused, double *out_820401519696436064);
void live_H_14(double *state, double *unused, double *out_3889954838964994141);
void live_h_33(double *state, double *unused, double *out_651685544947035238);
void live_H_33(double *state, double *unused, double *out_3119954965712011651);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}