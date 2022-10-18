#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4673356681073551746) {
   out_4673356681073551746[0] = delta_x[0] + nom_x[0];
   out_4673356681073551746[1] = delta_x[1] + nom_x[1];
   out_4673356681073551746[2] = delta_x[2] + nom_x[2];
   out_4673356681073551746[3] = delta_x[3] + nom_x[3];
   out_4673356681073551746[4] = delta_x[4] + nom_x[4];
   out_4673356681073551746[5] = delta_x[5] + nom_x[5];
   out_4673356681073551746[6] = delta_x[6] + nom_x[6];
   out_4673356681073551746[7] = delta_x[7] + nom_x[7];
   out_4673356681073551746[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5111845765447207624) {
   out_5111845765447207624[0] = -nom_x[0] + true_x[0];
   out_5111845765447207624[1] = -nom_x[1] + true_x[1];
   out_5111845765447207624[2] = -nom_x[2] + true_x[2];
   out_5111845765447207624[3] = -nom_x[3] + true_x[3];
   out_5111845765447207624[4] = -nom_x[4] + true_x[4];
   out_5111845765447207624[5] = -nom_x[5] + true_x[5];
   out_5111845765447207624[6] = -nom_x[6] + true_x[6];
   out_5111845765447207624[7] = -nom_x[7] + true_x[7];
   out_5111845765447207624[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2465835830106009632) {
   out_2465835830106009632[0] = 1.0;
   out_2465835830106009632[1] = 0;
   out_2465835830106009632[2] = 0;
   out_2465835830106009632[3] = 0;
   out_2465835830106009632[4] = 0;
   out_2465835830106009632[5] = 0;
   out_2465835830106009632[6] = 0;
   out_2465835830106009632[7] = 0;
   out_2465835830106009632[8] = 0;
   out_2465835830106009632[9] = 0;
   out_2465835830106009632[10] = 1.0;
   out_2465835830106009632[11] = 0;
   out_2465835830106009632[12] = 0;
   out_2465835830106009632[13] = 0;
   out_2465835830106009632[14] = 0;
   out_2465835830106009632[15] = 0;
   out_2465835830106009632[16] = 0;
   out_2465835830106009632[17] = 0;
   out_2465835830106009632[18] = 0;
   out_2465835830106009632[19] = 0;
   out_2465835830106009632[20] = 1.0;
   out_2465835830106009632[21] = 0;
   out_2465835830106009632[22] = 0;
   out_2465835830106009632[23] = 0;
   out_2465835830106009632[24] = 0;
   out_2465835830106009632[25] = 0;
   out_2465835830106009632[26] = 0;
   out_2465835830106009632[27] = 0;
   out_2465835830106009632[28] = 0;
   out_2465835830106009632[29] = 0;
   out_2465835830106009632[30] = 1.0;
   out_2465835830106009632[31] = 0;
   out_2465835830106009632[32] = 0;
   out_2465835830106009632[33] = 0;
   out_2465835830106009632[34] = 0;
   out_2465835830106009632[35] = 0;
   out_2465835830106009632[36] = 0;
   out_2465835830106009632[37] = 0;
   out_2465835830106009632[38] = 0;
   out_2465835830106009632[39] = 0;
   out_2465835830106009632[40] = 1.0;
   out_2465835830106009632[41] = 0;
   out_2465835830106009632[42] = 0;
   out_2465835830106009632[43] = 0;
   out_2465835830106009632[44] = 0;
   out_2465835830106009632[45] = 0;
   out_2465835830106009632[46] = 0;
   out_2465835830106009632[47] = 0;
   out_2465835830106009632[48] = 0;
   out_2465835830106009632[49] = 0;
   out_2465835830106009632[50] = 1.0;
   out_2465835830106009632[51] = 0;
   out_2465835830106009632[52] = 0;
   out_2465835830106009632[53] = 0;
   out_2465835830106009632[54] = 0;
   out_2465835830106009632[55] = 0;
   out_2465835830106009632[56] = 0;
   out_2465835830106009632[57] = 0;
   out_2465835830106009632[58] = 0;
   out_2465835830106009632[59] = 0;
   out_2465835830106009632[60] = 1.0;
   out_2465835830106009632[61] = 0;
   out_2465835830106009632[62] = 0;
   out_2465835830106009632[63] = 0;
   out_2465835830106009632[64] = 0;
   out_2465835830106009632[65] = 0;
   out_2465835830106009632[66] = 0;
   out_2465835830106009632[67] = 0;
   out_2465835830106009632[68] = 0;
   out_2465835830106009632[69] = 0;
   out_2465835830106009632[70] = 1.0;
   out_2465835830106009632[71] = 0;
   out_2465835830106009632[72] = 0;
   out_2465835830106009632[73] = 0;
   out_2465835830106009632[74] = 0;
   out_2465835830106009632[75] = 0;
   out_2465835830106009632[76] = 0;
   out_2465835830106009632[77] = 0;
   out_2465835830106009632[78] = 0;
   out_2465835830106009632[79] = 0;
   out_2465835830106009632[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1423529977948406845) {
   out_1423529977948406845[0] = state[0];
   out_1423529977948406845[1] = state[1];
   out_1423529977948406845[2] = state[2];
   out_1423529977948406845[3] = state[3];
   out_1423529977948406845[4] = state[4];
   out_1423529977948406845[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1423529977948406845[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1423529977948406845[7] = state[7];
   out_1423529977948406845[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2292801506792658940) {
   out_2292801506792658940[0] = 1;
   out_2292801506792658940[1] = 0;
   out_2292801506792658940[2] = 0;
   out_2292801506792658940[3] = 0;
   out_2292801506792658940[4] = 0;
   out_2292801506792658940[5] = 0;
   out_2292801506792658940[6] = 0;
   out_2292801506792658940[7] = 0;
   out_2292801506792658940[8] = 0;
   out_2292801506792658940[9] = 0;
   out_2292801506792658940[10] = 1;
   out_2292801506792658940[11] = 0;
   out_2292801506792658940[12] = 0;
   out_2292801506792658940[13] = 0;
   out_2292801506792658940[14] = 0;
   out_2292801506792658940[15] = 0;
   out_2292801506792658940[16] = 0;
   out_2292801506792658940[17] = 0;
   out_2292801506792658940[18] = 0;
   out_2292801506792658940[19] = 0;
   out_2292801506792658940[20] = 1;
   out_2292801506792658940[21] = 0;
   out_2292801506792658940[22] = 0;
   out_2292801506792658940[23] = 0;
   out_2292801506792658940[24] = 0;
   out_2292801506792658940[25] = 0;
   out_2292801506792658940[26] = 0;
   out_2292801506792658940[27] = 0;
   out_2292801506792658940[28] = 0;
   out_2292801506792658940[29] = 0;
   out_2292801506792658940[30] = 1;
   out_2292801506792658940[31] = 0;
   out_2292801506792658940[32] = 0;
   out_2292801506792658940[33] = 0;
   out_2292801506792658940[34] = 0;
   out_2292801506792658940[35] = 0;
   out_2292801506792658940[36] = 0;
   out_2292801506792658940[37] = 0;
   out_2292801506792658940[38] = 0;
   out_2292801506792658940[39] = 0;
   out_2292801506792658940[40] = 1;
   out_2292801506792658940[41] = 0;
   out_2292801506792658940[42] = 0;
   out_2292801506792658940[43] = 0;
   out_2292801506792658940[44] = 0;
   out_2292801506792658940[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2292801506792658940[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2292801506792658940[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2292801506792658940[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2292801506792658940[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2292801506792658940[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2292801506792658940[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2292801506792658940[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2292801506792658940[53] = -9.8000000000000007*dt;
   out_2292801506792658940[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2292801506792658940[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2292801506792658940[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2292801506792658940[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2292801506792658940[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2292801506792658940[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2292801506792658940[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2292801506792658940[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2292801506792658940[62] = 0;
   out_2292801506792658940[63] = 0;
   out_2292801506792658940[64] = 0;
   out_2292801506792658940[65] = 0;
   out_2292801506792658940[66] = 0;
   out_2292801506792658940[67] = 0;
   out_2292801506792658940[68] = 0;
   out_2292801506792658940[69] = 0;
   out_2292801506792658940[70] = 1;
   out_2292801506792658940[71] = 0;
   out_2292801506792658940[72] = 0;
   out_2292801506792658940[73] = 0;
   out_2292801506792658940[74] = 0;
   out_2292801506792658940[75] = 0;
   out_2292801506792658940[76] = 0;
   out_2292801506792658940[77] = 0;
   out_2292801506792658940[78] = 0;
   out_2292801506792658940[79] = 0;
   out_2292801506792658940[80] = 1;
}
void h_25(double *state, double *unused, double *out_274171705799066031) {
   out_274171705799066031[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1362543829109268691) {
   out_1362543829109268691[0] = 0;
   out_1362543829109268691[1] = 0;
   out_1362543829109268691[2] = 0;
   out_1362543829109268691[3] = 0;
   out_1362543829109268691[4] = 0;
   out_1362543829109268691[5] = 0;
   out_1362543829109268691[6] = 1;
   out_1362543829109268691[7] = 0;
   out_1362543829109268691[8] = 0;
}
void h_24(double *state, double *unused, double *out_8682175579901995671) {
   out_8682175579901995671[0] = state[4];
   out_8682175579901995671[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2195381769693054445) {
   out_2195381769693054445[0] = 0;
   out_2195381769693054445[1] = 0;
   out_2195381769693054445[2] = 0;
   out_2195381769693054445[3] = 0;
   out_2195381769693054445[4] = 1;
   out_2195381769693054445[5] = 0;
   out_2195381769693054445[6] = 0;
   out_2195381769693054445[7] = 0;
   out_2195381769693054445[8] = 0;
   out_2195381769693054445[9] = 0;
   out_2195381769693054445[10] = 0;
   out_2195381769693054445[11] = 0;
   out_2195381769693054445[12] = 0;
   out_2195381769693054445[13] = 0;
   out_2195381769693054445[14] = 1;
   out_2195381769693054445[15] = 0;
   out_2195381769693054445[16] = 0;
   out_2195381769693054445[17] = 0;
}
void h_30(double *state, double *unused, double *out_3363277822454956319) {
   out_3363277822454956319[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1155789129397979936) {
   out_1155789129397979936[0] = 0;
   out_1155789129397979936[1] = 0;
   out_1155789129397979936[2] = 0;
   out_1155789129397979936[3] = 0;
   out_1155789129397979936[4] = 1;
   out_1155789129397979936[5] = 0;
   out_1155789129397979936[6] = 0;
   out_1155789129397979936[7] = 0;
   out_1155789129397979936[8] = 0;
}
void h_26(double *state, double *unused, double *out_5967683143263653970) {
   out_5967683143263653970[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5104047147983324915) {
   out_5104047147983324915[0] = 0;
   out_5104047147983324915[1] = 0;
   out_5104047147983324915[2] = 0;
   out_5104047147983324915[3] = 0;
   out_5104047147983324915[4] = 0;
   out_5104047147983324915[5] = 0;
   out_5104047147983324915[6] = 0;
   out_5104047147983324915[7] = 1;
   out_5104047147983324915[8] = 0;
}
void h_27(double *state, double *unused, double *out_2144573907268787995) {
   out_2144573907268787995[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1018974182402444975) {
   out_1018974182402444975[0] = 0;
   out_1018974182402444975[1] = 0;
   out_1018974182402444975[2] = 0;
   out_1018974182402444975[3] = 1;
   out_1018974182402444975[4] = 0;
   out_1018974182402444975[5] = 0;
   out_1018974182402444975[6] = 0;
   out_1018974182402444975[7] = 0;
   out_1018974182402444975[8] = 0;
}
void h_29(double *state, double *unused, double *out_8364951090115801964) {
   out_8364951090115801964[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1666020473712372120) {
   out_1666020473712372120[0] = 0;
   out_1666020473712372120[1] = 1;
   out_1666020473712372120[2] = 0;
   out_1666020473712372120[3] = 0;
   out_1666020473712372120[4] = 0;
   out_1666020473712372120[5] = 0;
   out_1666020473712372120[6] = 0;
   out_1666020473712372120[7] = 0;
   out_1666020473712372120[8] = 0;
}
void h_28(double *state, double *unused, double *out_8652824993129429279) {
   out_8652824993129429279[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3416378543357158454) {
   out_3416378543357158454[0] = 1;
   out_3416378543357158454[1] = 0;
   out_3416378543357158454[2] = 0;
   out_3416378543357158454[3] = 0;
   out_3416378543357158454[4] = 0;
   out_3416378543357158454[5] = 0;
   out_3416378543357158454[6] = 0;
   out_3416378543357158454[7] = 0;
   out_3416378543357158454[8] = 0;
}
void h_31(double *state, double *unused, double *out_6718782836820559302) {
   out_6718782836820559302[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5730255250216676391) {
   out_5730255250216676391[0] = 0;
   out_5730255250216676391[1] = 0;
   out_5730255250216676391[2] = 0;
   out_5730255250216676391[3] = 0;
   out_5730255250216676391[4] = 0;
   out_5730255250216676391[5] = 0;
   out_5730255250216676391[6] = 0;
   out_5730255250216676391[7] = 0;
   out_5730255250216676391[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4673356681073551746) {
  err_fun(nom_x, delta_x, out_4673356681073551746);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5111845765447207624) {
  inv_err_fun(nom_x, true_x, out_5111845765447207624);
}
void car_H_mod_fun(double *state, double *out_2465835830106009632) {
  H_mod_fun(state, out_2465835830106009632);
}
void car_f_fun(double *state, double dt, double *out_1423529977948406845) {
  f_fun(state,  dt, out_1423529977948406845);
}
void car_F_fun(double *state, double dt, double *out_2292801506792658940) {
  F_fun(state,  dt, out_2292801506792658940);
}
void car_h_25(double *state, double *unused, double *out_274171705799066031) {
  h_25(state, unused, out_274171705799066031);
}
void car_H_25(double *state, double *unused, double *out_1362543829109268691) {
  H_25(state, unused, out_1362543829109268691);
}
void car_h_24(double *state, double *unused, double *out_8682175579901995671) {
  h_24(state, unused, out_8682175579901995671);
}
void car_H_24(double *state, double *unused, double *out_2195381769693054445) {
  H_24(state, unused, out_2195381769693054445);
}
void car_h_30(double *state, double *unused, double *out_3363277822454956319) {
  h_30(state, unused, out_3363277822454956319);
}
void car_H_30(double *state, double *unused, double *out_1155789129397979936) {
  H_30(state, unused, out_1155789129397979936);
}
void car_h_26(double *state, double *unused, double *out_5967683143263653970) {
  h_26(state, unused, out_5967683143263653970);
}
void car_H_26(double *state, double *unused, double *out_5104047147983324915) {
  H_26(state, unused, out_5104047147983324915);
}
void car_h_27(double *state, double *unused, double *out_2144573907268787995) {
  h_27(state, unused, out_2144573907268787995);
}
void car_H_27(double *state, double *unused, double *out_1018974182402444975) {
  H_27(state, unused, out_1018974182402444975);
}
void car_h_29(double *state, double *unused, double *out_8364951090115801964) {
  h_29(state, unused, out_8364951090115801964);
}
void car_H_29(double *state, double *unused, double *out_1666020473712372120) {
  H_29(state, unused, out_1666020473712372120);
}
void car_h_28(double *state, double *unused, double *out_8652824993129429279) {
  h_28(state, unused, out_8652824993129429279);
}
void car_H_28(double *state, double *unused, double *out_3416378543357158454) {
  H_28(state, unused, out_3416378543357158454);
}
void car_h_31(double *state, double *unused, double *out_6718782836820559302) {
  h_31(state, unused, out_6718782836820559302);
}
void car_H_31(double *state, double *unused, double *out_5730255250216676391) {
  H_31(state, unused, out_5730255250216676391);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
