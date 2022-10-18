#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2312207852105212569) {
   out_2312207852105212569[0] = delta_x[0] + nom_x[0];
   out_2312207852105212569[1] = delta_x[1] + nom_x[1];
   out_2312207852105212569[2] = delta_x[2] + nom_x[2];
   out_2312207852105212569[3] = delta_x[3] + nom_x[3];
   out_2312207852105212569[4] = delta_x[4] + nom_x[4];
   out_2312207852105212569[5] = delta_x[5] + nom_x[5];
   out_2312207852105212569[6] = delta_x[6] + nom_x[6];
   out_2312207852105212569[7] = delta_x[7] + nom_x[7];
   out_2312207852105212569[8] = delta_x[8] + nom_x[8];
   out_2312207852105212569[9] = delta_x[9] + nom_x[9];
   out_2312207852105212569[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3492233489560761085) {
   out_3492233489560761085[0] = -nom_x[0] + true_x[0];
   out_3492233489560761085[1] = -nom_x[1] + true_x[1];
   out_3492233489560761085[2] = -nom_x[2] + true_x[2];
   out_3492233489560761085[3] = -nom_x[3] + true_x[3];
   out_3492233489560761085[4] = -nom_x[4] + true_x[4];
   out_3492233489560761085[5] = -nom_x[5] + true_x[5];
   out_3492233489560761085[6] = -nom_x[6] + true_x[6];
   out_3492233489560761085[7] = -nom_x[7] + true_x[7];
   out_3492233489560761085[8] = -nom_x[8] + true_x[8];
   out_3492233489560761085[9] = -nom_x[9] + true_x[9];
   out_3492233489560761085[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6464982121424259234) {
   out_6464982121424259234[0] = 1.0;
   out_6464982121424259234[1] = 0;
   out_6464982121424259234[2] = 0;
   out_6464982121424259234[3] = 0;
   out_6464982121424259234[4] = 0;
   out_6464982121424259234[5] = 0;
   out_6464982121424259234[6] = 0;
   out_6464982121424259234[7] = 0;
   out_6464982121424259234[8] = 0;
   out_6464982121424259234[9] = 0;
   out_6464982121424259234[10] = 0;
   out_6464982121424259234[11] = 0;
   out_6464982121424259234[12] = 1.0;
   out_6464982121424259234[13] = 0;
   out_6464982121424259234[14] = 0;
   out_6464982121424259234[15] = 0;
   out_6464982121424259234[16] = 0;
   out_6464982121424259234[17] = 0;
   out_6464982121424259234[18] = 0;
   out_6464982121424259234[19] = 0;
   out_6464982121424259234[20] = 0;
   out_6464982121424259234[21] = 0;
   out_6464982121424259234[22] = 0;
   out_6464982121424259234[23] = 0;
   out_6464982121424259234[24] = 1.0;
   out_6464982121424259234[25] = 0;
   out_6464982121424259234[26] = 0;
   out_6464982121424259234[27] = 0;
   out_6464982121424259234[28] = 0;
   out_6464982121424259234[29] = 0;
   out_6464982121424259234[30] = 0;
   out_6464982121424259234[31] = 0;
   out_6464982121424259234[32] = 0;
   out_6464982121424259234[33] = 0;
   out_6464982121424259234[34] = 0;
   out_6464982121424259234[35] = 0;
   out_6464982121424259234[36] = 1.0;
   out_6464982121424259234[37] = 0;
   out_6464982121424259234[38] = 0;
   out_6464982121424259234[39] = 0;
   out_6464982121424259234[40] = 0;
   out_6464982121424259234[41] = 0;
   out_6464982121424259234[42] = 0;
   out_6464982121424259234[43] = 0;
   out_6464982121424259234[44] = 0;
   out_6464982121424259234[45] = 0;
   out_6464982121424259234[46] = 0;
   out_6464982121424259234[47] = 0;
   out_6464982121424259234[48] = 1.0;
   out_6464982121424259234[49] = 0;
   out_6464982121424259234[50] = 0;
   out_6464982121424259234[51] = 0;
   out_6464982121424259234[52] = 0;
   out_6464982121424259234[53] = 0;
   out_6464982121424259234[54] = 0;
   out_6464982121424259234[55] = 0;
   out_6464982121424259234[56] = 0;
   out_6464982121424259234[57] = 0;
   out_6464982121424259234[58] = 0;
   out_6464982121424259234[59] = 0;
   out_6464982121424259234[60] = 1.0;
   out_6464982121424259234[61] = 0;
   out_6464982121424259234[62] = 0;
   out_6464982121424259234[63] = 0;
   out_6464982121424259234[64] = 0;
   out_6464982121424259234[65] = 0;
   out_6464982121424259234[66] = 0;
   out_6464982121424259234[67] = 0;
   out_6464982121424259234[68] = 0;
   out_6464982121424259234[69] = 0;
   out_6464982121424259234[70] = 0;
   out_6464982121424259234[71] = 0;
   out_6464982121424259234[72] = 1.0;
   out_6464982121424259234[73] = 0;
   out_6464982121424259234[74] = 0;
   out_6464982121424259234[75] = 0;
   out_6464982121424259234[76] = 0;
   out_6464982121424259234[77] = 0;
   out_6464982121424259234[78] = 0;
   out_6464982121424259234[79] = 0;
   out_6464982121424259234[80] = 0;
   out_6464982121424259234[81] = 0;
   out_6464982121424259234[82] = 0;
   out_6464982121424259234[83] = 0;
   out_6464982121424259234[84] = 1.0;
   out_6464982121424259234[85] = 0;
   out_6464982121424259234[86] = 0;
   out_6464982121424259234[87] = 0;
   out_6464982121424259234[88] = 0;
   out_6464982121424259234[89] = 0;
   out_6464982121424259234[90] = 0;
   out_6464982121424259234[91] = 0;
   out_6464982121424259234[92] = 0;
   out_6464982121424259234[93] = 0;
   out_6464982121424259234[94] = 0;
   out_6464982121424259234[95] = 0;
   out_6464982121424259234[96] = 1.0;
   out_6464982121424259234[97] = 0;
   out_6464982121424259234[98] = 0;
   out_6464982121424259234[99] = 0;
   out_6464982121424259234[100] = 0;
   out_6464982121424259234[101] = 0;
   out_6464982121424259234[102] = 0;
   out_6464982121424259234[103] = 0;
   out_6464982121424259234[104] = 0;
   out_6464982121424259234[105] = 0;
   out_6464982121424259234[106] = 0;
   out_6464982121424259234[107] = 0;
   out_6464982121424259234[108] = 1.0;
   out_6464982121424259234[109] = 0;
   out_6464982121424259234[110] = 0;
   out_6464982121424259234[111] = 0;
   out_6464982121424259234[112] = 0;
   out_6464982121424259234[113] = 0;
   out_6464982121424259234[114] = 0;
   out_6464982121424259234[115] = 0;
   out_6464982121424259234[116] = 0;
   out_6464982121424259234[117] = 0;
   out_6464982121424259234[118] = 0;
   out_6464982121424259234[119] = 0;
   out_6464982121424259234[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_4539264253469581544) {
   out_4539264253469581544[0] = dt*state[3] + state[0];
   out_4539264253469581544[1] = dt*state[4] + state[1];
   out_4539264253469581544[2] = dt*state[5] + state[2];
   out_4539264253469581544[3] = state[3];
   out_4539264253469581544[4] = state[4];
   out_4539264253469581544[5] = state[5];
   out_4539264253469581544[6] = dt*state[7] + state[6];
   out_4539264253469581544[7] = dt*state[8] + state[7];
   out_4539264253469581544[8] = state[8];
   out_4539264253469581544[9] = state[9];
   out_4539264253469581544[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8063886126610994275) {
   out_8063886126610994275[0] = 1;
   out_8063886126610994275[1] = 0;
   out_8063886126610994275[2] = 0;
   out_8063886126610994275[3] = dt;
   out_8063886126610994275[4] = 0;
   out_8063886126610994275[5] = 0;
   out_8063886126610994275[6] = 0;
   out_8063886126610994275[7] = 0;
   out_8063886126610994275[8] = 0;
   out_8063886126610994275[9] = 0;
   out_8063886126610994275[10] = 0;
   out_8063886126610994275[11] = 0;
   out_8063886126610994275[12] = 1;
   out_8063886126610994275[13] = 0;
   out_8063886126610994275[14] = 0;
   out_8063886126610994275[15] = dt;
   out_8063886126610994275[16] = 0;
   out_8063886126610994275[17] = 0;
   out_8063886126610994275[18] = 0;
   out_8063886126610994275[19] = 0;
   out_8063886126610994275[20] = 0;
   out_8063886126610994275[21] = 0;
   out_8063886126610994275[22] = 0;
   out_8063886126610994275[23] = 0;
   out_8063886126610994275[24] = 1;
   out_8063886126610994275[25] = 0;
   out_8063886126610994275[26] = 0;
   out_8063886126610994275[27] = dt;
   out_8063886126610994275[28] = 0;
   out_8063886126610994275[29] = 0;
   out_8063886126610994275[30] = 0;
   out_8063886126610994275[31] = 0;
   out_8063886126610994275[32] = 0;
   out_8063886126610994275[33] = 0;
   out_8063886126610994275[34] = 0;
   out_8063886126610994275[35] = 0;
   out_8063886126610994275[36] = 1;
   out_8063886126610994275[37] = 0;
   out_8063886126610994275[38] = 0;
   out_8063886126610994275[39] = 0;
   out_8063886126610994275[40] = 0;
   out_8063886126610994275[41] = 0;
   out_8063886126610994275[42] = 0;
   out_8063886126610994275[43] = 0;
   out_8063886126610994275[44] = 0;
   out_8063886126610994275[45] = 0;
   out_8063886126610994275[46] = 0;
   out_8063886126610994275[47] = 0;
   out_8063886126610994275[48] = 1;
   out_8063886126610994275[49] = 0;
   out_8063886126610994275[50] = 0;
   out_8063886126610994275[51] = 0;
   out_8063886126610994275[52] = 0;
   out_8063886126610994275[53] = 0;
   out_8063886126610994275[54] = 0;
   out_8063886126610994275[55] = 0;
   out_8063886126610994275[56] = 0;
   out_8063886126610994275[57] = 0;
   out_8063886126610994275[58] = 0;
   out_8063886126610994275[59] = 0;
   out_8063886126610994275[60] = 1;
   out_8063886126610994275[61] = 0;
   out_8063886126610994275[62] = 0;
   out_8063886126610994275[63] = 0;
   out_8063886126610994275[64] = 0;
   out_8063886126610994275[65] = 0;
   out_8063886126610994275[66] = 0;
   out_8063886126610994275[67] = 0;
   out_8063886126610994275[68] = 0;
   out_8063886126610994275[69] = 0;
   out_8063886126610994275[70] = 0;
   out_8063886126610994275[71] = 0;
   out_8063886126610994275[72] = 1;
   out_8063886126610994275[73] = dt;
   out_8063886126610994275[74] = 0;
   out_8063886126610994275[75] = 0;
   out_8063886126610994275[76] = 0;
   out_8063886126610994275[77] = 0;
   out_8063886126610994275[78] = 0;
   out_8063886126610994275[79] = 0;
   out_8063886126610994275[80] = 0;
   out_8063886126610994275[81] = 0;
   out_8063886126610994275[82] = 0;
   out_8063886126610994275[83] = 0;
   out_8063886126610994275[84] = 1;
   out_8063886126610994275[85] = dt;
   out_8063886126610994275[86] = 0;
   out_8063886126610994275[87] = 0;
   out_8063886126610994275[88] = 0;
   out_8063886126610994275[89] = 0;
   out_8063886126610994275[90] = 0;
   out_8063886126610994275[91] = 0;
   out_8063886126610994275[92] = 0;
   out_8063886126610994275[93] = 0;
   out_8063886126610994275[94] = 0;
   out_8063886126610994275[95] = 0;
   out_8063886126610994275[96] = 1;
   out_8063886126610994275[97] = 0;
   out_8063886126610994275[98] = 0;
   out_8063886126610994275[99] = 0;
   out_8063886126610994275[100] = 0;
   out_8063886126610994275[101] = 0;
   out_8063886126610994275[102] = 0;
   out_8063886126610994275[103] = 0;
   out_8063886126610994275[104] = 0;
   out_8063886126610994275[105] = 0;
   out_8063886126610994275[106] = 0;
   out_8063886126610994275[107] = 0;
   out_8063886126610994275[108] = 1;
   out_8063886126610994275[109] = 0;
   out_8063886126610994275[110] = 0;
   out_8063886126610994275[111] = 0;
   out_8063886126610994275[112] = 0;
   out_8063886126610994275[113] = 0;
   out_8063886126610994275[114] = 0;
   out_8063886126610994275[115] = 0;
   out_8063886126610994275[116] = 0;
   out_8063886126610994275[117] = 0;
   out_8063886126610994275[118] = 0;
   out_8063886126610994275[119] = 0;
   out_8063886126610994275[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6421619825315491896) {
   out_6421619825315491896[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1129091903658590088) {
   out_1129091903658590088[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1129091903658590088[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1129091903658590088[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1129091903658590088[3] = 0;
   out_1129091903658590088[4] = 0;
   out_1129091903658590088[5] = 0;
   out_1129091903658590088[6] = 1;
   out_1129091903658590088[7] = 0;
   out_1129091903658590088[8] = 0;
   out_1129091903658590088[9] = 0;
   out_1129091903658590088[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5307285830992173381) {
   out_5307285830992173381[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8963679878168358303) {
   out_8963679878168358303[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8963679878168358303[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8963679878168358303[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8963679878168358303[3] = 0;
   out_8963679878168358303[4] = 0;
   out_8963679878168358303[5] = 0;
   out_8963679878168358303[6] = 1;
   out_8963679878168358303[7] = 0;
   out_8963679878168358303[8] = 0;
   out_8963679878168358303[9] = 1;
   out_8963679878168358303[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3350868428259612938) {
   out_3350868428259612938[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_738916608976085322) {
   out_738916608976085322[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[6] = 0;
   out_738916608976085322[7] = 1;
   out_738916608976085322[8] = 0;
   out_738916608976085322[9] = 0;
   out_738916608976085322[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3350868428259612938) {
   out_3350868428259612938[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_738916608976085322) {
   out_738916608976085322[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_738916608976085322[6] = 0;
   out_738916608976085322[7] = 1;
   out_738916608976085322[8] = 0;
   out_738916608976085322[9] = 0;
   out_738916608976085322[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2312207852105212569) {
  err_fun(nom_x, delta_x, out_2312207852105212569);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3492233489560761085) {
  inv_err_fun(nom_x, true_x, out_3492233489560761085);
}
void gnss_H_mod_fun(double *state, double *out_6464982121424259234) {
  H_mod_fun(state, out_6464982121424259234);
}
void gnss_f_fun(double *state, double dt, double *out_4539264253469581544) {
  f_fun(state,  dt, out_4539264253469581544);
}
void gnss_F_fun(double *state, double dt, double *out_8063886126610994275) {
  F_fun(state,  dt, out_8063886126610994275);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6421619825315491896) {
  h_6(state, sat_pos, out_6421619825315491896);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1129091903658590088) {
  H_6(state, sat_pos, out_1129091903658590088);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5307285830992173381) {
  h_20(state, sat_pos, out_5307285830992173381);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8963679878168358303) {
  H_20(state, sat_pos, out_8963679878168358303);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3350868428259612938) {
  h_7(state, sat_pos_vel, out_3350868428259612938);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_738916608976085322) {
  H_7(state, sat_pos_vel, out_738916608976085322);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3350868428259612938) {
  h_21(state, sat_pos_vel, out_3350868428259612938);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_738916608976085322) {
  H_21(state, sat_pos_vel, out_738916608976085322);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
