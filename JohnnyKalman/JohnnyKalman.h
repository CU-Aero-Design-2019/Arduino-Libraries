#ifndef KALMAN2_H
#define KALMAN2_H

#include <cmath>
#include "SpecGPS.h"

class JohnnyKalman{
	
	const int debugEnabled = 1;

	struct LLAT_in {
	  double lat_in, lon_in, alt_in, minutes, seconds, centiseconds;
	};
	struct Kalman_out {
	  double x_pos, y_pos, z_pos, x_vel, y_vel, z_vel;
	};

	const double pi = 3.14159265359;
	const double deg_to_rad = pi / 180;

	const double x_sig_pos = 3.41;
	const double y_sig_pos = 3.41;
	const double z_sig_pos = 5;
	
	const double cov_px = pow(x_sig_pos, 2);
	const double cov_py = pow(y_sig_pos, 2);
	const double cov_pz = pow(z_sig_pos, 2);
	
	const double cov_vx = pow(x_sig_vel, 2);
	const double cov_vy = pow(y_sig_vel, 2);
	const double cov_vz = pow(z_sig_vel, 2);
	
	// velocity sigmas 
	const double x_sig_vel = 0.5;
	const double y_sig_vel = 0.5;
	const double z_sig_vel = 0.5;
	
	// initial velocity estimate
	const double init_vel_x_enu = 2;
	const double init_vel_y_enu = 2;
	const double init_vel_z_enu = 0;
	
	// alpha values
	// knobs for the process noise matrix
	const double x_enu_alpha = 1000;
	const double y_enu_alpha = 1000;
	const double z_enu_alpha = 1000;
	
	const double x_sig_acc = 0.05;
	const double y_sig_acc = 0.05;
	const double z_sig_acc = 0.05;
	
	double delta_time, t_curr, t_prev;
	
	double meas_vector_x[2];
	double meas_vector_y[2];
	double meas_vector_z[2];
	
	double acc_control_vec_x[2] = { 0, 0 };
	double acc_control_vec_y[2] = { 0, 0 };
	double acc_control_vec_z[2] = { 0, 0 };
	
	double F2[2][2];
	double F2_t[2][2];
	double B2[2][2];
	
	double Q_sub_x_in[2][2];
	double Q_sub_x_scalar_temp;
	double Q_sub_x_out[2][2];

	double Q_sub_y_in[2][2];
	double Q_sub_y_scalar_temp;
	double Q_sub_y_out[2][2];

	double Q_sub_z_in[2][2];
	double Q_sub_z_scalar_temp;
	double Q_sub_z_out[2][2];

	double xp_x[2];
	double xp_x_vec_1[2];
	double xp_x_vec_2[2];

	double xp_y[2];
	double xp_y_vec_1[2];
	double xp_y_vec_2[2];

	double xp_z[2];
	double xp_z_vec_1[2];
	double xp_z_vec_2[2];

	double P0x[2][2];
	double P0y[2][2];
	double P0z[2][2];

	double Px[2][2];
	double F2_P0x[2][2];
	double F2_P0x_F_t[2][2];

	double Py[2][2];
	double F2_P0y[2][2];
	double F2_P0y_F_t[2][2];

	double Pz[2][2];
	double F2_P0z[2][2];
	double F2_P0z_F_t[2][2];

	double y_x[2];
	double H2_xp_x[2];

	double y_y[2];
	double H2_xp_y[2];

	double y_z[2];
	double H2_xp_z[2];

	double Sx[2][2];
	double H2_Px[2][2];
	double H2_Px_H2_t[2][2];

	double Sy[2][2];
	double H2_Py[2][2];
	double H2_Py_H2_t[2][2];

	double Sz[2][2];
	double H2_Pz[2][2];
	double H2_Pz_H2_t[2][2];

	double Kx[2][2];
	double Px_H2_t[2][2];
	double Sx_inv[2][2];
	double Px_H2_t_inv_Sx[2][2];

	double Ky[2][2];
	double Py_H2_t[2][2];
	double Sy_inv[2][2];
	double Py_H2_t_inv_Sy[2][2];

	double Kz[2][2];
	double Pz_H2_t[2][2];
	double Sz_inv[2][2];
	double Pz_H2_t_inv_Sz[2][2];

	double x_updated_x[2];
	double Kx_y_x[2];

	double x_updated_y[2];
	double Ky_y_y[2];

	double x_updated_z[2];
	double Kz_y_z[2];

	double eye_2[2][2] = { { 1, 0 },
	{ 0, 1 } };

	double Px_updated[2][2];
	double Kx_H_2[2][2];
	double eye_2_Kx_H2[2][2];

	double Py_updated[2][2];
	double Ky_H_2[2][2];
	double eye_2_Ky_H2[2][2];

	double Pz_updated[2][2];
	double Kz_H_2[2][2];
	double eye_2_Kz_H2[2][2];

	double x0_x[2];
	double x0_y[2];
	double x0_z[2];
	
	// observation matrix of 2 state ... will be assuming no cross correlation
	// and thus can use the 2x2 observation matrix
	double H2[2][2] = { { 1, 0 },
	{ 0, 0 } };
	//double H2_t[2][2]; // might want to transpose ahead of time
	double H2_t[2][2] = { { 1, 0 },
	{ 0, 0 } };
	
	// measurement covariance matrix ... deals with noise/error in the measurements
	// 2 state measurement covariance matrices
	double Rx[2][2] = { { cov_px, 0 },
	{ 0, cov_vx } };
	double Ry[2][2] = { { cov_py, 0 },
	{ 0, cov_vy } };
	double Rz[2][2] = { { cov_pz, 0 },
	{ 0, cov_vz } };
	
	//Kalman Filter
	void initial_kf_setup(LLAT_in gps_input);
	void kalman_update(SpecGPS::LLA ref_pt_lla, LLAT_in gps_input, Kalman_out filter_output);

	//Matrix and vector math prototypes here
	void my_mx_vec_mult(double *mx_in, double *vec_in, double *vec_out, int n_dim);
	void my_mx_add(double *mx_a, double *mx_b, double *mx_out, int n_dim);
	void my_mx_subtract(double *mx_a, double *mx_b, double *mx_out, int n_dim);
	void my_mx_mx_mult(double *mx_a, double *mx_b, double *mx_out, int n_dim);
	void my_mx_transpose(double *mx_in, double *mx_out, int n_dim);
	void my_scalar_mx_mult(double *mx_in, double *mx_out, double num_mult, int n_dim);
	void my_mx_copy(double *mx_in, double *mx_out, int n_dim);
	void my_mx_identity(double *mx_out, int n_dim);
	void my_mx_2x2_inv(double *mx_in, double *mx_out);
	void my_vec_add(double *vec_a, double *vec_b, double *vec_out, int n_len);
	void my_vec_subtract(double *vec_first, double *vec_second, double *vec_out, int n_len);
	void my_vec_scalar_mult(double *vec_in, double *vec_out, double num_mult, int n_len);
	void my_vec_copy(double *vec_in, double *vec_out, int n_len);

};


#endif