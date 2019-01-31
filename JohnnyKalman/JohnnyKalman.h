#ifndef KALMAN2_H
#define KALMAN2_H

#include <cmath>
#include <SpecGPS.h>
#include <SpecBMP180.h>

namespace JohnnyKalman {
	
	const int debugEnabled = 0;
	unsigned long nextTime = 0;

	struct LLAT_in {
		double lat_in;
		double lon_in;
		double alt_in;
		double minutes;
		double seconds;
		double centiseconds;
	};
	
	struct Kalman_out {
		double x_pos;
		double y_pos;
		double z_pos;
		double x_vel;
		double y_vel;
		double z_vel;
	};
	
	SpecGPS::LLA lla_ref;
	SpecGPS::ECEF XYZ_ecef_ref;
	SpecGPS::ENU xyz_enu_ref;
	
	Kalman_out filter_output;

	const double pi = 3.14159265359;
	const double deg_to_rad = pi / 180;

	const double x_sig_pos = 3.41;
	const double y_sig_pos = 3.41;
	const double z_sig_pos = 5;
	
	const double cov_px = pow(x_sig_pos, 2);
	const double cov_py = pow(y_sig_pos, 2);
	const double cov_pz = pow(z_sig_pos, 2);
	
	// velocity sigmas 
	const double x_sig_vel = 0.5;
	const double y_sig_vel = 0.5;
	const double z_sig_vel = 0.5;
	
	const double cov_vx = pow(x_sig_vel, 2);
	const double cov_vy = pow(y_sig_vel, 2);
	const double cov_vz = pow(z_sig_vel, 2);
	
	// initial velocity estimate
	const double init_vel_x_enu = 2;
	const double init_vel_y_enu = 2;
	const double init_vel_z_enu = 0;
	
	// alpha values
	// knobs for the process noise matrix
	const double x_enu_alpha = 1000;
	const double y_enu_alpha = 1000;
	const double z_enu_alpha = 1000;
	// TODO: Make these bigger ^V
	const double x_sig_acc = 0.05;
	const double y_sig_acc = 0.05;
	const double z_sig_acc = 0.05;
	
	double delta_time;
	double t_curr;
	double t_prev;
	
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
	
	bool hasDoneSetup = false;
	
	//Kalman Filter
	void initial_kf_setup();
	void kalman_update(LLAT_in gps_input, Kalman_out filter_output);

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
	 
	void initial_kf_setup(SpecGPS::LLA targetLLA) {
		hasDoneSetup = true;
		
		LLAT_in gps_input;
		gps_input.lat_in = SpecGPS::gps.location.lat();
		gps_input.lon_in = SpecGPS::gps.location.lng();
		gps_input.alt_in = bmp.readOffsetAltitude();
		gps_input.minutes = SpecGPS::gps.time.minute();
		gps_input.seconds = SpecGPS::gps.time.second();
		gps_input.centiseconds = SpecGPS::gps.time.centisecond();
		
		if (debugEnabled == 1) {
			Serial.println("Test: beginning of initial kf setup");
		}
		
		meas_vector_x[1] = 0;
		meas_vector_y[1] = 0;
		meas_vector_y[1] = 0;

		F2[0][0] = 1;
		F2[1][0] = 0;
		F2[1][1] = 1;

		F2_t[0][0] = 1;
		F2_t[0][1] = 0;
		F2_t[1][1] = 1;

		B2[0][1] = 0;
		B2[1][0] = 0;

		// set the initial state covariance matrix to 'R...' (measurement covariance matrix)
		my_mx_copy(Rx[0], P0x[0], 2);
		my_mx_copy(Ry[0], P0y[0], 2);
		my_mx_copy(Rz[0], P0z[0], 2);

		// first time previous will be set to the first value
		t_prev = millis()/1000;

		// initial 2 state measurement
		x0_x[0] = 0;
		x0_x[1] = init_vel_x_enu;

		x0_y[0] = 0;
		x0_y[1] = init_vel_y_enu;

		x0_z[0] = 0;
		x0_z[1] = init_vel_z_enu;
		
		if (debugEnabled) {
			Serial.print("x0_x = ");
			Serial.print(x0_x[0]);
			Serial.print(" , ");
			Serial.println(x0_x[1]);
			Serial.print("x0_y = ");
			Serial.print(x0_y[0]);
			Serial.print(" , ");
			Serial.println(x0_y[1]);
			Serial.print("x0_z = ");
			Serial.print(x0_z[0]);
			Serial.print(" , ");
			Serial.println(x0_z[1]);
		}
		
		lla_ref = targetLLA;
		if (debugEnabled) {
			Serial.print("lla target: ");
			Serial.print(lla_ref.lat, 8);
			Serial.print("\t");
			Serial.print(lla_ref.lng, 8);
			Serial.print("\t");
			Serial.print(lla_ref.alt, 1);
			Serial.print("\t");
			Serial.print("\n");
		}
		SpecGPS::lla_to_ecef(lla_ref, XYZ_ecef_ref);
		xyz_enu_ref.e = 0;
		xyz_enu_ref.n = 0;
		xyz_enu_ref.u = 0;

	}

	void kalman_update() {
		
		// get GPS location and put in struct
		SpecGPS::LLA lla_coor;
		// lla_coor.lat = SpecGPS::gps.location.lat();
		// lla_coor.lng = SpecGPS::gps.location.lng();
		// lla_coor.alt = bmp.readOffsetAltitude();
		lla_coor.lat = 39.747511;
		lla_coor.lng = -83.813272;
		lla_coor.alt = 25;
		
		SpecGPS::ENU xyz_enu;
		// Coordinate Transformation
		SpecGPS::lla_to_enu(lla_coor, lla_ref, XYZ_ecef_ref, xyz_enu); // call this instead of the two functions

		if (debugEnabled) {
			Serial.print("enu target: ");
			Serial.print(xyz_enu_ref.e);
			Serial.print("\t");
			Serial.print(xyz_enu_ref.n);
			Serial.print("\t");
			Serial.print(xyz_enu_ref.u);
			Serial.print("\t");
			Serial.print("\n");
			Serial.print("enu current location: ");
			Serial.print(xyz_enu.e);
			Serial.print("\t");
			Serial.print(xyz_enu.n);
			Serial.print("\t");
			Serial.print(xyz_enu.u);
			Serial.print("\t");
			Serial.print("\n");
		}

		// delta time between measurements
		t_curr = ((double)millis())/1000.0;
		delta_time = t_curr - t_prev;
		
		if (debugEnabled) {
			Serial.print("delta time: ");
			Serial.print(delta_time);
			Serial.print("\n");
		}

		// measurement vector for 2 state ... just change first element in array since second
		// element will always be zero
		meas_vector_x[0] = xyz_enu.e; // x enu data value
		meas_vector_y[0] = xyz_enu.n; // y enu data value
		meas_vector_z[0] = xyz_enu.u; // z enu data value

		// state transition matrix for 2 state ... 
		// used in part of the calculation of the equation of motion
		// pre-declared up top with certain indices already assigned values
		F2[0][1] = delta_time;
		
		// if (debugEnabled) {
			// Serial.println("F2 initialized");
		// }

		// acceleration control matrix for 2 state
		// pre-declared up top with certain indices already assigned values
		B2[0][0] = 0.5 * pow(delta_time, 2);
		B2[1][1] = delta_time;
		
		// if (debugEnabled) {
			// Serial.println("B2 initialized");
		// }

		// process noise matrix ... adds to the uncertainty of the prediction
		// break up for 2 state instead of one large matrix
		Q_sub_x_in[0][0] = 0.25 * pow(delta_time, 4);
		Q_sub_x_in[0][1] = 0.5 * pow(delta_time, 3);
		Q_sub_x_in[1][0] = 0.5 * pow(delta_time, 3);
		Q_sub_x_in[1][1] = pow(delta_time, 2);
		Q_sub_x_scalar_temp = 2 * pow(x_sig_acc, 2) * x_enu_alpha;
		my_scalar_mx_mult(Q_sub_x_in[0], Q_sub_x_out[0], Q_sub_x_scalar_temp, 2);

		Q_sub_y_in[0][0] = 0.25 * pow(delta_time, 4);
		Q_sub_y_in[0][1] = 0.5 * pow(delta_time, 3);
		Q_sub_y_in[1][0] = 0.5 * pow(delta_time, 3);
		Q_sub_y_in[1][1] = pow(delta_time, 2);
		Q_sub_y_scalar_temp = 2 * pow(y_sig_acc, 2) * y_enu_alpha;
		my_scalar_mx_mult(Q_sub_y_in[0], Q_sub_y_out[0], Q_sub_y_scalar_temp, 2);

		Q_sub_z_in[0][0] = 0.25 * pow(delta_time, 4);
		Q_sub_z_in[0][1] = 0.5 * pow(delta_time, 3);
		Q_sub_z_in[1][0] = 0.5 * pow(delta_time, 3);
		Q_sub_z_in[1][1] = pow(delta_time, 2);
		Q_sub_z_scalar_temp = 2 * pow(z_sig_acc, 2) * z_enu_alpha;
		my_scalar_mx_mult(Q_sub_z_in[0], Q_sub_z_out[0], Q_sub_z_scalar_temp, 2);

		// if (debugEnabled == 1) {
			// Serial.println("Q subs matrices initialized");
		// }

		// PREDICTING STAGE

		// predict state ... predicted position and velocity in x,y,z directions
		// equation of motion ... x = x0 + v*dt + (1/2)*a*dt^2
		my_mx_vec_mult(F2[0], x0_x, xp_x_vec_1, 2);
		my_mx_vec_mult(B2[0], acc_control_vec_x, xp_x_vec_2, 2);
		my_vec_add(xp_x_vec_1, xp_x_vec_2, xp_x, 2);

		my_mx_vec_mult(F2[0], x0_y, xp_y_vec_1, 2);
		my_mx_vec_mult(B2[0], acc_control_vec_y, xp_y_vec_2, 2);
		my_vec_add(xp_y_vec_1, xp_y_vec_2, xp_y, 2);

		my_mx_vec_mult(F2[0], x0_z, xp_z_vec_1, 2);
		my_mx_vec_mult(B2[0], acc_control_vec_z, xp_z_vec_2, 2);
		my_vec_add(xp_z_vec_1, xp_z_vec_2, xp_z, 2);

		// if (debugEnabled == 1) {
			// Serial.println("xp partials initialized");
		// }

		// updating the state covariance matrix ... propogating the covariances ahead 
		// in order to predict ... this gives you the error in the state prediction and estimate
		F2_t[1][0] = delta_time; // this should be fine for the transpose

		my_mx_mx_mult(F2[0], P0x[0], F2_P0x[0], 2);
		my_mx_mx_mult(F2_P0x[0], F2_t[0], F2_P0x_F_t[0], 2);
		my_mx_add(F2_P0x_F_t[0], Q_sub_x_out[0], Px[0], 2);

		my_mx_mx_mult(F2[0], P0y[0], F2_P0y[0], 2);
		my_mx_mx_mult(F2_P0y[0], F2_t[0], F2_P0y_F_t[0], 2);
		my_mx_add(F2_P0y_F_t[0], Q_sub_y_out[0], Py[0], 2);

		my_mx_mx_mult(F2[0], P0z[0], F2_P0z[0], 2);
		my_mx_mx_mult(F2_P0z[0], F2_t[0], F2_P0z_F_t[0], 2);
		my_mx_add(F2_P0z_F_t[0], Q_sub_z_out[0], Pz[0], 2);

		// if (debugEnabled == 1) {
			// Serial.println("P subs initialized");
		// }

		// UPDATE STAGE

		// residual vector ... (measurement - predicted) ... "Innovation"
		my_mx_vec_mult(H2[0], xp_x, H2_xp_x, 2);
		my_vec_subtract(meas_vector_x, H2_xp_x, y_x, 2);

		my_mx_vec_mult(H2[0], xp_y, H2_xp_y, 2);
		my_vec_subtract(meas_vector_y, H2_xp_y, y_y, 2);

		my_mx_vec_mult(H2[0], xp_z, H2_xp_z, 2);
		my_vec_subtract(meas_vector_z, H2_xp_z, y_z, 2);

		// if (debugEnabled == 1) {
			// Serial.println("y subs 'innovation' initialized");
		// }

		// residual covariance ... "Innovation Covariance"
		//matrix transpose should be done already ...

		my_mx_mx_mult(H2[0], Px[0], H2_Px[0], 2);
		my_mx_mx_mult(H2_Px[0], H2_t[0], H2_Px_H2_t[0], 2);
		my_mx_add(H2_Px_H2_t[0], Rx[0], Sx[0], 2);

		my_mx_mx_mult(H2[0], Py[0], H2_Py[0], 2);
		my_mx_mx_mult(H2_Py[0], H2_t[0], H2_Py_H2_t[0], 2);
		my_mx_add(H2_Py_H2_t[0], Ry[0], Sy[0], 2);

		my_mx_mx_mult(H2[0], Pz[0], H2_Pz[0], 2);
		my_mx_mx_mult(H2_Pz[0], H2_t[0], H2_Pz_H2_t[0], 2);
		my_mx_add(H2_Pz_H2_t[0], Rz[0], Sz[0], 2);

		// Kalman Gain ... weights whether to use the prediction or measurements more
		my_mx_mx_mult(Px[0], H2_t[0], Px_H2_t[0], 2);
		my_mx_2x2_inv(Sx[0], Sx_inv[0]);
		my_mx_mx_mult(Px_H2_t[0], Sx_inv[0], Kx[0], 2);

		my_mx_mx_mult(Py[0], H2_t[0], Py_H2_t[0], 2);
		my_mx_2x2_inv(Sy[0], Sy_inv[0]);
		my_mx_mx_mult(Py_H2_t[0], Sy_inv[0], Ky[0], 2);

		my_mx_mx_mult(Pz[0], H2_t[0], Pz_H2_t[0], 2);
		my_mx_2x2_inv(Sz[0], Sz_inv[0]);
		my_mx_mx_mult(Pz_H2_t[0], Sz_inv[0], Kz[0], 2);

		// if (debugEnabled == 1) {
			// Serial.println("K subs initialized");
		// }

		// state update
		// update state vector
		my_mx_vec_mult(Kx[0], y_x, Kx_y_x, 2);
		my_vec_add(xp_x, Kx_y_x, x_updated_x, 2);

		my_mx_vec_mult(Ky[0], y_y, Ky_y_y, 2);
		my_vec_add(xp_y, Ky_y_y, x_updated_y, 2);

		my_mx_vec_mult(Kz[0], y_z, Kz_y_z, 2);
		my_vec_add(xp_z, Kz_y_z, x_updated_z, 2);

		// if (debugEnabled == 1) {
			// Serial.println("x_updated subs initialized");
		// }

		// updated covariance matrix
		my_mx_mx_mult(Kx[0], H2[0], Kx_H_2[0], 2);
		my_mx_subtract(eye_2[0], Kx_H_2[0], eye_2_Kx_H2[0], 2);
		my_mx_mx_mult(eye_2_Kx_H2[0], Px[0], Px_updated[0], 2);

		my_mx_mx_mult(Ky[0], H2[0], Ky_H_2[0], 2);
		my_mx_subtract(eye_2[0], Ky_H_2[0], eye_2_Ky_H2[0], 2);
		my_mx_mx_mult(eye_2_Ky_H2[0], Py[0], Py_updated[0], 2);

		my_mx_mx_mult(Kz[0], H2[0], Kz_H_2[0], 2);
		my_mx_subtract(eye_2[0], Kz_H_2[0], eye_2_Kz_H2[0], 2);
		my_mx_mx_mult(eye_2_Kz_H2[0], Pz[0], Pz_updated[0], 2);

		// if (debugEnabled == 1) {
			// Serial.println("P_updated subs initialized");
		// }

		// update P0 and x0
		my_mx_copy(Px_updated[0], P0x[0], 2);
		my_mx_copy(Py_updated[0], P0y[0], 2);
		my_mx_copy(Pz_updated[0], P0z[0], 2);

		my_vec_copy(x_updated_x, x0_x, 2);
		my_vec_copy(x_updated_y, x0_y, 2);
		my_vec_copy(x_updated_z, x0_z, 2);

		// filtered data! ... this will be used for the calculations and such by the plane and gliders
		filter_output.x_pos = x_updated_x[0];
		filter_output.y_pos = x_updated_y[0];
		filter_output.z_pos = x_updated_z[0];

		filter_output.x_vel = x_updated_x[1];
		filter_output.y_vel = x_updated_y[1];
		filter_output.z_vel = x_updated_z[1];
		
		if (debugEnabled) {
			Serial.print("State x,y,z position and velocity: ");
			Serial.print(filter_output.x_pos);
			Serial.print("\t");
			Serial.print(filter_output.y_pos);
			Serial.print("\t");
			Serial.print(filter_output.z_pos);
			Serial.print("\t");
			Serial.print(filter_output.x_vel);
			Serial.print("\t");
			Serial.print(filter_output.y_vel);
			Serial.print("\t");
			Serial.print(filter_output.z_vel);
			Serial.print("\t");
			Serial.print("\n");
		}

		t_prev = ((double)millis())/1000.0;
		if (debugEnabled == 1) {
			Serial.println("t_prev: " + String(t_prev));
			Serial.println();
		}
	}

	//Create matrix and vector math functions here
	//Matrix
	void my_mx_vec_mult(double *mx_in, double *vec_in, double *vec_out, int n_dim) {
	  int i, j;
	  for (i = 0; i < n_dim; i++) {
		vec_out[i] = 0;
		for (j = 0; j < n_dim; j++) {
		  vec_out[i] += *(mx_in + i * n_dim + j) * vec_in[j];
		}
	  }
	}

	void my_mx_add(double *mx_a, double *mx_b, double *mx_out, int n_dim) {
	  int i, j;
	  for (i = 0; i < n_dim; i++) {

		for (j = 0; j < n_dim; j++) {
		  *(mx_out + i * n_dim + j) = *(mx_a + i * n_dim + j) + *(mx_b + i * n_dim + j);
		}
	  }
	}

	void my_mx_subtract(double *mx_a, double *mx_b, double *mx_out, int n_dim) {
	  // this matrix does 'a' - 'b' not the other way!!
	  int i, j;
	  for (i = 0; i < n_dim; i++) {
		for (j = 0; j < n_dim; j++) {
		  *(mx_out + i * n_dim + j) = *(mx_a + i * n_dim + j) - *(mx_b + i * n_dim + j);
		}
	  }
	}

	void my_mx_mx_mult(double *mx_a, double *mx_b, double *mx_out, int n_dim) {
	  int i, j, k;
	  double temp;
	  int counter = 0;
	  for (i = 0; i < n_dim; i++) {
		for (j = 0; j < n_dim; j++) {
		  temp = 0;

		  for (k = 0; k < n_dim; k++) {
			temp += *(mx_a + i * n_dim + k) * *(mx_b + k * n_dim + j);
		  }
		  *(mx_out + counter) = temp;
		  counter = counter + 1;
		}
	  }
	}

	void my_mx_transpose(double *mx_in, double *mx_out, int n_dim) {
	  int i, j;
	  for (i = 0; i < n_dim; i++) {
		for (j = 0; j < n_dim; j++) {
		  *(mx_out + j * n_dim + i) = *(mx_in + i * n_dim + j);
		}
	  }
	}

	void my_scalar_mx_mult(double *mx_in, double *mx_out, double num_mult, int n_dim) {
	  int i, j;
	  for (i = 0; i < n_dim; i++) {
		for (j = 0; j < n_dim; j++) {
		  *(mx_out + i * n_dim + j) = *(mx_in + i * n_dim + j)*num_mult;
		}
	  }
	}

	void my_mx_copy(double *mx_in, double *mx_out, int n_dim) {
	  int i, j;
	  for (i = 0; i < n_dim; i++) {
		for (j = 0; j < n_dim; j++) {
		  *(mx_out + i * n_dim + j) = *(mx_in + i * n_dim + j);
		}
	  }
	}

	void my_mx_identity(double *mx_out, int n_dim) {
	  int i, j;
	  for (i = 0; i < n_dim; i++) {
		for (j = 0; j < n_dim; j++) {
		  if (i == j) {
			*(mx_out + i * n_dim + j) = 1;
		  }
		  else {
			*(mx_out + i * n_dim + j) = 0;
		  }
		}
	  }
	}

	void my_mx_2x2_inv(double *mx_in, double *mx_out) {
	  int i, j;
	  double temp = 1 / (*(mx_in) * *(mx_in + 3) - *(mx_in + 1) * *(mx_in + 2));
	  *(mx_out) = *(mx_in + 3) * temp;
	  *(mx_out + 1) = *(mx_in + 1) * -1 * temp;
	  *(mx_out + 2) = *(mx_in + 2) * -1 * temp;
	  *(mx_out + 3) = *(mx_in)* temp;
	}

	//Vectors
	void my_vec_add(double *vec_a, double *vec_b, double *vec_out, int n_len) {
	  //Add two vectors
	  int i;
	  for (i = 0; i < n_len; i++) {
		*(vec_out + i) = *(vec_a + i) + *(vec_b + i);
	  }
	}

	void my_vec_subtract(double *vec_first, double *vec_second, double *vec_out, int n_len) {
	  //Subtract two vectors ... first minus second
	  int i;
	  for (i = 0; i < n_len; i++) {
		*(vec_out + i) = *(vec_first + i) - *(vec_second + i);
	  }
	}

	void my_vec_scalar_mult(double *vec_in, double *vec_out, double num_mult, int n_len) {
	  int i;
	  for (i = 0; i < n_len; i++) {
		*(vec_out + i) = *(vec_in + i) * num_mult;
	  }
	}

	void my_vec_copy(double *vec_in, double *vec_out, int n_len) {
	  int i;
	  for (i = 0; i < n_len; i++) {
		*(vec_out + i) = *(vec_in + i);
	  }
	}

};


#endif