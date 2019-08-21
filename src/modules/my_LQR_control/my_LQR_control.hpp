/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
 /**
 * @file my_LQR_control.hpp
 *
 * my_LQR_control module header file
 *
 * @author Juraj Mihalik <jm1e16@soton.ac.uk>
 */

#include <px4_module.h> // to exist as a module
#include <px4_module_params.h> // to exist as a module
#include <px4_getopt.h> // for the module::instantiate functionality
//#include <px4_log.h>

//#include <px4_defines.h>
//#include <platforms/px4_defines.h>
//#include <px4_config.h>
//#include <px4_tasks.h>
//#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h> // for the ioctl definition
//#include <drivers/drv_mixer.h> // for the ioctl mixer defines (MIXERIOCLOADBUF)
//#include "systemlib/err.h"

#include <px4_posix.h> // for the module polling
#include <matrix/math.hpp> // for the matrix:: library
//#include <mathlib/mathlib.h>
//#include <matrix/Vector.hpp>
//#include <matrix/Matrix.hpp>
#include <drivers/drv_hrt.h> // for the hrt_absolute_time fn
//#include <lib/mixer/mixer.h> // for the load_mixer_file function

#include <perf/perf_counter.h> // for performance counter

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/angular_rates_filtered.h>
#include <uORB/topics/my_LQR_setpoints.h>
#include <uORB/topics/debug_value.h>

using matrix::Matrix;
using matrix::Vector3f;
using matrix::Eulerf;
using matrix::Dcmf;
using matrix::Quatf;

#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>

#define MY_PI 3.14159265359f


extern "C" __EXPORT int my_LQR_control_main(int argc, char *argv[]);

class My_LQR_control : public ModuleBase<My_LQR_control>, public ModuleParams
{
public:
	My_LQR_control(int arg_param, bool arg_flag);

	virtual ~My_LQR_control() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static My_LQR_control *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:


        /**
        * My functions
        */
        int parameter_update_poll();
        int vehicle_local_position_poll();
        int vehicle_local_position_setpoint_poll();
        int manual_control_setpoint_poll();
        int rc_channels_poll();
        int actuator_controls_virtual_poll();
		int home_position_poll();
		int actuator_controls_publish();
		int angular_rates_filtered_publish();
		int setpoints_publish();
		int debug_publish();
		int timer_clock();
		int update_parameters(bool force = false);
		int initialize_variables();
		int local_parameters_update();
		int control_fun();
        int read_y_state();
        int convert_quaternions();
        int filter_rates();
        int read_setpoints();
        int read_y_setpoint();
        int read_c_setpoint();
        int gains_schedule();
        int gains_scale();
        int gains_tune();
        int setpoints_scale();
        int supporting_outputs();
		int bound_control_c();
		int rc_loss_failsafe();
		int printouts();
		int controller_mode();
		int px4_override();
		int project_theta();
		int project_del_psi();
		int del_epsilon_to_body_frame();
		int perturb_control();
		int flip();
		float deg2rad(float);
		float rad2deg(float);
		
		

		/**
        * My subscription/publication topics structures
		*/
        struct parameter_update_s parameter_update{};
        struct vehicle_attitude_s vehicle_attitude{};
        struct vehicle_local_position_s vehicle_local_position{};
        struct vehicle_local_position_setpoint_s vehicle_local_position_setpoint{};
        struct manual_control_setpoint_s manual_control_setpoint{};
        struct rc_channels_s rc_channels{};
		struct actuator_controls_s actuator_controls_0{};        
		struct actuator_controls_s actuator_controls_1{};
		struct actuator_controls_s actuator_controls_virtual{};
		struct home_position_s home_position{};		
		struct angular_rates_filtered_s angular_rates_filtered{};
		struct my_LQR_setpoints_s setpoints_struct{};
		struct debug_value_s dbg_val{};

        /**
        * My subscription topics subsriptors
        */
        int parameter_update_sub = -1;
        int vehicle_attitude_sub = -1;
        int vehicle_local_position_sub = -1;
        int vehicle_local_position_setpoint_sub = -1;
        int manual_control_setpoint_sub = -1;
        int rc_channels_sub = -1;
        int actuator_controls_virtual_sub = -1;
		int home_position_sub = -1;
		
		/**
		* My publication topics publictors
		*/
		orb_advert_t actuator_controls_0_pub = nullptr;		
		orb_advert_t actuator_controls_1_pub = nullptr;
		orb_advert_t angular_rates_filtered_pub = nullptr;
		orb_advert_t setpoints_pub = nullptr;
		orb_advert_t dbg_val_pub = nullptr;

		/**
		* My ORB IDs
		*/
		orb_id_t actuator_controls_virtual_id = ORB_ID(actuator_controls_virtual_fw);

		/**
		* My performance counters
		*/
		perf_counter_t	_loop_perf;			/**< loop performance counter */

        /**
        * My global variables
        */
		uint64_t time_last_run = 0.0f;
		float dt = 0.0f;
		float dt_print = 0.0f;
		float dt_perturb = 0.0f;
		float dt_rcloss = 0.0f;
		float dt_loop = 0.0f;
		float loop_counter = 0.0f;
		float t_start = -1.0f;
		float loop_update_freqn = 250.0f;

        Matrix<float,12,1> y; // state vector
        Matrix<float,4,1> cf; // filtered control
        Matrix<float,6,1> r; // integral of the error in states 
        Matrix<float,8,1> uf; // filtered control surfaces

        Matrix<float,12,1> y_setpoint; 
        Matrix<float,4,1> c_setpoint;
        Matrix<float,6,1> r_setpoint; 
		
		Matrix<float,12,1> Del_y; 
		Matrix<float,4,1> Del_cf; 
		Matrix<float,6,1> Del_r; 
		Matrix<float,4,1> Del_c; 

		Matrix<float,4,12> K_feedback_y; // feedback matrix for the state terms
		Matrix<float,4,6> K_feedback_int; // feedback matrix for the integral terms
		Matrix<float,4,4> K_feedback_cf; // feedback matrix for the filtered control terms
		
		Matrix<float,4,4> Tf; // diagonal matrix of control time delays inverted
		Matrix<float,6,12> Ci; // matrix to pick the states from y to integrate in r

		Matrix<float,4,1> c_nominal_control;

		Matrix<float,12,1> y_max;
		float RC_scale = 1.0f;
		
		Matrix<float,3,1> attitude;
		Eulerf euler_angles;
		Dcmf Qdcm;
		Dcmf Qdcm_proj;

		float thrust_setpoint = 0.0f;
		float pitch_setpoint = 0.0f;
		float yaw_setpoint = 0.0f;

		Vector3f omg;
		Vector3f omg_filtered;
		float angular_rates_cutoff_freqn = 50.0f;
		math::LowPassFilter2pVector3f lp_filter_angular_rates{loop_update_freqn, angular_rates_cutoff_freqn};
		int filter_status = 0;
		
		int case_int = 1;
		int case_int_last = 1;
		float f_int = 0.0f;
		int n_int = 7;
		Matrix<float,8,1> tht_ints;
		Matrix<float,8,8> k_scheds; // [pp, pr, rp, rr, ...]
		Matrix<float,8,8> k_scheds_sc;
		Matrix<float,8,8> k_scheds_sc_tun;
		Matrix<float,8,1> k_scheds_sc_tun_int;
		Matrix<float,4,12> K_feedback_y_sc_tun_sched;
		float rc_sc_omg_last = 0;
		float rc_sc_eps_last = 0;
		int tuner_status = 0;
		float tune_expo = 10.0f;

		Matrix<float,4,12> K_feedback_y_scaled;
		Matrix<float,4,6> K_feedback_int_scaled; 
		Matrix<float,4,4> K_feedback_cf_scaled; 
		Matrix<float,4,12> K_feedback_y_scaled_tuned;
		Matrix<float,4,6> K_feedback_int_scaled_tuned; 
		Matrix<float,4,4> K_feedback_cf_scaled_tuned; 
		Matrix<float,11,1> k_sc_vec; // x,v, p,q,r,phi,theta,psi, cc, cf,ri

		Matrix<float,4,1> Del_c_x; 
		Matrix<float,4,1> Del_c_v; 
		Matrix<float,4,1> Del_c_omg; 
		Matrix<float,4,1> Del_c_eps; 
		Matrix<float,4,1> Del_c_lim; // limits on max Del_c for x,v,omg,eps

		float theta0 = 0.0f;
		bool proj_theta = 0;
		int proj_theta_status = 0;
		bool proj_dpsi = 0;
		int proj_dpsi_status = 0;
		bool schedule_K = 0;
		int schedule_K_status = 0;

		Matrix<float,3,3> E2B; // matrix to convert from euler to body frame
		bool e2b;
		Matrix<float,3,1> Del_y_eps; // the Del eps to be used in feedback

		float tailerons_scaling = 0.3f;
		float motorons_p_scaling = 0.15f;
		float motorons_r_scaling = 0.15f;


		bool do_printouts = 0;

		int pert_idx;
		float pert_time;
		float pert_magnitude;
		bool do_perturb_control = 0;

		float flip_sign;
		bool in_flip = false;
		bool do_flip = 0;

		/**
        * My global decision variables
        */
		int vehicle_id = 0;
		
		


		
		
		
		


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MY_LQR_RC_SCALE>) rc_scale, 
		(ParamFloat<px4::params::MY_LQR_MAX_U>) param_max_u,   
		(ParamFloat<px4::params::MY_LQR_MAX_V>) param_max_v,
		(ParamFloat<px4::params::MY_LQR_MAX_W>) param_max_w,
		(ParamFloat<px4::params::MY_LQR_MAX_P>) param_max_p,
		(ParamFloat<px4::params::MY_LQR_MAX_Q>) param_max_q,
		(ParamFloat<px4::params::MY_LQR_MAX_R>) param_max_r,
		(ParamFloat<px4::params::MY_LQR_MAX_PHI>) param_max_phi,
		(ParamFloat<px4::params::MY_LQR_MAX_THETA>) param_max_theta,
		(ParamFloat<px4::params::MY_LQR_MAX_PSI>) param_max_psi,
        (ParamFloat<px4::params::MY_LQR_F_LAG>) f_lag,
        (ParamFloat<px4::params::MY_LQR_K_SC_X>) k_sc_x,
        (ParamFloat<px4::params::MY_LQR_K_SC_V>) k_sc_v,
        (ParamFloat<px4::params::MY_LQR_K_SC_P>) k_sc_p,
        (ParamFloat<px4::params::MY_LQR_K_SC_Q>) k_sc_q,
        (ParamFloat<px4::params::MY_LQR_K_SC_R>) k_sc_r, 
        (ParamFloat<px4::params::MY_LQR_K_SC_PHI>) k_sc_phi, 
        (ParamFloat<px4::params::MY_LQR_K_SC_THT>) k_sc_tht, 
        (ParamFloat<px4::params::MY_LQR_K_SC_PSI>) k_sc_psi, 
        (ParamFloat<px4::params::MY_LQR_K_SC_CC>) k_sc_cc, // ok this is not consistent but may be useful to be able to tweak it
        (ParamFloat<px4::params::MY_LQR_K_SC_CF>) k_sc_cf,
        (ParamFloat<px4::params::MY_LQR_K_SC_RI>) k_sc_ri,
        (ParamFloat<px4::params::MY_LQR_TUNE_EX>) tune_ex,
        (ParamFloat<px4::params::MY_LQR_DX_LIM>) dx_lim,
        (ParamFloat<px4::params::MY_LQR_DV_LIM>) dv_lim,
        (ParamFloat<px4::params::MY_LQR_DOMG_LIM>) domg_lim,
        (ParamFloat<px4::params::MY_LQR_DEPS_LIM>) deps_lim,
        (ParamInt<px4::params::MY_LQR_PERT_I>) param_pert_idx,
        (ParamFloat<px4::params::MY_LQR_PERT_T>) param_pert_time,
        (ParamFloat<px4::params::MY_LQR_PERT_M>) param_pert_magnitude,
        (ParamInt<px4::params::MY_LQR_DO_FLIP>) bool_do_flip,
        (ParamInt<px4::params::MY_LQR_DO_PERT>) bool_perturb_control,
        (ParamFloat<px4::params::MY_LQR_RTS_CTF>) angular_rates_cutoff_fn,
        (ParamFloat<px4::params::MY_LQR_TAILERONS>) tailerons_sc,
        (ParamFloat<px4::params::MY_LQR_MOTORONSP>) motorons_p_sc,
        (ParamFloat<px4::params::MY_LQR_MOTORONSR>) motorons_r_sc,
        (ParamInt<px4::params::MY_LQR_BOOL_SCHD>) bool_K_sched,
        (ParamInt<px4::params::MY_LQR_BOOL_PRNT>) bool_printouts,
        (ParamInt<px4::params::MY_LQR_BOOL_PTT>) bool_proj_tht,
        (ParamInt<px4::params::MY_LQR_BOOL_PDP>) bool_proj_dpsi,
        (ParamInt<px4::params::MY_LQR_BOOL_E2B>) bool_e2b
        )// Just the handles, need to .get() them
};

