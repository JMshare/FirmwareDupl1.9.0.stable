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
 * @file my_LQR_control.cpp
 *
 * my_LQR_control module
 *
 * @author Juraj Mihalik <jm1e16@soton.ac.uk>
 */

#include "my_LQR_control.hpp"
#include <mathlib/mathlib.h>


int My_LQR_control::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
This implements the LQR controller. 
So far only attitude control.

### Implementation
The control loop polls on the sensors combined topic.

### Examples
To load S500 quadcopter gains:
$ my_LQR_control start -p 1
To load Custer gains:
$ my_LQR_control start -p 2
To load Custer HITL gains:
$ my_LQR_control start -p 3

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("my_LQR_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Vehicle ID.\n\t\tQuadS500: 1\n\t\tCuster: 2", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int My_LQR_control::print_status()
{
        PX4_INFO("Running in mode %d", vehicle_id);
        // TODO: print additional runtime information about the state of the module

        return 0;
}

int My_LQR_control::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("my_LQR_control",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_ATTITUDE_CONTROL,
                                      4000, // stack size
                                      (px4_main_t)&run_trampoline,
                                      (char *const *)argv);

        if (_task_id < 0) {
                _task_id = -1;
                return -errno;
        }

        return 0;
}

My_LQR_control *My_LQR_control::instantiate(int argc, char *argv[])
{
        int arg_param = 0;
        bool arg_flag = false;
        bool error_flag = false;

        int myoptind = 1;
        int ch;
        const char *myoptarg = nullptr;

        // parse CLI arguments
        while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
                switch (ch) {
                case 'p':
                        arg_param = (int)strtol(myoptarg, nullptr, 10);
                        break;

                case 'f':
                        arg_flag = true;
                        break;

                case '?':
                        error_flag = true;
                        break;

                default:
                        PX4_WARN("unrecognized flag");
                        error_flag = true;
                        break;
                }
        }

        if (error_flag) {
                return nullptr;
        }

        My_LQR_control *instance = new My_LQR_control(arg_param, arg_flag);

        if (instance == nullptr) {
                PX4_ERR("alloc failed");
        }

        return instance;
}

My_LQR_control::My_LQR_control(int arg_param, bool arg_flag)
        : ModuleParams(nullptr), _loop_perf(perf_alloc(PC_ELAPSED, "my_LQR_control"))
{
    vehicle_id = arg_param;
}

int My_LQR_control::custom_command(int argc, char *argv[])
{
        /*
        if (!is_running()) {
                print_usage("not running");
                return 1;
        }

        // additional custom commands can be handled like this:
        if (!strcmp(argv[0], "do-something")) {
                get_instance()->do_something();
                return 0;
        }
         */

        return print_usage("unknown command");
}



int My_LQR_control::parameter_update_poll()
{
    bool parameter_update_updated;
    orb_check(parameter_update_sub, &parameter_update_updated);
    if(parameter_update_updated){
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &parameter_update);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::vehicle_local_position_poll(){
    bool vehicle_local_position_updated;
    orb_check(vehicle_local_position_sub, &vehicle_local_position_updated);
    if(vehicle_local_position_updated){
        orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::vehicle_local_position_setpoint_poll(){
    bool vehicle_local_position_setpoint_updated;
    orb_check(vehicle_local_position_setpoint_sub, &vehicle_local_position_setpoint_updated);
    if(vehicle_local_position_setpoint_updated){
        orb_copy(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_sub, &vehicle_local_position_setpoint);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::manual_control_setpoint_poll(){
    bool manual_control_setpoint_updated;
    orb_check(manual_control_setpoint_sub, &manual_control_setpoint_updated);
    if(manual_control_setpoint_updated){
        orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_setpoint);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::rc_channels_poll(){
    bool rc_channels_updated;
    orb_check(rc_channels_sub, &rc_channels_updated);
    if(rc_channels_updated){
        orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::actuator_controls_virtual_poll(){
    bool actuator_controls_virtual_updated;
    orb_check(actuator_controls_virtual_sub, &actuator_controls_virtual_updated);
    if(actuator_controls_virtual_updated){
        orb_copy(actuator_controls_virtual_id, actuator_controls_virtual_sub, &actuator_controls_virtual);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::home_position_poll(){
    bool home_position_updated;
    orb_check(home_position_sub, &home_position_updated);
    if(home_position_updated){
        orb_copy(ORB_ID(home_position), home_position_sub, &home_position);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::actuator_controls_publish(){
    bound_controls(); // bounds pqr control to (-1,1) and thrust to (0,1)

    for(int i=0; i < 4; i++){
        actuator_controls_0.control[i] = cf(i,0);
    }
    actuator_controls_0.control[4] = 0.0f;
    actuator_controls_0.control[5] = 0.0f;
    actuator_controls_0.control[6] = 0.0f;
    actuator_controls_0.control[7] = -1.0f;
    actuator_controls_0.timestamp = hrt_absolute_time();
    actuator_controls_0.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(actuator_controls_0), actuator_controls_0_pub, &actuator_controls_0) != PX4_OK){
        PX4_WARN("Unable to publish actuator_controls0!");
    }

    for(int i=0; i < 8; i++){
        actuator_controls_1.control[i] = uf(i,0);
    }
    actuator_controls_1.timestamp = hrt_absolute_time();
    actuator_controls_1.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(actuator_controls_1), actuator_controls_1_pub, &actuator_controls_1) != PX4_OK){
        PX4_WARN("Unable to publish actuator_controls1!");
    }
    
    return PX4_OK;
}

int My_LQR_control::angular_rates_filtered_publish(){
    angular_rates_filtered.rollspeed = omg_filtered(0);
    angular_rates_filtered.pitchspeed = omg_filtered(1);
    angular_rates_filtered.yawspeed = omg_filtered(2);
    angular_rates_filtered.loop_update_freqn = loop_update_freqn;
    angular_rates_filtered.cutoff_freqn = angular_rates_cutoff_freqn;
    angular_rates_filtered.filter_status = filter_status;
    
    angular_rates_filtered.timestamp = hrt_absolute_time();
    angular_rates_filtered.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(angular_rates_filtered), angular_rates_filtered_pub, &angular_rates_filtered) != PX4_OK){
        PX4_WARN("Unable to publish angular_rates_filtered!");
    }
    
    return PX4_OK;
}

int My_LQR_control::setpoints_publish(){
    setpoints_struct.y0 = y(6,0);
    setpoints_struct.y1 = y(7,0);
    setpoints_struct.y2 = y(8,0);
    setpoints_struct.y3 = y(9,0);
    setpoints_struct.y4 = y(10,0);
    setpoints_struct.y5 = y(11,0);
    setpoints_struct.y6 = y_setpoint(6,0);
    setpoints_struct.y7 = y_setpoint(7,0);
    setpoints_struct.y8 = y_setpoint(8,0);
    setpoints_struct.y9 = y_setpoint(9,0);
    setpoints_struct.y10 = y_setpoint(10,0);
    setpoints_struct.y11 = y_setpoint(11,0);

    setpoints_struct.c0 = c_setpoint(0,0);
    setpoints_struct.c1 = c_setpoint(1,0);
    setpoints_struct.c2 = c_setpoint(2,0);
    setpoints_struct.c3 = c_setpoint(3,0);

    setpoints_struct.k_omg_pp = K_feedback_y_sc_tun_sched(0,6);
    setpoints_struct.k_omg_pr = K_feedback_y_sc_tun_sched(0,8);
    setpoints_struct.k_omg_rp = K_feedback_y_sc_tun_sched(2,6);
    setpoints_struct.k_omg_rr = K_feedback_y_sc_tun_sched(2,8);
    setpoints_struct.k_omg_qq = K_feedback_y_sc_tun_sched(1,7);

    setpoints_struct.k_eps_phiphi = K_feedback_y_sc_tun_sched(0,9);
    setpoints_struct.k_eps_phipsi = K_feedback_y_sc_tun_sched(0,11);
    setpoints_struct.k_eps_psiphi = K_feedback_y_sc_tun_sched(2,9);
    setpoints_struct.k_eps_psipsi = K_feedback_y_sc_tun_sched(2,11);
    setpoints_struct.k_eps_thttht = K_feedback_y_sc_tun_sched(1,10);

    setpoints_struct.del_c_omg_p = Del_c_omg(0,0);
    setpoints_struct.del_c_omg_q = Del_c_omg(1,0);
    setpoints_struct.del_c_omg_r = Del_c_omg(2,0);
    setpoints_struct.del_c_eps_phi = Del_c_eps(0,0);
    setpoints_struct.del_c_eps_tht = Del_c_eps(1,0);
    setpoints_struct.del_c_eps_psi = Del_c_eps(2,0);

    setpoints_struct.pitch_setpoint = rad2deg(pitch_setpoint);

    setpoints_struct.proj_dpsi_status = proj_dpsi_status;
    setpoints_struct.dpsi = rad2deg(Del_y_eps(2,0));
    setpoints_struct.proj_theta_status = proj_theta_status;

    setpoints_struct.tuner_status = tuner_status;
    setpoints_struct.gain_scale_p = tune_p;
    setpoints_struct.gain_scale_d = tune_d;
    setpoints_struct.case_int_f_int = 1.0f*case_int + f_int;

    setpoints_struct.do_adaptive = do_adaptive;
    setpoints_struct.do_adapt_p = adapt_A(0,0);
    setpoints_struct.do_adapt_q = adapt_A(1,0);
    setpoints_struct.do_adapt_r = adapt_A(2,0);
    setpoints_struct.alpha_p = alpha_A(0,0);
    setpoints_struct.alpha_q = alpha_A(1,0);
    setpoints_struct.alpha_r = alpha_A(2,0);
    setpoints_struct.i_error_p = I_error_A(0,0);
    setpoints_struct.i_error_q = I_error_A(1,0);
    setpoints_struct.i_error_r = I_error_A(2,0);
    setpoints_struct.error_p = error_A(0,0);
    setpoints_struct.error_q = error_A(1,0);
    setpoints_struct.error_r = error_A(2,0);
    setpoints_struct.domg_p = Domg_A(0,0);
    setpoints_struct.domg_q = Domg_A(1,0);
    setpoints_struct.domg_r = Domg_A(2,0);
    setpoints_struct.domg_pred_p = Domg_pred_A(0,0);
    setpoints_struct.domg_pred_q = Domg_pred_A(1,0);
    setpoints_struct.domg_pred_r = Domg_pred_A(2,0); 
    setpoints_struct.p_a = p_A;
    setpoints_struct.gamma = gamma_A;
    setpoints_struct.dt = dt;

    setpoints_struct.x_rls1 = X_RLS(0,0);
    setpoints_struct.x_rls2 = X_RLS(1,0);

    setpoints_struct.kp_rls = K_feedback_y_sc_tun_sched_adapt(0,6);
    setpoints_struct.kphi_rls = K_feedback_y_sc_tun_sched_adapt(0,9);



    setpoints_struct.timestamp = hrt_absolute_time();
    setpoints_struct.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(my_LQR_setpoints), setpoints_pub, &setpoints_struct) != PX4_OK){
        PX4_WARN("Unable to publish my_setpoints!");
    }
    
    return PX4_OK;
}

int My_LQR_control::debug_publish(){
    dbg_val.ind = 0;
    dbg_val.value = y(6,0); // roll rate to see filtered vibrations
    dbg_val.value = alpha_A(0,0); // adaptive factor
    dbg_val.value = X_RLS(1,0); // identified values
    
    dbg_val.timestamp = hrt_absolute_time();

    if(orb_publish(ORB_ID(debug_value), dbg_val_pub, &dbg_val) != PX4_OK){
        PX4_WARN("Unable to publish debug value!");
    }
    
    return PX4_OK;
}

int My_LQR_control::publish_topics(){
    actuator_controls_publish();
    angular_rates_filtered_publish();
    setpoints_publish();
    debug_publish();
    return PX4_OK;
}




/*----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------*/
void My_LQR_control::run(){
    // subscribe to topics
    parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    actuator_controls_virtual_sub = orb_subscribe(actuator_controls_virtual_id);
    home_position_sub = orb_subscribe(ORB_ID(home_position));
    
    // advertise topics
    actuator_controls_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuator_controls_0);    
    actuator_controls_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &actuator_controls_1);
    angular_rates_filtered_pub = orb_advertise(ORB_ID(angular_rates_filtered), &angular_rates_filtered);
    setpoints_pub = orb_advertise(ORB_ID(my_LQR_setpoints), &setpoints_struct);
    dbg_val_pub = orb_advertise(ORB_ID(debug_value), &dbg_val);
    
    // initialize parameters
    initialize_variables();
    

    // Run the loop synchronized to the vehicle_attitude topic.
    px4_pollfd_struct_t fds[1];
    fds[0].fd = vehicle_attitude_sub;
    fds[0].events = POLLIN;
    int pret;

    while (!should_exit()) {
        
        // wait for up to 100ms for data
        pret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 100);

        if (pret == 0) {
            continue;
        }
        else if (pret < 0) {
            // this is undesirable but not much we can do
            PX4_ERR("poll error %d, %d", pret, errno);
            px4_usleep(50000);
            continue;
        }
        else {
            perf_begin(_loop_perf);
            if (fds[0].revents & POLLIN) {
                timer_clock(); // manages time to have a reliable dt variable
                
                read_y_state();
                
                read_setpoints();

                gains_tune(); // gains tune based on RC knobs
                gains_schedule(); // gains schedule based on pitch angle

                recursiveLS();
                adaptive_control();
                
                control_fun(); // computes the actuator controls

                manual_override(); // overrides the controls by manual RC input based on RC switches

                px4_override(); // overrides the controls by the PX4 controller based on RC switches

                supporting_outputs(); // front engine and tailerons and differential thrust to mixer based on RC switches

                rc_loss_failsafe();
                
                publish_topics();

                printouts();
                
            }
            perf_end(_loop_perf);
        }
        update_parameters();
    }
    
    
    // Unsubscribe
    orb_unsubscribe(parameter_update_sub);
    orb_unsubscribe(vehicle_attitude_sub);
    orb_unsubscribe(vehicle_local_position_sub);
    orb_unsubscribe(vehicle_local_position_setpoint_sub);
    orb_unsubscribe(manual_control_setpoint_sub);
    orb_unsubscribe(rc_channels_sub);
    orb_unsubscribe(actuator_controls_virtual_sub);
    orb_unsubscribe(home_position_sub);

    // unadvertise topics
    orb_unadvertise(actuator_controls_0_pub);    
    orb_unadvertise(actuator_controls_1_pub);
    orb_unadvertise(angular_rates_filtered_pub);
    orb_unadvertise(setpoints_pub);
    orb_unadvertise(dbg_val_pub);
}
/* -----------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------*/




int My_LQR_control::timer_clock(){
    if(t_start < 0.0f){
        t_start = vehicle_attitude.timestamp;
    }
    dt = (vehicle_attitude.timestamp - time_last_run)/1000000.0f;
    time_last_run = vehicle_attitude.timestamp;
    
    loop_counter = loop_counter + 1.0f;
    dt_loop = dt_loop + dt;
    if(dt_loop >= 10.0f){ // use 10 seconds to compute the loop update rate
        if(fabsf(loop_update_freqn - loop_counter/dt_loop) > 10.0f){ // if loop freqn change by more than 10 Hz update the filter
            loop_update_freqn = loop_counter/dt_loop;
            lp_filter_angular_rates.set_cutoff_frequency(loop_update_freqn, angular_rates_cutoff_freqn);
        }
        dt_loop = 0.0f;
        loop_counter = 0.0f;
    }
    return PX4_OK;
}




int My_LQR_control::read_y_state(){
    orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);
    vehicle_local_position_poll();
    //vehicle_local_position_setpoint_poll();
    convert_quaternions();
    filter_rates();
        
    y( 0,0) = 0.0f*vehicle_local_position.x;
    y( 1,0) = 0.0f*vehicle_local_position.y;
    y( 2,0) = 0.0f*vehicle_local_position.z;
    y( 3,0) = 0.0f*vehicle_local_position.vx;
    y( 4,0) = 0.0f*vehicle_local_position.vy;
    y( 5,0) = 0.0f*vehicle_local_position.vz;
    y( 6,0) = 1.0f*omg_filtered(0);
    y( 7,0) = 1.0f*omg_filtered(1);
    y( 8,0) = 1.0f*omg_filtered(2);
    y( 9,0) = 1.0f*attitude(0,0);
    y(10,0) = 1.0f*attitude(1,0);
    y(11,0) = 1.0f*attitude(2,0);

    return PX4_OK;
}
int My_LQR_control::filter_rates(){
    omg(0) = vehicle_attitude.rollspeed;
    omg(1) = vehicle_attitude.pitchspeed;
    omg(2) = vehicle_attitude.yawspeed;

    if(angular_rates_cutoff_freqn <= 100.0f){
        omg_filtered = lp_filter_angular_rates.apply(omg);
        filter_status = 0; // ok
        if(!(omg_filtered(0) > -1000000.0f && omg_filtered(1) > -1000000.0f && omg_filtered(2) > -1000000.0f)){ // safety check, if NAN this should come to false
            omg_filtered = omg*0.0f; // turn it off to prevent feeding vibrations to servos
            filter_status = 1; // whops
        }
    }
    else{
        omg_filtered = omg; 
        filter_status = 2; // no cutoff
    }

    return PX4_OK;
}
int My_LQR_control::convert_quaternions(){
    Qdcm = Quatf(vehicle_attitude.q);
    euler_angles = Qdcm; // this is how you convert the Dcm into Euler angles (readme.md in matrix lib)
    project_theta();

    attitude(0,0) = euler_angles.phi();
    attitude(1,0) = euler_angles.theta();
    attitude(2,0) = euler_angles.psi();
    return PX4_OK;
}
int My_LQR_control::project_theta(){
// Extending the -90 to +90 deg range on theta to -110 to 110 deg
    proj_theta_status = 0;
    theta0 = euler_angles.theta();
    if(proj_theta){
        proj_theta_status = 1;
        Qdcm_proj = Qdcm; // predefine
        if(euler_angles.theta() > deg2rad(50.0f)){ 
            // rotate the device by -90 deg
            Qdcm_proj(0,0) = Qdcm(0,2); // z to x
            Qdcm_proj(1,0) = Qdcm(1,2);
            Qdcm_proj(2,0) = Qdcm(2,2);

            Qdcm_proj(0,2) = -Qdcm(0,0); // x to -z
            Qdcm_proj(1,2) = -Qdcm(1,0);
            Qdcm_proj(2,2) = -Qdcm(2,0);

            euler_angles = Qdcm_proj; // get the corresponding euler angles
            euler_angles.theta() = euler_angles.theta() + deg2rad(90.0f); // bring back the unrotated theta
            proj_theta_status = 10; // log status
        }
        else if(euler_angles.theta() < -deg2rad(50.0f)){ 
            // rotate the device by 90 deg
            Qdcm_proj(0,0) = -Qdcm(0,2); // z to -x
            Qdcm_proj(1,0) = -Qdcm(1,2);
            Qdcm_proj(2,0) = -Qdcm(2,2);

            Qdcm_proj(0,2) = Qdcm(0,0); // x to z
            Qdcm_proj(1,2) = Qdcm(1,0);
            Qdcm_proj(2,2) = Qdcm(2,0);

            euler_angles = Qdcm_proj; // get the corresponding euler angles
            euler_angles.theta() = euler_angles.theta() - deg2rad(90.0f); // bring back the unrotated theta
            proj_theta_status = -10; // log status
        }
    }

    return PX4_OK;
}




int My_LQR_control::read_setpoints(){
    // manual_control_setpoint_poll();
    rc_channels_poll();
    read_y_setpoint();
    read_c_setpoint();
    setpoints_scale();
    return PX4_OK;
}
int My_LQR_control::read_c_setpoint(){
    c_setpoint = c_nominal_control;

    // Position/ velocity control
    /* all c_setpoints are equal to nominal control */

    // Attitude control
    /* we control thrust directly by rc */
    thrust_setpoint = rc_channels.channels[3] - 0.5f; 
    c_setpoint(3,0) = math::constrain(c_nominal_control(3,0) + thrust_setpoint, 0.0f, 1.0f);

    return PX4_OK;
}
int My_LQR_control::read_y_setpoint(){
    y_setpoint.setAll(0.0f);

    pitch_setpoint = ((rc_channels.channels[9] + 1.0f)/2.0f)*deg2rad(pitch_sp_max); // 0 to pitch_sp_max deg based on RS stick input
    //pitch_setpoint = rc_channels.channels[9]*deg2rad(pitch_sp_max); // -90 to 90 deg based on RS stick input just for test

    /* 
    // Position/ velocity control
    y_setpoint(3,0) = rc_deadband(manual_control_setpoint.x)*y_setpoint_scale(3,0);
    y_setpoint(4,0) = rc_deadband(manual_control_setpoint.y)*y_setpoint_scale(4,0);
    y_setpoint(5,0) = -(2.0f*rc_deadband(manual_control_setpoint.z) - 1.0f)*y_setpoint_scale(5,0);
    y_setpoint(8,0) = rc_deadband(manual_control_setpoint.r)*y_setpoint_scale(8,0);
    */

    // Attitude control
    y_setpoint(6,0) =  rc_channels.channels[0];
    y_setpoint(7,0) = -rc_channels.channels[1];
    y_setpoint(8,0) =  rc_channels.channels[2];
    y_setpoint(9,0) = 0.0f;
    y_setpoint(10,0) = pitch_setpoint;
    y_setpoint(11,0) = yaw_setpoint;
    if(fabsf(y_setpoint(8,0)) > 0.1f || ((fabsf(y_setpoint(6,0)) > 0.1f) && ((vehicle_id == 2) || (vehicle_id == 3))) || fabsf(yaw_setpoint) <= 0.0f){ // at yaw||(roll if fw) rate command or at the startup
        yaw_setpoint = y(11,0);
    }

    return PX4_OK;
}
int My_LQR_control::setpoints_scale(){
// RC pass through scaled by RC_scale cos i.e. for multicopters we don't want such high rates
// also p,q,r can react differently to cp,cq,cr in fixed-wing plane

    RC_scale = RC_scale_base;
    if(do_rc_scale){
        for(int i=0; i<3; i++){ // scaling the RC input up based on K_eps gains, so that if K_eps high, I can stil move the plane without the p-compensation pushing me back to zero
            f_scale = K_feedback_y_sc_tun_sched_adapt(i,9+i);
            p_scale = fabsf(y(9+i,0))/RC_scale_base(i,0);
            if(p_scale < 1.0f){
                f_scale = 1.0f + (K_feedback_y_sc_tun_sched_adapt(i,9+i) - 1)*p_scale;
            }
            RC_scale(i,0) *= f_scale;
        }
    }

    y_setpoint(6,0) *= RC_scale(0,0);
    y_setpoint(7,0) *= RC_scale(1,0);
    y_setpoint(8,0) *= RC_scale(2,0);

    return PX4_OK;
}




int My_LQR_control::gains_scale(){
    k_sc_vec(0,0) = k_sc_x.get();
    k_sc_vec(1,0) = k_sc_v.get();
    k_sc_vec(2,0) = k_sc_p.get();
    k_sc_vec(3,0) = k_sc_q.get();
    k_sc_vec(4,0) = k_sc_r.get();
    k_sc_vec(5,0) = k_sc_phi.get();
    k_sc_vec(6,0) = k_sc_tht.get();
    k_sc_vec(7,0) = k_sc_psi.get();
    k_sc_vec(8,0) = k_sc_ccd.get();
    k_sc_vec(9,0) = k_sc_ccp.get();
    k_sc_vec(10,0) = k_sc_cf.get();
    k_sc_vec(11,0) = k_sc_ri.get();

    K_feedback_y_scaled = K_feedback_y;
    for(int i=0; i<4; i++){
        for(int j=0; j<3; j++){
            K_feedback_y_scaled(i,j) *= k_sc_vec(0,0); // x
        }
        for(int j=3; j<6; j++){
            K_feedback_y_scaled(i,j) *= k_sc_vec(1,0); // v
        }
        K_feedback_y_scaled(i,6) *= k_sc_vec(2,0); // p
        K_feedback_y_scaled(i,7) *= k_sc_vec(3,0); // q
        K_feedback_y_scaled(i,8) *= k_sc_vec(4,0); // r
        K_feedback_y_scaled(i,9) *= k_sc_vec(5,0); // phi
        K_feedback_y_scaled(i,10) *= k_sc_vec(6,0); // tht
        K_feedback_y_scaled(i,11) *= k_sc_vec(7,0); // psi
    }
    K_feedback_y_scaled(0,8) *= k_sc_vec(8,0); // cross-coupling d
    K_feedback_y_scaled(2,6) *= k_sc_vec(8,0);
    K_feedback_y_scaled(0,11) *= k_sc_vec(9,0); // cross-coupling p
    K_feedback_y_scaled(2,9) *= k_sc_vec(9,0);


    K_feedback_cf_scaled = K_feedback_cf;
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            K_feedback_cf_scaled *= k_sc_vec(10,0);
        }
    }

    K_feedback_int_scaled = K_feedback_int;
    for(int i=0; i<4; i++){
        for(int j=0; j<6; j++){
            K_feedback_int_scaled *= k_sc_vec(11,0);
        }
    }


    k_scheds_sc = k_scheds;
    for(int j=0; j<n_int+1; j++){ // for all thetas
        k_scheds_sc(0,j) *= k_sc_vec(2,0); // p
        k_scheds_sc(1,j) *= k_sc_vec(2,0); // pr
        k_scheds_sc(2,j) *= k_sc_vec(4,0); // rp
        k_scheds_sc(3,j) *= k_sc_vec(4,0); // r
        k_scheds_sc(4,j) *= k_sc_vec(5,0); // phi
        k_scheds_sc(5,j) *= k_sc_vec(5,0); // phipsi
        k_scheds_sc(6,j) *= k_sc_vec(7,0); // psiphi
        k_scheds_sc(7,j) *= k_sc_vec(7,0); // psi
        k_scheds_sc(1,j) *= k_sc_vec(8,0); // cross-coupling [pr, rp]
        k_scheds_sc(2,j) *= k_sc_vec(8,0); 
        k_scheds_sc(5,j) *= k_sc_vec(9,0); // cross-coupling [phipsi, psiphi]
        k_scheds_sc(6,j) *= k_sc_vec(9,0); 
        k_scheds_sc(8,j) *= k_sc_vec(3,0); // q
        k_scheds_sc(9,j) *= k_sc_vec(6,0); // tht
    }

    rc_sc_eps_last = 0.0f; // to wake up the tuner as well
    rc_sc_omg_last = 0.0f;
    K_feedback_y_scaled_tuned = K_feedback_y_scaled;
    K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned;
    k_scheds_sc_tun = k_scheds_sc;

    return PX4_OK;
}
int My_LQR_control::gains_tune(){
// Tune the gains using the radio knobs
    tuner_status = 0;
    if(fabsf(rc_sc_omg_last - rc_channels.channels[11]) > 0.002f || fabsf(rc_sc_eps_last - rc_channels.channels[10]) > 0.002f){
        tuner_status = 1;
        rc_sc_omg_last = rc_channels.channels[11];
        rc_sc_eps_last = rc_channels.channels[10];

        tune_d = powf(tune_expo, rc_channels.channels[11]);
        tune_p = powf(tune_expo, rc_channels.channels[10]);

        K_feedback_y_scaled_tuned = K_feedback_y_scaled;
        for(int i=0; i<4; i++){
            for(int j=6; j<9; j++){
                K_feedback_y_scaled_tuned(i,j) *= tune_d;
            }
            for(int j=9; j<12; j++){
                K_feedback_y_scaled_tuned(i,j) *= tune_p;
            }
        }

        k_scheds_sc_tun = k_scheds_sc;
        for(int j=0; j<n_int+1; j++){
            for(int i=0; i<4; i++){
                k_scheds_sc_tun(i,j) *= tune_d; // omg
            }
            k_scheds_sc_tun(8,j) *= tune_d; // p
            for(int i=4; i<8; i++){
                k_scheds_sc_tun(i,j) *= tune_p; // eps
            }
            k_scheds_sc_tun(9,j) *= tune_p; // tht
        }

        // trigger the scheduler as well if tuning changed
        case_int_last = -1;
        K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned; // if schedule_K == 0 it updates here 
    }

    return PX4_OK;
}
int My_LQR_control::gains_schedule(){
    if(schedule_K == 1){ 
        schedule_K_status = 1;
        for(int i = 0; i < n_int; i++){ // find interval
            f_int = (theta0 - tht_ints(0,i))/(tht_ints(0,i+1) - tht_ints(0,i));
            if(f_int >= 0.0f && f_int <= 1.0f){
                case_int = i;
                break;
            }
        }
        case_int_last = 100; // comment if you just want a step function
        if(case_int_last != case_int){ // interpolate
            // f_int = 0.0f; // zero order interpolation
            case_int_last = case_int;
            for(int i=0; i<10; i++){
                k_scheds_sc_tun_int(i,0) = (1.0f-f_int)*k_scheds_sc_tun(i,case_int) + f_int*k_scheds_sc_tun(i,case_int+1);
            }
            K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned;
            K_feedback_y_sc_tun_sched(0,6) = k_scheds_sc_tun_int(0,0);
            K_feedback_y_sc_tun_sched(0,8) = k_scheds_sc_tun_int(1,0);
            K_feedback_y_sc_tun_sched(1,7) = k_scheds_sc_tun_int(8,0);
            K_feedback_y_sc_tun_sched(2,6) = k_scheds_sc_tun_int(2,0);
            K_feedback_y_sc_tun_sched(2,8) = k_scheds_sc_tun_int(3,0);
            K_feedback_y_sc_tun_sched(0,9) = k_scheds_sc_tun_int(4,0);
            K_feedback_y_sc_tun_sched(0,11) = k_scheds_sc_tun_int(5,0);
            K_feedback_y_sc_tun_sched(1,10) = k_scheds_sc_tun_int(9,0);
            K_feedback_y_sc_tun_sched(2,9) = k_scheds_sc_tun_int(6,0);
            K_feedback_y_sc_tun_sched(2,11) = k_scheds_sc_tun_int(7,0);
        }
    }
    else{
        schedule_K_status = 0;
        case_int = -1;
        f_int = 0.0f;
    }

    return PX4_OK;
}
/*
int My_LQR_control::adaptive_control(){
    if(do_adaptive){
        Omg_A = omg_filtered;

        Domg_A = (Omg_A - Omg_prev_A)/dt;
        Domg_pred_A = p_A*(A_A*Omg_prev_A + B_A*C_prev_A);
        error_A = Domg_pred_A - Domg_A;

        for(int i=0; i<3; i++){
            if(fabsf(Domg_A(i,0)) > 60.0f || fabsf(error_A(i,0)) > p_A*30.0f || dt > 0.3f || dt < 0.002f|| fabsf(Omg_A(i,0)) < 0.001f){
                error_A(i,0) = 0.0f;
            }
            I_error_A(i,0) = I_error_A(i,0) + gamma_A*error_A(i,0)*copysignf(1.0f, Domg_pred_A(i,0));
            I_error_A(i,0) = math::constrain(I_error_A(i,0), -2.0f, 2.0f);
            alpha_A(i,0) = powf(2.0f, I_error_A(i,0)*adapt_A(i,0));
            alpha_A(i,0) = math::constrain(alpha_A(i,0), 0.25f, 4.0f);

            C_prev_A(i,0) = Del_c(i,0);
            Del_c_adapt(i,0) = Del_c_eps(i,0)*c_eps_bool(i,0)*alpha_A(i,0) + Del_c_omg(i,0)/alpha_A(i,0);
        }

        Omg_prev_A = Omg_A;
    }

    else{
        Del_c_adapt = Del_c;
        alpha_A.setAll(1.0f);
        I_error_A.setAll(0.0f);
        C_prev_A.setAll(0.0f);
    }

    return PX4_OK;
}
*/
int My_LQR_control::recursiveLS(){
    if(do_recursiveLS){
        M_RLS(0,0) = p_prev_RLS(0,0);
        M_RLS(0,1) = cf(0,0);
        Y_RLS(0,0) = (omg_filtered(0) - p_prev_RLS(0,0))/dt;
        E_RLS = (Y_RLS - M_RLS*X_RLS);
        E_RLS(0,0) = math::constrain(E_RLS(0,0), -5.0f, 5.0f);
        if(fabsf(Y_RLS(0,0)) < 50.0f && fabsf(Y_RLS(0,0)) > 1.0f && dt < 0.3f && dt > 0.002f){
            fract_RLS = M_RLS*P_RLS*M_RLS.T();
            P_RLS = (P_RLS - (P_RLS*M_RLS.T()*M_RLS*P_RLS)/(lambda_RLS + fract_RLS(0,0))) / lambda_RLS;
            X_RLS = X_RLS + P_RLS*M_RLS.T()*E_RLS;
            X_RLS(0,0) = math::constrain(X_RLS(0,0), -30.0f, 0.0f);
            X_RLS(1,0) = math::constrain(X_RLS(1,0), 0.0f, 60.0f);
        }
        p_prev_RLS(0,0) = omg_filtered(0);
    }
    else{
        X_RLS = X0_RLS;
        P_RLS = P0_RLS;
    }
    return PX4_OK;
}
int My_LQR_control::adaptive_control(){
    K_feedback_y_sc_tun_sched_adapt = K_feedback_y_sc_tun_sched;
    if(do_adaptive){
        K_feedback_y_sc_tun_sched_adapt(0,6) = -(ap_adapt - bp_adapt*kp_adapt - X_RLS(0,0))/X_RLS(1,0);
        K_feedback_y_sc_tun_sched_adapt(0,9) = -(0.0f - bp_adapt*kphi_adapt - 0.0f)/X_RLS(1,0);

        K_feedback_y_sc_tun_sched_adapt(0,6) = math::constrain(K_feedback_y_sc_tun_sched_adapt(0,6), 0.0f, 10.0f);
        K_feedback_y_sc_tun_sched_adapt(0,9) = math::constrain(K_feedback_y_sc_tun_sched_adapt(0,9), 0.0f, 10.0f);
    }

    return PX4_OK;
}



int My_LQR_control::control_fun(){
// Main controller function
    Del_y  = y  - y_setpoint;
    //Del_y(11,0) = math::constrain(Del_y(11,0), -0.26f, 0.26f); // limit the yaw error to +-15 deg (0.26rad) not to freak out when heading too off course

    //Del_cf = cf - c_setpoint; // not used if not filter
    //Del_r  = r  - r_setpoint; // not used if no integral compensation

    //Del_c = K_feedback_y*Del_y ... ; // not used if not filter

    if(dt < 0.5f){ // ignore too large dt steps (probably some glitches, startup etc)
        //r  = r  + dt*Ci*Del_y; // not used if no integral compensation
        //cf = cf + dt*Tf*(-Del_cf + Del_c); // not used if not filter
    }
    
    Del_y_eps = Del_y.slice<3,1>(9,0);
    project_del_psi();
    del_epsilon_to_body_frame(); // depreciated

    //Del_c_x   = -K_feedback_y_sc_tun_sched.T().slice<3,4>(0,0).T()*Del_y.slice<3,1>(0,0); // slice x contribution
    //Del_c_v   = -K_feedback_y_sc_tun_sched.T().slice<3,4>(3,0).T()*Del_y.slice<3,1>(3,0); // slice v contribution
    // Del_c_omg = -K_feedback_y_sc_tun_sched.T().slice<3,4>(6,0).T()*Del_y.slice<3,1>(6,0); // slice omg contribution
    Del_c_omg = -K_feedback_y_sc_tun_sched_adapt.T().slice<3,4>(6,0).T()*y.slice<3,1>(6,0); // slice omg contribution, this way RC input indep of K
    Del_c_omg(0,0) += y_setpoint(6,0); // p
    Del_c_omg(1,0) += y_setpoint(7,0); // q
    Del_c_omg(2,0) += y_setpoint(8,0); // r
    Del_c_eps = -K_feedback_y_sc_tun_sched_adapt.T().slice<3,4>(9,0).T()*Del_y_eps; // sliced eps contribution
    for(int i = 0; i < 4; i++){ // not necessarily (-1,1), just a sanity check against NaNs
        Del_c_x(i,0)   = math::constrain(Del_c_x(i,0)  , -Del_c_lim(0,0), Del_c_lim(0,0));
        Del_c_v(i,0)   = math::constrain(Del_c_v(i,0)  , -Del_c_lim(1,0), Del_c_lim(1,0));
        Del_c_omg(i,0) = math::constrain(Del_c_omg(i,0), -Del_c_lim(2,0), Del_c_lim(2,0));
        Del_c_eps(i,0) = math::constrain(Del_c_eps(i,0), -Del_c_lim(3,0), Del_c_lim(3,0));
    }

    stabilisation_mode();
    Del_c = Del_c_eps.emult(c_eps_bool) + Del_c_omg;
    cf = c_setpoint + Del_c;
        
    return PX4_OK;
}
int My_LQR_control::project_del_psi(){
// If the heading error is more than 180 degree, it is faster to correct for it by turning the opposite way.
    proj_dpsi_status = 0;
    if(proj_dpsi){
        proj_dpsi_status = 1;
        if(Del_y_eps(2,0) >  MY_PI){
            Del_y_eps(2,0) = -2*MY_PI + Del_y_eps(2,0);
            proj_dpsi_status = 10;
        }
        else if(Del_y_eps(2,0) < -MY_PI){
            Del_y_eps(2,0) =  2*MY_PI + Del_y_eps(2,0);
            proj_dpsi_status = -10;
        }
    }
/* not true
// If pitch over 90deg, the yaw compensation must flip
    if(proj_dpsi && fabsf(y(10,0)) > deg2rad(90.0f)){
        Del_y_eps(2,0) = -Del_y_eps(2,0);
    }
*/

    return PX4_OK;
}
int My_LQR_control::stabilisation_mode(){
// Decide whether to disable pitch or yaw compensation
    c_eps_bool.setAll(1.0f);
    if(rc_channels.channels[14] < 0.5f){ // pitch rate compensation only
        c_eps_bool(1,0) = 0.0f;
    }
    if(rc_channels.channels[5] < 0.5f){ // yaw rate compensation only
        c_eps_bool(2,0) = 0.0f;
    }

    return PX4_OK;
}




int My_LQR_control::manual_override(){
    cm.setAll(0.0f);
    cm(0,0) =  rc_channels.channels[0] * RC_scale_base(0,0);
    cm(1,0) = -rc_channels.channels[1] * RC_scale_base(1,0);
    cm(2,0) =  rc_channels.channels[2] * RC_scale_base(2,0);
    cm(3,0) =  rc_channels.channels[3];
    
    if(rc_channels.channels[14] < -0.5f){ // manual override pitch
        cf(1,0) = cm(1,0);
    }
    if(rc_channels.channels[5] < -0.5f){ // manual override yaw
        cf(2,0) = cm(2,0);
    }
    if(rc_channels.channels[13] < -0.5f){ // manual override all
        cf.setAll(0.0f);
        cf(0,0) = cm(0,0);
        cf(1,0) = cm(1,0);
        cf(2,0) = cm(2,0);
        cf(3,0) = cm(3,0);
    }

    return PX4_OK;
}

int My_LQR_control::px4_override(){
    if(rc_channels.channels[13] > 0.5f){ // PX4 override
        actuator_controls_virtual_poll();
        for(int i=0; i<8; i++){
            cf(i,0) = actuator_controls_virtual.control[i];
        }
    }
    return PX4_OK;
}

int My_LQR_control::supporting_outputs(){
// Outputs to the actuator_controls_1

    uf.setAll(0.0f);

    // front propeller thrust
    uf(3,0) = math::constrain(c_nominal_control(3,0) + rc_channels.channels[8]/2.0f, 0.0f, 1.0f);

    // roll support to the spilt elevators
    if(rc_channels.channels[12] > -0.5f){
        uf(0,0) = tailerons_scaling*cf(0,0);
    }

    // roll and yaw support to the wing motors
    if(rc_channels.channels[12] > 0.5f){
        uf(6,0) = motorons_p_scaling*cf(0,0);
        uf(7,0) = motorons_r_scaling*cf(2,0);
    }
    
    return PX4_OK;
}

int My_LQR_control::rc_loss_failsafe(){
    if(rc_channels.signal_lost == true){
        dt_rcloss = dt_rcloss + dt;
        if(dt_rcloss >= 4.0f){
            uf.setAll(0.0f);
            cf.setAll(0.0f);
        }
        if(dt_rcloss >= 1000000.0f){ // not to get an overflow
            dt_rcloss = 5.0f;
        }
    }
    else{
        dt_rcloss = 0.0f;
    }

    return PX4_OK;
}

int My_LQR_control::bound_controls(){
    
    for(int i = 0; i < 3; i++){
        cf(i,0) = math::constrain(cf(i,0), -1.0f, 1.0f);
    }
    cf(3,0) = math::constrain(cf(3,0), 0.0f, 1.0f); // thrust only positive

    for(int i = 0; i < 3; i++){
        uf(i,0) = math::constrain(uf(i,0), -1.0f, 1.0f);
    }
    uf(3,0) = math::constrain(uf(3,0), 0.0f, 1.0f); // thrust only positive
    for(int i = 4; i < 8; i++){
        uf(i,0) = math::constrain(uf(i,0), -1.0f, 1.0f);
    }

    return PX4_OK;
}





int My_LQR_control::printouts(){
    if(do_printouts){
        dt_print = dt_print + dt;
        if(dt_print > 3.0f){
            //PX4_INFO("dt:%2.5f", (double)dt);
            
            //PX4_INFO("x:%2.2f, xd:%2.2f", (double)y(0,0), (double)y_setpoint(0,0));
            
            //PX4_INFO("m1:%2.4f, m2:%2.4f, m3:%2.4f, m4:%2.4f\n", (double)uf(0,0), (double)uf(1,0), (double)uf(2,0), (double)uf(3,0));

            //PX4_INFO("m5:%2.4f, m6:%2.4f, m7:%2.4f, m8:%2.4f\n", (double)uf(4,0), (double)uf(5,0), (double)uf(6,0), (double)uf(7,0));

            PX4_INFO("c1(roll):%2.4f, c2(pitch):%2.4f, c3(yaw):%2.4f, c4(thrust):%2.4f", (double)cf(0,0), (double)cf(1,0), (double)cf(2,0), (double)cf(3,0));

            //PX4_INFO("Dcf1:%2.2f, Dcf2:%2.2f, Dcf3:%2.2f, Dcf4:%2.2f\n", (double)Del_cf(0,0), (double)Del_cf(1,0), (double)Del_cf(2,0), (double)Del_cf(3,0));

            //PX4_INFO("Dc1:%2.4f, Dc2:%2.4f, Dc3:%2.4f, Dc4:%2.4f\n", (double)Del_c(0,0), (double)Del_c(1,0), (double)Del_c(2,0), (double)Del_c(3,0));

            //PX4_INFO("Dy1:%2.2f, Dy2:%2.2f, Dy3:%2.2f, Dy4:%2.2f, Dy5:%2.2f, Dy6:%2.2f\n", (double)Del_y(6,0), (double)Del_y(7,0), (double)Del_y(8,0), (double)Del_y(9,0), (double)Del_y(10,0), (double)Del_y(11,0));

            if(filter_status == 1){
                PX4_ERR("Filtering rates results in NANs!");
            }
            if(filter_status == 2){
                PX4_ERR("Filtering freqn off range 100Hz, disabled!");
            }

            PX4_INFO("dpsi projected [deg]: %3.1f", (double)rad2deg(Del_y_eps(2,0)));
            PX4_INFO("pitch setpoint [deg]: %3.1f", (double)rad2deg(pitch_setpoint));
            PX4_INFO("theta0 [deg]: %3.1f, theta proj [deg]: %3.1f", (double)rad2deg(theta0), (double)rad2deg(y(10,0)));    
            PX4_INFO("Scheduler interval: %d, f_int: %2.4f", case_int, (double)f_int);

            (K_feedback_y_sc_tun_sched_adapt.T().slice<6,4>(6,0)).T().print();

            PX4_INFO("alpha %2.2f, Ierr %2.2f, err %2.2f, Domgpred %2.6f, Domg %2.6f, Cprev %1.2f, Omgprev %2.2f.", (double)alpha_A(0,0), (double)I_error_A(0,0), (double)error_A(0,0), (double)Domg_pred_A(0,0), (double)Domg_A(0,0), (double)C_prev_A(0,0), (double)Omg_prev_A(0,0));

            //(-K_feedback_y*SC_Del_y_eps*Del_y.slice<3,1>(9,0)).print();
            //Del_c_eps.print();
            //(-K_feedback_y.T().slice<3,8>(9,0)).T().print();

            //PX4_INFO("angular_rates_timestamp:%2.2f", (double)angular_rates_filtered.timestamp_sample);
            //omg_filtered.print();
            //omg.print();

            dt_print = 0.0f;
        }
    }
    return PX4_OK;
}




/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int My_LQR_control::initialize_variables(){
    y_setpoint.setAll(0.0f);
    c_setpoint.setAll(0.0f);
    r_setpoint.setAll(0.0f);
    pitch_setpoint = 0.0f;
    attitude.setAll(0.0f);

    y.setAll(0.0f);
    r.setAll(0.0f);
    cf.setAll(0.0f);
    uf.setAll(0.0f);

    K_feedback_y.setAll(0.0f);
    K_feedback_int.setAll(0.0f);
    K_feedback_cf.setAll(0.0f);
    c_nominal_control.setAll(0.0f);

    k_scheds.setAll(0.0f);

    if(vehicle_id == 1){ // S500 quad
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.11f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =   0.00f; K_feedback_y(0,9) =   0.50f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.11f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   0.50f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =   0.00f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.22f; K_feedback_y(2,9) =   0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   0.50f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
    }
    else if(vehicle_id == 2){ // Custer
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.4000f; K_feedback_y(0,7) =  -0.0000f; K_feedback_y(0,8) =   0.0000f; K_feedback_y(0,9) =   1.7500f; K_feedback_y(0,10) =  -0.0000f; K_feedback_y(0,11) =   0.0000f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =  -0.0000f; K_feedback_y(1,7) =   0.0500f; K_feedback_y(1,8) =  -0.0000f; K_feedback_y(1,9) =  -0.0000f; K_feedback_y(1,10) =   1.7500f; K_feedback_y(1,11) =  -0.0000f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =  -0.0000f; K_feedback_y(2,7) =  -0.0000f; K_feedback_y(2,8) =   0.4000f; K_feedback_y(2,9) =  -0.0000f; K_feedback_y(2,10) =  -0.0000f; K_feedback_y(2,11) =   1.7500f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.0000f; K_feedback_y(3,7) =   0.0000f; K_feedback_y(3,8) =   0.0000f; K_feedback_y(3,9) =   0.0000f; K_feedback_y(3,10) =   0.0000f; K_feedback_y(3,11) =   0.0000f; 
k_scheds(0,0) =   0.40f; k_scheds(0,1) =   0.40f; k_scheds(0,2) =   0.92f; k_scheds(0,3) =   0.92f; k_scheds(0,4) =   1.45f; k_scheds(0,5) =   1.45f; 
k_scheds(1,0) =   0.00f; k_scheds(1,1) =   0.00f; k_scheds(1,2) =   0.00f; k_scheds(1,3) =   0.74f; k_scheds(1,4) =   1.17f; k_scheds(1,5) =   1.17f;  
k_scheds(2,0) =   0.00f; k_scheds(2,1) =   0.00f; k_scheds(2,2) =   0.00f; k_scheds(2,3) =  -0.74f; k_scheds(2,4) =  -1.17f; k_scheds(2,5) =  -1.17f;  
k_scheds(3,0) =   0.40f; k_scheds(3,1) =   0.40f; k_scheds(3,2) =   0.92f; k_scheds(3,3) =   0.92f; k_scheds(3,4) =   1.45f; k_scheds(3,4) =   1.45f;  
k_scheds(4,0) =   1.75f; k_scheds(4,1) =   1.75f; k_scheds(4,2) =   1.75f; k_scheds(4,3) =   1.75f; k_scheds(4,4) =   1.75f; k_scheds(4,5) =   1.75f; 
k_scheds(5,0) =  -0.0000f; k_scheds(5,1) =  -0.0000f; k_scheds(5,2) =  -0.0000f; k_scheds(5,3) =  -0.0000f; k_scheds(5,4) =  -0.0000f; k_scheds(5,5) =  -0.0000f; 
k_scheds(6,0) =  -0.0000f; k_scheds(6,1) =  -0.0000f; k_scheds(6,2) =  -0.0000f; k_scheds(6,3) =  -0.0000f; k_scheds(6,4) =  -0.0000f; k_scheds(6,5) =  -0.0000f; 
k_scheds(7,0) =   1.75f; k_scheds(7,1) =   1.75f; k_scheds(7,2) =   1.75f; k_scheds(7,3) =   1.75f; k_scheds(7,4) =   1.75f; k_scheds(7,5) =   1.75f; 
k_scheds(8,0) =   0.05f; k_scheds(8,1) =   0.05f; k_scheds(8,2) =   0.12f; k_scheds(8,3) =   0.12f; k_scheds(8,4) =   0.19f; k_scheds(8,5) =   0.19f; 
k_scheds(9,0) =   1.75f; k_scheds(9,1) =   1.75f; k_scheds(9,2) =   1.75f; k_scheds(9,3) =   1.75f; k_scheds(9,4) =   1.75f; k_scheds(9,5) =   1.75f; 
        tht_ints(0,0) =  -1.5708f; tht_ints(0,1) =   0.0000f; tht_ints(0,2) =   0.5236f; tht_ints(0,3) =   0.7854f; tht_ints(0,4) =   1.0472f; tht_ints(0,5) =   1.5708f; 
        // pitch angles (0.35, 0.52, 0.70, 0.87 rad = 20, 30, 40, 50 deg)
    }
    else if(vehicle_id == 3){ // Custer HITL
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.7414f; K_feedback_y(0,7) =  -0.0000f; K_feedback_y(0,8) =   0.2000f; K_feedback_y(0,9) =   1.2910f; K_feedback_y(0,10) =  -0.0000f; K_feedback_y(0,11) =   0.2000f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =  -0.0000f; K_feedback_y(1,7) =   0.7414f; K_feedback_y(1,8) =  -0.0000f; K_feedback_y(1,9) =  -0.0000f; K_feedback_y(1,10) =   1.2910f; K_feedback_y(1,11) =  -0.0000f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =  -0.2000f; K_feedback_y(2,7) =  -0.0000f; K_feedback_y(2,8) =   0.7414f; K_feedback_y(2,9) =  -0.0000f; K_feedback_y(2,10) =  -0.0000f; K_feedback_y(2,11) =   1.2910f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.0000f; K_feedback_y(3,7) =   0.0000f; K_feedback_y(3,8) =   0.0000f; K_feedback_y(3,9) =   0.0000f; K_feedback_y(3,10) =   0.0000f; K_feedback_y(3,11) =   0.0000f; 
k_scheds(0,0) =   0.30f; k_scheds(0,1) =   0.30f; k_scheds(0,2) =   0.58f; k_scheds(0,3) =   0.92f; k_scheds(0,4) =   1.45f; k_scheds(0,5) =   1.45f; 
k_scheds(1,0) =   0.00f; k_scheds(1,1) =   0.00f; k_scheds(1,2) =   0.00f; k_scheds(1,3) =   0.74f; k_scheds(1,4) =   1.17f; k_scheds(1,5) =   1.17f;  
k_scheds(2,0) =   0.00f; k_scheds(2,1) =   0.00f; k_scheds(2,2) =   0.00f; k_scheds(2,3) =  -0.74f; k_scheds(2,4) =  -1.17f; k_scheds(2,5) =  -1.17f;  
k_scheds(3,0) =   0.30f; k_scheds(3,1) =   0.30f; k_scheds(3,2) =   0.58f; k_scheds(3,3) =   0.92f; k_scheds(3,4) =   1.45f; k_scheds(3,4) =   1.45f;  
k_scheds(4,0) =   1.2910f; k_scheds(4,1) =   1.2910f; k_scheds(4,2) =   1.2910f; k_scheds(4,3) =   1.2910f; k_scheds(4,4) =   1.2910f; k_scheds(4,5) =   1.2910f; 
k_scheds(5,0) =  -0.0000f; k_scheds(5,1) =  -0.0000f; k_scheds(5,2) =  -0.0000f; k_scheds(5,3) =  -0.0000f; k_scheds(5,4) =  -0.0000f; k_scheds(5,5) =  -0.0000f; 
k_scheds(6,0) =  -0.0000f; k_scheds(6,1) =  -0.0000f; k_scheds(6,2) =  -0.0000f; k_scheds(6,3) =  -0.0000f; k_scheds(6,4) =  -0.0000f; k_scheds(6,5) =  -0.0000f; 
k_scheds(7,0) =   1.2910f; k_scheds(7,1) =   1.2910f; k_scheds(7,2) =   1.2910f; k_scheds(7,3) =   1.2910f; k_scheds(7,4) =   1.2910f; k_scheds(7,5) =   1.2910f; 
k_scheds(8,0) =   0.04f; k_scheds(8,1) =   0.04f; k_scheds(8,2) =   0.07f; k_scheds(8,3) =   0.12f; k_scheds(8,4) =   0.19f; k_scheds(8,5) =   0.19f; 
k_scheds(9,0) =   1.2910f; k_scheds(9,1) =   1.2910f; k_scheds(9,2) =   1.2910f; k_scheds(9,3) =   1.2910f; k_scheds(9,4) =   1.2910f; k_scheds(9,5) =   1.2910f; 
        tht_ints(0,0) =  -1.5708f; tht_ints(0,1) =   0.0000f; tht_ints(0,2) =   0.5236f; tht_ints(0,3) =   0.7854f; tht_ints(0,4) =   1.0472f; tht_ints(0,5) =   1.5708f; 
        // pitch angles (0.35, 0.52, 0.70, 0.87 rad = 20, 30, 40, 50 deg)
    }
    else{ // Not specified
        PX4_WARN("No airframe specified, using unit gains K");
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.01f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =   0.00f; K_feedback_y(0,9) =   0.01f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.01f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   0.01f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =   0.00f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.01f; K_feedback_y(2,9) =   0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   0.01f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
    }

    c_nominal_control(3,0) = 0.5f; // don't change this, otherwise the RC is not able to go to full 1 or full 0. Add a trim variable to c_setpoint after rc is added if you want to change it

    K_feedback_y_scaled = K_feedback_y;
    K_feedback_y_scaled_tuned = K_feedback_y_scaled;
    K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned;

    k_scheds_sc = k_scheds;
    k_scheds_sc_tun = k_scheds_sc;

    Ci.setAll(0.0f);
    Ci(0, 0) = 1.0f;
    Ci(1, 1) = 1.0f;
    Ci(2, 2) = 1.0f;
    Ci(3, 9) = 1.0f;
    Ci(4,10) = 1.0f;
    Ci(5,11) = 1.0f;

    manual_control_setpoint.x = 0.0f;
    manual_control_setpoint.y = 0.0f;
    manual_control_setpoint.z = 0.0f;
    manual_control_setpoint.r = 0.0f;
    manual_control_setpoint.aux1 = 0.0f;
    manual_control_setpoint.aux2 = 0.0f;

    rc_channels.channels[5] = 1.0f; // yaw switch
    rc_channels.channels[14] = 1.0f; // pitch switch
    rc_channels.channels[10] = 0.0f; // S1
    rc_channels.channels[11] = 0.0f; // S2
    rc_channels.channels[12] = 0.0f; // SB
    rc_channels.channels[13] = 0.0f; // SA

    vehicle_attitude.rollspeed = 0.0f;
    vehicle_attitude.pitchspeed = 0.0f;
    vehicle_attitude.yawspeed = 0.0f;
    vehicle_attitude.q[0] = 0.0f;
    vehicle_attitude.q[1] = 0.0f;
    vehicle_attitude.q[2] = 0.0f;
    vehicle_attitude.q[3] = 0.0f;

    vehicle_local_position.x = 0.0f;
    vehicle_local_position.y = 0.0f;
    vehicle_local_position.z = 0.0f;
    vehicle_local_position.vx = 0.0f;
    vehicle_local_position.vy = 0.0f;
    vehicle_local_position.vz = 0.0f;

    actuator_controls_virtual.control[0] = 0.0f;
    actuator_controls_virtual.control[1] = 0.0f;
    actuator_controls_virtual.control[2] = 0.0f;
    actuator_controls_virtual.control[3] = 0.0f;

    omg.setAll(0.0f);
    omg_filtered.setAll(0.0f);
    angular_rates_filtered.rollspeed = 0.0f;
    angular_rates_filtered.pitchspeed = 0.0f;
    angular_rates_filtered.yawspeed = 0.0f;
    angular_rates_filtered.loop_update_freqn = loop_update_freqn;
    angular_rates_filtered.cutoff_freqn = angular_rates_cutoff_freqn;

    Del_c_lim.setAll(1.0f);

    E2B.setAll(0.0f);
    E2B(0,0) = 1.0f;
    E2B(1,1) = 1.0f;
    E2B(2,2) = 1.0f;

    alpha_A.setAll(1.0f);
    I_error_A.setAll(0.0f);
    Omg_prev_A.setAll(0.0f);
    Omg_A.setAll(0.0f);
    A_A.setAll(0.0f);
    B_A.setAll(0.0f);
    A_A(0,0) = -8.0f;
    A_A(1,0) = -8.0f;
    A_A(2,0) = -8.0f;
    B_A(0,0) = 24.0f;
    B_A(1,0) = 24.0f;
    B_A(2,0) = 24.0f;

    X0_RLS(0,0) = -7.0f;
    X0_RLS(1,0) = 25.0f;
    P0_RLS(0,0) = 0.1481f/1000.0f;
    P0_RLS(0,1) = -0.1784f/1000.0f;
    P0_RLS(1,0) = -0.1784f/1000.0f;
    P0_RLS(1,1) = 0.7632f/1000.0f;
    X_RLS = X0_RLS;
    P_RLS = P0_RLS;

    update_parameters(true);

    return PX4_OK;
}

int My_LQR_control::local_parameters_update(){
    y_max.setAll(0.0f);
    y_max(3,0) = param_max_u.get();
    y_max(4,0) = param_max_v.get();
    y_max(5,0) = param_max_w.get();
    y_max(6,0) = param_max_p.get();
    y_max(7,0) = param_max_q.get();
    y_max(8,0) = param_max_r.get();
    y_max(9,0) = param_max_phi.get();
    y_max(10,0) = param_max_theta.get();
    y_max(11,0) = param_max_psi.get();

    RC_scale_base(0,0) = rc_scale_p.get();
    RC_scale_base(1,0) = rc_scale_q.get();
    RC_scale_base(2,0) = rc_scale_r.get();
    
    Tf.setAll(0.0f);
    for(int i = 0; i < 4; i++){
        Tf(i,i) = f_lag.get();
    }

    gains_scale();

    Del_c_lim(0,0) = dx_lim.get();
    Del_c_lim(1,0) = dv_lim.get();
    Del_c_lim(2,0) = domg_lim.get();
    Del_c_lim(3,0) = deps_lim.get();

    if(fabsf(angular_rates_cutoff_freqn - math::min(angular_rates_cutoff_fn.get(), 250.0f)) > 0.1f){
        angular_rates_cutoff_freqn = math::min(angular_rates_cutoff_fn.get(), 250.0f);
        lp_filter_angular_rates.set_cutoff_frequency(loop_update_freqn, angular_rates_cutoff_freqn);
    }

    tailerons_scaling = tailerons_sc.get();
    motorons_p_scaling = motorons_p_sc.get();
    motorons_r_scaling = motorons_r_sc.get();


    e2b = bool_e2b.get() == 1;

    schedule_K = bool_K_sched.get() == 1;

    do_rc_scale = bool_rc_sc.get() == 1;

    proj_theta = bool_proj_tht.get() == 1;

    proj_dpsi = bool_proj_dpsi.get() == 1;

    do_printouts = bool_printouts.get() == 1;

    tune_expo = tune_ex.get();

    pitch_sp_max = tht_sp_m.get();

    do_adaptive = bool_adaptive.get() == 1;
    adapt_A(0,0) = bool_adaptive_p.get();
    adapt_A(1,0) = bool_adaptive_q.get();
    adapt_A(2,0) = bool_adaptive_r.get();
    gamma_A = gamma_adaptive.get();
    p_A = p_adaptive.get();

    do_recursiveLS = bool_recursiveLS.get() == 1;
    lambda_RLS = lambda_rls.get();

    
    return PX4_OK;
}

int My_LQR_control::update_parameters(bool force){
    bool updated = false;
    updated = parameter_update_poll() == PX4_OK;
    if(updated || force){
        updateParams();
        local_parameters_update();
    }
    return PX4_OK;
}

float My_LQR_control::deg2rad(float degs){
    return degs*0.01745329252f;
}

float My_LQR_control::rad2deg(float rads){
    return rads/0.01745329252f;
}




/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int My_LQR_control::del_epsilon_to_body_frame(){
// Non-linear transformation on the Del_epsilon     
    if(e2b){
        E2B(0,0) = 1.0f;
        E2B(0,2) = -sin(y(10,0));
        E2B(1,1) = cos(y(9,0));
        E2B(1,2) = sin(y(9,0))*cos(y(10,0));
        E2B(2,1) = -sin(y(9,0));
        E2B(2,2) = cos(y(9,0))*cos(y(10,0));

        Del_y_eps = E2B*Del_y.slice<3,1>(9,0);
    }

    return PX4_OK;
}


/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int my_LQR_control_main(int argc, char *argv[])
{
    return My_LQR_control::main(argc, argv);
}
