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

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("my_LQR_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Vehicle ID.\nQuadS500: 1\nCuster: 2", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int My_LQR_control::print_status()
{
        PX4_INFO("Running");
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

    setpoints_struct.pitch_setpoint = pitch_setpoint;

    setpoints_struct.timestamp = hrt_absolute_time();
    setpoints_struct.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(my_LQR_setpoints), setpoints_pub, &setpoints_struct) != PX4_OK){
        PX4_WARN("Unable to publish my_setpoints!");
    }
    
    return PX4_OK;
}


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
                timer_clock();
                
                read_y_state();

                angular_rates_filtered_publish();
                
                read_setpoints();

                setpoints_publish();

                gains_tune();

                flip();

                setpoints_scale(); 

                controller_mode();
                
                control_fun();
                
                perturb_control(); 

                px4_override();

                supporting_outputs();

                rc_loss_failsafe();
                
                actuator_controls_publish();

                //printouts();
                
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
}

int My_LQR_control::timer_clock(){
    if(t_start < 0.0f){
        t_start = hrt_absolute_time();
    }
    dt = (hrt_absolute_time() - time_last_run)/1000000.0f;
    time_last_run = hrt_absolute_time();
    
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

int My_LQR_control::rc_loss_failsafe(){
    if(rc_channels.signal_lost == true){
        dt_rcloss = dt_rcloss + dt;
        if(dt_rcloss >= 2.0f){
            uf.setAll(0.0f);
            cf.setAll(0.0f);

            uf(3,0) = math::min(0.0f, uf(3,0)); // not to kill the petrol engine if possible. should set PWM min to idle at 0 and PWM disarmed to kill
            cf(0,0) = 0.0f; 
            cf(1,0) = 0.0f; 
        }
        if(dt_rcloss >= 1000000.0f){ // not to get an overflow
            dt_rcloss = 3.0f;
        }
    }
    else{
        dt_rcloss = 0.0f;
    }

    return PX4_OK;
}

int My_LQR_control::control_fun(){

    Del_y  = y  - y_setpoint;
    //Del_y(11,0) = math::constrain(Del_y(11,0), -0.26f, 0.26f); // limit the yaw error to +-15 deg (0.26rad) not to freak out when heading too off course

    //Del_cf = cf - c_setpoint; // not used if not filter
    Del_r  = r  - r_setpoint;

    //Del_c = K_feedback_y*Del_y ... ; // not used if not filter

    if(dt < 0.5f){ // ignore too large dt steps (probably some glitches, startup etc)
        r  = r  + dt*Ci*Del_y;
        //cf = cf + dt*Tf*(-Del_cf + Del_c); // not used if not filter
    }
    
    Del_y_eps = Del_y.slice<3,1>(9,0);
    project_del_psi();
    del_epsilon_to_body_frame();

    Del_c_x   = -K_feedback_y_scaled_tuned.T().slice<3,4>(0,0).T()*Del_y.slice<3,1>(0,0); // slice x contribution
    Del_c_v   = -K_feedback_y_scaled_tuned.T().slice<3,4>(3,0).T()*Del_y.slice<3,1>(3,0); // slice v contribution
    // Del_c_omg = -K_feedback_y_scaled_tuned.T().slice<3,4>(6,0).T()*Del_y.slice<3,1>(6,0); // slice omg contribution
    Del_c_omg = -K_feedback_y_scaled_tuned.T().slice<3,4>(6,0).T()*y.slice<3,1>(6,0); // slice omg contribution
    Del_c_omg(0,0) += y_setpoint(6,0); // p
    Del_c_omg(1,0) += y_setpoint(7,0); // q
    Del_c_omg(2,0) += y_setpoint(8,0); // r
    Del_c_eps = -K_feedback_y_scaled_tuned.T().slice<3,4>(9,0).T()*Del_y_eps; // slice eps contribution
    for(int i = 0; i < 4; i++){
        Del_c_x(i,0)   = math::constrain(Del_c_x(i,0)  , -Del_c_lim(0,0), Del_c_lim(0,0));
        Del_c_v(i,0)   = math::constrain(Del_c_v(i,0)  , -Del_c_lim(1,0), Del_c_lim(1,0));
        Del_c_omg(i,0) = math::constrain(Del_c_omg(i,0), -Del_c_lim(2,0), Del_c_lim(2,0));
        Del_c_eps(i,0) = math::constrain(Del_c_eps(i,0), -Del_c_lim(3,0), Del_c_lim(3,0));
    }

    Del_c = Del_c_eps + Del_c_omg;
    cf = c_setpoint + Del_c;

    bound_control_c();
        
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

int My_LQR_control::gains_tune(){
// Tune the gains using the radio knobs

    K_feedback_y_scaled_tuned = K_feedback_y_scaled;

    for(int i=0; i<4; i++){
        for(int j=6; j<9; j++){
            K_feedback_y_scaled_tuned(i,j) *= powf(20.0f, rc_channels.channels[11]);
        }
        for(int j=9; j<12; j++){
            K_feedback_y_scaled_tuned(i,j) *= powf(20.0f, rc_channels.channels[10]);
        }
    }

    return PX4_OK;
}

int My_LQR_control::setpoints_scale(){
/*
// Scale the setpoints such that at Del_eps=0 and omg=0 the y_setpoint results in RC pass through
    
    y_setpoint(6,0) /= K_feedback_y_scaled_tuned(0,6);
    y_setpoint(7,0) /= K_feedback_y_scaled_tuned(1,7);
    y_setpoint(8,0) /= K_feedback_y_scaled_tuned(2,8);
commented out cos now this happens directly in control_fun to avoid division by zero here 
*/

// RC pass through scaled by RC_scale cos i.e. for multicopters we don't want such high rates
    y_setpoint(6,0) *= RC_scale;
    y_setpoint(7,0) *= RC_scale;
    y_setpoint(8,0) *= RC_scale;

    return PX4_OK;
}

int My_LQR_control::supporting_outputs(){
// Outputs to the actuator_controls_1

    uf.setAll(0.0f);

    // front propeller thrust
    uf(3,0) = math::constrain(c_nominal_control(3,0) + manual_control_setpoint.aux1/2.0f, 0.0f, 1.0f);

    // roll support to the spilt elevators
    if(rc_channels.channels[12] > -0.5f){
        uf(0,0) = tailerons_scaling*cf(0,0);
    }

    // roll and yaw support to the wing motors
    if(rc_channels.channels[12] > 0.5f){
        uf(6,0) = motorons_p_scaling*cf(0,0);
        // uf(7,0) = motorons_r_scaling*cf(2,0);
    }
    
    return PX4_OK;
}

int My_LQR_control::controller_mode(){
// Decide whether to use the LQR feedback or just RC passthrough

    if(rc_channels.channels[13] < -0.5f){ // RC only
        y(6,0)  = 0.0f;
        y(7,0)  = 0.0f;
        y(8,0)  = 0.0f;
        y(9,0)  = y_setpoint(9,0);
        y(10,0) = y_setpoint(10,0);
        y(11,0) = y_setpoint(11,0);
    }
    

// Decide whether to disable pitch or yaw stabilisation and damping
    if(rc_channels.channels[14] < -0.5f){ // RC override pitch
        y(7,0)  = 0.0f;
        y(10,0) = y_setpoint(10,0);
    }
    else if(rc_channels.channels[14] < 0.5f){ // pitch rate compensation only
        y(10,0) = y_setpoint(10,0);
    }
    
    if(rc_channels.channels[5] < -0.5f){ // RC override yaw
        y(8,0)  = 0.0f;
        y(11,0) = y_setpoint(11,0);
    }
    else if(rc_channels.channels[5] < 0.5f){ // yaw rate compensation only
        y(11,0) = y_setpoint(11,0);
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

int My_LQR_control::bound_control_c(){
    
    for(int i = 0; i < 3; i++){
        cf(i,0) = math::constrain(cf(i,0), -1.0f, 1.0f);
    }
    cf(3,0) = math::constrain(cf(3,0), 0.0f, 1.0f); // thrust only positive

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
            omg_filtered = omg;
            filter_status = 1; // whops
        }
    }
    else{
        omg_filtered = omg;
        filter_status = 2; // whops 2
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
    if(proj_theta){
        proj_theta_status = 1;
        Qdcm_proj = Qdcm; // predefine
        if(euler_angles.theta() > deg2rad(50.0f)){ // if > 50 deg
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
        else if(euler_angles.theta() < -deg2rad(50.0f)){ // if < -50 deg
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
    manual_control_setpoint_poll();
    rc_channels_poll();
    read_y_setpoint();
    read_c_setpoint();
    return PX4_OK;
}
int My_LQR_control::read_c_setpoint(){
    c_setpoint = c_nominal_control;

    // Position/ velocity control
    /* all c_setpoints are equal to nominal control */

    // Attitude control
    /* we control thrust directly by rc */
    thrust_setpoint = manual_control_setpoint.z - 0.5f; 
    c_setpoint(3,0) = math::constrain(c_nominal_control(3,0) + thrust_setpoint, 0.0f, 1.0f);

    //pitch_setpoint = ((rc_channels.channels[9] + 1.0f)/2.0f)*deg2rad(60f); // 0 to 60 deg based on RS stick input
    pitch_setpoint = rc_channels.channels[9]*deg2rad(90.0f); // -90 to 90 deg based on RS stick input just for test
    
    return PX4_OK;
}

int My_LQR_control::read_y_setpoint(){
    y_setpoint.setAll(0.0f);

    /* 
    // Position/ velocity control
    y_setpoint(3,0) = rc_deadband(manual_control_setpoint.x)*y_setpoint_scale(3,0);
    y_setpoint(4,0) = rc_deadband(manual_control_setpoint.y)*y_setpoint_scale(4,0);
    y_setpoint(5,0) = -(2.0f*rc_deadband(manual_control_setpoint.z) - 1.0f)*y_setpoint_scale(5,0);
    y_setpoint(8,0) = rc_deadband(manual_control_setpoint.r)*y_setpoint_scale(8,0);
    */

    // Attitude control
    y_setpoint(6,0) = manual_control_setpoint.y;
    y_setpoint(7,0) = -manual_control_setpoint.x;
    y_setpoint(8,0) = manual_control_setpoint.r;
    y_setpoint(9,0) = 0.0f;
    y_setpoint(10,0) = pitch_setpoint;
    y_setpoint(11,0) = yaw_setpoint;
    if(fabsf(y_setpoint(8,0)) > 0.1f || ((fabsf(y_setpoint(6,0)) > 0.1f) && (vehicle_id == 2)) || fabsf(yaw_setpoint) <= 0.0f){ // at yaw||(roll if fw) rate command or at the startup
        yaw_setpoint = y(11,0);
    }

    return PX4_OK;
}

int My_LQR_control::printouts(){
    dt_print = dt_print + dt;
    if(dt_print > 3.0f){
        PX4_INFO("dt:%2.5f", (double)dt);
        
        PX4_INFO("x:%2.2f, xd:%2.2f", (double)y(0,0), (double)y_setpoint(0,0));
        
        PX4_INFO("m1:%2.4f, m2:%2.4f, m3:%2.4f, m4:%2.4f\n", (double)uf(0,0), (double)uf(1,0), (double)uf(2,0), (double)uf(3,0));

        PX4_INFO("m5:%2.4f, m6:%2.4f, m7:%2.4f, m8:%2.4f\n", (double)uf(4,0), (double)uf(5,0), (double)uf(6,0), (double)uf(7,0));

        PX4_INFO("c1(roll):%2.4f, c2(pitch):%2.4f, c3(yaw):%2.4f, c4(thrust):%2.4f\n", (double)cf(0,0), (double)cf(1,0), (double)cf(2,0), (double)cf(3,0));

        //PX4_INFO("Dcf1:%2.2f, Dcf2:%2.2f, Dcf3:%2.2f, Dcf4:%2.2f\n", (double)Del_cf(0,0), (double)Del_cf(1,0), (double)Del_cf(2,0), (double)Del_cf(3,0));

        PX4_INFO("Dc1:%2.4f, Dc2:%2.4f, Dc3:%2.4f, Dc4:%2.4f\n", (double)Del_c(0,0), (double)Del_c(1,0), (double)Del_c(2,0), (double)Del_c(3,0));

        PX4_INFO("Dy1:%2.2f, Dy2:%2.2f, Dy3:%2.2f, Dy4:%2.2f, Dy5:%2.2f, Dy6:%2.2f\n", (double)Del_y(6,0), (double)Del_y(7,0), (double)Del_y(8,0), (double)Del_y(9,0), (double)Del_y(10,0), (double)Del_y(11,0));

        if(filter_status == 1){
            PX4_ERR("Filtering rates results in NANs!");
        }

        //(-K_feedback_y*SC_Del_y_eps*Del_y.slice<3,1>(9,0)).print();
        //Del_c_eps.print();
        //(-K_feedback_y.T().slice<3,8>(9,0)).T().print();

        //PX4_INFO("angular_rates_timestamp:%2.2f", (double)angular_rates_filtered.timestamp_sample);
        //omg_filtered.print();
        //omg.print();

        dt_print = 0.0f;
    }
    return PX4_OK;
}

int My_LQR_control::initialize_variables(){
    y_setpoint.setAll(0.0f);
    c_setpoint.setAll(0.0f);
    r_setpoint.setAll(0.0f);
    attitude.setAll(0.0f);

    y.setAll(0.0f);
    r.setAll(0.0f);
    cf.setAll(0.0f);
    uf.setAll(0.0f);

    K_feedback_y.setAll(0.0f);
    K_feedback_int.setAll(0.0f);
    K_feedback_cf.setAll(0.0f);
    c_nominal_control.setAll(0.0f);

    if(vehicle_id == 1){ // S500 quad
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.11f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =   0.00f; K_feedback_y(0,9) =   0.50f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.11f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   0.50f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =   0.00f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.22f; K_feedback_y(2,9) =   0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   0.50f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
    }
    else if(vehicle_id == 2){ // Custer
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.70f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =  -0.20f; K_feedback_y(0,9) =   1.30f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.70f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   1.30f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =  -0.20f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.70f; K_feedback_y(2,9) =  -0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   1.30f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
        pitch_setpoint = 0.3491; // 20 deg pitch setpoint (0.35, 0.52, 0.70, 0.87 rad = 20, 30, 40, 50 deg)
    }
    else{ // Not specified
        PX4_WARN("No airframe specified, using unit gains K");
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.01f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =   0.00f; K_feedback_y(0,9) =   0.01f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.01f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   0.01f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =   0.00f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.01f; K_feedback_y(2,9) =   0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   0.01f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
    }

    c_nominal_control(3,0) = 0.5f; // don't change this, otherwise the RC is not able to go to full 1 or full 0. Add a trim variable to c_setpoint after rc is added if you want to change it


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

    RC_scale = rc_scale.get();
    
    Tf.setAll(0.0f);
    for(int i = 0; i < 4; i++){
        Tf(i,i) = f_lag.get();
    }


    K_feedback_y_scaled = K_feedback_y;
    for(int i=0; i<4; i++){
        for(int j=0; j<3; j++){
            K_feedback_y_scaled(i,j) *= k_sc_x.get();
        }
        for(int j=3; j<6; j++){
            K_feedback_y_scaled(i,j) *= k_sc_v.get();
        }
        for(int j=6; j<9; j++){
            K_feedback_y_scaled(i,j) *= k_sc_omg.get();
        }
        for(int j=9; j<12; j++){
            K_feedback_y_scaled(i,j) *= k_sc_eps.get();
        }
        K_feedback_y_scaled(i,8) *= k_sc_yawr.get();
    }
    K_feedback_y_scaled(0,8) *= k_sc_cc.get();
    K_feedback_y_scaled(2,6) *= k_sc_cc.get();
    K_feedback_y_scaled(0,11) *= k_sc_cc.get();
    K_feedback_y_scaled(2,9) *= k_sc_cc.get();


    K_feedback_cf_scaled = K_feedback_cf;
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            K_feedback_cf_scaled *= k_sc_cf.get();
        }
    }

    K_feedback_int_scaled = K_feedback_int;
    for(int i=0; i<4; i++){
        for(int j=0; j<6; j++){
            K_feedback_int_scaled *= k_sc_r.get();
        }
    }
    

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

    pert_idx = param_pert_idx.get();
    pert_time = param_pert_time.get();
    pert_magnitude = param_pert_magnitude.get();

    e2b = bool_e2b.get() == 1;

    proj_theta = bool_proj_tht.get() == 1;

    proj_dpsi = bool_proj_dpsi.get() == 1;

    do_flip = bool_do_flip.get() == 1;

    do_perturb_control = bool_perturb_control.get() == 1;
    
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


int My_LQR_control::flip(){
    if(do_flip){
        if(fabsf(manual_control_setpoint.y) >= 0.98f){ // if the roll stick input == 1 or -1 (with some tolerance)
            in_flip = true;
            flip_sign = manual_control_setpoint.y/fabsf(manual_control_setpoint.y);
        }
        
        if(in_flip){
            Del_c_lim(2,0) = 1.0f; // to allow for larger Del_c_omg when flipping
            y_setpoint(6,0) = flip_sign*1.0f; // flip in the roll direction
            y_setpoint(9,0) = y(9,0); // annulate the eps compensation basically
            if((flip_sign > 0) && (y(9,0) > -1.57f)){
                in_flip = false;
                local_parameters_update(); // to reset the Del_c_lim
            }
            if((flip_sign < 0) && (y(9,0) < 1.57f)){
                in_flip = false;
                local_parameters_update(); // to reset the Del_c_lim
            }
        }
    }

    return PX4_OK;
}

int My_LQR_control::perturb_control(){
    if(do_perturb_control){
        dt_perturb = dt_perturb + dt;

        if(dt_perturb <= pert_time){
            uf(pert_idx,0) = uf(pert_idx,0) + pert_magnitude;
        }
        else if(dt_perturb <= 2.0f*pert_time){
            uf(pert_idx,0) = uf(pert_idx,0) - pert_magnitude;
        }

        if(dt_perturb >= 10.0f*pert_time){
            dt_perturb = 0.0f;
        }
    }
    
    return PX4_OK;
}

float My_LQR_control::deg2rad(float degs){
    return degs*0.01745329252f;
}


int my_LQR_control_main(int argc, char *argv[])
{
    return My_LQR_control::main(argc, argv);
}
