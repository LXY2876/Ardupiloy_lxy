#include "mode.h"
#include "Plane.h"
// #include <AP_Param/AP_Param.h>
const AP_Param::GroupInfo ModeNewMode::var_info[] = {
     
    AP_GROUPINFO("rate_gain",    1, ModeNewMode, rate_gain, 7.0f),
    AP_GROUPINFO("angle_pgain",    2, ModeNewMode, angle_P_gain, 2.0f),
    AP_GROUPINFO("C_l_delta_a",    3, ModeNewMode, C_l_delta_a, 0.25f),
    AP_GROUPINFO("Ixx",    4, ModeNewMode, Ixx, 1.0f),
    AP_GROUPINFO("S_wing",    5, ModeNewMode, S_wing, 0.45f),
    AP_GROUPINFO("b",    6, ModeNewMode, b, 1.88f),
    AP_GROUPINFO("sigma",    7, ModeNewMode, sigma,0.5f),
    AP_GROUPINFO("LMS_enable",    8, ModeNewMode, LMS_enable,1),
    AP_GROUPINFO("max_inc",    9, ModeNewMode, max_increment,0.2f),
    AP_GROUPINFO("lms_ctf_frq",    10, ModeNewMode, lms_cutoff_freq,10), 
    AP_GROUPINFO("auto_flap",    11, ModeNewMode, auto_flap,0), 
    AP_GROUPINFO("cutf_freq",    12, ModeNewMode, cutoff_frequency,18), 
    AP_GROUPEND
};


struct PACKED log_AngAcc{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float AGAC_x_flt;
    float AGAC_y_flt;
    float AGAC_z_flt;
    float AGAC_x_raw;
    float AGAC_y_raw;
    // float ang_acc_z_raw;
};
void ModeNewMode::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
     
    if (rc().channel(4)->norm_input()>0.5){
        if(start_test<100)
        plane.nav_roll_cd=2500;
        else if (start_test<200)
        plane.nav_roll_cd=-2500;
        else {plane.nav_roll_cd=0;}
        start_test++;
    }else{start_test=0;}
    
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

 void ModeNewMode::run()
{

   
    Vector3f gyro_latest = ahrs.get_gyro_latest();
    // printf("rate_gain=%f",(float)angle_gain);
    float aspeed=0.0,pitch_out=0.0,yaw_out=0.0;
    if (!ahrs.airspeed_estimate(aspeed)){
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO,"ahrs_aspeed=%f",aspeed);   
        aspeed=ahrs.groundspeed();
        
    } 
    // else{
    //     plane.stabilize_roll();

    // }
    aspeed=plane.airspeed.get_airspeed();
    // 
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO,"aspeed=%f",aspeed);  
    aspeed=MAX(plane.aparm.airspeed_min,aspeed);
    aspeed=MIN(plane.aparm.airspeed_max,aspeed);
    float aspeed_flt=ARSP_filter.apply(aspeed);
 
    float desired_phi_dot=angle_P_gain*cd_to_rad(plane.nav_roll_cd-ahrs.roll_sensor);
    
     
    Vector3f ang_acc_raw=(gyro_latest -_gyro_prev)/AP::scheduler().get_loop_period_s() ;                                                             
    Vector3f ang_acc_flt =_ang_acc_filter.apply(ang_acc_raw);
    float desired_psi_dot=GRAVITY_MSS/aspeed_flt*tanf(cd_to_rad(plane.nav_roll_cd))*cosf(cd_to_rad(plane.nav_pitch_cd));
    float desired_theta_dot=angle_P_gain*cd_to_rad(plane.nav_pitch_cd-ahrs.pitch_sensor);
    _gyro_prev=gyro_latest;
    float desired_q=cosf(cd_to_rad(plane.nav_roll_cd))*desired_theta_dot+cosf(cd_to_rad(plane.nav_pitch_cd))*sinf(cd_to_rad(plane.nav_roll_cd))*desired_psi_dot;
    float desired_p=desired_phi_dot-sinf(cd_to_rad(plane.nav_pitch_cd))*desired_psi_dot;
    float desired_r=-sinf(cd_to_rad(plane.nav_roll_cd))*desired_theta_dot+cosf(cd_to_rad(plane.nav_pitch_cd))*cosf(cd_to_rad(plane.nav_roll_cd))*desired_psi_dot;
    Vector3f rate_ref(desired_p,desired_q,desired_r);
    Vector3f target_ang_acc =  (rate_ref-gyro_latest)*rate_gain ; 
    // Matrix3f moment_of_inertia_xyz (
    //     Ixx, 0.0f, 0.0f,
    //     0.0f, Iyy, 0.0f,
    //     0.0f, 0.0f, Izz
    // );
    float gain_roll=C_l_delta_a;
    Matrix3f cofficient_inverse (
        1.0f/gain_roll, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 10.0f
    );

    delta_u=cofficient_inverse*(target_ang_acc-ang_acc_flt);
    // float last_out=SRV_Channels::get_output_norm(SRV_Channel::k_aileron);
    float roll_out_cmd;
    float delta_roll_out;
    delta_roll_out=Ixx*sigma*delta_u.x/(0.5*1.225*aspeed_flt*S_wing*b);
    delta_roll_out=constrain_float(delta_roll_out,-max_increment,max_increment);
    roll_out_cmd=roll_out_flt+delta_roll_out;
    // roll_out_cmd=_roll_output_post_filter.apply(roll_out_cmd);//后置滤波
    roll_out_cmd=constrain_float(roll_out_cmd,-0.9,0.9);//实际输出到执行器的控制指令
    roll_out_real=(1-0.67)*roll_out_cmd+0.67*roll_out_flt;//考虑一阶惯性环节的实际输出
    roll_out_flt=_roll_output_filter.apply(roll_out_real);//考虑执行器与传感器的时间同步
    if (LMS_enable==1){
    val=lms_filter.apply(C_l_delta_a,ang_acc_raw.x,roll_out_real,(0.5*1.225*aspeed_flt*S_wing*b)/(Ixx*sigma));
    C_l_delta_a.set(val);
    }
    // printf("C_l_delta_a=%f",val);
    pitch_out=SRV_Channels::get_output_norm(SRV_Channel::k_elevator)+sigma*delta_u.y/(0.5*1.225*aspeed_flt*S_wing*0.24);
    yaw_out=SRV_Channels::get_output_norm(SRV_Channel::k_rudder)+sigma*delta_u.z/(0.5*1.225*aspeed_flt*S_wing*b);
    roll_out_cmd=plane.roll_in_expo(false);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out_cmd);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out*4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_out*4500);
    
    output=Vector3f(gyro_latest.x,aspeed_flt,roll_out_cmd);
    output1=Vector3f(gyro_latest.z,val,0);
    log_write_ang_acc(output,output1);
    
    plane.stabilize_pitch();
    plane.stabilize_yaw();
    // stabilize_stick_mixing_direct();
    output_pilot_throttle();
}

float ModeNewMode::update_actuator_dynamics(float cmd_norm){

 
    float delayed_cmd;
    if (_delay_samples == 0) {
        delayed_cmd = cmd_norm;  // 无延迟
    } else {
        // === Step 1: 纯延迟环节（环缓冲实现）===
        // 1.2 读取延迟后的指令（注意：首次延迟未填满时返回0或保持初值）
        delayed_cmd = _cmd_buffer[_buf_cmd_idx];
        // _buf_read_idx = (_buf_cmd_idx + 1) % (_delay_samples);
        // 1.1 写入当前指令到缓冲区
        _cmd_buffer[_buf_cmd_idx] = cmd_norm;
        _buf_cmd_idx = (_buf_cmd_idx + 1) % (_delay_samples);

    }
    
    // === Step 2: 一阶惯性环节 ===
    // 离散形式: y[k] = α·y[k-1] + (1-α)·u[k]
    _actuator_state = _actuator_alpha * _actuator_state + 
                      (1.0f - _actuator_alpha) * delayed_cmd;
    
    return _actuator_state;

}

void ModeNewMode::log_write_ang_acc(Vector3f vec1,Vector3f vec2){

    struct log_AngAcc pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ANGAC_MSG),
        time_us                : AP_HAL::micros64(),
        AGAC_x_flt     : vec1.x,
        AGAC_y_flt   : vec1.y,
        AGAC_z_flt             : vec1.z,
        AGAC_x_raw     : vec2.x,
        AGAC_y_raw   : vec2.y,
        // AGAC_z_raw             : vec2.z,
 
    };
    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
bool ModeNewMode::_enter()
{
    _gyro_prev = ahrs.get_gyro_latest();
    roll_out_real = SRV_Channels::get_output_norm(SRV_Channel::k_aileron);
    
    ARSP_filter.set_cutoff_frequency(50,cutoff_frequency);
    _roll_output_filter.set_cutoff_frequency(50,cutoff_frequency);
    _ang_acc_filter.set_cutoff_frequency(50,cutoff_frequency);
    lms_filter.acc_x_filter.set_cutoff_frequency(50,lms_cutoff_freq);
    lms_filter.output_filter.set_cutoff_frequency(50,lms_cutoff_freq);
    roll_out_flt = roll_out_real;
    delta_u.zero();
    output.zero();
    output1.zero();
    
    return true;
}