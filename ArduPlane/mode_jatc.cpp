#include "mode.h"
#include "Plane.h"
#include <AP_QPOASES/AP_QPOASES_Test.h>

 
const AP_Param::GroupInfo ModeJATC::var_info[] = {
    AP_GROUPINFO("rate_gain",    1, ModeJATC, rate_gain, 7.0f),
    AP_GROUPINFO("angle_pgain",    2, ModeJATC, angle_P_gain, 3.0f),
    AP_GROUPINFO("C_l_delta_a",    3, ModeJATC, C_l_delta_a, 0.25f),
    AP_GROUPINFO("S_wing",    4, ModeJATC, S_wing, 0.2325f),
    AP_GROUPINFO("b",    5, ModeJATC, b, 0.93f),
    AP_GROUPINFO("sigma",    6, ModeJATC, sigma,0.5f),
    AP_GROUPINFO("max_inc",    7, ModeJATC, max_increment,0.2f),
    AP_GROUPINFO("auto_flap",    8, ModeJATC, auto_flap,0), 
    AP_GROUPINFO("cutf_freq",    9, ModeJATC, cutoff_frequency,20), 
    AP_GROUPINFO("L1_freq",    10, ModeJATC, L1_frequency,20), 
    AP_GROUPINFO("C_m_delta_e",    11, ModeJATC, C_m_delta_e, 0.25f),
    AP_GROUPEND
};

void ModeJATC::update()
{
    // set nav_roll and nav_pitch using sticks
    // plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    target_w = -plane.channel_pitch->norm_input()*3;
    target_u = plane.channel_throttle->norm_input()*20;
    // printf("desired_w_u=%f,%f",target_w,target_u);
    plane.nav_pitch_cd=0;
    // plane.update_load_factor();
    // plane.adjust_nav_pitch_throttle();
    // plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    // if (plane.fly_inverted()) {
    //     plane.nav_pitch_cd = -plane.nav_pitch_cd;
    // }
    // if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
    //     // FBWA failsafe glide
    //     plane.nav_roll_cd = 0;
    //     plane.nav_pitch_cd = 0;
    //     SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    // }
    // RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    // if (chan != nullptr) {
    //     // check for the user enabling FBWA taildrag takeoff mode
    //     bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
    //     if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
    //         if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
    //             plane.auto_state.fbwa_tdrag_takeoff_mode = true;
    //             plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
    //         }
    //     }
    // }
}

void ModeJATC::run()
{
    Vector3f gyro_latest = ahrs.get_gyro_latest(); 
    float aspeed=0.0;
    Vector3f vel_ned;
    actuator_dynamic_update();
    if (ahrs.get_velocity_NED(vel_ned)){
    // const Matrix3f &dcm = ahrs.get_rotation_body_to_ned();
    Vector3f target_vel_h(target_u,0,target_w);
    const Vector2f vel_horizontal =
        ahrs.earth_to_body2D(vel_ned.xy());
    const Vector3f vel_heading(
        vel_horizontal.x,  // 沿航向的水平速度
        vel_horizontal.y,  // 航向右侧的水平速度
        vel_ned.z);        // 向下速度
    plane.nav_roll_cd=plane.channel_roll->norm_input() * rad_to_cd(atan(target_u*0.5/GRAVITY_MSS));
    Vector3f target_vel_acc_h=(target_vel_h-vel_heading)*2.0;//期望的航向坐标系下速度导数
    // const float phi = ahrs.get_roll_rad();
    float phi = cd_to_rad(plane.nav_roll_cd);
    const float theta = ahrs.get_pitch_rad();
    const float N=sinf(phi)*((GRAVITY_MSS-target_vel_acc_h.z)*cosf(theta)-target_vel_acc_h.x*sinf(theta))-target_vel_acc_h.y*cosf(phi);
    const float D=vel_heading.x*cosf(phi)-vel_heading.y*sinf(phi)*sinf(theta);
    float desired_psi_dot;
    // if (D<0.01f)
    //     desired_psi_dot=0;
    // else
    desired_psi_dot = constrain_float(N*D/MAX(D*D,1),-0.5f,0.5f);
    
    
    Vector3f   vel_accel_earth_cmd(
        target_vel_acc_h.x-desired_psi_dot*vel_horizontal.y,
        target_vel_acc_h.y+desired_psi_dot*vel_horizontal.x,
        target_vel_acc_h.z
    );
    const Vector2f accel_ne =
        ahrs.body_to_earth2D(vel_accel_earth_cmd.xy());

    const Vector3f target_vel_acc_body =
        ahrs.earth_to_body(
            Vector3f(accel_ne.x, accel_ne.y,
                    vel_accel_earth_cmd.z));
    const Vector3f specific_force_body = ahrs.get_accel() - ahrs.get_accel_bias();
    const Matrix3f& R_body_to_ned = ahrs.get_rotation_body_to_ned();
    const Vector3f gravity_body =  R_body_to_ned.transposed() * Vector3f(0, 0, GRAVITY_MSS);
    vel_accel_body = specific_force_body + gravity_body;
    Vector3f vel_accel_flt=_vel_acc_filter.apply(vel_accel_body);
    Vector3f delta_vel_acc=target_vel_acc_body-vel_accel_flt;
    
    if (!ahrs.airspeed_estimate(aspeed)){
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO,"ahrs_aspeed=%f",aspeed);   
        aspeed=ahrs.groundspeed();
    } 
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO,"aspeed=%f",aspeed);  
    aspeed=MAX(0,aspeed);
    aspeed=MIN(plane.aparm.airspeed_max,aspeed);
    float aspeed_flt=ARSP_filter.apply(aspeed);

    float desired_phi_dot=angle_P_gain*cd_to_rad(plane.nav_roll_cd-ahrs.roll_sensor);
    Vector3f ang_acc_raw=(gyro_latest -_gyro_prev)/AP::scheduler().get_loop_period_s() ;                                                             
    Vector3f ang_acc_flt =_ang_acc_filter.apply(ang_acc_raw);
    // float desired_psi_dot=GRAVITY_MSS/aspeed_flt*tanf(cd_to_rad(plane.nav_roll_cd))*cosf(cd_to_rad(plane.nav_pitch_cd));
    float desired_theta_dot=angle_P_gain*cd_to_rad(plane.nav_pitch_cd-ahrs.pitch_sensor);
    _gyro_prev=gyro_latest;
    float desired_q=cosf(cd_to_rad(plane.nav_roll_cd))*desired_theta_dot+cosf(cd_to_rad(plane.nav_pitch_cd))*sinf(cd_to_rad(plane.nav_roll_cd))*desired_psi_dot;
    float desired_p=desired_phi_dot-sinf(cd_to_rad(plane.nav_pitch_cd))*desired_psi_dot;
    float desired_r=-sinf(cd_to_rad(plane.nav_roll_cd))*desired_theta_dot+cosf(cd_to_rad(plane.nav_pitch_cd))*cosf(cd_to_rad(plane.nav_roll_cd))*desired_psi_dot;
    Vector3f rate_ref(desired_p,desired_q,desired_r);
    Vector3f target_ang_acc =  (rate_ref-gyro_latest)*rate_gain ;
    delta_ang_acc=target_ang_acc-ang_acc_flt; 
    Vector3f delta_long_v(delta_vel_acc.x,-delta_vel_acc.z,delta_ang_acc.y);
    Vector2f delta_lateral_v(delta_ang_acc.x,delta_ang_acc.z);
    if(!longitudinal_update(delta_long_v,aspeed_flt)){
        
        return;
    }
    lateral_update(delta_lateral_v,aspeed_flt);
    setup_actuator();
     
    AP::logger().Write("JATC", "TimeUS,sev_cmd,sev_real,sev_flt,d_psi_dem", "Qffff",
                                AP_HAL::micros64(),
                                servo_actuator.cmd.x,
                            servo_actuator.state.x,
                            servo_actuator.state_flt.x,
                            desired_psi_dot);
    }else{
        plane.stabilize_roll();    
        plane.stabilize_pitch();
        plane.stabilize_yaw();
        }

}   

bool ModeJATC::_enter()
{
    _gyro_prev = ahrs.get_gyro_latest();
    roll_out_real = SRV_Channels::get_output_norm(SRV_Channel::k_aileron);
    float loop_rate=AP::scheduler().get_loop_rate_hz();
    ARSP_filter.set_cutoff_frequency(loop_rate,cutoff_frequency);
    control_surface_filter.set_cutoff_frequency(loop_rate,cutoff_frequency);
    _ang_acc_filter.set_cutoff_frequency(loop_rate,cutoff_frequency);
    _vel_acc_filter.set_cutoff_frequency(loop_rate,cutoff_frequency);
    L1_filter.set_cutoff_frequency(loop_rate,L1_frequency);
    motor_actuator.alpha=expf(-AP::scheduler().get_loop_period_s()/0.02);
    control_surface_actuator.alpha=expf(-AP::scheduler().get_loop_period_s()/0.02);
    servo_actuator.alpha=expf(-AP::scheduler().get_loop_period_s()/0.1);
    motor_actuator.state_flt = motor_actuator.state = motor_actuator.cmd = Vector3f(
         SRV_Channels::get_output_scaled(SRV_Channel::k_motor1),
         SRV_Channels::get_output_scaled(SRV_Channel::k_motor2),
         SRV_Channels::get_output_scaled(SRV_Channel::k_motor3))/4500;
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL    
    servo_actuator.state_flt = servo_actuator.state = servo_actuator.cmd =Vector2f(
         AP::sitl()->get_rangefinder(0),0);
    #endif     

    control_surface_actuator.state_flt = control_surface_actuator.state = control_surface_actuator.cmd =Vector2f(
        SRV_Channels::get_output_scaled(SRV_Channel::k_elevator),
        SRV_Channels::get_output_scaled(SRV_Channel::k_aileron ))/ 4500.0f;
    SRV_Channels::set_range(SRV_Channel::k_motor1, 4500);
    SRV_Channels::set_range(SRV_Channel::k_motor2, 4500);
    SRV_Channels::set_range(SRV_Channel::k_motor3, 4500);
    // if (!test_qpoases()) {
    // plane.gcs().send_text(
    //     MAV_SEVERITY_ERROR,
    //     "JATC: QP self-test failed");

    // return false;
    // }
    return true;
}

void ModeJATC::lateral_update(Vector2f delta_v,float aspeed){
    //左右电机差动量，副翼
    float c=cosf(servo_actuator.state_flt.x);
    float s=sinf(servo_actuator.state_flt.x);
    float ddT_du_F=22;
    if (motor_actuator.state_flt.x+motor_actuator.state_flt.y<0.001) 
        ddT_du_F=22;
    else{
        ddT_du_F=(2*19.2389*motor_actuator.state_flt.x+2*19.2389*motor_actuator.state_flt.y)/2;
    }
    
 
    Vector2f A(s*74.195/(74.195*L*ddT_du_F),c*74.195/(74.195*L*ddT_du_F));
    Vector2f B(c*ddT_du_F*L/(74.195*L*ddT_du_F), -s*ddT_du_F*L/(74.195*L*ddT_du_F));
    Vector2f C(Ixx*delta_v.x-Ixz*delta_v.y,-Ixz*delta_v.x+Izz*delta_v.y);
    front_diff = constrain_float(A*C,-motor_actuator.max_increment.x-motor_actuator.max_increment.y,motor_actuator.max_increment.x+motor_actuator.max_increment.y);
    float delta_aileron=constrain_float(B * C,-control_surface_actuator.max_increment.y,control_surface_actuator.max_increment.y);
    control_surface_actuator.cmd.y = constrain_float(control_surface_actuator.state_flt.y+delta_aileron,control_surface_actuator.min.y,control_surface_actuator.max.y); 
}

bool ModeJATC::longitudinal_update(Vector3f delta_v,float aspeed){
    using qpOASES::real_t;
    float pre_fac=0.5*1.225*aspeed*aspeed*S_wing;
    constexpr int NV = 4;
    constexpr int NY = 3;
    float T=MAX(19.2389 *motor_actuator.state_flt.x*motor_actuator.state_flt.x-0.0546*motor_actuator.state_flt.x-0.0589+\
    19.2389 *motor_actuator.state_flt.y*motor_actuator.state_flt.y-0.0546*motor_actuator.state_flt.y-0.0589,0);
    float c=cosf(servo_actuator.state_flt.x);
    float s=sinf(servo_actuator.state_flt.x);
    float dT_du_F=22;
    if (motor_actuator.state_flt.x+motor_actuator.state_flt.y<0.01){
        dT_du_F=22;
    }else{
        dT_du_F=(2*19.2389*motor_actuator.state_flt.x+2*19.2389*motor_actuator.state_flt.y)/2;
    }
    
    float dT_du_R;
    if (motor_actuator.state_flt.z<0.01){
        dT_du_R=8.32;
    }else{
        dT_du_R= 7.4326*motor_actuator.state_flt.z;
    }
    
    // float dT_du_F= 2*22.85;
    // float dT_du_R= 8.32;
    float  G[] = {dT_du_F*c/mass,0,-s*T/mass,0,\
        dT_du_F*s/mass, dT_du_R/mass,(c*T+pre_fac*CL_alpha)/mass,0,\
        s*h1*dT_du_F/Iyy, -h2*dT_du_R/Iyy,c*T*h1/Iyy,pre_fac*C_m_delta_e/Iyy};
    const real_t v[NY] = {delta_v.x, delta_v.y, delta_v.z};
    const real_t weight[NY] = {1.0, 1.0, 1.0}; // 按各行量纲调整
    constexpr real_t lambda = 1.0e-3;

    real_t H[NV * NV] = {};
    real_t gradient[NV] = {};

    for (int i = 0; i < NV; i++) {
        for (int k = 0; k < NY; k++) {
            gradient[i] -= G[k * NV + i] * weight[k] * v[k];
            for (int j = 0; j < NV; j++) {
                H[i * NV + j] += G[k * NV + i] *
                                weight[k] *
                                G[k * NV + j];
            }
        }
        H[i * NV + i] += lambda;
    }
    //前螺旋桨合力，后螺旋桨拉力，倾斜角，升降舵偏转角
    
    real_t lb[NV]={0,0,0,0};
    real_t ub[NV]={0,0,0,0};

    lb[0]=-MIN((motor_actuator.state_flt.x+motor_actuator.state_flt.y-motor_actuator.min.x-motor_actuator.min.y),2*motor_actuator.max_increment.x);
    lb[1]=-MIN((motor_actuator.state_flt.z-motor_actuator.min.z),motor_actuator.max_increment.z);
    lb[2]=-MIN((servo_actuator.state_flt.x-servo_actuator.min.x),servo_actuator.max_increment.x);
    lb[3]=-MIN((control_surface_actuator.state_flt.x-control_surface_actuator.min.x),control_surface_actuator.max_increment.x);

    ub[0]=MIN((motor_actuator.max.x+motor_actuator.max.y-motor_actuator.state_flt.x-motor_actuator.state_flt.y),2*motor_actuator.max_increment.x);
    ub[1]=MIN((motor_actuator.max.z-motor_actuator.state_flt.z),motor_actuator.max_increment.z);
    ub[2]=MIN((servo_actuator.max.x-servo_actuator.state_flt.x),servo_actuator.max_increment.x);
    ub[3]=MIN((control_surface_actuator.max.x-control_surface_actuator.state_flt.x),control_surface_actuator.max_increment.x);

    qpOASES::QProblemB qp(NV, qpOASES::HST_POSDEF);
    static  qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);

    real_t delta_u[NV] = {};
    qpOASES::int_t nWSR = 30;
    bool solved = qp.init(H, gradient, lb, ub, nWSR) ==
                qpOASES::SUCCESSFUL_RETURN;
    if (solved) {
        solved = qp.getPrimalSolution(delta_u) ==
                qpOASES::SUCCESSFUL_RETURN;
    }
    if (!solved) {
        Matrix3<real_t> M{};
        for (int r = 0; r < NY; ++r) {
            for (int i = 0; i < NY; ++i) {
                for (int j = 0; j < NV; ++j) {
                    M[r][i] += real_t(G[r * NV + j]) * G[i * NV + j];
                }
            }
            M[r][r] += lambda;
        }
        Matrix3<real_t> inv;
        if (!M.inverse(inv)) {
            return false;
        }
        const Vector3<real_t> y = inv * Vector3<real_t>(v[0], v[1], v[2]);
        for (int j = 0; j < NV; ++j) {
            delta_u[j] = 0;
            for (int r = 0; r < NY; ++r) {
                delta_u[j] += real_t(G[r * NV + j]) * y[r];
            }
        }
    }
    for (int j = 0; j < NV; ++j) {
        if (!isfinite(delta_u[j])) {
            return false;
        }
        delta_u[j] = MAX(lb[j], MIN(ub[j], delta_u[j]));
    }
    front_tot=delta_u[0];
    
    motor_actuator.cmd.z=constrain_float(motor_actuator.state_flt.z+delta_u[1],motor_actuator.min.z,motor_actuator.max.z);
    servo_actuator.cmd.x=constrain_float(servo_actuator.state_flt.x+delta_u[2],servo_actuator.min.x,servo_actuator.max.x);
    control_surface_actuator.cmd.x=constrain_float(control_surface_actuator.state_flt.x+delta_u[3],control_surface_actuator.min.x,control_surface_actuator.max.x);
    return true;
}
void ModeJATC::actuator_dynamic_update(){

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    motor_actuator.state =  motor_actuator.state* motor_actuator.alpha + 
                    motor_actuator.cmd*(1.0 - motor_actuator.alpha) ;
    servo_actuator.state.x=AP::sitl()->get_rangefinder(0);
    #elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // STM32H743 等 ChibiOS 飞控的执行器输出代码
 
    #endif     
    // printf("servo=%f",AP::sitl()->get_rangefinder(0));
    // servo_actuator.state =  servo_actuator.state* servo_actuator.alpha + 
    //                  servo_actuator.cmd*(1.0 - servo_actuator.alpha) ;
    control_surface_actuator.state =  control_surface_actuator.state* control_surface_actuator.alpha + 
    control_surface_actuator.cmd*(1.0 - control_surface_actuator.alpha) ;

    motor_actuator.state_flt=motor_filter.apply(motor_actuator.state);
    servo_actuator.state_flt=servo_filter.apply(servo_actuator.state);
    control_surface_actuator.state_flt=control_surface_filter.apply(control_surface_actuator.state);

 
}
void ModeJATC::setup_actuator()
{
    motor_actuator.cmd.x=constrain_float(motor_actuator.state_flt.x+(front_tot+front_diff)/2,motor_actuator.min.x,motor_actuator.max.x);
    motor_actuator.cmd.y=constrain_float(motor_actuator.state_flt.y+(front_tot-front_diff)/2,motor_actuator.min.y,motor_actuator.max.y);
    

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, motor_actuator.cmd.x*4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, motor_actuator.cmd.y*4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, motor_actuator.cmd.z*4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, control_surface_actuator.cmd.x*4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, control_surface_actuator.cmd.y*4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, (servo_actuator.cmd.x/2.0f*3.0f/3.14159265f)*1000);
    // printf("motor_actuator_x_y=%f,%f",motor_actuator.cmd.x,motor_actuator.cmd.y);
    #elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // STM32H743 等 ChibiOS 飞控的执行器输出代码

 
    #endif
}

void ModeJATC::state_estimate(float d_u,float x,float Gn,float dot_x){

    Vector3f ep=hat_x-x;


    float expm=expf(-Kp*AP::scheduler().get_loop_period_s());
    float Phi=(expm-1)/Kp;
    float miu=expm*ep;
    hat_sigma=miu/Phi/Gn;
    
    float dot_hat_x=dot_x+Gn*(d_u+hat_sigma)-Kp*ep;//更新状态估计器
    hat_x+=(dot_hat_x*AP::scheduler().get_loop_period_s());
    AP::logger().Write("L1", "TimeUS,ep,hsigma", "Qff",
                                        AP_HAL::micros64(),
                                        ep,
                                    hat_sigma);
}

bool ModeJATC::test_qpoases()
{
    AP_QPOASES::BasicTestResult result;

    const bool passed = AP_QPOASES::run_basic_test(result);

    if (!passed) {
        plane.gcs().send_text(
            MAV_SEVERITY_ERROR,
            "JATC QP FAIL st=%d it=%d",
            static_cast<int>(result.status),
            static_cast<int>(result.iterations));

        return false;
    }

    plane.gcs().send_text(
        MAV_SEVERITY_INFO,
        "JATC QP PASS it=%d",
        static_cast<int>(result.iterations));

    plane.gcs().send_text(
        MAV_SEVERITY_INFO,
        "QP x=%.3f %.3f J=%.3f",
        static_cast<double>(result.solution[0]),
        static_cast<double>(result.solution[1]),
        static_cast<double>(result.objective));

    return true;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
bool ModeJATC::poll_servo_position(uint16_t &raw_position)
{
    AP_HAL::UARTDriver *uart = hal.serial(5);
    if (uart == nullptr || !uart->is_initialized()) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();

    // 每次只处理有限的已到达字节，不等待新字节。
    for (uint8_t n = 0; n < 32 && uart->available() > 0; n++) {
        uint8_t byte;
        if (!uart->read(byte)) {
            break;
        }
        if (!servo_waiting) {
            continue;
        }

        if (servo_rx_count == 0) {
            if (byte == 0xFF) {
                servo_rx[servo_rx_count++] = byte;
            }
        } else if (servo_rx_count == 1) {
            if (byte == 0xF5) {
                servo_rx[servo_rx_count++] = byte;
            } else if (byte != 0xFF) {
                servo_rx_count = 0;
            }
        } else {
            servo_rx[servo_rx_count++] = byte;
            if (servo_rx_count != sizeof(servo_rx)) {
                continue;
            }

            servo_rx_count = 0;
            servo_waiting = false;

            uint8_t sum = 0;
            for (uint8_t i = 2; i < 7; i++) {
                sum += servo_rx[i];
            }
            const uint8_t checksum = static_cast<uint8_t>(~sum);
            const uint16_t raw =
                (uint16_t(servo_rx[5]) << 8) | servo_rx[6];

            if (servo_rx[2] == 0x01 && servo_rx[3] == 0x04 &&
                servo_rx[4] == 0x00 && servo_rx[7] == checksum &&
                raw <= 4095) {
                raw_position = raw;
                servo_last_good_ms = now;
                return true;
            }
        }
    }

    if (servo_waiting && now - servo_request_ms >= 30) {
        servo_waiting = false;
        servo_rx_count = 0;
    }

    // 最多每 20 ms 查询一次；超时后也不会阻塞等待。
    if (!servo_waiting && now - servo_request_ms >= 20 &&
        uart->txspace() >= 8) {
        static const uint8_t request[8] =
            {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
        uart->discard_input();
        if (uart->write(request, sizeof(request)) == sizeof(request)) {
            servo_request_ms = now;
            servo_waiting = true;
        }
    }
    return false;
}
#endif