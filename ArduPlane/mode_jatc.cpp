#include "mode.h"
#include "Plane.h"

const AP_Param::GroupInfo ModeJATC::var_info[] = {
     
    AP_GROUPINFO("auto_jet",    1, ModeJATC, use_auto_jet, 0.0f),
    AP_GROUPINFO("min_servo",    2, ModeJATC, min_servo_value, 1000),
    AP_GROUPINFO("max_servo",    3, ModeJATC, max_servo_value, 2000),
    AP_GROUPINFO("jet2theta_I",    4, ModeJATC, jet2theta_I, 0.05),
    AP_GROUPINFO("jet2theta_P",    5, ModeJATC, jet2theta_P, 0.05),
    AP_GROUPEND
};

bool ModeJATC::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();
    jet_last_output=SRV_Channels::get_output_norm(SRV_Channel::k_flaperon_left);
   
    // SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto,2000);
    printf("last_jet_out=%f",jet_last_output);
    return true;
}

void ModeJATC::update()
{
    
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();
    int32_t commanded_pitch=plane.TECS_controller.get_pitch_pid_demand();
    plane.nav_pitch_cd = constrain_int32(commanded_pitch, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    float aspeed=plane.airspeed.get_airspeed();
    aspeed=constrain_float(aspeed,plane.aparm.airspeed_min,plane.aparm.airspeed_max);
    float az_des=4*(plane.TECS_controller.get_hgtdot_dem()-plane.TECS_controller.get_climb_rate());
    az_des=constrain_float(az_des,-7.0f,7.0f);
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    float az_body = -(AP::ins().get_accel().z + rotMat.c.z * GRAVITY_MSS);
    float delta_v=az_des-az_body;
    float delta_jet=2*delta_v/(0.5*1.225*aspeed*0.2325*6.5/2);
    // delta_jet=constrain_float(delta_jet,-0.1,0.1);
    float jet_output_cmd=jet_output_real+delta_jet;
    jet_output_cmd=constrain_float(jet_output_cmd,-0.5,0.5);
    jet_output_real=(1-0.67)*jet_output_cmd+0.67*jet_last_output;
    jet_last_output=jet_output_real;
    float interjet_cmd_error_filter=(0-jet_output_cmd);
    const bool inhibit_integrator =
        (integ_jet2theta >= 0.1 && interjet_cmd_error_filter < 0.0f) ||
        (integ_jet2theta <= -0.1 && interjet_cmd_error_filter > 0.0f);
    if (!inhibit_integrator) {
        
        integ_jet2theta=integ_jet2theta-(jet2theta_P*interjet_cmd_error_filter+jet2theta_I*interjet_cmd_error_filter*AP::scheduler().get_loop_period_s()); 
    }

    
    integ_jet2theta=constrain_float(integ_jet2theta,-0.1,0.1);
    // printf("integ_jet2theta=%f,jet_output_cmd=%f\n",integ_jet2theta,jet_output_cmd);
  
    AP::logger().Write("JATC", "TimeUS,hdot_dem,hdot,az_des,az_body,deltav,deltau", "Qffffff",
                                        AP_HAL::micros64(),
                                        plane.TECS_controller.get_hgtdot_dem(),
                                        plane.TECS_controller.get_climb_rate(),
                                        az_des,
                                        az_body,
                                        delta_v,
                                        delta_jet
                                        );
}   
 
 