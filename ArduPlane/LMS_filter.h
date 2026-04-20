#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
class LMS//自适应最小均方滤波器
{
    // friend class Plane;
public:
    LowPassFilter2pFloat acc_x_filter{50,13};
    LowPassFilter2pFloat output_filter{50,13}; 
    float miu=0.005;
    float Gain;
    float last_v;
    float last_K_flt;
    
    float last_u_flt;
    float llast_u_flt;
    bool initialised=false;
    // 定义物理合理的增益范围
    const float GAIN_MIN = 0.1f;  // 例如：最小不能低于标称值的 50%
    const float GAIN_MAX = 1.2f;  // 例如：最大不能高于标称值的 200%
    uint32_t last_time;
    float apply(float G,float delta_v, float u_flt,float K_flt);

};