#include "LMS_filter.h"
#include "Plane.h"
 
float LMS::apply(float G,float v, float u_flt,float K_flt){
    if (!initialised) {
        initialised=true;
        Gain=G;
        last_v=v;
        llast_u_flt=u_flt;
        last_K_flt=K_flt;
        return Gain;
    }
    else{
        float delta_u=output_filter.apply(last_u_flt-llast_u_flt);
        float delta_v=acc_x_filter.apply(v-last_v);
        float error=(Gain*last_K_flt*delta_u-delta_v);
        float input_power = last_K_flt*(delta_u + 0.001f) * last_K_flt*(delta_u + 0.001f);
        float mu_normalized = miu / input_power;
        if (fabsf(error) > 0.05f && fabsf(delta_u) > 0.01f) {
            
        Gain = Gain - mu_normalized * error * last_K_flt*delta_u;
        printf("Gain=%f",Gain);
        Gain=constrain_float(Gain,GAIN_MIN,GAIN_MAX);
        }
        last_v=v;
        llast_u_flt=last_u_flt;
        last_u_flt=u_flt;
        last_K_flt=K_flt;
        return Gain;
    }
}