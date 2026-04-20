#include "AP_AOA_SITL.h"

#if AP_AOA_SITL_ENABLED

 
#include <SITL/SITL.h>

// return the current differential_pressure in Pascal
bool AP_Aoa_SITL::get_differential_pressure(float &pressure)
{
    const uint8_t _instance = get_instance();
    
    if (_instance >= AIRSPEED_MAX_SENSORS) {
        return false;
    }

    pressure = atan2(AP::sitl()->state.velocity_air_bf.z, AP::sitl()->state.velocity_air_bf.x);

    return true;
}

bool AP_Aoa_SITL::get_temperature(float &temperature)
{
 
    temperature =1.0;

    return true;
}

#endif // AP_AOA_SITL_ENABLED
