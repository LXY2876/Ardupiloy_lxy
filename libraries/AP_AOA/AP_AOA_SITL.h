/*
  SITL airspeed backend - a perfect airspeed sensor
 */
#pragma once

#include "AP_AOA_config.h"

#if AP_AOA_SITL_ENABLED

#include "AP_AOA_Backend.h"

class AP_Aoa_SITL : public AP_Aoa_Backend
{
public:

    using AP_Aoa_Backend::AP_Aoa_Backend;

    bool init(void) override {
        return true;
    }

  
    bool get_differential_pressure(float &pressure) override;
    bool get_temperature(float &temperature) override;
 

private:
};

#endif // AP_AOA_SITL_ENABLED
