/*
  external AHRS aoa backend
 */
#pragma once

#include "AP_AOA_config.h"

#if AP_AOA_EXTERNAL_ENABLED

#include "AP_AOA_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

class AP_Aoa_External : public AP_Aoa_Backend
{
public:
    AP_Aoa_External(AP_Aoa &aoa, uint8_t instance);

    bool init(void) override {
        return true;
    }

    void handle_external(const AP_ExternalAHRS::aoa_data_message_t &pkt) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;
    
private:
    float sum_pressure;
    uint8_t press_count;
    float sum_temperature;
    uint8_t temperature_count;
};

#endif // AP_AOA_EXTERNAL_ENABLED

