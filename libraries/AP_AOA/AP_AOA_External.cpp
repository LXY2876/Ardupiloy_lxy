#include "AP_AOA_External.h"

#if AP_AOA_EXTERNAL_ENABLED

AP_Aoa_External::AP_Aoa_External(AP_Aoa &_frontend, uint8_t _instance) :
    AP_Aoa_Backend(_frontend, _instance)
{
    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,0,_instance,0));
}

// return the current differential_pressure in Pascal
bool AP_Aoa_External::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);
    if (press_count == 0) {
        return false;
    }
    pressure = sum_pressure/press_count;
    press_count = 0;
    sum_pressure = 0;
    return true;
}

// get last temperature
bool AP_Aoa_External::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);
    if (temperature_count == 0) {
        return false;
    }
    temperature = sum_temperature/temperature_count;
    temperature_count = 0;
    sum_temperature = 0;
    return true;
}

void AP_Aoa_External::handle_external(const AP_ExternalAHRS::aoa_data_message_t &pkt)
{
    WITH_SEMAPHORE(sem);

    sum_pressure += pkt.aoa;
    press_count++;
    if (press_count > 100) {
        // prevent overflow
        sum_pressure /= 2;
        press_count /= 2;
    }

    sum_temperature += pkt.aoa;
    temperature_count++;
    if (temperature_count > 100) {
        // prevent overflow
        sum_temperature /= 2;
        temperature_count /= 2;
    }
}

#endif // AP_AOA_EXTERNAL_ENABLED
