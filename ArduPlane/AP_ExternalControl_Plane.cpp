/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  Sets the target global position for a loiter point.
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}

/*
  Sets the target airspeed.
*/
bool AP_ExternalControl_Plane::set_airspeed(const float airspeed)
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // The command is only valid in guided mode.
    if (plane.control_mode != &plane.mode_guided) {
        return false;
    }
    
    // Assume the user wanted maximum acceleration.
    const float acc_instant = 0.0;
    return plane.mode_guided.handle_change_airspeed(airspeed, acc_instant);
#else 
  return false;
#endif
}

bool AP_ExternalControl_Plane::set_attitude(const Vector3f &attitude)
{
  // hal.console->printf("target_roll=%f\n", (float)attitude.x);
  uint32_t now = AP_HAL::millis();
  plane.guided_state.last_forced_rpy_ms.x = now;
  
  plane.guided_state.forced_rpy_cd.x = degrees(attitude.x) * 100.0f;
  plane.guided_state.last_forced_rpy_ms.y = now;
  plane.guided_state.forced_rpy_cd.y=degrees(attitude.y) * 100.0f;
  plane.set_target_altitude_location(plane.current_loc);
  // plane.set_target_altitude_location(plane.current_loc); // 始终保持当前高度
  return true;

}

#endif // AP_EXTERNAL_CONTROL_ENABLED
