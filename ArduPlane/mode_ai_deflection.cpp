#include "mode.h"
#include "Plane.h"

ModeAI_Deflection::ModeAI_Deflection()
{
    
}

bool ModeAI_Deflection::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;
    
    _servoOutput.stickAileron = plane.channel_roll->get_control_mid();
    _servoOutput.stickElevator = plane.channel_pitch->get_control_mid();
    _servoOutput.stickThrottle = plane.channel_throttle->get_control_mid();
    _servoOutput.stickRudder = plane.channel_rudder->get_control_mid();

    return true;
}

void ModeAI_Deflection::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}

void ModeAI_Deflection::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, _servoOutput.stickAileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _servoOutput.stickElevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _servoOutput.stickThrottle);

    plane.steering_control.steering = plane.steering_control.rudder = _servoOutput.stickRudder;
    AP::logger().WriteAI(AP_HAL::micros64(), 1, 1000);

/*
    manual_override(plane.channel_roll, packet.y, 1000, 2000, tnow);
    manual_override(plane.channel_pitch, packet.x, 1000, 2000, tnow, true);
    manual_override(plane.channel_throttle, packet.z, 0, 1000, tnow);
    manual_override(plane.channel_rudder, packet.r, 1000, 2000, tnow);
    */
}

bool ModeAI_Deflection::is_AI_control() const
{
    return true;
}

bool ModeAI_Deflection::handleLongCommand(const mavlink_command_long_t &packet)
{
    printf("We are going to handle the command.");

    //take the params and map them to the servos
    return true;
}
